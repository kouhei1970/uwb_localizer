"""UWB シミュレータ — ハードがなくてもアルゴリズムを評価する.

実機の HAL と**同じ** :class:`~uwb_loc.hal.base.UwbHal` を実装しているので,
測位側のコードは実機かシミュレータかを知らない. UI もテストも同じ経路を通る.

誤差要因は実運用で効く順に入れてある.

* 測距ノイズ (距離比例項つき)
* **NLOS** — 見通しが切れると距離は正側にしか伸びない. 2 状態マルコフ連鎖で
  時間相関を持たせてある (実際の遮蔽は 1 サンプルで消えたりしない)
* パケットロス, 更新レート
* アンテナ遅延の取り残し, アンカー設置座標の誤差
  (設営の失敗がどれくらい効くかを見るためのもの)
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from .hal.base import UwbHal
from .types import Anchor, MeasKind, Measurement, MeasurementBatch

__all__ = [
    "ErrorModel",
    "SimulatedHal",
    "Scenario",
    "trajectory",
    "make_anchors",
    "room_anchors",
]

Trajectory = Callable[[float], np.ndarray]


# --------------------------------------------------------------------------- 誤差


@dataclass
class ErrorModel:
    """測距の誤差モデル.

    Attributes
    ----------
    sigma0, sigma_per_m:
        測距ノイズ [m] と距離比例分 [m/m]. 実効 σ は ``sigma0 + sigma_per_m*d``.
    nlos_prob:
        あるリンクが NLOS 状態にある定常確率.
    nlos_hold:
        NLOS 状態の平均継続時間 [s]. 遮蔽は数サンプル続くのが普通なので,
        毎サンプル独立に振ると NLOS の難しさを過小評価してしまう.
    nlos_bias_mean:
        NLOS 時に上乗せされる距離の平均 [m] (指数分布, 正側のみ).
    loss_rate:
        パケットロス率.
    max_range:
        これを超えるリンクは受信できない [m].
    antenna_delay:
        実機のアンテナ遅延 [m]. 測距値に一律で乗る. 測位側の
        :attr:`Anchor.antenna_delay_m` で補正しなければ, そのまま系統誤差になる.
    anchor_position_error:
        アンカーの設置座標誤差 [m] (1σ). 測位側には公称座標を渡し,
        真の測距はずれた座標から計算する.
    report_sigma / report_quality:
        HAL が σ / 品質値を申告するか. 実機でこれを上げてこないチップを
        模擬したいときに False にする.
    """

    sigma0: float = 0.08
    sigma_per_m: float = 0.004
    nlos_prob: float = 0.15
    nlos_hold: float = 1.5
    nlos_bias_mean: float = 0.8
    loss_rate: float = 0.03
    max_range: float = 40.0
    antenna_delay: float = 0.0
    anchor_position_error: float = 0.0
    report_sigma: bool = True
    report_quality: bool = True

    def sigma_at(self, d: float) -> float:
        """その距離での測距標準偏差 [m]."""
        return self.sigma0 + self.sigma_per_m * d


# --------------------------------------------------------------------------- 軌道


class trajectory:  # noqa: N801 - 名前空間として使う
    """よく使う軌道."""

    @staticmethod
    def static(p: np.ndarray) -> Trajectory:
        """静止."""
        p = np.asarray(p, dtype=float).reshape(3)
        return lambda t: p.copy()

    @staticmethod
    def line(p0: np.ndarray, p1: np.ndarray, period: float = 20.0) -> Trajectory:
        """2 点間を往復."""
        p0 = np.asarray(p0, dtype=float).reshape(3)
        p1 = np.asarray(p1, dtype=float).reshape(3)

        def f(t: float) -> np.ndarray:
            s = 0.5 - 0.5 * math.cos(2.0 * math.pi * t / period)
            return p0 + (p1 - p0) * s

        return f

    @staticmethod
    def circle(
        center: np.ndarray, radius: float = 2.0, period: float = 20.0, z_amp: float = 0.0
    ) -> Trajectory:
        """水平円 (``z_amp`` を与えると螺旋)."""
        center = np.asarray(center, dtype=float).reshape(3)

        def f(t: float) -> np.ndarray:
            w = 2.0 * math.pi * t / period
            return center + np.array(
                [radius * math.cos(w), radius * math.sin(w), z_amp * math.sin(w)]
            )

        return f

    @staticmethod
    def figure8(
        center: np.ndarray, size: float = 2.0, period: float = 24.0, z_amp: float = 0.3
    ) -> Trajectory:
        """8 の字. 加減速と旋回が入るのでフィルタの追従性を見るのに向く."""
        center = np.asarray(center, dtype=float).reshape(3)

        def f(t: float) -> np.ndarray:
            w = 2.0 * math.pi * t / period
            return center + np.array(
                [size * math.sin(w), size * math.sin(w) * math.cos(w), z_amp * math.sin(2.0 * w)]
            )

        return f

    @staticmethod
    def random_walk(
        start: np.ndarray,
        speed: float = 0.5,
        bounds: tuple[tuple[float, float], ...] | None = None,
        seed: int = 0,
        dt: float = 0.05,
    ) -> Trajectory:
        """ランダムウォーク (時刻をキャッシュして再現性を保つ)."""
        start = np.asarray(start, dtype=float).reshape(3)
        rng = np.random.default_rng(seed)
        cache: dict[int, np.ndarray] = {0: start.copy()}
        vel = np.zeros(3)

        def f(t: float) -> np.ndarray:
            k = int(round(t / dt))
            nonlocal vel
            last = max(cache)
            while last < k:
                vel = 0.9 * vel + rng.normal(0.0, speed * 0.3, 3)
                p = cache[last] + vel * dt
                if bounds is not None:
                    for i, (lo, hi) in enumerate(bounds):
                        if p[i] < lo or p[i] > hi:
                            p[i] = float(np.clip(p[i], lo, hi))
                            vel[i] *= -1.0
                last += 1
                cache[last] = p
            return cache[min(k, last)].copy()

        return f


# --------------------------------------------------------------------------- アンカー


def make_anchors(positions: np.ndarray, prefix: str = "A", **kw: object) -> list[Anchor]:
    """座標配列からアンカー一覧を作る."""
    positions = np.atleast_2d(np.asarray(positions, dtype=float))
    return [Anchor(id=f"{prefix}{i}", position=p, **kw) for i, p in enumerate(positions)]  # type: ignore[arg-type]


def room_anchors(
    size: tuple[float, float, float] = (8.0, 6.0, 2.6),
    *,
    n_low: int = 4,
    z_low: float = 0.3,
    z_high: float | None = None,
) -> list[Anchor]:
    """部屋の四隅に上下 2 段でアンカーを置く既定配置.

    3 次元測位では**アンカーが同一平面に並ばないこと**が本質的に効くので,
    既定では高さを 2 段にしてある. ``n_low=0`` にすると天井のみの
    平面配置になり, 高さが観測できなくなる様子を確認できる.
    """
    lx, ly, lz = size
    z_high = lz - 0.2 if z_high is None else z_high
    corners = [(0.2, 0.2), (lx - 0.2, 0.2), (lx - 0.2, ly - 0.2), (0.2, ly - 0.2)]
    pts: list[list[float]] = []
    for i, (x, y) in enumerate(corners):
        pts.append([x, y, z_high])
        if i < n_low:
            pts.append([x, y, z_low])
    return make_anchors(np.array(pts))


# --------------------------------------------------------------------------- HAL


class SimulatedHal(UwbHal):
    """模擬 UWB.

    Parameters
    ----------
    anchors:
        **測位側に渡す公称座標**のアンカー一覧.
    traj:
        真の位置を返す関数 ``t -> (3,)``.
    error:
        誤差モデル.
    rate_hz:
        測位更新レート [Hz].
    kind:
        ``MeasKind.RANGE`` (DS-TWR) か ``MeasKind.TDOA``.
    seed:
        乱数種.
    t0:
        開始時刻 [s].
    """

    def __init__(
        self,
        anchors: list[Anchor],
        traj: Trajectory,
        error: ErrorModel | None = None,
        *,
        rate_hz: float = 10.0,
        kind: MeasKind = MeasKind.RANGE,
        seed: int = 0,
        t0: float = 0.0,
        tag_id: str = "tag0",
    ) -> None:
        self.name = "sim"
        self._anchors = list(anchors)
        self.traj = traj
        self.error = error or ErrorModel()
        self.dt = 1.0 / float(rate_hz)
        self.kind = kind
        self.tag_id = tag_id
        self.rng = np.random.default_rng(seed)
        self.t = float(t0)

        # 真のアンカー座標 (設置誤差を乗せたもの). 測位側は公称値しか知らない.
        self.true_positions = {}
        for a in self._anchors:
            off = (
                self.rng.normal(0.0, self.error.anchor_position_error, 3)
                if self.error.anchor_position_error > 0.0
                else np.zeros(3)
            )
            self.true_positions[a.id] = a.position + off

        # リンクごとの NLOS 状態 (2 状態マルコフ連鎖).
        self._nlos = {a.id: False for a in self._anchors}

    # ------------------------------------------------------------------

    @property
    def anchors(self) -> list[Anchor]:
        return list(self._anchors)

    def truth(self, t: float | None = None) -> np.ndarray:
        """真の位置."""
        return np.asarray(self.traj(self.t if t is None else t), dtype=float).reshape(3)

    # ------------------------------------------------------------------

    def _advance_nlos(self, dt: float) -> None:
        """NLOS 状態を時間相関つきで遷移させる."""
        e = self.error
        if e.nlos_prob <= 0.0:
            return
        # 定常確率 p, 平均継続 T から遷移確率を作る.
        p_on = float(np.clip(e.nlos_prob, 0.0, 0.999))
        hold = max(e.nlos_hold, dt)
        p_off_to_on = dt / hold * p_on / max(1.0 - p_on, 1e-6)
        p_on_to_off = dt / hold
        for k, on in self._nlos.items():
            u = self.rng.random()
            self._nlos[k] = (u > p_on_to_off) if on else (u < p_off_to_on)

    def _range_measurement(self, a: Anchor, p: np.ndarray) -> Measurement | None:
        e = self.error
        d_true = float(np.linalg.norm(p - self.true_positions[a.id]))
        if d_true > e.max_range or self.rng.random() < e.loss_rate:
            return None

        sigma = e.sigma_at(d_true)
        value = d_true + self.rng.normal(0.0, sigma) + e.antenna_delay
        nlos = self._nlos[a.id]
        if nlos and e.nlos_bias_mean > 0.0:
            value += self.rng.exponential(e.nlos_bias_mean)

        # 品質値: 実機の CIR 特徴から作る想定. LOS/NLOS を完璧には
        # 見分けられないので, 分布を重ねてから 0-1 に潰す.
        quality = None
        if e.report_quality:
            base = 0.35 if nlos else 0.85
            quality = float(np.clip(self.rng.normal(base, 0.15), 0.0, 1.0))

        return Measurement(
            anchor_id=a.id,
            value=value,
            kind=MeasKind.RANGE,
            t=self.t,
            sigma=sigma if e.report_sigma else None,
            quality=quality,
            tag_id=self.tag_id,
            raw={"nlos_truth": bool(nlos), "d_true": d_true},
        )

    def step(self) -> tuple[np.ndarray, MeasurementBatch]:
        """1 エポック進めて (真位置, 観測) を返す."""
        p = self.truth()
        self._advance_nlos(self.dt)

        ms: list[Measurement] = []
        for a in self._anchors:
            if not a.enabled:
                continue
            m = self._range_measurement(a, p)
            if m is not None:
                ms.append(m)

        if self.kind is MeasKind.TDOA and len(ms) >= 2:
            ref = ms[0]
            ms = [
                Measurement(
                    anchor_id=m.anchor_id,
                    value=m.value - ref.value,
                    kind=MeasKind.TDOA,
                    t=m.t,
                    sigma=None if m.sigma is None else float(np.hypot(m.sigma, ref.sigma or 0.0)),
                    quality=m.quality,
                    ref_anchor_id=ref.anchor_id,
                    tag_id=self.tag_id,
                    raw=m.raw,
                )
                for m in ms[1:]
            ]

        batch = MeasurementBatch(t=self.t, measurements=ms, tag_id=self.tag_id)
        self.t += self.dt
        return p, batch

    def poll(self, timeout: float = 0.0) -> list[MeasurementBatch]:
        return [self.step()[1]]

    def generate(self, duration: float) -> tuple[np.ndarray, list[np.ndarray], list[MeasurementBatch]]:
        """``duration`` 秒ぶんまとめて生成する.

        Returns
        -------
        (times, truths, batches)
        """
        n = max(int(round(duration / self.dt)), 1)
        times, truths, batches = [], [], []
        for _ in range(n):
            t = self.t
            p, b = self.step()
            times.append(t)
            truths.append(p)
            batches.append(b)
        return np.array(times), truths, batches


@dataclass
class Scenario:
    """シミュレーション条件一式 (UI と CLI が共有する)."""

    anchors: list[Anchor]
    traj: Trajectory
    error: ErrorModel = field(default_factory=ErrorModel)
    rate_hz: float = 10.0
    duration: float = 60.0
    kind: MeasKind = MeasKind.RANGE
    seed: int = 0

    def hal(self) -> SimulatedHal:
        """この設定どおりの :class:`SimulatedHal` を作る."""
        return SimulatedHal(
            self.anchors,
            self.traj,
            self.error,
            rate_hz=self.rate_hz,
            kind=self.kind,
            seed=self.seed,
        )
