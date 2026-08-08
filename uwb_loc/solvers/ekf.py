"""Lv3 — 密結合 EKF による追跡.

スナップショット位置をフィルタに入れる (疎結合) のではなく,
**測距値そのものを観測としてカルマンフィルタに直接入れる**.

そうする理由:

* アンカーが 3 本未満しか見えないエポックでも**更新できる**.
  疎結合だとそのエポックは丸ごと捨てになる
* TWR はアンカーを順にポーリングするので観測はもともと非同期に届く.
  「1 スキャン = 1 エポック」に束ねる必要がなく, 届いた瞬間に
  predict→update すればよい. レイテンシが下がり, 速い機体で効く.
  ただし**立ち上げだけは 1 本では足りない** — 測距 1 本は球面 1 枚でしかなく
  位置が決まらないため, 揃うまで貯めてから始める (:meth:`_bootstrap`)
* 幾何が悪い方向の情報だけを部分的に取り込める (共分散が正しく効く)

更新は 1 本ずつ逐次に行い, イノベーションゲート (マハラノビス距離) で
NLOS を弾く. 共分散更新は Joseph 形にしてあるので, 逐次更新を繰り返しても
対称性と正定値性が崩れにくい.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..geometry import gdop_from_jacobian
from ..types import Anchor, Fix, MeasurementBatch
from .base import PositionEstimator, SolveConfig
from .nls import Lv2RobustNLS

__all__ = ["Lv3TightlyCoupledEKF"]


class Lv3TightlyCoupledEKF(PositionEstimator):
    """密結合拡張カルマンフィルタ.

    Parameters
    ----------
    anchors:
        アンカー一覧.
    config:
        共通設定. ``dim`` が 2 なら状態から z を外す.
    motion:
        ``"cv"`` 等速度モデル (既定) / ``"ca"`` 等加速度モデル.
        停止と高速移動が混ざるなら ``"ca"`` の方が追従が良いが,
        静止時のふらつきは増える.
    sigma_a:
        プロセスノイズ. ``"cv"`` では加速度の白色雑音強度 [m/s^2],
        ``"ca"`` では加加速度の白色雑音強度 [m/s^3].
        **調整するのは実質このパラメータだけ**にしてある.

        目安は「対象が実際に出す加速度の RMS」. 小さすぎるとフィルタが
        自分の予測を信じすぎて機動に追従できず, 大きすぎると静止時に
        ふらつく. ただし**外すなら大きめに外す方が安全**で, 小さすぎると
        追従できずに発散しうるのに対し, 大きすぎても精度が少し落ちるだけ.
        既定の 1.0 (≒ 1 m/s^2 の機動を想定) は追従を優先した値.
        歩行者や台車なら 0.2-0.5, 機敏なドローンなら 2-5 が目安.
    gate:
        イノベーションゲートのしきい値 (σ の倍数). これを超えた観測は
        そのエポックで使わない. NLOS 対策の本体.
    max_dt:
        予測ステップの上限 [s]. 観測が長く途切れたらフィルタを初期化する.
    max_rejects:
        連続して全観測が弾かれた回数の上限. 超えたら発散とみなして
        スナップショット測位からやり直す. ゲートで自分の誤りを
        守り続ける (棺桶化) のを防ぐ.
    """

    level = "Lv3"

    def __init__(
        self,
        anchors: list[Anchor],
        config: SolveConfig | None = None,
        *,
        motion: str = "cv",
        sigma_a: float = 1.0,
        gate: float = 3.0,
        max_dt: float = 2.0,
        max_rejects: int = 5,
        init_estimator: PositionEstimator | None = None,
    ) -> None:
        super().__init__(anchors, config)
        if motion not in ("cv", "ca"):
            raise ValueError("motion は 'cv' か 'ca'")
        self.motion = motion
        self.sigma_a = float(sigma_a)
        self.gate = float(gate)
        self.max_dt = float(max_dt)
        self.max_rejects = int(max_rejects)
        self._init = init_estimator or Lv2RobustNLS(anchors, self.config)

        self.nd = self.config.dim
        self.norder = 2 if motion == "cv" else 3
        self.nx = self.nd * self.norder
        self.reset()

    # ------------------------------------------------------------------

    def set_anchors(self, anchors: list[Anchor]) -> None:
        super().set_anchors(anchors)
        if getattr(self, "_init", None) is not None:
            self._init.set_anchors(anchors)

    def reset(self) -> None:
        super().reset()
        self.x = np.zeros(self.nx)
        self.P = np.eye(self.nx) * 1e6
        self.t: float | None = None
        self._rejects = 0
        self._initialized = False
        # 立ち上げ時に鏡像解が決められなかったかどうか. フィルタは連続性で
        # 側を保つだけなので, 一度どちらか分からないまま始まったら, その track
        # 全体が鏡像側である可能性が残り続ける.
        self._ambiguous = False
        # 立ち上げ待ちの測距 (1 本ずつ届く経路で使う)
        self._pending: list[Any] = []
        if getattr(self, "_init", None) is not None:
            self._init.reset()

    # ------------------------------------------------------------------ 予測

    def _transition(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """状態遷移行列とプロセスノイズ共分散.

        軸ごとに独立とみなし, 1 軸分の小行列をクロネッカー積で広げる.

        遷移行列は積分器の連鎖なので ``F[i][j] = dt^(j-i)/(j-i)!``.

        プロセスノイズは **「最上位の微分に連続時間の白色雑音が乗る」**
        モデルで統一してある (CV なら加速度, CA なら加加速度).
        ``Q = ∫_0^dt F(τ) G σ² Gᵀ F(τ)ᵀ dτ`` を解いた形で,

        * CV: ``σ_a² [[dt³/3, dt²/2], [dt²/2, dt]]``
        * CA: ``σ_j² [[dt⁵/20, dt⁴/8, dt³/6], [dt⁴/8, dt³/3, dt²/2],
          [dt³/6, dt²/2, dt]]``

        離散版 (1 ステップの間だけ加速度が一定と見なす ``Γ Γᵀ`` 形) と
        混ぜないこと. ``sigma_a`` の意味がモードによって変わってしまう.
        """
        k = self.norder
        f1 = np.eye(k)
        for i in range(k):
            for j in range(i + 1, k):
                f1[i, j] = dt ** (j - i) / math.factorial(j - i)

        if k == 2:  # 等速度 + 加速度の連続白色雑音
            q1 = np.array([[dt**3 / 3.0, dt**2 / 2.0], [dt**2 / 2.0, dt]])
        else:  # 等加速度 + 加加速度の連続白色雑音
            q1 = np.array(
                [
                    [dt**5 / 20.0, dt**4 / 8.0, dt**3 / 6.0],
                    [dt**4 / 8.0, dt**3 / 3.0, dt**2 / 2.0],
                    [dt**3 / 6.0, dt**2 / 2.0, dt],
                ]
            )
        q1 = q1 * self.sigma_a**2

        eye = np.eye(self.nd)
        return np.kron(f1, eye), np.kron(q1, eye)

    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        fmat, qmat = self._transition(dt)
        self.x = fmat @ self.x
        self.P = fmat @ self.P @ fmat.T + qmat

    # ------------------------------------------------------------------ 位置

    def _position(self) -> np.ndarray:
        p = np.zeros(3)
        p[: self.nd] = self.x[: self.nd]
        if self.nd == 2:
            p[2] = self.config.z_fixed
        return p

    def _velocity(self) -> np.ndarray:
        v = np.zeros(3)
        v[: self.nd] = self.x[self.nd : 2 * self.nd]
        return v

    def _position_cov(self) -> np.ndarray:
        cov = np.zeros((3, 3))
        cov[: self.nd, : self.nd] = self.P[: self.nd, : self.nd]
        return cov

    # ------------------------------------------------------------------ 更新

    def update(self, batch: MeasurementBatch) -> Fix:
        meas = self._usable(batch)
        n_total = len(batch)

        # 時刻の巻き戻り (遅れて届いた観測) は捨てる.
        if self.t is not None and batch.t < self.t - 1e-9:
            return self._fix(batch.t, 0, n_total, [], float("nan"), float("nan"), ok=self._initialized)

        if self._initialized and self.t is not None:
            dt = batch.t - self.t
            if dt > self.max_dt:
                self._initialized = False
            else:
                self.predict(dt)

        if not self._initialized:
            if not self._bootstrap(batch):
                self.t = batch.t
                return Fix.failed(batch.t, n_total, self.level)
            self.t = batch.t
            fix = self._diagnostics(batch.t, meas, n_total, [])
            return fix

        self.t = batch.t
        excluded: list[str] = []
        n_used = 0

        for m in meas:
            p = self._position()
            e, jac3, sigma = self.model.evaluate(p, m)
            hvec = np.zeros(self.nx)
            hvec[: self.nd] = jac3[: self.nd]

            ph = self.P @ hvec
            s = float(hvec @ ph + sigma**2)
            if s <= 0.0 or not np.isfinite(s):
                excluded.append(m.anchor_id)
                continue
            if abs(e) > self.gate * np.sqrt(s):
                excluded.append(m.anchor_id)
                continue

            kgain = ph / s
            self.x = self.x + kgain * e
            imkh = np.eye(self.nx) - np.outer(kgain, hvec)
            self.P = imkh @ self.P @ imkh.T + np.outer(kgain, kgain) * sigma**2
            self.P = 0.5 * (self.P + self.P.T)
            n_used += 1

        if n_used == 0 and meas:
            self._rejects += 1
            if self._rejects >= self.max_rejects:
                # ゲートが自分の誤りを守り続けている. 一度捨てて組み直す.
                self._initialized = False
                self._rejects = 0
                if self._bootstrap(batch):
                    return self._diagnostics(batch.t, meas, n_total, [])
                return Fix.failed(batch.t, n_total, self.level)
        else:
            self._rejects = 0

        return self._diagnostics(batch.t, meas, n_total, excluded, n_used=n_used)

    # ------------------------------------------------------------------

    def _bootstrap(self, batch: MeasurementBatch) -> bool:
        """スナップショット測位でフィルタを立ち上げる.

        **立ち上げだけは 1 本では足りない** — 測距 1 本は球面 1 枚でしかなく、
        位置が決まらないため。走り出したあとは 1 本ずつでも更新できるので、
        非同期に 1 本ずつ届く経路 (順繰りの TWR、BLE の通知など) のために、
        直近の測距を貯めておいて揃った時点で立ち上げる。

        貯める窓は ``max_dt`` 秒。同じアンカーが複数あれば新しい方を採る。
        """
        self._pending.extend(batch.measurements)
        cutoff = batch.t - self.max_dt
        self._pending = [m for m in self._pending if m.t >= cutoff]
        # アンカーごとに最新の 1 本だけ残す (古い測距で薄めない)
        newest: dict[str, Any] = {}
        for m in self._pending:
            prev = newest.get(m.anchor_id)
            if prev is None or m.t >= prev.t:
                newest[m.anchor_id] = m
        seed = MeasurementBatch(t=batch.t, tag_id=batch.tag_id,
                                measurements=list(newest.values()))

        # 解ける最小本数 (dim+1) ちょうどで立ち上げると初期値が悪く、
        # 収束するまでの過渡が長く尾を引く。少し余分に揃うまで待つ。
        # ただし本当にアンカーが少ない現場で永久に待たないよう、
        # 溜め始めてから max_dt 経ったら最小本数でも始める。
        want = self.config.dim + 2
        if len(seed) < want:
            oldest = min((m.t for m in self._pending), default=batch.t)
            if len(seed) < self.config.dim + 1 or batch.t - oldest < self.max_dt:
                return False

        fix = self._init.update(seed)
        if not fix.ok:
            return False
        self._pending.clear()
        self.x = np.zeros(self.nx)
        self.x[: self.nd] = fix.position[: self.nd]
        self.P = np.eye(self.nx)
        cov = fix.covariance[: self.nd, : self.nd]
        if np.all(np.isfinite(cov)):
            self.P[: self.nd, : self.nd] = cov + np.eye(self.nd) * 1e-4
        else:
            self.P[: self.nd, : self.nd] *= 4.0
        # 速度 (と加速度) は未知. 大きめの分散から始める.
        for k in range(1, self.norder):
            sl = slice(k * self.nd, (k + 1) * self.nd)
            self.P[sl, sl] = np.eye(self.nd) * (10.0 ** (2 - k))
        self._initialized = True
        self._rejects = 0
        # 立ち上げに使ったスナップショットが鏡像を決められなかったなら,
        # この track はまるごと鏡像側の可能性がある.
        self._ambiguous = bool(fix.ambiguous)
        return True

    def _diagnostics(
        self,
        t: float,
        meas: list,
        n_total: int,
        excluded: list[str],
        *,
        n_used: int | None = None,
    ) -> Fix:
        p = self._position()
        if meas:
            e, jac, sig = self.model.assemble(p, meas)
            rms = float(np.sqrt(np.mean(e**2)))
            g = gdop_from_jacobian(jac, self.config.free_mask)
        else:
            rms, g = float("nan"), float("nan")
        return self._fix(
            t,
            len(meas) if n_used is None else n_used,
            n_total,
            excluded,
            rms,
            g,
            ok=True,
        )

    def _fix(
        self,
        t: float,
        n_used: int,
        n_total: int,
        excluded: list[str],
        rms: float,
        g: float,
        *,
        ok: bool,
    ) -> Fix:
        return Fix(
            position=self._position(),
            covariance=self._position_cov(),
            t=t,
            ok=ok,
            n_used=n_used,
            n_total=n_total,
            residual_rms=rms,
            gdop=g,
            excluded=excluded,
            level=self.level,
            ambiguous=self._ambiguous,
            velocity=self._velocity(),
        )
