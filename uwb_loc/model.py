"""観測モデル — 位置から観測値を予測し, ヤコビアンを返す.

距離・距離差・方位角・仰角のどれが来ても同じ形 ``(残差, ヤコビアン, σ)`` を
返すので, 下流のソルバ (WNLS も EKF も) は観測種別を意識しなくてよい.
種別を増やしたければここに 1 分岐足すだけで全レベルに反映される.

残差の符号は ``e = 観測値 - 予測値`` で統一する. NLOS は電波の回り込みで
距離が伸びる側にしか出ないので, **距離観測の e は NLOS のとき必ず正**になる.
:mod:`uwb_loc.solvers.robust` の片側損失はこの符号を使う.
"""

from __future__ import annotations

import numpy as np

from .types import Anchor, MeasKind, Measurement

__all__ = ["MeasurementModel", "wrap_angle"]

_EPS = 1e-9


def wrap_angle(x: float | np.ndarray) -> float | np.ndarray:
    """角度を (-pi, pi] に畳む."""
    return (np.asarray(x) + np.pi) % (2.0 * np.pi) - np.pi


class MeasurementModel:
    """アンカー表を保持して観測を評価する.

    Parameters
    ----------
    anchors:
        アンカー一覧.
    apply_antenna_delay:
        True なら距離観測から :attr:`Anchor.antenna_delay_m` を差し引く.
        HAL 側で補正済みなら False にする.
    """

    def __init__(self, anchors: list[Anchor], *, apply_antenna_delay: bool = True) -> None:
        self.anchors = {a.id: a for a in anchors}
        self.apply_antenna_delay = apply_antenna_delay

    # ------------------------------------------------------------------

    def known(self, m: Measurement) -> bool:
        """その観測を評価できるか (アンカー座標が既知で有効か)."""
        a = self.anchors.get(m.anchor_id)
        if a is None or not a.enabled:
            return False
        if m.kind is MeasKind.TDOA:
            ref = self.anchors.get(m.ref_anchor_id or "")
            if ref is None or not ref.enabled:
                return False
        return True

    def corrected_value(self, m: Measurement) -> float:
        """アンテナ遅延を補正した観測値."""
        if not self.apply_antenna_delay:
            return float(m.value)
        if m.kind is MeasKind.RANGE:
            return float(m.value) - self.anchors[m.anchor_id].antenna_delay_m
        if m.kind is MeasKind.TDOA:
            a = self.anchors[m.anchor_id]
            ref = self.anchors[m.ref_anchor_id or ""]
            return float(m.value) - (a.antenna_delay_m - ref.antenna_delay_m)
        return float(m.value)

    def sigma(self, m: Measurement, distance: float) -> float:
        """観測の 1σ.

        HAL が ``sigma`` を入れてきたらそれを使い, なければアンカーの
        ノイズモデルから作る. ``quality`` (0-1) が入っていれば
        見通しが悪いほど σ を膨らませる — 観測を捨てずに効きを落とす方が,
        アンカー本数が少ないときに粘れる.
        """
        if m.sigma is not None:
            s = float(m.sigma)
        elif m.kind in (MeasKind.AZIMUTH, MeasKind.ELEVATION):
            s = np.deg2rad(5.0)
        else:
            a = self.anchors[m.anchor_id]
            s = a.range_sigma(distance)
            if m.kind is MeasKind.TDOA:
                ref = self.anchors[m.ref_anchor_id or ""]
                s = float(np.hypot(s, ref.range_sigma(distance)))
        q = 1.0 if m.quality is None else float(np.clip(m.quality, 0.0, 1.0))
        # q=1 で等倍, q=0 で 4 倍. NLOS 分類器の出力をそのまま重みに使える.
        return s * (1.0 + 3.0 * (1.0 - q))

    # ------------------------------------------------------------------

    def evaluate(self, p: np.ndarray, m: Measurement) -> tuple[float, np.ndarray, float]:
        """観測 1 本を評価する.

        Returns
        -------
        residual:
            ``観測値 - 予測値``. 角度観測は畳んである.
        jac:
            予測値の位置微分 ``dh/dp``, shape (3,).
        sigma:
            観測の 1σ.
        """
        p = np.asarray(p, dtype=float)
        a = self.anchors[m.anchor_id]
        z = self.corrected_value(m)

        if m.kind is MeasKind.RANGE:
            dv = p - a.position
            d = float(np.linalg.norm(dv))
            if d < _EPS:
                return 0.0, np.zeros(3), self.sigma(m, d)
            return z - d, dv / d, self.sigma(m, d)

        if m.kind is MeasKind.TDOA:
            ref = self.anchors[m.ref_anchor_id or ""]
            dv, dvr = p - a.position, p - ref.position
            d, dr = float(np.linalg.norm(dv)), float(np.linalg.norm(dvr))
            if d < _EPS or dr < _EPS:
                return 0.0, np.zeros(3), self.sigma(m, max(d, dr))
            return z - (d - dr), dv / d - dvr / dr, self.sigma(m, max(d, dr))

        dx, dy, dz = p - a.position
        rho2 = dx * dx + dy * dy
        rho = float(np.sqrt(rho2))
        r2 = rho2 + dz * dz

        if m.kind is MeasKind.AZIMUTH:
            if rho2 < _EPS:
                return 0.0, np.zeros(3), self.sigma(m, rho)
            jac = np.array([-dy / rho2, dx / rho2, 0.0])
            return float(wrap_angle(z - np.arctan2(dy, dx))), jac, self.sigma(m, rho)

        if m.kind is MeasKind.ELEVATION:
            if rho < _EPS or r2 < _EPS:
                return 0.0, np.zeros(3), self.sigma(m, float(np.sqrt(r2)))
            jac = np.array([-dz * dx / (r2 * rho), -dz * dy / (r2 * rho), rho / r2])
            return (
                float(wrap_angle(z - np.arctan2(dz, rho))),
                jac,
                self.sigma(m, float(np.sqrt(r2))),
            )

        raise ValueError(f"未対応の観測種別: {m.kind}")

    # ------------------------------------------------------------------

    def assemble(
        self, p: np.ndarray, meas: list[Measurement]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """観測のリストをまとめて評価する.

        Returns
        -------
        e:
            残差ベクトル, shape (n,).
        J:
            ヤコビアン, shape (n, 3).
        sigma:
            各観測の 1σ, shape (n,).
        """
        n = len(meas)
        e = np.zeros(n)
        jac = np.zeros((n, 3))
        sig = np.ones(n)
        for i, m in enumerate(meas):
            e[i], jac[i], sig[i] = self.evaluate(p, m)
        return e, jac, sig
