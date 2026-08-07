"""測位器の共通インターフェイス.

忠実度レベル (Lv0 三辺測量 〜 Lv3 密結合 EKF) をこの 1 つの型で差し替える.
呼び出し側のコードはレベルが変わっても一切変えなくてよい.

    est = make_estimator("Lv2", anchors)
    fix = est.update(batch)

Lv0-Lv2 はステートレス (エポック単位の独立推定), Lv3 だけ内部状態を持つが,
``update()`` の呼び方は同じ.
"""

from __future__ import annotations

import warnings
from abc import ABC, abstractmethod

import numpy as np

from ..geometry import anchor_plane, mirror_point
from ..model import MeasurementModel
from ..types import Anchor, Fix, Measurement, MeasurementBatch

__all__ = ["PositionEstimator", "SolveConfig"]


class SolveConfig:
    """測位の共通設定.

    Parameters
    ----------
    dim:
        解く次元. 3 なら xyz, 2 なら xy のみ (z は ``z_fixed`` に固定).
        アンカーが同一平面上にしかない構成では 2 にすること — z がほぼ
        不可観測なまま 3 次元で解くと, xy まで巻き添えで悪化する.
    z_fixed:
        ``dim=2`` のときの高さ [m].
    z_bounds:
        3 次元で解くときの高さの許容範囲 [m]. 解が範囲外に出たら丸める.
        アンカー平面に対する鏡像解を落とすのに使う. None なら無制限.
    max_iter:
        反復ソルバの最大反復数.
    tol:
        位置更新量の収束判定 [m].
    """

    def __init__(
        self,
        *,
        dim: int = 3,
        z_fixed: float = 0.0,
        z_bounds: tuple[float, float] | None = None,
        max_iter: int = 30,
        tol: float = 1e-4,
    ) -> None:
        if dim not in (2, 3):
            raise ValueError("dim は 2 か 3")
        self.dim = dim
        self.z_fixed = z_fixed
        self.z_bounds = z_bounds
        self.max_iter = max_iter
        self.tol = tol

    @property
    def free_mask(self) -> np.ndarray:
        """自由に動かす座標のマスク, shape (3,)."""
        return np.array([True, True, self.dim == 3])

    def project(self, p: np.ndarray) -> np.ndarray:
        """拘束を位置に適用する."""
        p = np.asarray(p, dtype=float).copy()
        if self.dim == 2:
            p[2] = self.z_fixed
        elif self.z_bounds is not None:
            p[2] = float(np.clip(p[2], *self.z_bounds))
        return p


class PositionEstimator(ABC):
    """測位器の基底クラス.

    Parameters
    ----------
    anchors:
        アンカー一覧.
    config:
        共通設定.
    """

    level: str = "Lv?"

    def __init__(self, anchors: list[Anchor], config: SolveConfig | None = None) -> None:
        self.config = config or SolveConfig()
        self.set_anchors(anchors)

    def set_anchors(self, anchors: list[Anchor]) -> None:
        """アンカー表を差し替える (自己測量の結果を反映するときなど)."""
        self.anchors = list(anchors)
        self.model = MeasurementModel(self.anchors)
        # 同一平面配置なら鏡像解が生じる. 平面はエポックごとに変わらないので
        # ここで一度だけ求めておく. 2 次元で解くときは高さが固定されるので無関係.
        self._plane = anchor_plane(self.anchors) if self.config.dim == 3 else None
        self._side: float | None = None

        # 同一平面 + 3D + 事前情報なし = 鏡像がどちらか決めようがない.
        # 黙って 1/2 の確率で track まるごと鏡像側になるのがいちばん困るので,
        # 使う前に気づけるようにしておく (Fix.ambiguous でも分かるが,
        # 見ない人の方が多い).
        if self._plane is not None and self.config.z_bounds is None:
            if not getattr(self, "_warned_coplanar", False):
                self._warned_coplanar = True
                warnings.warn(
                    "アンカーが同一平面に並んでいます。3 次元で解くと、その平面に関する"
                    "鏡像が測距値では区別できず、推定が丸ごと反対側になることがあります"
                    "（水平位置は正しく、高さだけ折り返る）。"
                    "SolveConfig(z_bounds=(下限, 上限)) で片側に絞るか、"
                    "SolveConfig(dim=2, z_fixed=...) で高さを固定してください。",
                    stacklevel=3,
                )

    @abstractmethod
    def update(self, batch: MeasurementBatch) -> Fix:
        """観測を 1 エポック分与えて位置を得る."""

    def reset(self) -> None:
        """内部状態を捨てる. ステートレスな実装では何もしない."""
        self._side = None

    # ------------------------------------------------------------------

    def _usable(self, batch: MeasurementBatch) -> list[Measurement]:
        """アンカー座標が分かっていて有効な観測だけ取り出す."""
        return [m for m in batch.measurements if self.model.known(m)]

    # ---------------------------------------------------------- 鏡像解の解消

    def resolve_mirror(
        self, p: np.ndarray, cov: np.ndarray | None = None
    ) -> tuple[np.ndarray, np.ndarray | None, bool]:
        """同一平面配置で生じる鏡像解を片側に寄せる.

        アンカーが平面上に並んでいると, その平面に関する鏡像は**測距値では
        まったく区別できない** (全アンカーからの距離が厳密に一致する).
        残差を比べても選べないので, 距離以外の情報で決めるしかない.

        使える情報を強い順に使う.

        1. ``SolveConfig.z_bounds`` — 「タグは天井より下」のような事前知識.
           片方だけが範囲に入るならそれを採る
        2. 直前に採った側 — 平面のどちら側にいたか (**上下の二択だけ**で,
           位置の初期値としては使わない). Lv2 が前回値から温め直さないのは
           外れ値への固着を避けるためで, 離散的な側の選択はそれと衝突しない
        3. どちらも無い — 決めようがないので ``ambiguous=True`` を立てて
           そのまま返す

        鏡映は直交変換なので, 共分散は ``R C Rᵀ`` で移せば厳密に対応する
        (``R = I - 2nnᵀ``). 残差 RMS と GDOP は鏡映で不変なのでそのままでよい.
        **解き直してはいけない** — 退化した幾何では最適点が平面をまたいで
        戻ってしまい、せっかく寄せた側から逃げる.

        Returns
        -------
        (位置, 共分散, 多義かどうか)
        """
        if self._plane is None:
            return p, cov, False
        normal, offset = self._plane
        signed = float(normal @ p) - offset
        if abs(signed) < 1e-12:  # 平面上なら鏡像は自分自身
            return p, cov, False

        def flip() -> tuple[np.ndarray, np.ndarray | None]:
            q = mirror_point(p, normal, offset)
            if cov is None:
                return q, None
            refl = np.eye(3) - 2.0 * np.outer(normal, normal)
            return q, refl @ cov @ refl.T

        bounds = self.config.z_bounds
        if bounds is not None:
            lo, hi = bounds
            other = mirror_point(p, normal, offset)
            in_p = lo - 1e-9 <= p[2] <= hi + 1e-9
            in_o = lo - 1e-9 <= other[2] <= hi + 1e-9
            if in_p and not in_o:
                self._side = np.sign(signed)
                return p, cov, False
            if in_o and not in_p:
                self._side = -np.sign(signed)
                q, c = flip()
                return q, c, False

        if self._side is not None:
            if np.sign(signed) != self._side:
                q, c = flip()
                return q, c, False
            return p, cov, False

        # 手掛かりなし. 最初の一発はどちらとも決められない.
        self._side = np.sign(signed)
        return p, cov, True

    def _remember_side(self, p: np.ndarray) -> None:
        """採用した位置がどちら側だったかを覚える."""
        if self._plane is None:
            return
        normal, offset = self._plane
        signed = float(normal @ p) - offset
        if abs(signed) > 1e-9:
            self._side = np.sign(signed)

    def __repr__(self) -> str:  # pragma: no cover - 表示のみ
        return f"<{type(self).__name__} {self.level} anchors={len(self.anchors)}>"
