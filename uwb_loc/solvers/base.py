"""測位器の共通インターフェイス.

忠実度レベル (Lv0 三辺測量 〜 Lv3 密結合 EKF) をこの 1 つの型で差し替える.
呼び出し側のコードはレベルが変わっても一切変えなくてよい.

    est = make_estimator("Lv2", anchors)
    fix = est.update(batch)

Lv0-Lv2 はステートレス (エポック単位の独立推定), Lv3 だけ内部状態を持つが,
``update()`` の呼び方は同じ.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

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

    @abstractmethod
    def update(self, batch: MeasurementBatch) -> Fix:
        """観測を 1 エポック分与えて位置を得る."""

    def reset(self) -> None:
        """内部状態を捨てる. ステートレスな実装では何もしない."""

    # ------------------------------------------------------------------

    def _usable(self, batch: MeasurementBatch) -> list[Measurement]:
        """アンカー座標が分かっていて有効な観測だけ取り出す."""
        return [m for m in batch.measurements if self.model.known(m)]

    def __repr__(self) -> str:  # pragma: no cover - 表示のみ
        return f"<{type(self).__name__} {self.level} anchors={len(self.anchors)}>"
