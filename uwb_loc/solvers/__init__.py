"""測位ソルバ (Lv0-Lv3).

レベルは名前で選べる::

    est = make_estimator("Lv2", anchors)
    fix = est.update(batch)

どのレベルでも ``update(batch) -> Fix`` は同じなので, 呼び出し側は
レベルを設定値として扱える (UI から切り替えて比較できる).
"""

from __future__ import annotations

from ..types import Anchor
from .base import PositionEstimator, SolveConfig
from .closed_form import beck_gtrs, chan_tdoa, lls_trilateration
from .ekf import Lv3TightlyCoupledEKF
from .nls import Lv0Trilateration, Lv1WeightedNLS, Lv2RobustNLS, solve_nls
from .robust import RobustLoss, physical_gate, ransac_ranges, robust_weights

__all__ = [
    "PositionEstimator",
    "SolveConfig",
    "RobustLoss",
    "Lv0Trilateration",
    "Lv1WeightedNLS",
    "Lv2RobustNLS",
    "Lv3TightlyCoupledEKF",
    "make_estimator",
    "LEVELS",
    "beck_gtrs",
    "chan_tdoa",
    "lls_trilateration",
    "solve_nls",
    "physical_gate",
    "ransac_ranges",
    "robust_weights",
]

#: 名前 -> クラス. UI とテストが同じ辞書を見る.
LEVELS: dict[str, type[PositionEstimator]] = {
    "Lv0": Lv0Trilateration,
    "Lv1": Lv1WeightedNLS,
    "Lv2": Lv2RobustNLS,
    "Lv3": Lv3TightlyCoupledEKF,
}


def make_estimator(
    level: str,
    anchors: list[Anchor],
    config: SolveConfig | None = None,
    **kwargs: object,
) -> PositionEstimator:
    """レベル名から測位器を作る.

    Parameters
    ----------
    level:
        ``"Lv0"`` 〜 ``"Lv3"`` (大文字小文字は問わない).
    anchors:
        アンカー一覧.
    config:
        共通設定.
    **kwargs:
        レベル固有の引数 (EKF の ``sigma_a`` など).
    """
    key = level.strip().capitalize().replace("v", "v")
    for name in LEVELS:
        if name.lower() == level.strip().lower():
            key = name
            break
    if key not in LEVELS:
        raise ValueError(f"未知のレベル: {level!r} (使えるのは {list(LEVELS)})")
    return LEVELS[key](anchors, config, **kwargs)  # type: ignore[arg-type]
