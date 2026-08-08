"""uwb_loc — チップ非依存の UWB 測位ライブラリ.

各 UWB 用に書いた HAL から観測をもらい, 位置・共分散・品質指標を返す.
HAL とアルゴリズムの間は :class:`Measurement` で切ってあるので,
DW1000 でも DW3000 でも SR150 でも, 距離になってしまえば同じコードが動く.

    import uwb_loc as ul

    anchors = ul.room_anchors((8, 6, 2.6))
    hal = ul.SimulatedHal(anchors, ul.trajectory.figure8([4, 3, 1.2]))
    for fix in ul.Pipeline(hal, level="Lv3").run(duration=30.0):
        print(fix.t, fix.position, fix.sigma)

測位レベル
----------
=====  ===================================================  ==================
Lv0    LLS 三辺測量 (閉形式)                                 動作確認・初期値
Lv1    重み付き非線形最小二乗 + χ² ゲート                     見通しの良い環境
Lv2    Beck 初期解 + Huber-IRLS + 片側損失 (+RANSAC)         NLOS のある屋内
Lv3    密結合 EKF (CV/CA)                                    移動体・ドローン
=====  ===================================================  ==================
"""

from __future__ import annotations

from . import calibration, geometry, metrics
from .calibration import (
    align_to_reference,
    estimate_antenna_delays,
    fit_range_bias,
    self_survey,
)
from .geometry import anchor_condition, crlb_at, gdop_at, gdop_map
from .hal import (
    JsonLinesHal,
    JsonLinesWriter,
    PushHal,
    Ryuw122Config,
    Ryuw122Hal,
    Ryuw122Tag,
    Ryuw122Terminal,
    TextHal,
    UwbHal,
    sniff,
)
from .metrics import error_cdf, error_series, error_stats
from .model import MeasurementModel
from .pipeline import Pipeline, run_offline
from .sim import ErrorModel, Scenario, SimulatedHal, make_anchors, room_anchors, trajectory
from .solvers import (
    LEVELS,
    Lv0Trilateration,
    Lv1WeightedNLS,
    Lv2RobustNLS,
    Lv3TightlyCoupledEKF,
    PositionEstimator,
    RobustLoss,
    SolveConfig,
    make_estimator,
)
from .types import WIRE_VERSION, Anchor, Fix, MeasKind, Measurement, MeasurementBatch

__version__ = "0.1.0"

__all__ = [
    "__version__",
    "WIRE_VERSION",
    # 型
    "Anchor",
    "Measurement",
    "MeasurementBatch",
    "MeasKind",
    "Fix",
    "MeasurementModel",
    # HAL
    "UwbHal",
    "TextHal",
    "PushHal",
    "Ryuw122Hal",
    "Ryuw122Config",
    "Ryuw122Terminal",
    "Ryuw122Tag",
    "sniff",
    "JsonLinesHal",
    "JsonLinesWriter",
    "SimulatedHal",
    # 測位
    "PositionEstimator",
    "SolveConfig",
    "RobustLoss",
    "Lv0Trilateration",
    "Lv1WeightedNLS",
    "Lv2RobustNLS",
    "Lv3TightlyCoupledEKF",
    "make_estimator",
    "LEVELS",
    "Pipeline",
    "run_offline",
    # シミュレータ
    "ErrorModel",
    "Scenario",
    "trajectory",
    "make_anchors",
    "room_anchors",
    # 評価・設営
    "geometry",
    "metrics",
    "calibration",
    "gdop_at",
    "gdop_map",
    "crlb_at",
    "anchor_condition",
    "error_stats",
    "error_series",
    "error_cdf",
    "self_survey",
    "align_to_reference",
    "fit_range_bias",
    "estimate_antenna_delays",
]
