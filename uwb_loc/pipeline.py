"""HAL と測位器をつなぐ実行系.

使い方はこれだけで済むようにしてある::

    hal = SimulatedHal(anchors, trajectory.figure8([4, 3, 1.2]))
    pipe = Pipeline(hal, level="Lv3")
    for fix in pipe.run(duration=30.0):
        print(fix.t, fix.position)

実機に差し替えるときは ``hal`` を変えるだけで, 他は 1 行も変わらない.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator

import numpy as np

from .hal.base import UwbHal
from .solvers import PositionEstimator, SolveConfig, make_estimator
from .types import Anchor, Fix, MeasurementBatch

__all__ = ["Pipeline", "run_offline"]


class Pipeline:
    """観測 -> 測位 -> 結果 の一連の流れ.

    Parameters
    ----------
    hal:
        観測源 (実機でもシミュレータでも同じ).
    level:
        測位レベル ``"Lv0"`` 〜 ``"Lv3"``.
    config:
        共通設定 (次元, 拘束, 反復).
    estimator:
        自前で組んだ測位器を使う場合. 指定すると ``level`` は無視される.
    anchors:
        アンカー一覧を外から与える場合. None なら HAL から取る.
    on_fix:
        1 エポックごとに呼ばれるコールバック.
    **kwargs:
        測位器のレベル固有引数 (EKF の ``sigma_a`` など).
    """

    def __init__(
        self,
        hal: UwbHal,
        *,
        level: str = "Lv2",
        config: SolveConfig | None = None,
        estimator: PositionEstimator | None = None,
        anchors: list[Anchor] | None = None,
        on_fix: Callable[[Fix], None] | None = None,
        **kwargs: object,
    ) -> None:
        self.hal = hal
        self.config = config or SolveConfig()
        self.anchors = list(anchors if anchors is not None else hal.anchors)
        self.estimator = estimator or make_estimator(level, self.anchors, self.config, **kwargs)
        self.on_fix = on_fix
        self.fixes: list[Fix] = []

    # ------------------------------------------------------------------

    def process(self, batch: MeasurementBatch) -> Fix:
        """観測 1 エポックを処理する."""
        # HAL が途中でアンカー表を送ってくることがある (自己測量の結果など).
        hal_anchors = self.hal.anchors
        if hal_anchors and len(hal_anchors) != len(self.anchors):
            self.anchors = list(hal_anchors)
            self.estimator.set_anchors(self.anchors)

        fix = self.estimator.update(batch)
        self.fixes.append(fix)
        if self.on_fix is not None:
            self.on_fix(fix)
        return fix

    def run(
        self,
        duration: float | None = None,
        *,
        max_epochs: int | None = None,
        timeout: float = 1.0,
    ) -> Iterator[Fix]:
        """HAL から読みながら測位し続けるジェネレータ.

        Parameters
        ----------
        duration:
            観測時刻ベースの実行時間 [s]. None なら HAL が尽きるまで.
        max_epochs:
            処理するエポック数の上限.
        """
        t0: float | None = None
        count = 0
        self.hal.open()
        try:
            for batch in self.hal.stream(timeout=timeout):
                if t0 is None:
                    t0 = batch.t
                if duration is not None and batch.t - t0 > duration:
                    break
                yield self.process(batch)
                count += 1
                if max_epochs is not None and count >= max_epochs:
                    break
        finally:
            self.hal.close()

    def positions(self) -> np.ndarray:
        """これまでの推定位置, shape (n, 3). 失敗したエポックは NaN."""
        if not self.fixes:
            return np.zeros((0, 3))
        return np.array([f.position for f in self.fixes])

    def times(self) -> np.ndarray:
        """これまでに測位した時刻の一覧 [s]."""
        return np.array([f.t for f in self.fixes])


def run_offline(
    batches: list[MeasurementBatch],
    anchors: list[Anchor],
    *,
    level: str = "Lv2",
    config: SolveConfig | None = None,
    **kwargs: object,
) -> list[Fix]:
    """記録済みの観測列をまとめて処理する.

    ログのリプレイと, レベル間の比較に使う. 同じ観測列を Lv0-Lv3 に
    通せば, アルゴリズムの差だけを純粋に比較できる.
    """
    est = make_estimator(level, anchors, config or SolveConfig(), **kwargs)
    return [est.update(b) for b in batches]
