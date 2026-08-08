"""測距をエポックに束ねる規則 (テキスト HAL とプッシュ HAL で共有).

順繰りにアンカーをポーリングするファームウェアは, 測距を 1 本ずつ
ばらばらに吐いてくる. Lv0-Lv2 は 1 エポックに 4 本以上ないと解けないので,
どこで区切るかを決める必要がある.

規則は 2 つだけ.

* **同じアンカーがもう一度出てきたら 1 巡完了** — 順繰りに回るファームなら
  これがいちばん素直で, レートを知らなくても効く
* **``max_span`` 秒を超えたら打ち切る** — 途中で取りこぼしても止まらない

なお **Lv3 (密結合 EKF) には束ねる必要がない**. 測距が届いた順に
1 本ずつ処理できるので, ``group=False`` で素通しにしてよい.
"""

from __future__ import annotations

from collections.abc import Callable

from ..types import Measurement, MeasurementBatch

__all__ = ["EpochGrouper"]


class EpochGrouper:
    """測距を受け取り, エポックが確定したらコールバックに渡す.

    Parameters
    ----------
    emit:
        エポックが確定したときに呼ばれる関数.
    group:
        False なら束ねずに 1 回の入力ごとに確定させる.
    max_span:
        束ねる場合の打ち切り時間 [s].
    """

    def __init__(
        self,
        emit: Callable[[MeasurementBatch], None],
        *,
        group: bool = True,
        max_span: float = 0.5,
    ) -> None:
        self._emit = emit
        self.group = group
        self.max_span = float(max_span)
        self._pending: list[Measurement] = []
        self._seen: set[str] = set()
        self._started = 0.0

    def add(self, measurements: list[Measurement], now: float) -> None:
        """測距を投入する. エポックが確定すれば ``emit`` が呼ばれる."""
        if not measurements:
            return
        if not self.group:
            self._emit(self._make(measurements))
            return

        for m in measurements:
            if m.anchor_id in self._seen or (
                self._pending and now - self._started > self.max_span
            ):
                self.flush()
            if not self._pending:
                self._started = now
            self._pending.append(m)
            self._seen.add(m.anchor_id)

    def flush(self) -> None:
        """溜まっている分を確定させる (ストリームの終わりで呼ぶ)."""
        if self._pending:
            self._emit(self._make(self._pending))
        self._pending, self._seen = [], set()

    # ------------------------------------------------------------------

    def _make(self, ms: list[Measurement]) -> MeasurementBatch:
        return MeasurementBatch(t=ms[0].t, measurements=list(ms), tag_id=ms[0].tag_id)
