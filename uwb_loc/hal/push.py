"""読みに行けない経路のための HAL — 観測を「押し込む」.

シリアルやソケットは ``readline()`` で**読みに行ける**が, そうでない経路も多い.

* BLE の通知 (bleak の ``start_notify`` コールバック)
* MQTT の ``on_message``
* ROS のサブスクライバ
* USB HID, gRPC, WebSocket, 共有メモリ, 別スレッドの受信ループ

これらは「届いたら呼ばれる」形なので, 読みに行く HAL には嵌まらない.
:class:`PushHal` は逆向きで, 受け取った側が :meth:`push` を呼ぶ::

    hal = PushHal(anchors)

    def on_ble_notify(_, data):                  # BLE の通知コールバック
        aid, dist = my_decode(data)
        hal.push(aid, dist)                      # ← 押し込むだけ

    for fix in Pipeline(hal, level="Lv2").run():
        print(fix.position)

**測位側のコードは他の HAL と一切変わらない。**

なお, 自分でループを回すなら HAL 自体が要らない
(``est.update(batch)`` を直接呼べばよい). :class:`PushHal` が要るのは,
:class:`~uwb_loc.pipeline.Pipeline` や UI に載せたいときと,
エポックへの束ね方を任せたいとき.
"""

from __future__ import annotations

import queue
import threading
import time

from ..types import Anchor, MeasKind, Measurement, MeasurementBatch
from .base import UwbHal
from .grouping import EpochGrouper

__all__ = ["PushHal"]


class PushHal(UwbHal):
    """外から観測を押し込む HAL.

    Parameters
    ----------
    anchors:
        アンカー一覧. あとから :meth:`set_anchors` で差し替えてもよい.
    group:
        True なら「同じアンカーが再び来たら 1 巡完了」とみなして束ねる.
        Lv3 だけを使うなら False で素通しにしてよい.
    max_span:
        束ねる場合の打ち切り時間 [s].
    clock:
        時刻を省略して :meth:`push` したときに使う時計. 既定は
        ``time.monotonic``. **速く動くものを追うなら, 受信側で測距が成立した
        時刻を取って明示的に渡すこと** — 通知が届いた時刻ではレイテンシぶん遅れる.
    """

    name = "push"

    def __init__(
        self,
        anchors: list[Anchor] | None = None,
        *,
        group: bool = True,
        max_span: float = 0.5,
        clock=time.monotonic,
    ) -> None:
        self._anchors = list(anchors or [])
        self._queue: queue.Queue[MeasurementBatch] = queue.Queue()
        self._lock = threading.Lock()
        self._closed = False
        self._clock = clock
        self._grouper = EpochGrouper(self._queue.put, group=group, max_span=max_span)
        #: 押し込まれた測距の総数 (立ち上げの切り分け用).
        self.n_pushed = 0

    # ------------------------------------------------------------------ 投入

    def push(
        self,
        anchor_id: str,
        distance: float,
        *,
        t: float | None = None,
        quality: float | None = None,
        sigma: float | None = None,
        tag_id: str = "tag0",
    ) -> None:
        """測距 1 本を押し込む.

        Parameters
        ----------
        anchor_id:
            アンカー ID (アンカー表の ``id`` と一致させる).
        distance:
            距離 [m]. **単位の換算は呼ぶ側で済ませておくこと.**
        t:
            測距が成立した時刻 [s]. 省略すると ``clock()`` を使う.
        quality:
            見通しの尤度 0-1. 入れておくと NLOS に強くなる.
        sigma:
            この観測の 1σ [m].
        """
        now = self._clock()
        m = Measurement(
            anchor_id=str(anchor_id),
            value=float(distance),
            kind=MeasKind.RANGE,
            t=now if t is None else float(t),
            sigma=sigma,
            quality=quality,
            tag_id=tag_id,
        )
        with self._lock:
            self.n_pushed += 1
            self._grouper.add([m], now)

    def push_many(self, readings, *, t: float | None = None, **kw) -> None:
        """``[(アンカー ID, 距離), ...]`` をまとめて押し込む.

        1 エポック分が一度に手に入る場合はこちら.
        """
        for aid, dist in readings:
            self.push(aid, dist, t=t, **kw)

    def push_batch(self, batch: MeasurementBatch) -> None:
        """組み立て済みのエポックをそのまま流す (束ね直さない)."""
        with self._lock:
            self.n_pushed += len(batch)
            self._queue.put(batch)

    # -------------------------------------------------------------- ライフサイクル

    def set_anchors(self, anchors: list[Anchor]) -> None:
        """アンカー表を差し替える (自己測量の結果を反映するときなど)."""
        self._anchors = list(anchors)

    def close(self) -> None:
        """もう押し込まれないことを伝える (溜まっている分は流し切る)."""
        with self._lock:
            self._grouper.flush()
            self._closed = True

    @property
    def is_open(self) -> bool:
        return not (self._closed and self._queue.empty())

    # -------------------------------------------------------------- インターフェイス

    @property
    def anchors(self) -> list[Anchor]:
        return list(self._anchors)

    def poll(self, timeout: float = 0.0) -> list[MeasurementBatch]:
        out: list[MeasurementBatch] = []
        try:
            out.append(self._queue.get(timeout=timeout) if timeout > 0
                       else self._queue.get_nowait())
        except queue.Empty:
            # 束ねている途中で入力が途切れたときに詰まらないよう、
            # 打ち切り時間を過ぎていたら確定させる。
            with self._lock:
                self._grouper.flush()
            try:
                out.append(self._queue.get_nowait())
            except queue.Empty:
                return out
        while True:
            try:
                out.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return out
