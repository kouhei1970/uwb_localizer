"""REYAX RYUW122 / RYUW122_Lite 用の HAL.

仕様書: ``docs/datasheets/RYUW122_AT_Command_Guide.pdf`` (2024-03-12 版)

このモジュールの性質
--------------------
**距離は ANCHOR 側の UART に出る.** ANCHOR が ``AT+ANCHOR_SEND`` で TAG を
指名して初めて測距が成立し, 応答として返る::

    AT+ANCHOR_SEND=DAVID123,4,RNGE
    +ANCHOR_RCV=DAVID123,5,HELLO,40 cm

つまり **1 台の ANCHOR は 1 度に 1 つの TAG としか測距しない**.
複数点の距離を集めるには, TAG の宛先を順番に変えて呼び出す (ポーリング).

配置の取り方が 2 通りある
-------------------------
**(A) 移動側を ANCHOR にする — 本 HAL が想定する構成**

PC に繋いだ 1 台を ANCHOR にして, 部屋に固定した TAG を順に呼ぶ.
**UART 1 本で全部の距離が揃う**ので配線が最も簡単.

    PC ── UART ── RYUW122 (ANCHOR, 移動する)
                     ├── 測距 ──> RYUW122 (TAG) 部屋の隅に固定
                     ├── 測距 ──> RYUW122 (TAG)
                     └── 測距 ──> RYUW122 (TAG) ...

この場合, ライブラリでいう「アンカー座標」は**固定した TAG の座標**で,
``Anchor.id`` には TAG の ``AT+ADDRESS`` (8 バイト ASCII) を入れる.

**(B) 移動側を TAG にする — 一般的な UWB の構成**

固定局を ANCHOR にすると距離は各 ANCHOR の UART に出るので,
**ANCHOR の数だけホスト接続が要る**. 各 ANCHOR の MCU から距離を集めて
1 箇所に流す仕組みを自分で用意し, :class:`~uwb_loc.hal.text.TextHal` か
:class:`~uwb_loc.hal.push.PushHal` に渡す方が早い.

TAG 側の準備 (重要)
-------------------
TAG は ``AT+TAG_SEND`` でデータを積んでおかないと応答できない.
TAG 側の MCU が定期的に ``AT+TAG_SEND=<len>,<data>`` を打つ必要がある.
また **ANCHOR と TAG のペイロード長の差は 3 バイト以内**でなければ
距離が計算されない (仕様書 13, 14 節).

単位
----
``+ANCHOR_RCV`` の距離は **cm**. 本 HAL が m に直して渡す.
"""

from __future__ import annotations

import re
import threading
import time
from typing import IO, Any

from ..types import Anchor, MeasKind, Measurement, MeasurementBatch
from .base import UwbHal
from .grouping import EpochGrouper

__all__ = ["Ryuw122Hal", "Ryuw122Config", "parse_anchor_rcv"]

#: ``+ANCHOR_RCV=<TAG Address>,<PAYLOAD LENGTH>,<TAG DATA>,<DISTANCE> cm[,<RSSI>]``
#: データ本体にカンマが入りうるので、宛先と長さだけ先に取り、
#: 残りは長さを使って切り出す (仕様書 16 節)。
_RCV_HEAD = re.compile(r"\+ANCHOR_RCV\s*=\s*(?P<addr>[^,]+)\s*,\s*(?P<len>\d+)\s*,")
_DIST = re.compile(r"(?P<dist>-?\d+(?:\.\d+)?)\s*cm", re.IGNORECASE)


def parse_anchor_rcv(line: str) -> tuple[str, float, float | None] | None:
    """``+ANCHOR_RCV`` 行から ``(TAG アドレス, 距離 [m], RSSI)`` を取り出す.

    距離が取れない行 (測距に失敗した応答など) は None を返す.
    """
    head = _RCV_HEAD.search(line)
    if head is None:
        return None
    addr = head.group("addr").strip()
    n = int(head.group("len"))

    rest = line[head.end():]
    # <TAG DATA> はちょうど n バイト。データ中のカンマに引っかからないよう
    # 長さで切り出してから、残りを距離と RSSI として読む。
    tail = rest[n:] if len(rest) >= n else rest
    m = _DIST.search(tail) or _DIST.search(rest)
    if m is None:
        return None
    dist_m = float(m.group("dist")) / 100.0        # cm -> m

    rssi = None
    after = tail[m.end():]
    r = re.search(r"-?\d+(?:\.\d+)?", after)
    if r is not None:
        rssi = float(r.group())
    return addr, dist_m, rssi


class Ryuw122Config:
    """モジュールに流し込む設定 (仕様書 3-12 節).

    None のままの項目は触らない (モジュールの Flash に残っている値を使う).

    Attributes
    ----------
    network_id:
        ``AT+NETWORKID`` 8 バイト ASCII. **全機で揃っていないと通信しない.**
    address:
        ``AT+ADDRESS`` この機体自身のアドレス, 8 バイト ASCII.
    password:
        ``AT+CPIN`` 32 文字の AES128 パスワード. 全機で揃える必要がある.
    channel:
        ``AT+CHANNEL`` 5 (6489.6 MHz) か 9 (7987.2 MHz).
    bandwidth:
        ``AT+BANDWIDTH`` 0 (850 kbps) か 1 (6.8 Mbps).
    power:
        ``AT+CRFOP`` 0-5. 5 が最大 (-32 dBm).
    calibration_cm:
        ``AT+CAL`` 距離校正 [cm], -100〜+100. アンテナ遅延の粗補正に使える
        が, 細かい補正は :attr:`Anchor.antenna_delay_m` の方が扱いやすい.
    rssi:
        ``AT+RSSI`` 受信強度を ``+ANCHOR_RCV`` に載せるか. 載せると
        品質値として使えるので既定で有効にする.
    """

    def __init__(
        self,
        *,
        network_id: str | None = None,
        address: str | None = None,
        password: str | None = None,
        channel: int | None = None,
        bandwidth: int | None = None,
        power: int | None = None,
        calibration_cm: int | None = None,
        rssi: bool = True,
    ) -> None:
        for name, v in (("network_id", network_id), ("address", address)):
            if v is not None and len(v) != 8:
                raise ValueError(f"{name} は 8 バイト ASCII (指定: {v!r})")
        if password is not None and len(password) != 32:
            raise ValueError("password は 32 文字 (AES128)")
        if channel is not None and channel not in (5, 9):
            raise ValueError("channel は 5 か 9")
        if bandwidth is not None and bandwidth not in (0, 1):
            raise ValueError("bandwidth は 0 か 1")
        if power is not None and not 0 <= power <= 5:
            raise ValueError("power は 0-5")
        if calibration_cm is not None and not -100 <= calibration_cm <= 100:
            raise ValueError("calibration_cm は -100〜100")
        self.network_id = network_id
        self.address = address
        self.password = password
        self.channel = channel
        self.bandwidth = bandwidth
        self.power = power
        self.calibration_cm = calibration_cm
        self.rssi = rssi

    def commands(self, *, as_anchor: bool = True) -> list[str]:
        """流し込む AT コマンドを順に並べる.

        ``AT+MODE`` を最初に置くのは仕様書の指示 (2 節).
        """
        out = [f"AT+MODE={1 if as_anchor else 0}"]
        if self.network_id is not None:
            out.append(f"AT+NETWORKID={self.network_id}")
        if self.address is not None:
            out.append(f"AT+ADDRESS={self.address}")
        if self.password is not None:
            out.append(f"AT+CPIN={self.password}")
        if self.channel is not None:
            out.append(f"AT+CHANNEL={self.channel}")
        if self.bandwidth is not None:
            out.append(f"AT+BANDWIDTH={self.bandwidth}")
        if self.power is not None:
            out.append(f"AT+CRFOP={self.power}")
        if self.calibration_cm is not None:
            out.append(f"AT+CAL={self.calibration_cm}")
        out.append(f"AT+RSSI={1 if self.rssi else 0}")
        return out


class Ryuw122Hal(UwbHal):
    """RYUW122 を ANCHOR にして, 固定した TAG を順に呼ぶ HAL.

    Parameters
    ----------
    stream:
        ``readline()`` と ``write()`` を持つテキストストリーム.
        通常は :meth:`from_serial` で作る.
    tag_addresses:
        呼び出す TAG のアドレス (8 バイト ASCII) を順に並べたもの.
        ライブラリ側の :attr:`Anchor.id` と一致させる.
    anchors:
        固定した TAG の座標 (= ライブラリでいうアンカー).
    config:
        モジュールに流し込む設定. None なら Flash の値のまま使う
        (``AT+MODE=1`` だけは必ず送る).
    payload:
        ``AT+ANCHOR_SEND`` で送る中身. **TAG 側の ``AT+TAG_SEND`` の長さと
        3 バイト以上違うと距離が出ない** (仕様書 13 節).
    timeout:
        1 回の測距を待つ上限 [s].
    period:
        1 巡したあとの待ち [s]. 0 なら間を空けずに回し続ける.
    """

    name = "ryuw122"

    def __init__(
        self,
        stream: IO[str],
        tag_addresses: list[str],
        anchors: list[Anchor] | None = None,
        *,
        config: Ryuw122Config | None = None,
        payload: str = "RNGE",
        timeout: float = 0.35,
        period: float = 0.0,
        group: bool = True,
    ) -> None:
        if not tag_addresses:
            raise ValueError("呼び出す TAG アドレスを 1 つ以上指定してください")
        self._stream = stream
        self.tag_addresses = list(tag_addresses)
        self._anchors = list(anchors or [])
        self.config = config
        self.payload = payload
        self.timeout = float(timeout)
        self.period = float(period)

        self._queue: list[MeasurementBatch] = []
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._closed = False
        self._grouper = EpochGrouper(self._emit, group=group, max_span=5.0)

        #: 立ち上げの切り分け用.
        self.n_ranged = 0
        self.n_timeout = 0
        self.last_error: str | None = None
        #: 設定を流し込んだときのモジュールの応答 (デバッグ用).
        self.setup_log: list[str] = []

    # ------------------------------------------------------------------ 生成

    @classmethod
    def from_serial(
        cls,
        port: str,
        tag_addresses: list[str],
        baudrate: int = 115200,
        **kw: Any,
    ) -> "Ryuw122Hal":
        """シリアルポートに繋ぐ (既定ボーレート 115200 は仕様書 4 節の既定値)."""
        import io

        import serial  # type: ignore[import-not-found]

        ser = serial.Serial(port, baudrate, timeout=0.2)
        stream = io.TextIOWrapper(ser, encoding="ascii", errors="replace",
                                  newline="\n", write_through=True)
        hal = cls(stream, tag_addresses, **kw)
        hal._serial = ser  # type: ignore[attr-defined]
        return hal

    # ------------------------------------------------------------------ AT

    def _write(self, line: str) -> None:
        self._stream.write(line + "\r\n")
        flush = getattr(self._stream, "flush", None)
        if flush is not None:
            flush()

    def command(self, cmd: str, timeout: float = 1.0) -> list[str]:
        """AT コマンドを 1 つ送り, ``+OK`` / ``+ERR`` が返るまでの行を返す.

        仕様書 3 節: 「``+OK`` が返るまで次のコマンドを実行しないこと」.
        """
        self._write(cmd)
        out: list[str] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._stream.readline()
            if not line:
                continue
            line = line.strip()
            if not line:
                continue
            out.append(line)
            if line.startswith("+OK") or line.startswith("+ERR"):
                break
        if out and out[-1].startswith("+ERR"):
            self.last_error = f"{cmd} -> {out[-1]}"
        return out

    def setup(self) -> bool:
        """モジュールを ANCHOR に設定する. 成功したら True."""
        self.setup_log = []
        ok = True
        cmds = (self.config or Ryuw122Config()).commands(as_anchor=True)
        for c in cmds:
            resp = self.command(c)
            self.setup_log.append(f"{c} -> {' / '.join(resp) or '(応答なし)'}")
            if not any(r.startswith("+OK") for r in resp):
                ok = False
        return ok

    # ------------------------------------------------------------------ 測距

    def _emit(self, batch: MeasurementBatch) -> None:
        with self._lock:
            self._queue.append(batch)

    def range_once(self, tag: str) -> Measurement | None:
        """TAG を 1 つ呼んで距離を得る.

        仕様書 13 節: ``AT+ANCHOR_SEND=<TAG Address>,<Payload Length>,<Data>``
        に対し ``+ANCHOR_RCV=...,<DISTANCE> cm`` が返る.
        """
        self._write(f"AT+ANCHOR_SEND={tag},{len(self.payload)},{self.payload}")
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            line = self._stream.readline()
            if not line:
                continue
            line = line.strip()
            if not line:
                continue
            if line.startswith("+ERR"):
                self.last_error = f"{tag} -> {line}"
                return None
            got = parse_anchor_rcv(line)
            if got is None:
                continue
            addr, dist_m, rssi = got
            self.n_ranged += 1
            # RSSI から見通しの尤度をつくる。-60 dBm 付近を良好、
            # -95 dBm 付近を悪いとみなす粗い写像 (現場で調整する前提)。
            quality = None
            if rssi is not None:
                quality = max(0.0, min(1.0, (rssi + 95.0) / 35.0))
            return Measurement(
                anchor_id=addr,
                value=dist_m,
                kind=MeasKind.RANGE,
                t=time.monotonic(),
                quality=quality,
                raw={"rssi": rssi, "line": line[:120]},
            )
        self.n_timeout += 1
        return None

    def _worker(self) -> None:
        while not self._stop.is_set():
            for tag in self.tag_addresses:
                if self._stop.is_set():
                    break
                m = self.range_once(tag)
                if m is not None:
                    self._grouper.add([m], m.t)
            self._grouper.flush()
            if self.period > 0:
                self._stop.wait(self.period)
        self._closed = True

    # -------------------------------------------------------------- ライフサイクル

    def open(self) -> None:
        if self._thread is not None:
            return
        self.setup()
        self._stop.clear()
        self._closed = False
        self._thread = threading.Thread(target=self._worker, daemon=True,
                                        name="uwb-ryuw122")
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._closed = True
        try:
            self._stream.close()
        except Exception:  # pragma: no cover
            pass

    @property
    def is_open(self) -> bool:
        with self._lock:
            pending = bool(self._queue)
        return not (self._closed and not pending)

    # -------------------------------------------------------------- インターフェイス

    @property
    def anchors(self) -> list[Anchor]:
        return list(self._anchors)

    def set_anchors(self, anchors: list[Anchor]) -> None:
        self._anchors = list(anchors)

    def poll(self, timeout: float = 0.0) -> list[MeasurementBatch]:
        deadline = time.monotonic() + max(timeout, 0.0)
        while True:
            with self._lock:
                if self._queue:
                    out, self._queue = self._queue, []
                    return out
            if time.monotonic() >= deadline:
                return []
            time.sleep(0.005)
