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
TAG 側も設定が要る. ANCHOR を設定しただけでは距離は出ない.

* ``AT+MODE=0`` (TAG). 工場出荷値は TAG なので, 買ってきたままなら不要
* ``NETWORKID`` / ``CPIN`` / ``CHANNEL`` / ``BANDWIDTH`` を **ANCHOR と同じ値**に
* ``AT+ADDRESS`` を **機体ごとに別の値**に. 工場出荷値はどの機体も同じなので,
  そのまま並べるとアドレスが衝突して測距できない
* ``AT+TAG_SEND=<len>,<data>`` でデータを積み続ける. 積んでいないと
  ANCHOR の呼びかけに応答できない. 本来は TAG 側の MCU の役目

設定の書き込みと読み出しは :class:`Ryuw122Terminal`, ``AT+TAG_SEND`` を
積み続ける役目は :class:`Ryuw122Tag` が持つ. 設定は Flash に残るので
**一度書けば次回からは不要**.

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

__all__ = [
    "Ryuw122Hal",
    "Ryuw122Config",
    "Ryuw122Terminal",
    "Ryuw122Tag",
    "parse_anchor_rcv",
]

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


def _write_line(stream: IO[str], line: str) -> None:
    stream.write(line + "\r\n")
    flush = getattr(stream, "flush", None)
    if flush is not None:
        flush()


def _at_command(stream: IO[str], cmd: str, timeout: float = 1.0) -> list[str]:
    """AT コマンドを 1 つ送り, ``+OK`` / ``+ERR`` が返るまでの行を返す.

    仕様書 3 節: 「``+OK`` が返るまで次のコマンドを実行しないこと」.
    """
    _write_line(stream, cmd)
    out: list[str] = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = stream.readline()
        if not line:
            continue
        line = line.strip()
        if not line:
            continue
        out.append(line)
        if line.startswith("+OK") or line.startswith("+ERR"):
            break
    return out


def _open_serial_stream(port: str, baudrate: int = 115200) -> tuple[IO[str], Any]:
    """シリアルポートをテキストストリームとして開く (既定 115200 は仕様書 4 節)."""
    import io

    import serial  # type: ignore[import-not-found]

    ser = serial.Serial(port, baudrate, timeout=0.2)
    stream = io.TextIOWrapper(ser, encoding="ascii", errors="replace",
                              newline="\n", write_through=True)
    return stream, ser


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
        ``AT+RSSI`` 受信強度を ``+ANCHOR_RCV`` / ``+TAG_RCV`` に載せるか.
        載せると品質値として使えるので既定で有効にする.
    duty_cycle:
        ``AT+TAGD`` TAG の RF デューティ ``(有効 [ms], 無効 [ms])``.
        既定 (工場出荷) は ``(0, 0)`` = 常時有効で, 測位用途ならそのままでよい.
        電池を持たせたいときだけ設定する. 0 以外は 10〜28000 ms.
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
        duty_cycle: tuple[int, int] | None = None,
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
        if duty_cycle is not None:
            for v in duty_cycle:
                if v != 0 and not 10 <= v <= 28000:
                    raise ValueError("duty_cycle は 0 か 10〜28000 ms (仕様書 11 節)")
        self.duty_cycle = duty_cycle
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
        ``as_anchor=False`` なら TAG (``AT+MODE=0``) として設定する.
        無線側の設定 (NETWORKID / CPIN / CHANNEL / BANDWIDTH) は
        **ANCHOR と TAG で同じ値**にしないと通信が成立しないので,
        同じ :class:`Ryuw122Config` を両方に流すのが安全.
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
        if self.duty_cycle is not None and not as_anchor:
            out.append(f"AT+TAGD={self.duty_cycle[0]},{self.duty_cycle[1]}")
        out.append(f"AT+RSSI={1 if self.rssi else 0}")
        return out

    def for_tag(self, address: str) -> "Ryuw122Config":
        """無線側の設定はそのままに, アドレスだけ差し替えた複製を返す.

        ANCHOR 用に作った設定から TAG 用の設定を起こすためのもの.
        **アドレスは機体ごとに変える必要がある** (工場出荷値は全機同じ).
        """
        return Ryuw122Config(
            network_id=self.network_id,
            address=address,
            password=self.password,
            channel=self.channel,
            bandwidth=self.bandwidth,
            power=self.power,
            calibration_cm=self.calibration_cm,
            rssi=self.rssi,
            duty_cycle=self.duty_cycle,
        )


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
        stream, ser = _open_serial_stream(port, baudrate)
        hal = cls(stream, tag_addresses, **kw)
        hal._serial = ser  # type: ignore[attr-defined]
        return hal

    # ------------------------------------------------------------------ AT

    def _write(self, line: str) -> None:
        _write_line(self._stream, line)

    def command(self, cmd: str, timeout: float = 1.0) -> list[str]:
        """AT コマンドを 1 つ送り, ``+OK`` / ``+ERR`` が返るまでの行を返す."""
        out = _at_command(self._stream, cmd, timeout)
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


class Ryuw122Terminal:
    """AT コマンドを打つだけの最小セッション (測距はしない).

    用途は 2 つ.

    1. **TAG の設定を書き込む.** 固定局を TAG にする構成では, TAG 側にも
       ``AT+MODE=0`` と NETWORKID / CPIN / CHANNEL / BANDWIDTH を
       ANCHOR と同じ値で入れ, **アドレスだけ機体ごとに変える**必要がある.
       工場出荷値はどの機体も同じなので, 買ってきたまま並べると
       アドレスが衝突して測距できない.
    2. **今の設定を読む.** :meth:`info` で MODE / ADDRESS / NETWORKID などを
       まとめて問い合わせる. 立ち上げでハマったときの最初の一手.

    設定はモジュールの Flash に残るので, **一度書けば次回からは不要**.

        with Ryuw122Terminal.from_serial("/dev/ttyUSB0") as t:
            t.provision(Ryuw122Config(network_id="REYAX123",
                                      address="TAG00001"), as_anchor=False)
            print(t.info())
    """

    #: :meth:`info` が問い合わせる項目. ``AT+UID?`` だけは読み出し専用の
    #: 機体固有値 (96 bit), 残りは書き換えられる設定.
    QUERIES: tuple[tuple[str, str], ...] = (
        ("mode", "AT+MODE?"),
        ("address", "AT+ADDRESS?"),
        ("network_id", "AT+NETWORKID?"),
        ("channel", "AT+CHANNEL?"),
        ("bandwidth", "AT+BANDWIDTH?"),
        ("power", "AT+CRFOP?"),
        ("cal", "AT+CAL?"),
        ("rssi", "AT+RSSI?"),
        ("tagd", "AT+TAGD?"),
        ("uid", "AT+UID?"),
        ("version", "AT+VER?"),
    )

    def __init__(self, stream: IO[str]) -> None:
        self._stream = stream
        self._serial: Any = None
        #: 送ったコマンドと応答 (デバッグ用).
        self.log: list[str] = []

    @classmethod
    def from_serial(cls, port: str, baudrate: int = 115200) -> "Ryuw122Terminal":
        stream, ser = _open_serial_stream(port, baudrate)
        t = cls(stream)
        t._serial = ser
        return t

    def __enter__(self) -> "Ryuw122Terminal":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def command(self, cmd: str, timeout: float = 1.0) -> list[str]:
        out = _at_command(self._stream, cmd, timeout)
        self.log.append(f"{cmd} -> {' / '.join(out) or '(応答なし)'}")
        return out

    def query(self, cmd: str, timeout: float = 1.0) -> str | None:
        """``AT+ADDRESS?`` → ``"DAVID123"`` のように値だけ取り出す."""
        for line in self.command(cmd, timeout):
            if line.startswith("+") and "=" in line and not line.startswith("+OK"):
                return line.split("=", 1)[1].strip()
        return None

    def info(self) -> dict[str, str | None]:
        """今の設定をまとめて読む. 応答しない項目は None."""
        return {name: self.query(cmd) for name, cmd in self.QUERIES}

    def provision(self, config: Ryuw122Config, *, as_anchor: bool) -> bool:
        """設定を流し込む. 全コマンドが ``+OK`` を返したら True."""
        ok = True
        for cmd in config.commands(as_anchor=as_anchor):
            resp = self.command(cmd)
            if not any(r.startswith("+OK") for r in resp):
                ok = False
        return ok

    def factory_reset(self) -> list[str]:
        """``AT+FACTORY``. 出荷時の値に戻す (仕様書 20 節).

        戻ると NETWORKID / ADDRESS / CPIN も既定値になるので,
        **他の機体とアドレスが衝突する**. 戻したら必ず設定し直すこと.
        """
        return self.command("AT+FACTORY", timeout=2.0)

    def close(self) -> None:
        try:
            self._stream.close()
        except Exception:  # pragma: no cover
            pass


class Ryuw122Tag:
    """TAG 側の面倒を見る: 設定を入れて ``AT+TAG_SEND`` を積み続ける.

    TAG は **データを積んでおかないと ANCHOR の呼びかけに応答できず,
    距離が出ない** (仕様書 14 節). 本来は TAG 側の MCU が定期的に打つ役目
    だが, MCU を用意する前に PC を繋いで動かしてみたいときにこれを使う.

    ``+TAG_RCV`` (ANCHOR に読まれた合図) を見たらすぐ次を積む. 見えなくても
    ``refill`` 秒ごとに積み直すので, 取りこぼしても止まらない.

    **ペイロード長は ANCHOR 側と 3 バイト以内**に揃える (仕様書 13 節).
    既定はどちらも ``"RNGE"`` (4 バイト) なので, 両方を既定のまま使う分には
    気にしなくてよい.

        tag = Ryuw122Tag.from_serial("/dev/ttyUSB1",
                                     config=Ryuw122Config(network_id="REYAX123",
                                                          address="TAG00001"))
        tag.open()      # 設定を入れて積み続ける
        ...
        tag.close()
    """

    def __init__(
        self,
        stream: IO[str],
        *,
        config: Ryuw122Config | None = None,
        payload: str = "RNGE",
        refill: float = 0.2,
        setup: bool = True,
    ) -> None:
        if not 0 < len(payload) <= 12:
            raise ValueError("payload は 1〜12 バイト (仕様書 14 節)")
        self._stream = stream
        self.config = config
        self.payload = payload
        self.refill = float(refill)
        self._do_setup = setup

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

        #: 立ち上げの切り分け用.
        self.n_sent = 0        # AT+TAG_SEND を積んだ回数
        self.n_rcv = 0         # ANCHOR に読まれた回数 (+TAG_RCV)
        self.last_error: str | None = None
        self.setup_log: list[str] = []

    @classmethod
    def from_serial(cls, port: str, baudrate: int = 115200, **kw: Any) -> "Ryuw122Tag":
        stream, ser = _open_serial_stream(port, baudrate)
        tag = cls(stream, **kw)
        tag._serial = ser  # type: ignore[attr-defined]
        return tag

    def setup(self) -> bool:
        """モジュールを TAG に設定する."""
        self.setup_log = []
        ok = True
        for cmd in (self.config or Ryuw122Config()).commands(as_anchor=False):
            resp = _at_command(self._stream, cmd)
            self.setup_log.append(f"{cmd} -> {' / '.join(resp) or '(応答なし)'}")
            if not any(r.startswith("+OK") for r in resp):
                ok = False
                if resp and resp[-1].startswith("+ERR"):
                    self.last_error = f"{cmd} -> {resp[-1]}"
        return ok

    def _stage(self) -> None:
        _write_line(self._stream, f"AT+TAG_SEND={len(self.payload)},{self.payload}")
        self.n_sent += 1

    def _worker(self) -> None:
        self._stage()
        last = time.monotonic()
        while not self._stop.is_set():
            line = self._stream.readline()
            now = time.monotonic()
            if line:
                s = line.strip()
                if s.startswith("+TAG_RCV"):
                    # 読まれた = 積んだデータが消費された。すぐ次を積む。
                    self.n_rcv += 1
                    self._stage()
                    last = now
                    continue
                if s.startswith("+ERR"):
                    self.last_error = s
            if now - last >= self.refill:
                self._stage()
                last = now

    def open(self) -> None:
        if self._thread is not None:
            return
        if self._do_setup:
            self.setup()
        self._stop.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True,
                                        name="uwb-ryuw122-tag")
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            self._stream.close()
        except Exception:  # pragma: no cover
            pass

    def __enter__(self) -> "Ryuw122Tag":
        self.open()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()
