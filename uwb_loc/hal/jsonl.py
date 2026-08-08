"""JSON Lines ワイヤ形式の汎用 HAL.

チップごとに Python の HAL クラスを書く代わりに, **ファームウェア側が
1 行 1 JSON を吐くだけ**で繋がるようにしたもの. シリアル・TCP・
ファイル・標準入力のどれでも同じ形式で扱える.

仕様は docs/UWB_PROTOCOL.md. 最小の送信例::

    {"v":1,"type":"anchors","anchors":[{"id":"A0","p":[0,0,0.2]}]}
    {"v":1,"type":"meas","t":12.345,"tag":"tag0",
     "meas":[{"a":"A0","d":3.214,"q":0.93},{"a":"A1","d":2.887,"q":0.41}]}

受信は別スレッドで行いキューに積むので, :meth:`poll` はブロックしない.
"""

from __future__ import annotations

import json
import queue
import threading
from typing import IO, Any

from ..types import Anchor, MeasurementBatch
from .base import UwbHal

__all__ = ["JsonLinesHal", "JsonLinesWriter", "parse_line"]


def parse_line(line: str) -> tuple[str, Any]:
    """1 行を ``(種別, 中身)`` に解釈する.

    種別は ``"meas"`` / ``"anchors"`` / ``"other"``.
    壊れた行は ``("other", None)`` にして落とす (実機のシリアルには
    起動メッセージやデバッグ出力が混ざるのが普通なので, 例外にしない).
    """
    line = line.strip()
    if not line or line[0] not in "{[":
        return "other", None
    try:
        obj = json.loads(line)
    except (json.JSONDecodeError, ValueError):
        return "other", None
    if not isinstance(obj, dict):
        return "other", None

    kind = obj.get("type")
    if kind == "anchors" or ("anchors" in obj and kind is None):
        try:
            return "anchors", [Anchor.from_dict(a) for a in obj["anchors"]]
        except (KeyError, TypeError, ValueError):
            return "other", None
    if kind == "meas" or ("meas" in obj and kind is None):
        try:
            return "meas", MeasurementBatch.from_dict(obj)
        except (KeyError, TypeError, ValueError):
            return "other", None
    return "other", None


class JsonLinesHal(UwbHal):
    """JSON Lines を読むだけの HAL.

    Parameters
    ----------
    stream:
        ``readline()`` を持つテキストストリーム. ファイル, ``sys.stdin``,
        ``serial.Serial`` を ``io.TextIOWrapper`` で包んだもの, ソケットの
        ``makefile("r")`` など.
    anchors:
        アンカー一覧を外から与える場合. ストリーム中の ``anchors`` メッセージが
        来たらそちらで上書きする.
    name:
        表示名.
    """

    def __init__(
        self,
        stream: IO[str],
        anchors: list[Anchor] | None = None,
        *,
        name: str = "jsonl",
    ) -> None:
        self._stream = stream
        self._anchors: list[Anchor] = list(anchors or [])
        self.name = name
        self._queue: queue.Queue[MeasurementBatch] = queue.Queue()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._eof = False

    # ------------------------------------------------------------------ 生成

    @classmethod
    def from_path(cls, path: str, **kw: Any) -> "JsonLinesHal":
        """ログファイルを読む (リプレイ用)."""
        return cls(open(path, "r", encoding="utf-8"), name=f"jsonl:{path}", **kw)

    @classmethod
    def from_tcp(cls, host: str, port: int, **kw: Any) -> "JsonLinesHal":
        """TCP でファームウェア/ブリッジに繋ぐ."""
        import socket

        sock = socket.create_connection((host, port))
        stream = sock.makefile("r", encoding="utf-8")
        hal = cls(stream, name=f"jsonl:{host}:{port}", **kw)
        hal._socket = sock  # type: ignore[attr-defined]
        return hal

    @classmethod
    def from_serial(cls, port: str, baudrate: int = 115200, **kw: Any) -> "JsonLinesHal":
        """シリアルポートから読む (pyserial が要る)."""
        import io

        import serial  # type: ignore[import-not-found]

        ser = serial.Serial(port, baudrate, timeout=1.0)
        stream = io.TextIOWrapper(ser, encoding="utf-8", errors="replace", newline="\n")
        hal = cls(stream, name=f"jsonl:{port}", **kw)
        hal._serial = ser  # type: ignore[attr-defined]
        return hal

    # -------------------------------------------------------------- ライフサイクル

    def open(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True, name="uwb-jsonl")
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        try:
            self._stream.close()
        except Exception:  # pragma: no cover - 閉じ損ねは致命的でない
            pass

    @property
    def is_open(self) -> bool:
        return not (self._eof and self._queue.empty())

    # ------------------------------------------------------------------ 読み取り

    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                line = self._stream.readline()
            except Exception:
                break
            if line == "":  # EOF
                break
            kind, payload = parse_line(line)
            if kind == "meas":
                self._queue.put(payload)
            elif kind == "anchors":
                self._anchors = payload
        self._eof = True

    # -------------------------------------------------------------- インターフェイス

    @property
    def anchors(self) -> list[Anchor]:
        return list(self._anchors)

    def poll(self, timeout: float = 0.0) -> list[MeasurementBatch]:
        if self._thread is None:
            self.open()
        out: list[MeasurementBatch] = []
        try:
            out.append(self._queue.get(timeout=timeout) if timeout > 0 else self._queue.get_nowait())
        except queue.Empty:
            return out
        # 溜まっている分は一気に吸い出す.
        while True:
            try:
                out.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return out


class JsonLinesWriter:
    """観測を JSON Lines で記録する.

    実機のログを取っておけば, あとから UI でリプレイして
    アルゴリズムのレベルを比較できる.
    """

    def __init__(self, path: str, anchors: list[Anchor] | None = None) -> None:
        self._f = open(path, "w", encoding="utf-8")
        if anchors:
            self.write_anchors(anchors)

    def write_anchors(self, anchors: list[Anchor]) -> None:
        """アンカー座標の行を書く. ログの先頭に 1 度だけ置く."""
        obj = {"v": 1, "type": "anchors", "anchors": [a.to_dict() for a in anchors]}
        self._f.write(json.dumps(obj, ensure_ascii=False) + "\n")

    def write(self, batch: MeasurementBatch) -> None:
        """観測 1 エポックを 1 行書く."""
        self._f.write(json.dumps(batch.to_dict(), ensure_ascii=False) + "\n")

    def flush(self) -> None:
        """バッファを吐き出す."""
        self._f.flush()

    def close(self) -> None:
        """閉じる (未書き出しがあれば吐く)."""
        self._f.close()

    def __enter__(self) -> "JsonLinesWriter":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
