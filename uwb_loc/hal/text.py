"""既存ファームウェアの出力をそのまま読む HAL.

多くの UWB モジュールは、何もしなくても既に距離をシリアルに吐いている.
形式がばらばらなだけで, 情報は出ている::

    A0: 3.21 m
    range,A1,2887          <- mm
    DIST,AN0,0x1234,3.214,AN1,...
    [12.345] anchor=2 dist=4.55 rssi=-79

:class:`TextHal` は**正規表現 1 本**でこれらを観測に変える. ファームウェアを
書き換える必要も, Python のクラスを書く必要もない.

    hal = TextHal.from_serial("/dev/ttyUSB0", 115200,
                              r"(?P<anchor>A\\d+):\\s*(?P<dist>[\\d.]+)\\s*m",
                              anchors=anchors)

うまく取れているかは :func:`sniff` で先に確かめられる::

    python -m uwb_loc sniff --serial /dev/ttyUSB0

必須の名前つきグループは ``anchor`` と ``dist`` の 2 つだけ.
``t`` / ``q`` / ``tag`` があれば使う.
"""

from __future__ import annotations

import queue
import re
import threading
import time
from typing import IO, Any

from ..types import Anchor, MeasKind, Measurement, MeasurementBatch
from .base import UwbHal

__all__ = ["TextHal", "UNITS", "sniff"]

#: 単位名 -> m への換算. **ここを間違えるのが実機立ち上げで一番多い事故**なので
#: 明示的に選ばせる (DW1000 系の生値はミリメートルであることが多い).
UNITS: dict[str, float] = {"m": 1.0, "cm": 0.01, "mm": 0.001}


class TextHal(UwbHal):
    """行指向のテキスト出力を正規表現で読む HAL.

    Parameters
    ----------
    stream:
        ``readline()`` を持つテキストストリーム.
    pattern:
        1 本の測距を表す正規表現. 名前つきグループ ``anchor`` と ``dist`` が必須.
        任意で ``t`` (時刻 [s]), ``q`` (品質 0-1), ``tag``.
        1 行に複数の測距が並んでいてもよい (``finditer`` で全部拾う).
    anchors:
        アンカー一覧. ID は ``anchor`` グループが返す文字列と一致させる.
    unit:
        ``dist`` の単位. ``"m"`` / ``"cm"`` / ``"mm"``.
    anchor_prefix:
        ``anchor`` グループが数字だけを返す場合の接頭辞.
        ``"A"`` なら ``2`` -> ``"A2"``.
    group:
        True なら「同じアンカーが再び出てきたら 1 巡完了」とみなして
        測距をエポックにまとめる. 順繰りにポーリングするファームウェアの
        出力に合う. False なら 1 行 1 エポックにする.

        **Lv3 (密結合 EKF) を使うならまとめなくてよい** — 測距が届いた順に
        1 本ずつ処理できる. Lv0-Lv2 は 1 エポックに 4 本以上必要なので
        まとめる必要がある.
    max_span:
        まとめる場合の打ち切り時間 [s]. これを超えたら溜まっている分で確定する.
    rate_hz:
        ファームウェアが時刻を出さない場合に, 時刻を等間隔で合成する [Hz].
        **記録したログを後から流す (リプレイ) ときは必ず指定すること** —
        指定しないとホストの時計を使うので, 一瞬で読み終わるリプレイでは
        全観測が同時刻になり, Lv3 の予測ステップが止まって精度が出ない
        (実測で RMSE 1.33 m 対 0.21 m).

        実機からリアルタイムに読む場合はホストの時計でもそれなりに合うが,
        シリアルの遅延ぶん遅れる. 速く動くものを追うならファームウェア側で
        時刻を出して ``t`` グループで拾うのが正しい.
    scale, offset:
        距離の補正 ``d = scale * 生値 + offset`` [m]. アンテナ遅延は
        :attr:`Anchor.antenna_delay_m` でも引けるので, 普通はそちらを使う.
    """

    def __init__(
        self,
        stream: IO[str],
        pattern: str,
        anchors: list[Anchor] | None = None,
        *,
        unit: str = "m",
        anchor_prefix: str = "",
        group: bool = True,
        max_span: float = 0.5,
        rate_hz: float | None = None,
        scale: float = 1.0,
        offset: float = 0.0,
        name: str = "text",
    ) -> None:
        if unit not in UNITS:
            raise ValueError(f"unit は {list(UNITS)} のいずれか (指定: {unit!r})")
        self._re = re.compile(pattern)
        if "anchor" not in self._re.groupindex or "dist" not in self._re.groupindex:
            raise ValueError(
                "pattern には名前つきグループ (?P<anchor>...) と (?P<dist>...) が必要です"
            )
        self._stream = stream
        self._anchors = list(anchors or [])
        self.unit = unit
        self.anchor_prefix = anchor_prefix
        self.group = group
        self.max_span = float(max_span)
        self.rate_hz = None if rate_hz is None else float(rate_hz)
        self.scale = float(scale)
        self.offset = float(offset)
        self.name = name

        self._queue: queue.Queue[MeasurementBatch] = queue.Queue()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._eof = False
        #: 解釈できなかった行数 / 解釈できた行数 (立ち上げの切り分け用).
        self.n_matched = 0
        self.n_unmatched = 0
        #: 時刻がストリーム由来か (False ならホストの時計 = 精度が落ちうる).
        self.has_stream_time = "t" in self._re.groupindex or rate_hz is not None
        self._n_batches = 0

    # ------------------------------------------------------------------ 生成

    @classmethod
    def from_path(cls, path: str, pattern: str, **kw: Any) -> "TextHal":
        return cls(open(path, "r", encoding="utf-8", errors="replace"), pattern,
                   name=f"text:{path}", **kw)

    @classmethod
    def from_serial(cls, port: str, baudrate: int, pattern: str, **kw: Any) -> "TextHal":
        import io

        import serial  # type: ignore[import-not-found]

        ser = serial.Serial(port, baudrate, timeout=1.0)
        stream = io.TextIOWrapper(ser, encoding="utf-8", errors="replace", newline="\n")
        hal = cls(stream, pattern, name=f"text:{port}", **kw)
        hal._serial = ser  # type: ignore[attr-defined]
        return hal

    @classmethod
    def from_tcp(cls, host: str, port: int, pattern: str, **kw: Any) -> "TextHal":
        import socket

        sock = socket.create_connection((host, port))
        hal = cls(sock.makefile("r", encoding="utf-8", errors="replace"), pattern,
                  name=f"text:{host}:{port}", **kw)
        hal._socket = sock  # type: ignore[attr-defined]
        return hal

    # ------------------------------------------------------------------ 解釈

    def parse(self, line: str, now: float) -> list[Measurement]:
        """1 行から測距を取り出す (テスト・sniff から直接呼べる)."""
        out: list[Measurement] = []
        for m in self._re.finditer(line):
            g = m.groupdict()
            try:
                raw = float(g["dist"])
            except (TypeError, ValueError):
                continue
            aid = self.anchor_prefix + str(g["anchor"]).strip()
            t = now
            if g.get("t"):
                try:
                    t = float(g["t"])
                except ValueError:
                    pass
            q = None
            if g.get("q"):
                try:
                    q = float(g["q"])
                except ValueError:
                    q = None
            out.append(
                Measurement(
                    anchor_id=aid,
                    value=self.scale * raw * UNITS[self.unit] + self.offset,
                    kind=MeasKind.RANGE,
                    t=t,
                    quality=q,
                    tag_id=str(g.get("tag") or "tag0"),
                    raw={"line": line.strip()[:120]},
                )
            )
        return out

    # -------------------------------------------------------------- ライフサイクル

    def open(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True, name="uwb-text")
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

    @property
    def is_open(self) -> bool:
        return not (self._eof and self._queue.empty())

    # ------------------------------------------------------------------ 読み取り

    def _reader(self) -> None:
        pending: list[Measurement] = []
        seen: set[str] = set()
        started = 0.0

        def flush() -> None:
            nonlocal pending, seen
            if pending:
                self._queue.put(self._make_batch(pending))
            pending, seen = [], set()

        while not self._stop.is_set():
            try:
                line = self._stream.readline()
            except Exception:
                break
            if line == "":
                break

            now = time.monotonic()
            found = self.parse(line, now)
            if found:
                self.n_matched += 1
            else:
                self.n_unmatched += 1
                continue

            if not self.group:
                self._queue.put(self._make_batch(found))
                continue

            for m in found:
                # 同じアンカーが再び出た = 次の巡回に入った、とみなす.
                if m.anchor_id in seen or (pending and now - started > self.max_span):
                    flush()
                if not pending:
                    started = now
                pending.append(m)
                seen.add(m.anchor_id)

        flush()
        self._eof = True

    def _make_batch(self, ms: list[Measurement]) -> MeasurementBatch:
        """測距をエポックにまとめる (必要なら時刻を合成する)."""
        if self.rate_hz is not None and "t" not in self._re.groupindex:
            t = self._n_batches / self.rate_hz
            for m in ms:
                m.t = t
        else:
            t = ms[0].t
        self._n_batches += 1
        return MeasurementBatch(t=t, measurements=ms, tag_id=ms[0].tag_id)

    # -------------------------------------------------------------- インターフェイス

    @property
    def anchors(self) -> list[Anchor]:
        return list(self._anchors)

    def poll(self, timeout: float = 0.0) -> list[MeasurementBatch]:
        if self._thread is None:
            self.open()
        out: list[MeasurementBatch] = []
        try:
            out.append(self._queue.get(timeout=timeout) if timeout > 0
                       else self._queue.get_nowait())
        except queue.Empty:
            return out
        while True:
            try:
                out.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return out


# --------------------------------------------------------------------------- sniff


def sniff(stream: IO[str], pattern: str | None = None, *, n: int = 40,
          unit: str = "m", anchor_prefix: str = "") -> dict[str, Any]:
    """流れてくる行を覗いて、正規表現が効いているか確かめる.

    実機立ち上げの**最初にやること**. 単位の取り違えとアンカー ID の
    不一致は、ここで潰しておかないと後段で原因が分からなくなる.

    ``pattern`` を省くと、よくある書式を順に当ててみて、いちばん多く
    引っかかったものを提案する.

    Returns
    -------
    dict
        ``lines`` 読んだ行数, ``matched`` 当たった行数, ``anchors`` 見つかった
        アンカー ID, ``ranges`` 距離の範囲 [m], ``samples`` 生の行の例,
        ``pattern`` 使った (または提案する) 正規表現.
    """
    #: よくある書式. 上から順に試す.
    GUESSES = [
        r"(?P<anchor>[A-Za-z]+\d+)\s*[:=,]\s*(?P<dist>-?[\d.]+)",
        r"anchor\s*[:=]\s*(?P<anchor>\w+).*?dist\w*\s*[:=]\s*(?P<dist>-?[\d.]+)",
        r"(?P<anchor>0[xX][0-9a-fA-F]+)\s*,\s*(?P<dist>-?[\d.]+)",
        r"(?P<anchor>\d+)\s*,\s*(?P<dist>-?[\d.]+)",
    ]
    lines = [stream.readline() for _ in range(n)]
    lines = [ln for ln in lines if ln]

    def try_pattern(pat: str) -> tuple[int, list[Measurement]]:
        try:
            hal = TextHal(_Null(), pat, unit=unit, anchor_prefix=anchor_prefix)
        except (re.error, ValueError):
            return 0, []
        hits, ms = 0, []
        for ln in lines:
            found = hal.parse(ln, 0.0)
            if found:
                hits += 1
                ms.extend(found)
        return hits, ms

    if pattern is None:
        best = max(((try_pattern(p)[0], p) for p in GUESSES), key=lambda x: x[0])
        pattern = best[1] if best[0] else GUESSES[0]
    hits, ms = try_pattern(pattern)

    dists = [m.value for m in ms]
    return {
        "lines": len(lines),
        "matched": hits,
        "pattern": pattern,
        "anchors": sorted({m.anchor_id for m in ms}),
        "ranges": (min(dists), max(dists)) if dists else None,
        "samples": [ln.rstrip()[:110] for ln in lines[:6]],
    }


class _Null:
    """sniff がパターンを試すためだけのダミーストリーム."""

    def readline(self) -> str:  # pragma: no cover - 呼ばれない
        return ""

    def close(self) -> None:  # pragma: no cover
        pass
