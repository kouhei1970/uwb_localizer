"""ローカル起動のブラウザ UI.

    python -m uwb_loc ui           # http://127.0.0.1:8765

外部 CDN を一切使わない単一ページなので, ネットワークのない現場でも動く.
できること:

* アンカーを置いて (ドラッグで移動), 誤差モデルと軌道を振ってシミュレーション
* Lv0-Lv3 を**同じ観測列**に通して比較 — アルゴリズムの差だけが見える
* GDOP ヒートマップで「部屋のどこが弱いか」を設置前に確認
* 実機の観測 (JSON Lines: ファイル/TCP/シリアル) を流し込んでライブ表示

標準ライブラリの ``http.server`` だけで動かしている. UI のために
Flask などを足すと, ライブラリ本体の依存まで重くなるため.
"""

from __future__ import annotations

import json
import threading
import traceback
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np

from ..geometry import anchor_condition, crlb_at, gdop_map
from ..hal.jsonl import JsonLinesHal
from ..hal.ryuw122 import Ryuw122Config, Ryuw122Hal
from ..hal.text import TextHal
from ..metrics import error_cdf, error_series, error_stats
from ..sim import ErrorModel, SimulatedHal, make_anchors, room_anchors, trajectory
from ..solvers import LEVELS, SolveConfig, make_estimator
from ..types import Anchor, MeasKind

__all__ = ["serve", "simulate", "make_app"]

_STATIC = Path(__file__).parent / "static"


# --------------------------------------------------------------------------- 設定の解釈


def _anchors_from(req: dict[str, Any]) -> list[Anchor]:
    """要求からアンカー一覧を取り出す.

    ``anchors`` が**あれば**それを使う — 空リストなら空のまま返す.
    「アンカーを全部消した」を「既定の部屋を使え」と読み替えてしまうと,
    実機のライブ表示で**座標が無いのにもっともらしい位置が出る**という
    最悪の挙動になる (捏造した配置で解いてしまう).
    キー自体が無いときだけ, 既定の部屋を組む.
    """
    if "anchors" in req:
        return [Anchor.from_dict(a) for a in (req["anchors"] or [])]
    room = req.get("room", [8.0, 6.0, 2.6])
    return room_anchors(tuple(room), n_low=int(req.get("n_low", 4)))


def _trajectory_from(req: dict[str, Any]):
    spec = req.get("traj", {}) or {}
    kind = spec.get("type", "figure8")
    room = req.get("room", [8.0, 6.0, 2.6])
    center = spec.get("center", [room[0] / 2, room[1] / 2, min(1.2, room[2] * 0.5)])
    period = float(spec.get("period", 24.0))
    size = float(spec.get("size", min(room[0], room[1]) * 0.3))

    if kind == "static":
        return trajectory.static(np.array(center, dtype=float))
    if kind == "circle":
        return trajectory.circle(np.array(center), size, period, float(spec.get("z_amp", 0.0)))
    if kind == "line":
        p0 = spec.get("p0", [1.0, 1.0, center[2]])
        p1 = spec.get("p1", [room[0] - 1.0, room[1] - 1.0, center[2]])
        return trajectory.line(np.array(p0), np.array(p1), period)
    if kind == "random_walk":
        bounds = ((0.3, room[0] - 0.3), (0.3, room[1] - 0.3), (0.3, room[2] - 0.3))
        return trajectory.random_walk(
            np.array(center), float(spec.get("speed", 0.6)), bounds, int(req.get("seed", 0))
        )
    return trajectory.figure8(np.array(center), size, period, float(spec.get("z_amp", 0.3)))


def _error_from(req: dict[str, Any]) -> ErrorModel:
    e = req.get("error", {}) or {}
    return ErrorModel(
        sigma0=float(e.get("sigma0", 0.08)),
        sigma_per_m=float(e.get("sigma_per_m", 0.004)),
        nlos_prob=float(e.get("nlos_prob", 0.15)),
        nlos_hold=float(e.get("nlos_hold", 1.5)),
        nlos_bias_mean=float(e.get("nlos_bias_mean", 0.8)),
        loss_rate=float(e.get("loss_rate", 0.03)),
        max_range=float(e.get("max_range", 40.0)),
        antenna_delay=float(e.get("antenna_delay", 0.0)),
        anchor_position_error=float(e.get("anchor_position_error", 0.0)),
        report_sigma=bool(e.get("report_sigma", True)),
        report_quality=bool(e.get("report_quality", True)),
    )


def _config_from(req: dict[str, Any]) -> SolveConfig:
    return SolveConfig(
        dim=int(req.get("dim", 3)),
        z_fixed=float(req.get("z_fixed", 1.0)),
        z_bounds=tuple(req["z_bounds"]) if req.get("z_bounds") else None,
    )


def _estimator_kwargs(level: str, req: dict[str, Any]) -> dict[str, Any]:
    if level == "Lv3":
        return {
            "motion": req.get("motion", "cv"),
            "sigma_a": float(req.get("sigma_a", 1.0)),
            "gate": float(req.get("gate", 3.0)),
        }
    return {}


def _clean(x: float) -> float | None:
    return None if not np.isfinite(x) else float(x)


# --------------------------------------------------------------------------- 本体


def simulate(req: dict[str, Any]) -> dict[str, Any]:
    """シミュレーションを実行して UI に返す辞書を作る.

    UI とは独立に呼べるので, 同じ結果をスクリプトからも再現できる.
    """
    anchors = _anchors_from(req)
    error = _error_from(req)
    config = _config_from(req)
    seed = int(req.get("seed", 0))
    rate = float(req.get("rate", 10.0))
    duration = float(req.get("duration", 40.0))
    kind = MeasKind.TDOA if req.get("kind") == "tdoa" else MeasKind.RANGE
    levels = [lv for lv in req.get("levels", ["Lv0", "Lv2", "Lv3"]) if lv in LEVELS]

    hal = SimulatedHal(anchors, _trajectory_from(req), error, rate_hz=rate, kind=kind, seed=seed)
    times, truths, batches = hal.generate(duration)
    truth = np.array(truths)

    results: dict[str, Any] = {}
    metrics: dict[str, Any] = {}
    cdf: dict[str, Any] = {}
    for lv in levels:
        est = make_estimator(lv, anchors, config, **_estimator_kwargs(lv, req))
        fixes = [est.update(b) for b in batches]
        pos = np.array([f.position for f in fixes])
        e3, _ = error_series(truth, pos)
        x, p = error_cdf(e3)
        results[lv] = {
            "p": [[_clean(v) for v in row] for row in pos],
            "ok": [bool(f.ok) for f in fixes],
            "sigma": [_clean(f.sigma) for f in fixes],
            "gdop": [_clean(f.gdop) for f in fixes],
            "n_used": [f.n_used for f in fixes],
            "n_excluded": [len(f.excluded) for f in fixes],
            "err": [_clean(v) for v in e3],
        }
        metrics[lv] = {k: _clean(v) for k, v in error_stats(truth, pos).items()}
        cdf[lv] = {"x": [float(v) for v in x], "p": [float(v) for v in p]}

    room = req.get("room", [8.0, 6.0, 2.6])
    gz = float(req.get("gdop_z", 1.2))
    gx, gy, gg = gdop_map(
        anchors, ((0.0, room[0]), (0.0, room[1])), gz, nx=48, ny=36, dim=config.dim
    )
    gg = np.where(np.isfinite(gg), gg, 99.0)

    n_meas = [len(b) for b in batches]
    n_nlos = sum(1 for b in batches for m in b.measurements if m.raw.get("nlos_truth"))
    total = max(sum(n_meas), 1)

    return {
        "t": [float(v) for v in times],
        "truth": [[float(v) for v in row] for row in truth],
        "anchors": [a.to_dict() for a in anchors],
        "results": results,
        "metrics": metrics,
        "cdf": cdf,
        "gdop": {
            "x": [float(v) for v in gx],
            "y": [float(v) for v in gy],
            "g": [[float(v) for v in row] for row in gg],
            "z": gz,
        },
        "crlb": _clean(crlb_at(truth.mean(axis=0), anchors, dim=config.dim)),
        "anchor_condition": {
            k: (bool(v) if isinstance(v, bool) else float(v))
            for k, v in anchor_condition(anchors).items()
        },
        "link_stats": {
            "mean_meas_per_epoch": float(np.mean(n_meas)) if n_meas else 0.0,
            "nlos_ratio": n_nlos / total,
            "epochs": len(batches),
        },
    }


# --------------------------------------------------------------------------- ライブ


class LiveSession:
    """実機 (または模擬 HAL) を裏で回して結果を溜める.

    ブラウザは ``/api/live/poll`` を叩いて差分だけ取りに来る.
    WebSocket を使わないのは, 標準ライブラリだけで完結させるため.
    """

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.thread: threading.Thread | None = None
        self.stop_flag = threading.Event()
        self.fixes: deque = deque(maxlen=20000)
        self.total = 0
        self.error: str | None = None
        self.source = ""
        self._hal: Any = None

    def start(self, req: dict[str, Any]) -> None:
        self.stop()
        self.stop_flag.clear()
        self.error = None
        with self.lock:
            self.fixes.clear()
            self.total = 0

        anchors = _anchors_from(req)
        config = _config_from(req)
        level = req.get("level", "Lv3")
        source = req.get("source", "sim")
        self.source = source

        # 経路 (source) と 形式 (fmt) は独立に選べる.
        # 形式が "text" ならファームの出力を正規表現で読む (JSON でなくてよい).
        fmt = req.get("format", "jsonl")
        text_kw = dict(
            anchors=anchors,
            unit=req.get("unit", "m"),
            anchor_prefix=req.get("prefix", ""),
        )
        if req.get("assume_rate"):
            text_kw["rate_hz"] = float(req.get("rate", 10.0))
        pattern = req.get("pattern") or r"(?P<anchor>[A-Za-z]*\d+)\s*[:=,]\s*(?P<dist>-?[\d.]+)"

        def build_hal():
            if source == "ryuw122":
                # TAG アドレス = アンカーの ID。別々に入力させると必ずずれるので
                # 1 箇所にまとめてある (アンカー一覧の ID をそのまま呼ぶ)。
                tags = [a.id for a in anchors if a.enabled]
                cfg = Ryuw122Config(
                    network_id=req.get("network_id") or None,
                    address=req.get("self_address") or None,
                    password=req.get("password") or None,
                    channel=int(req["channel"]) if req.get("channel") else None,
                    bandwidth=int(req["bandwidth"]) if req.get("bandwidth") not in (None, "") else None,
                    calibration_cm=int(req["cal_cm"]) if req.get("cal_cm") not in (None, "") else None,
                )
                return Ryuw122Hal.from_serial(
                    req["port"], tags, int(req.get("baud", 115200)),
                    anchors=anchors, config=cfg,
                    payload=req.get("payload") or "RNGE",
                    timeout=float(req.get("range_timeout", 0.35)),
                )
            if source == "file":
                return (TextHal.from_path(req["path"], pattern, **text_kw)
                        if fmt == "text"
                        else JsonLinesHal.from_path(req["path"], anchors=anchors))
            if source == "tcp":
                host, port = req["host"], int(req["port"])
                return (TextHal.from_tcp(host, port, pattern, **text_kw)
                        if fmt == "text"
                        else JsonLinesHal.from_tcp(host, port, anchors=anchors))
            if source == "serial":
                port, baud = req["port"], int(req.get("baud", 115200))
                return (TextHal.from_serial(port, baud, pattern, **text_kw)
                        if fmt == "text"
                        else JsonLinesHal.from_serial(port, baud, anchors=anchors))
            return SimulatedHal(
                anchors,
                _trajectory_from(req),
                _error_from(req),
                rate_hz=float(req.get("rate", 10.0)),
                seed=int(req.get("seed", 0)),
            )

        def run() -> None:
            import time

            try:
                hal = build_hal()
                self._hal = hal
                hal.open()
                realtime = source == "sim"
                dt = 1.0 / float(req.get("rate", 10.0))

                # 測位器はアンカーが分かってから作る。JSON Lines は観測源が
                # anchors メッセージで座標を送ってくることがあり、それは
                # open() 直後にはまだ読めていない (読み取りは別スレッド)。
                est = None
                while not self.stop_flag.is_set() and hal.is_open:
                    for batch in hal.poll(0.5):
                        if est is None:
                            use = hal.anchors or anchors
                            if not use:
                                self.error = (
                                    "アンカー座標がありません。左パネルで配置するか、"
                                    "観測源から anchors メッセージを送ってください。")
                                hal.close()
                                return
                            est = make_estimator(level, use, config,
                                                 **_estimator_kwargs(level, req))
                        fix = est.update(batch)
                        row = fix.to_dict()
                        if isinstance(hal, SimulatedHal):
                            row["truth"] = [float(v) for v in hal.truth(batch.t)]
                        with self.lock:
                            self.fixes.append(row)
                            self.total += 1
                    if realtime:
                        time.sleep(dt)
                hal.close()
            except Exception as exc:  # pragma: no cover - 実機依存
                self.error = f"{type(exc).__name__}: {exc}"

        self.thread = threading.Thread(target=run, daemon=True, name="uwb-live")
        self.thread.start()

    def stop(self) -> None:
        self.stop_flag.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            self.thread = None

    def poll(self, since: int) -> dict[str, Any]:
        with self.lock:
            start = max(self.total - len(self.fixes), 0)
            skip = max(since - start, 0)
            rows = list(self.fixes)[skip:]
            hal = self._hal
            return {
                "n": self.total,
                "fixes": rows,
                "running": self.thread is not None and self.thread.is_alive(),
                "error": self.error,
                "source": self.source,
                # テキスト源で位置が出ないとき、行が読めていないのか
                # 正規表現が当たっていないのかを切り分けるための数字
                "matched": getattr(hal, "n_matched", None),
                "unmatched": getattr(hal, "n_unmatched", None),
                "anchors_known": len(hal.anchors) if hal is not None else 0,
                "setup_log": list(getattr(hal, "setup_log", []) or []),
                "n_ranged": getattr(hal, "n_ranged", None),
                "n_timeout": getattr(hal, "n_timeout", None),
                "last_error": getattr(hal, "last_error", None),
            }


# --------------------------------------------------------------------------- HTTP


def make_app(live: LiveSession | None = None):
    live = live or LiveSession()

    class Handler(BaseHTTPRequestHandler):
        server_version = "uwb_loc-ui"

        def log_message(self, fmt: str, *args: Any) -> None:  # noqa: A003
            pass  # アクセスログは出さない (ターミナルが埋まるため)

        # ------------------------------------------------------------

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj: Any, code: int = 200) -> None:
            self._send(code, json.dumps(obj, ensure_ascii=False).encode("utf-8"),
                       "application/json; charset=utf-8")

        def _body(self) -> dict[str, Any]:
            n = int(self.headers.get("Content-Length", 0))
            if n <= 0:
                return {}
            return json.loads(self.rfile.read(n).decode("utf-8"))

        # ------------------------------------------------------------

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(200, (_STATIC / "index.html").read_bytes(),
                           "text/html; charset=utf-8")
                return
            if path == "/api/defaults":
                self._json(_defaults())
                return
            if path == "/api/live/poll":
                q = self.path.split("?", 1)
                since = 0
                if len(q) == 2:
                    for kv in q[1].split("&"):
                        if kv.startswith("since="):
                            since = int(kv.split("=", 1)[1] or 0)
                self._json(live.poll(since))
                return
            self._json({"error": "not found"}, 404)

        def do_POST(self) -> None:  # noqa: N802
            path = self.path.split("?")[0]
            try:
                if path == "/api/simulate":
                    self._json(simulate(self._body()))
                    return
                if path == "/api/live/start":
                    live.start(self._body())
                    self._json({"ok": True})
                    return
                if path == "/api/live/stop":
                    live.stop()
                    self._json({"ok": True})
                    return
                self._json({"error": "not found"}, 404)
            except Exception as exc:
                self._json({"error": f"{type(exc).__name__}: {exc}",
                            "trace": traceback.format_exc()}, 500)

    return Handler


def _defaults() -> dict[str, Any]:
    room = [8.0, 6.0, 2.6]
    return {
        "room": room,
        "anchors": [a.to_dict() for a in room_anchors(tuple(room))],
        "error": ErrorModel().__dict__,
        "levels": list(LEVELS),
        "traj": {"type": "figure8", "center": [4.0, 3.0, 1.2], "size": 2.0,
                 "period": 24.0, "z_amp": 0.3},
        "duration": 40.0,
        "rate": 10.0,
        "dim": 3,
        "z_fixed": 1.0,
        "seed": 0,
        "sigma_a": 1.0,
        "motion": "cv",
        "gate": 3.0,
        "gdop_z": 1.2,
    }


def _lan_address() -> str | None:
    """LAN 側の IP を調べる (スマホから開くとき用).

    外向きのソケットを作って自分側のアドレスを見るだけで, 実際の通信はしない.
    """
    import socket

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("192.0.2.1", 9))  # TEST-NET-1 (到達しなくてよい)
            return str(s.getsockname()[0])
    except OSError:  # pragma: no cover - ネットワーク構成依存
        return None


def serve(host: str = "127.0.0.1", port: int = 8765, *, open_browser: bool = True) -> None:
    """UI を起動する (Ctrl-C で終了).

    ``host="0.0.0.0"`` にすると同じ LAN の端末 (スマホ・タブレット) からも
    開ける. 認証は一切ないので, 信用できるネットワークでだけ使うこと.
    """
    httpd = ThreadingHTTPServer((host, port), make_app())
    url = f"http://{host}:{port}"
    print(f"UWB 測位 UI: {url}")
    if host in ("0.0.0.0", "::"):
        lan = _lan_address()
        if lan:
            print(f"  同じ LAN の端末からは http://{lan}:{port}")
        print("  警告: LAN 全体に公開されます (認証なし)。信用できる回線でのみ使ってください。")
        open_browser = False
    if open_browser:
        try:
            import webbrowser

            webbrowser.open(url)
        except Exception:  # pragma: no cover
            pass
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n終了")
    finally:
        httpd.server_close()
