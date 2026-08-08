"""ブラウザ UI のライブ機能 — 何があれば推定が出るかを固定する."""

from __future__ import annotations

import json
import time

import numpy as np
import pytest

import uwb_loc as ul
from uwb_loc.ui.server import LiveSession, _anchors_from


@pytest.fixture
def logs(tmp_path):
    """実機の代わり: 同じ観測をテキストと JSON Lines の両方で書き出す."""
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    hal = ul.SimulatedHal(anchors, ul.trajectory.figure8([4, 3, 1.2]),
                          ul.ErrorModel(nlos_prob=0.1), rate_hz=10, seed=11)
    _, _, batches = hal.generate(8.0)

    text = tmp_path / "fw.txt"
    text.write_text("boot ok\n" + "".join(
        f"range,{m.anchor_id[1:]},{int(m.value * 1000)}\n"
        for b in batches for m in b.measurements), encoding="utf-8")

    jsonl = tmp_path / "fw.jsonl"
    with open(jsonl, "w", encoding="utf-8") as f:
        f.write(json.dumps({"type": "anchors",
                            "anchors": [a.to_dict() for a in anchors]}) + "\n")
        for b in batches:
            f.write(json.dumps(b.to_dict()) + "\n")
    return anchors, str(text), str(jsonl)


def _run(req, wait=3.0):
    live = LiveSession()
    live.start(req)
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait:
        time.sleep(0.1)
        if not (live.thread and live.thread.is_alive()):
            break
    out = live.poll(0)
    live.stop()
    return out


def test_anchors_empty_is_not_the_same_as_absent():
    """「全消去した」を「既定の部屋を使え」と読み替えないこと.

    読み替えると、座標が無いのにもっともらしい位置が出てしまう。
    """
    assert _anchors_from({"anchors": []}) == []
    assert len(_anchors_from({"anchors": [{"id": "A0", "p": [0, 0, 2.4]}]})) == 1
    assert len(_anchors_from({"room": [8, 6, 2.6]})) > 0     # キーが無ければ既定


def test_live_needs_anchor_coordinates(logs):
    """アンカー座標が無ければ、位置を出さずに理由を返す."""
    _, text, _ = logs
    out = _run({"source": "file", "format": "text", "path": text,
                "pattern": r"range,(?P<anchor>\d+),(?P<dist>\d+)",
                "unit": "mm", "prefix": "A", "anchors": []})
    assert out["n"] == 0
    assert "アンカー座標がありません" in (out["error"] or "")


def test_live_from_plain_text(logs):
    """テキスト出力 + UI で置いたアンカー座標 → 位置が出る."""
    anchors, text, _ = logs
    out = _run({"source": "file", "format": "text", "path": text,
                "pattern": r"range,(?P<anchor>\d+),(?P<dist>\d+)",
                "unit": "mm", "prefix": "A", "assume_rate": True, "rate": 10,
                "level": "Lv3",
                "anchors": [a.to_dict() for a in anchors]})
    ok = [f for f in out["fixes"] if f["ok"]]
    assert len(ok) > 50
    assert out["matched"] > 100 and out["unmatched"] <= 2
    # 部屋の中に収まっている
    p = np.array([f["p"] for f in ok])
    assert (p[:, 0] > -1).all() and (p[:, 0] < 9).all()


def test_live_from_json_lines_takes_anchors_from_the_log(logs):
    """JSON Lines なら anchors メッセージで座標も送れる (UI 側の設定不要)."""
    _, _, jsonl = logs
    out = _run({"source": "file", "format": "jsonl", "path": jsonl,
                "level": "Lv2", "anchors": []})
    assert out["anchors_known"] == 8
    assert len([f for f in out["fixes"] if f["ok"]]) > 50


def test_live_reports_when_pattern_never_matches(logs):
    """正規表現が当たらないことを数字で分かるようにする."""
    anchors, text, _ = logs
    out = _run({"source": "file", "format": "text", "path": text,
                "pattern": r"NOPE(?P<anchor>x)(?P<dist>y)",
                "anchors": [a.to_dict() for a in anchors]})
    assert out["n"] == 0
    assert out["matched"] == 0
    assert out["unmatched"] > 100          # 行は読めているが解釈できていない
