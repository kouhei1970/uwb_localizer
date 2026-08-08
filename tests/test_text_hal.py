"""既存ファームの出力をそのまま読む TextHal の検証.

実機立ち上げで実際に踏む所 (単位の取り違え、アンカー ID の不一致、
時刻が無い) を再現して、期待どおりに扱えるかを確かめる。
"""

from __future__ import annotations

import io
import json
import time

import numpy as np
import pytest

import uwb_loc as ul
from uwb_loc.hal.text import sniff

# 実機でよく見る 4 通りの出力
FORMATS = {
    "arduino_m": (
        "A0: 3.214 m\nA1: 2.887 m\nA2: 4.550 m\nA3: 5.102 m\n",
        r"(?P<anchor>A\d+):\s*(?P<dist>[\d.]+)\s*m", "m", "",
    ),
    "csv_mm": (
        "range,0,3214\nrange,1,2887\nrange,2,4550\nrange,3,5102\n",
        r"range,(?P<anchor>\d+),(?P<dist>\d+)", "mm", "A",
    ),
    "key_value": (
        "[12.3] anchor=0 dist=3.214 rssi=-79\n[12.3] anchor=1 dist=2.887 rssi=-86\n"
        "[12.3] anchor=2 dist=4.550 rssi=-80\n[12.3] anchor=3 dist=5.102 rssi=-77\n",
        r"anchor=(?P<anchor>\d+)\s+dist=(?P<dist>[\d.]+)", "m", "A",
    ),
    "one_line": (
        "DIST,4,AN0,3.214,AN1,2.887,AN2,4.550,AN3,5.102\n",
        r"(?P<anchor>AN\d+),(?P<dist>[\d.]+)", "m", "",
    ),
}


def _drain(hal, timeout=1.0):
    hal.open()
    out, t0 = [], time.monotonic()
    while time.monotonic() - t0 < timeout:
        out.extend(hal.poll(0.05))
        if not hal.is_open:
            break
    out.extend(hal.poll(0.05))
    hal.close()
    return out


@pytest.mark.parametrize("key", sorted(FORMATS))
def test_common_firmware_formats_parse(key):
    """よくある 4 書式が、正規表現 1 本ずつで同じ観測になること."""
    text, pattern, unit, prefix = FORMATS[key]
    hal = ul.TextHal(io.StringIO(text), pattern, unit=unit, anchor_prefix=prefix)
    ms = [m for b in _drain(hal) for m in b.measurements]

    assert len(ms) == 4
    # ID は 4 台ぶん重複なく取れていること (体系は書式によって A0.. / AN0.. と違う)
    assert len({m.anchor_id for m in ms}) == 4
    # どの書式でも同じ距離に落ちる (単位換算が効いている)
    assert np.allclose(sorted(m.value for m in ms),
                       [2.887, 3.214, 4.550, 5.102], atol=1e-6)


def test_unit_conversion_is_explicit():
    """単位の指定を間違えると値が 1000 倍ずれる (だから明示させている)."""
    text = "range,0,3214\n"
    pat = r"range,(?P<anchor>\d+),(?P<dist>\d+)"
    mm = ul.TextHal(io.StringIO(text), pat, unit="mm").parse(text, 0.0)[0]
    m = ul.TextHal(io.StringIO(text), pat, unit="m").parse(text, 0.0)[0]
    assert mm.value == pytest.approx(3.214)
    assert m.value == pytest.approx(3214.0)

    with pytest.raises(ValueError):
        ul.TextHal(io.StringIO(""), pat, unit="inch")


def test_pattern_must_have_required_groups():
    with pytest.raises(ValueError, match="anchor"):
        ul.TextHal(io.StringIO(""), r"(\d+),(\d+)")


def test_noise_lines_are_ignored():
    """起動メッセージやデバッグ出力が混ざっても止まらない."""
    text = ("Booting fw v1.2.3\nSPI init ok\nA0: 3.214 m\n"
            "!!! warning: low battery\nA1: 2.887 m\n")
    hal = ul.TextHal(io.StringIO(text), r"(?P<anchor>A\d+):\s*(?P<dist>[\d.]+)")
    ms = [m for b in _drain(hal) for m in b.measurements]
    assert len(ms) == 2
    assert hal.n_unmatched == 3


def test_grouping_flushes_on_anchor_repeat():
    """同じアンカーが再び出たら 1 巡完了とみなしてエポックを切る."""
    text = "".join(f"A{i}: {3.0 + i * 0.1:.3f} m\n" for i in range(4)) * 3
    hal = ul.TextHal(io.StringIO(text), r"(?P<anchor>A\d+):\s*(?P<dist>[\d.]+)")
    batches = _drain(hal)
    assert len(batches) == 3
    assert all(len(b) == 4 for b in batches)


def test_ungrouped_gives_one_measurement_per_batch():
    """Lv3 は 1 本ずつでも処理できるので、まとめない選択肢もある."""
    text = "".join(f"A{i}: 3.2 m\n" for i in range(4))
    hal = ul.TextHal(io.StringIO(text), r"(?P<anchor>A\d+):\s*(?P<dist>[\d.]+)", group=False)
    batches = _drain(hal)
    assert len(batches) == 4
    assert all(len(b) == 1 for b in batches)


def test_end_to_end_positions_from_plain_text():
    """テキスト出力だけから測位まで通ること (実機の代わりに模擬出力を使う)."""
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    sim = ul.SimulatedHal(anchors, ul.trajectory.figure8([4, 3, 1.2]),
                          ul.ErrorModel(nlos_prob=0.1), rate_hz=10, seed=5)
    _, truth, batches = sim.generate(15.0)
    truth = np.array(truth)

    # ミリメートル CSV を吐くファームウェアを模す
    lines = [f"range,{m.anchor_id[1:]},{int(m.value * 1000)}"
             for b in batches for m in b.measurements]
    text = "boot ok\n" + "\n".join(lines) + "\n"
    pat = r"range,(?P<anchor>\d+),(?P<dist>\d+)"

    hal = ul.TextHal(io.StringIO(text), pat, anchors=anchors,
                     unit="mm", anchor_prefix="A", rate_hz=10.0)
    est = np.array([f.position for f in ul.Pipeline(hal, level="Lv3").run()])
    n = min(len(est), len(truth))
    assert n > 100
    assert ul.error_stats(truth[:n], est[:n])["rmse_3d"] < 0.4


def test_missing_timestamps_break_the_filter_but_rate_hz_fixes_it():
    """時刻が無いログを一気に流すと Lv3 の予測が止まる. rate_hz で回復する.

    「どんな情報を渡す必要があるか」の中で、単位の次に見落とされやすい所.
    """
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    sim = ul.SimulatedHal(anchors, ul.trajectory.figure8([4, 3, 1.2]),
                          ul.ErrorModel(nlos_prob=0.1), rate_hz=10, seed=5)
    _, truth, batches = sim.generate(15.0)
    truth = np.array(truth)
    lines = [f"range,{m.anchor_id[1:]},{int(m.value * 1000)}"
             for b in batches for m in b.measurements]
    text = "\n".join(lines) + "\n"
    pat = r"range,(?P<anchor>\d+),(?P<dist>\d+)"

    def rmse(**kw):
        hal = ul.TextHal(io.StringIO(text), pat, anchors=anchors,
                         unit="mm", anchor_prefix="A", **kw)
        est = np.array([f.position for f in ul.Pipeline(hal, level="Lv3").run()])
        n = min(len(est), len(truth))
        return ul.error_stats(truth[:n], est[:n])["rmse_3d"], hal.has_stream_time

    bad, has_bad = rmse()
    good, has_good = rmse(rate_hz=10.0)
    assert has_bad is False and has_good is True
    assert good < 0.4 < bad          # 時刻が無いと大きく崩れる


# --------------------------------------------------------------------- sniff


def test_sniff_guesses_pattern_and_reports_anchors():
    text = "boot ok\n" + "".join(f"range,{i},{3000 + i * 100}\n" for i in range(4))
    r = sniff(io.StringIO(text), unit="mm", anchor_prefix="A")
    assert r["matched"] == 4
    assert r["anchors"] == ["A0", "A1", "A2", "A3"]
    assert 2.9 < r["ranges"][0] < 3.1

def test_sniff_reports_nothing_matched_for_unparseable():
    r = sniff(io.StringIO("hello\nworld\n"))
    assert r["matched"] == 0
    assert r["anchors"] == []


# ------------------------------------------------- JSON Lines 前提を残さない

def test_sniff_flags_json_lines_instead_of_guessing_a_regex():
    """JSON Lines を渡されたら、正規表現をひねり出さずにそう教える."""
    text = "".join(
        '{"t":%.1f,"meas":[{"a":"A0","d":3.2},{"a":"A1","d":2.9}]}\n' % (i * 0.1)
        for i in range(10))
    r = sniff(io.StringIO(text))
    assert r["looks_like_json"] is True

    plain = "".join(f"A{i}: 3.2 m\n" for i in range(10))
    assert sniff(io.StringIO(plain))["looks_like_json"] is False


def test_replay_reads_both_formats(tmp_path):
    """CLI の replay が JSON Lines とテキストの両方を読めること."""
    from uwb_loc.cli import main

    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    sim = ul.SimulatedHal(anchors, ul.trajectory.circle([4, 3, 1.2], 2.0),
                          ul.ErrorModel(nlos_prob=0.05), rate_hz=10, seed=3)
    _, _, batches = sim.generate(6.0)

    apath = tmp_path / "anchors.json"
    apath.write_text(json.dumps({"anchors": [a.to_dict() for a in anchors]}),
                     encoding="utf-8")

    jsonl = tmp_path / "log.jsonl"
    with ul.JsonLinesWriter(str(jsonl), anchors) as w:
        for b in batches:
            w.write(b)
    assert main(["replay", str(jsonl), "--level", "Lv2"]) == 0

    text = tmp_path / "log.txt"
    text.write_text("".join(f"range,{m.anchor_id[1:]},{int(m.value * 1000)}\n"
                            for b in batches for m in b.measurements), encoding="utf-8")
    assert main(["replay", str(text), "--format", "text",
                 "--pattern", r"range,(?P<anchor>\d+),(?P<dist>\d+)",
                 "--unit", "mm", "--prefix", "A",
                 "--anchors", str(apath), "--level", "Lv3"]) == 0


def test_replay_text_with_wrong_pattern_fails_loudly(tmp_path):
    """解釈できないまま「成功」と言わないこと."""
    from uwb_loc.cli import main

    log = tmp_path / "log.txt"
    log.write_text("A0: 3.2 m\nA1: 2.9 m\n", encoding="utf-8")
    apath = tmp_path / "anchors.json"
    apath.write_text(json.dumps({"anchors": [
        ul.Anchor("A0", [0, 0, 2.4]).to_dict()]}), encoding="utf-8")
    assert main(["replay", str(log), "--format", "text",
                 "--pattern", r"NOPE(?P<anchor>x)(?P<dist>y)",
                 "--anchors", str(apath)]) == 2
