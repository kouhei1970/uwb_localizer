"""データ交換仕様 (型と JSON Lines) の検証."""

from __future__ import annotations

import io
import json
import time

import numpy as np
import pytest

import uwb_loc as ul
from uwb_loc.hal.jsonl import parse_line


def test_measurement_roundtrip():
    m = ul.Measurement(
        "A3", 4.25, kind=ul.MeasKind.RANGE, t=12.5, sigma=0.07, quality=0.8,
        raw={"rx_power": -81.2},
    )
    back = ul.Measurement.from_dict(m.to_dict())
    assert back.anchor_id == "A3"
    assert back.value == pytest.approx(4.25)
    assert back.kind is ul.MeasKind.RANGE
    assert back.t == pytest.approx(12.5)
    assert back.sigma == pytest.approx(0.07)
    assert back.quality == pytest.approx(0.8)
    assert back.raw["rx_power"] == pytest.approx(-81.2)


def test_measurement_accepts_d_as_distance_alias():
    """ファームウェアが距離を 'd' と書いてきても読める."""
    m = ul.Measurement.from_dict({"a": "A0", "d": 3.5}, t=1.0)
    assert m.value == pytest.approx(3.5)
    assert m.kind is ul.MeasKind.RANGE
    assert m.t == pytest.approx(1.0)


def test_measurement_requires_a_value():
    with pytest.raises(ValueError):
        ul.Measurement.from_dict({"a": "A0"})


def test_batch_roundtrip():
    b = ul.MeasurementBatch(
        t=3.0, tag_id="tagX",
        measurements=[ul.Measurement("A0", 1.0, t=3.0), ul.Measurement("A1", 2.0, t=3.0)],
    )
    back = ul.MeasurementBatch.from_dict(json.loads(json.dumps(b.to_dict())))
    assert back.t == pytest.approx(3.0)
    assert back.tag_id == "tagX"
    assert [m.anchor_id for m in back.measurements] == ["A0", "A1"]


def test_anchor_roundtrip():
    a = ul.Anchor("A9", [1.0, 2.0, 3.0], antenna_delay_m=0.15, sigma0=0.05)
    back = ul.Anchor.from_dict(a.to_dict())
    assert back.id == "A9"
    assert np.allclose(back.position, [1.0, 2.0, 3.0])
    assert back.antenna_delay_m == pytest.approx(0.15)


def test_parse_line_tolerates_garbage():
    """実機のシリアルには起動メッセージが混ざる. 例外にせず捨てること."""
    for junk in ["", "boot ok", "{broken json", "[1,2,3]", "\n"]:
        kind, payload = parse_line(junk)
        assert kind == "other"
        assert payload is None


def test_jsonl_hal_reads_anchors_and_measurements():
    anchors = ul.room_anchors((6.0, 4.0, 2.5))
    lines = [json.dumps({"v": 1, "type": "anchors",
                         "anchors": [a.to_dict() for a in anchors]})]
    lines.append("起動メッセージ (JSON ではない)")
    for i in range(3):
        lines.append(json.dumps({
            "v": 1, "type": "meas", "t": i * 0.1, "tag": "tag0",
            "meas": [{"a": a.id, "d": 2.0 + i, "q": 0.9} for a in anchors],
        }))

    hal = ul.JsonLinesHal(io.StringIO("\n".join(lines) + "\n"))
    hal.open()
    got = []
    for _ in range(20):
        got.extend(hal.poll(0.05))
        if len(got) >= 3:
            break
        time.sleep(0.02)
    hal.close()

    assert len(got) == 3
    assert len(hal.anchors) == len(anchors)
    assert len(got[0]) == len(anchors)
    assert got[0].measurements[0].quality == pytest.approx(0.9)


def test_jsonl_writer_then_replay(tmp_path):
    """記録 -> 読み直しで測位結果が変わらないこと."""
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    hal = ul.SimulatedHal(anchors, ul.trajectory.circle([4, 3, 1.2], 2.0), seed=7)
    _, truth, batches = hal.generate(5.0)

    path = tmp_path / "log.jsonl"
    with ul.JsonLinesWriter(str(path), anchors) as w:
        for b in batches:
            w.write(b)

    hal2 = ul.JsonLinesHal.from_path(str(path))
    hal2.open()
    replayed = []
    for _ in range(50):
        replayed.extend(hal2.poll(0.05))
        if len(replayed) >= len(batches):
            break
    hal2.close()

    assert len(replayed) == len(batches)
    a = ul.run_offline(batches, anchors, level="Lv2")
    b = ul.run_offline(replayed, hal2.anchors, level="Lv2")
    assert np.allclose([f.position for f in a], [f.position for f in b], atol=1e-6)


def test_fix_to_dict_is_json_safe():
    fix = ul.Fix.failed(t=1.0, n_total=3, level="Lv2")
    s = json.dumps(fix.to_dict())
    assert "NaN" not in s  # NaN は JSON として無効なので None に落とすこと


# ------------------------------------------------------- ドキュメントの最小形

def test_three_minimal_integrations_agree():
    """docs/BRINGUP.md に載せた 3 通りが同じ結果になること.

    「ID と距離が取れていれば、あとはこれだけ」という主張を固定する。
    """
    anchors = [ul.Anchor("A0", [0.2, 0.2, 2.4]), ul.Anchor("A1", [7.8, 0.2, 2.4]),
               ul.Anchor("A2", [7.8, 5.8, 0.3]), ul.Anchor("A3", [0.2, 5.8, 0.3]),
               ul.Anchor("A4", [0.2, 3.0, 0.3])]
    truth = np.array([3.0, 2.0, 1.2])
    readings = [(a.id, float(np.linalg.norm(truth - a.position))) for a in anchors]

    # A. HAL なし — Measurement を並べて update するだけ
    est = ul.make_estimator("Lv2", anchors)
    batch = ul.MeasurementBatch(
        t=0.0, measurements=[ul.Measurement(aid, d) for aid, d in readings])
    fix_a = est.update(batch)

    # B. JSON Lines — ファームが 1 行吐く形
    meas = ",".join(f'{{"a":"{aid}","d":{d:.6f}}}' for aid, d in readings)
    line = f'{{"t":0.0,"meas":[{meas}]}}\n'
    hal = ul.JsonLinesHal(io.StringIO(line), anchors=anchors)
    fix_b = ul.run_offline(_collect(hal), anchors, level="Lv2")[0]

    # C. HAL クラス — anchors と poll だけ実装
    class MyHal(ul.UwbHal):
        def __init__(self):
            self._done = False

        @property
        def anchors(self):
            return anchors

        @property
        def is_open(self):
            return not self._done

        def poll(self, timeout=0.0):
            self._done = True
            return [ul.MeasurementBatch(
                t=0.0, measurements=[ul.Measurement(aid, d) for aid, d in readings])]

    fix_c = list(ul.Pipeline(MyHal(), level="Lv2").run())[0]

    for fix in (fix_a, fix_b, fix_c):
        assert fix.ok
        assert np.linalg.norm(fix.position - truth) < 0.02
    assert np.allclose(fix_a.position, fix_b.position, atol=1e-4)
    assert np.allclose(fix_a.position, fix_c.position, atol=1e-4)


def _collect(hal, timeout=1.0):
    hal.open()
    out, t0 = [], time.monotonic()
    while time.monotonic() - t0 < timeout:
        out.extend(hal.poll(0.05))
        if not hal.is_open:
            break
    out.extend(hal.poll(0.05))
    hal.close()
    return out
