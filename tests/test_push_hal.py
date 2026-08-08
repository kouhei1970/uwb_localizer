"""読みに行けない経路 (BLE 通知・MQTT・UDP …) の検証.

「インターフェースの前提はあるか」への答えを固定する — 前提は無い。
"""

from __future__ import annotations

import json
import socket
import threading
import time

import numpy as np
import pytest

import uwb_loc as ul


@pytest.fixture
def scenario():
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    sim = ul.SimulatedHal(anchors, ul.trajectory.figure8([4, 3, 1.2]),
                          ul.ErrorModel(nlos_prob=0.1), rate_hz=20, seed=4)
    _, truth, batches = sim.generate(6.0)
    return anchors, np.array(truth), batches


def _rmse(truth, est):
    e = np.array(est)
    n = min(len(e), len(truth))
    return ul.error_stats(truth[:n], e[:n])["rmse_3d"]


def test_callback_transport_needs_no_stream(scenario):
    """BLE 通知や MQTT のように「届いたら呼ばれる」経路でも測位できる.

    readline() できるものは何も要らない。押し込むだけ。
    """
    anchors, truth, batches = scenario
    hal = ul.PushHal(anchors)

    def feed():
        for b in batches:
            for m in b.measurements:          # 通知が 1 本ずつ届く想定
                hal.push(m.anchor_id, m.value, t=m.t, quality=m.quality)
        hal.close()

    threading.Thread(target=feed, daemon=True).start()
    est = [f.position for f in ul.Pipeline(hal, level="Lv3").run()]

    assert len(est) > 50
    assert hal.n_pushed > 200
    assert _rmse(truth, est) < 0.3


def test_udp_datagrams_work(scenario):
    """readline() できないデータグラムでも、受信ループから押し込めばよい."""
    anchors, truth, batches = scenario
    hal = ul.PushHal(anchors)

    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("127.0.0.1", 0))
    srv.settimeout(0.4)
    port = srv.getsockname()[1]

    def rx():
        while True:
            try:
                data, _ = srv.recvfrom(4096)
            except socket.timeout:
                break
            d = json.loads(data.decode())
            hal.push(d["a"], d["d"], t=d["t"])
        hal.close()

    threading.Thread(target=rx, daemon=True).start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for b in batches:
        for m in b.measurements:
            tx.sendto(json.dumps({"a": m.anchor_id, "d": m.value, "t": m.t}).encode(),
                      ("127.0.0.1", port))
        time.sleep(0.001)

    est = [f.position for f in ul.Pipeline(hal, level="Lv3").run()]
    assert len(est) > 50
    assert _rmse(truth, est) < 0.3


def test_push_groups_epochs_on_anchor_repeat(scenario):
    """1 本ずつ押し込んでも、Lv0-Lv2 が解けるエポックに束ねてくれる."""
    anchors, _, batches = scenario
    hal = ul.PushHal(anchors)
    for m in batches[0].measurements:
        hal.push(m.anchor_id, m.value, t=m.t)
    for m in batches[1].measurements:
        hal.push(m.anchor_id, m.value, t=m.t)
    hal.close()

    got = hal.poll(0.2)
    assert len(got) == 2
    assert all(len(b) >= 4 for b in got)


def test_one_measurement_at_a_time_matches_grouping(scenario):
    """Lv3 は 1 本ずつ届いても、束ねた場合と同じ精度が出ること.

    密結合フィルタの売りそのもの。1 本ごとに predict→update しても
    エポック単位で処理したのと変わらない、が成り立たなければ
    「届いた瞬間に処理してよい」という設計の前提が崩れる。

    比較は**時刻で対応付ける** — 1 本ずつだと推定の個数が測距の本数分
    出るので、添字で真値と突き合わせると別の時刻同士を比べてしまう。
    """
    anchors, truth, batches = scenario

    grouped = ul.make_estimator("Lv3", anchors)
    ge = np.array([grouped.update(b).position for b in batches])

    hal = ul.PushHal(anchors, group=False)
    for b in batches:
        for m in b.measurements:
            hal.push(m.anchor_id, m.value, t=m.t)
    hal.close()
    fixes = list(ul.Pipeline(hal, level="Lv3").run())
    assert len(fixes) > 200                    # 1 本 = 1 更新

    # 各エポックの最後の推定を拾う
    last = {}
    for f in fixes:
        if f.ok:
            last[round(f.t, 4)] = f.position
    se = np.array([last.get(round(b.t, 4), np.full(3, np.nan)) for b in batches])

    n = min(len(se), len(truth))
    r_single = ul.error_stats(truth[:n], se[:n])["rmse_3d"]
    r_group = ul.error_stats(truth[:n], ge[:n])["rmse_3d"]
    assert r_single < 0.3
    assert r_single < r_group * 1.2            # まとめた場合と同程度


def test_filter_state_is_identical_either_way(scenario):
    """束ねても 1 本ずつでも、フィルタの状態が厳密に一致すること."""
    anchors, _, batches = scenario
    a = ul.make_estimator("Lv3", anchors)
    b = ul.make_estimator("Lv3", anchors)
    for i in range(3):                          # 立ち上げを揃える
        a.update(batches[i])
        b.update(batches[i])

    for bat in batches[3:]:
        a.update(bat)
        for m in bat.measurements:
            b.update(ul.MeasurementBatch(t=bat.t, measurements=[m]))
        assert np.allclose(a.x, b.x, atol=1e-9)


def test_bootstrap_waits_for_enough_ranges(scenario):
    """立ち上げは最小本数ちょうどでは行わない (初期値が悪く過渡が尾を引く)."""
    anchors, _, batches = scenario
    est = ul.make_estimator("Lv3", anchors)
    seen = 0
    for m in batches[0].measurements:
        seen += 1
        fix = est.update(ul.MeasurementBatch(t=m.t, measurements=[m]))
        if fix.ok:
            break
    assert seen >= 5              # 3D なら dim+2 = 5 本以上待つ


def test_push_batch_passes_through(scenario):
    """組み立て済みのエポックはそのまま流せる."""
    anchors, _, batches = scenario
    hal = ul.PushHal(anchors)
    hal.push_batch(batches[0])
    hal.close()
    got = hal.poll(0.2)
    assert len(got) == 1
    assert len(got[0]) == len(batches[0])


def test_push_many_takes_id_distance_pairs(scenario):
    anchors, _, batches = scenario
    hal = ul.PushHal(anchors)
    hal.push_many([(m.anchor_id, m.value) for m in batches[0].measurements], t=0.0)
    hal.close()
    got = hal.poll(0.2)
    assert sum(len(b) for b in got) == len(batches[0])
