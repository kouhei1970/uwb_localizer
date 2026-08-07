"""測位ソルバの検証."""

from __future__ import annotations

import numpy as np
import pytest

import uwb_loc as ul
from uwb_loc.solvers import SolveConfig, beck_gtrs, chan_tdoa, lls_trilateration, make_estimator


@pytest.fixture
def anchors() -> list[ul.Anchor]:
    return ul.room_anchors((8.0, 6.0, 2.6))


def _batch(anchors, p, *, noise=0.0, seed=0, t=0.0):
    rng = np.random.default_rng(seed)
    ms = []
    for a in anchors:
        d = float(np.linalg.norm(np.asarray(p) - a.position))
        ms.append(ul.Measurement(a.id, d + rng.normal(0.0, noise), t=t, sigma=max(noise, 0.01)))
    return ul.MeasurementBatch(t=t, measurements=ms)


# --------------------------------------------------------------------- 閉形式


def test_closed_form_exact_without_noise(anchors):
    """無雑音なら閉形式は真値をそのまま返すはず."""
    p = np.array([3.1, 2.2, 1.4])
    pos = np.array([a.position for a in anchors])
    r = np.linalg.norm(pos - p, axis=1)

    assert np.allclose(beck_gtrs(pos, r), p, atol=1e-6)
    assert np.allclose(lls_trilateration(pos, r), p, atol=1e-6)


def test_beck_is_global_and_beats_lls_with_noise(anchors):
    """Beck は初期値に依存せず, LLS より偏りが小さい."""
    rng = np.random.default_rng(0)
    pos = np.array([a.position for a in anchors])
    err_beck, err_lls = [], []
    for _ in range(200):
        p = np.array([rng.uniform(1, 7), rng.uniform(1, 5), rng.uniform(0.5, 2.0)])
        r = np.linalg.norm(pos - p, axis=1) + rng.normal(0.0, 0.1, len(pos))
        err_beck.append(np.linalg.norm(beck_gtrs(pos, r) - p))
        err_lls.append(np.linalg.norm(lls_trilateration(pos, r) - p))
    assert np.mean(err_beck) < np.mean(err_lls)
    assert np.mean(err_beck) < 0.3


def test_beck_needs_enough_anchors(anchors):
    pos = np.array([a.position for a in anchors])[:3]
    r = np.linalg.norm(pos - np.array([3.0, 2.0, 1.0]), axis=1)
    assert beck_gtrs(pos, r) is None  # 3D には 4 台必要


def test_chan_tdoa_exact_without_noise(anchors):
    p = np.array([2.5, 4.0, 1.1])
    pos = np.array([a.position for a in anchors])
    d = np.linalg.norm(pos - p, axis=1)
    est = chan_tdoa(pos, d - d[0], 0)
    assert np.allclose(est, p, atol=1e-6)


# --------------------------------------------------------------------- 推定器


@pytest.mark.parametrize("level", ["Lv0", "Lv1", "Lv2", "Lv3"])
def test_estimator_recovers_truth_without_noise(anchors, level):
    p = np.array([5.0, 1.7, 0.9])
    est = make_estimator(level, anchors)
    fix = est.update(_batch(anchors, p))
    if level == "Lv3":  # 初回はブートストラップなので 2 回流す
        fix = est.update(_batch(anchors, p, t=0.1))
    assert fix.ok
    assert np.linalg.norm(fix.position - p) < 0.05
    assert fix.n_used >= 4
    assert np.isfinite(fix.gdop)


def test_fix_reports_covariance_and_gdop(anchors):
    fix = make_estimator("Lv1", anchors).update(_batch(anchors, [4, 3, 1.2], noise=0.05, seed=1))
    assert fix.covariance.shape == (3, 3)
    assert np.all(np.isfinite(fix.covariance))
    assert fix.sigma > 0.0
    assert 1.0 < fix.gdop < 10.0


def test_insufficient_anchors_fails_cleanly(anchors):
    batch = _batch(anchors, [4, 3, 1.2])
    batch.measurements = batch.measurements[:2]
    fix = make_estimator("Lv2", anchors).update(batch)
    assert not fix.ok
    assert np.all(np.isnan(fix.position))


def test_unknown_anchor_is_ignored(anchors):
    batch = _batch(anchors, [4, 3, 1.2])
    batch.measurements.append(ul.Measurement("UNKNOWN", 3.0))
    fix = make_estimator("Lv1", anchors).update(batch)
    assert fix.ok
    assert fix.n_used == len(anchors)


# --------------------------------------------------------------------- ロバスト性


def test_robust_level_rejects_nlos_outlier(anchors):
    """1 本だけ 3 m 伸ばしたとき, Lv2 は Lv1 より影響を受けない."""
    p = np.array([4.0, 3.0, 1.2])
    batch = _batch(anchors, p, noise=0.03, seed=2)
    batch.measurements[0].value += 3.0  # NLOS を模した正バイアス

    e1 = np.linalg.norm(make_estimator("Lv1", anchors).update(batch).position - p)
    fix2 = make_estimator("Lv2", anchors).update(batch)
    e2 = np.linalg.norm(fix2.position - p)
    assert e2 < e1
    assert e2 < 0.15
    assert anchors[0].id in fix2.excluded


def test_physical_gate_drops_impossible_range(anchors):
    p = np.array([4.0, 3.0, 1.2])
    batch = _batch(anchors, p)
    batch.measurements[1].value = -5.0
    fix = make_estimator("Lv2", anchors).update(batch)
    assert fix.ok
    assert anchors[1].id in fix.excluded


# --------------------------------------------------------------------- 次元拘束


def test_2d_mode_pins_height(anchors):
    est = make_estimator("Lv2", anchors, SolveConfig(dim=2, z_fixed=1.0))
    fix = est.update(_batch(anchors, [4.0, 3.0, 1.0], noise=0.05, seed=3))
    assert fix.position[2] == pytest.approx(1.0)
    assert np.linalg.norm(fix.position[:2] - np.array([4.0, 3.0])) < 0.2


def test_2d_beats_3d_on_coplanar_anchors():
    """同一平面配置では, 高さを固定した方が水平精度まで良くなる."""
    flat = ul.room_anchors((8.0, 6.0, 2.6), n_low=0)
    hal = ul.SimulatedHal(
        flat, ul.trajectory.circle([4, 3, 1.0], 2.0), ul.ErrorModel(nlos_prob=0.0),
        rate_hz=10, seed=5
    )
    _, truth, batches = hal.generate(15.0)
    truth = np.array(truth)

    e3 = ul.error_stats(truth, np.array(
        [f.position for f in ul.run_offline(batches, flat, level="Lv2")]))
    e2 = ul.error_stats(truth, np.array([
        f.position for f in ul.run_offline(
            batches, flat, level="Lv2", config=SolveConfig(dim=2, z_fixed=1.0))]))
    assert e2["rmse_2d"] < e3["rmse_2d"]


# --------------------------------------------------------------------- EKF


def test_ekf_tracks_with_fewer_than_three_anchors(anchors):
    """アンカーが 2 本しか見えないエポックでも EKF は更新を続けられる.

    これがスナップショット測位に対する密結合フィルタの一番の利点.
    """
    est = make_estimator("Lv3", anchors, sigma_a=0.5)
    p = np.array([4.0, 3.0, 1.2])
    for i in range(20):  # まず十分な観測で立ち上げる
        est.update(_batch(anchors, p, noise=0.03, seed=i, t=i * 0.1))

    for i in range(10):  # 以降は 2 本だけ
        b = _batch(anchors, p, noise=0.03, seed=100 + i, t=2.0 + i * 0.1)
        b.measurements = b.measurements[:2]
        fix = est.update(b)
    assert fix.ok
    assert fix.n_used == 2
    assert np.linalg.norm(fix.position - p) < 0.3


def test_ekf_recovers_after_teleport(anchors):
    """ゲートが自分の誤りを守り続けない (棺桶化しない) こと."""
    est = make_estimator("Lv3", anchors, sigma_a=0.5, gate=3.0, max_rejects=5)
    for i in range(20):
        est.update(_batch(anchors, [1.5, 1.5, 1.0], noise=0.03, seed=i, t=i * 0.1))

    far = np.array([6.5, 4.5, 1.5])
    for i in range(40):
        fix = est.update(_batch(anchors, far, noise=0.03, seed=200 + i, t=2.0 + i * 0.1))
    assert fix.ok
    assert np.linalg.norm(fix.position - far) < 0.3


def test_ekf_ignores_out_of_sequence(anchors):
    est = make_estimator("Lv3", anchors)
    for i in range(10):
        est.update(_batch(anchors, [4, 3, 1.2], noise=0.03, seed=i, t=i * 0.1))
    before = est.x.copy()
    est.update(_batch(anchors, [1, 1, 1], t=0.2))  # 巻き戻った時刻
    assert np.allclose(est.x, before)


def test_levels_are_monotonically_better_under_nlos():
    """同じ観測列に対して Lv0 < Lv1 <= Lv2 < Lv3 の順で誤差が小さくなる."""
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    errs = {}
    for seed in range(3):
        hal = ul.SimulatedHal(
            anchors, ul.trajectory.figure8([4, 3, 1.2]), ul.ErrorModel(nlos_prob=0.2),
            rate_hz=10, seed=seed
        )
        _, truth, batches = hal.generate(20.0)
        truth = np.array(truth)
        for lv in ("Lv0", "Lv1", "Lv2", "Lv3"):
            s = ul.error_stats(truth, np.array(
                [f.position for f in ul.run_offline(batches, anchors, level=lv)]))
            errs.setdefault(lv, []).append(s["rmse_3d"])
    m = {k: float(np.mean(v)) for k, v in errs.items()}
    assert m["Lv1"] < m["Lv0"]
    assert m["Lv2"] <= m["Lv1"]
    assert m["Lv3"] < m["Lv2"]
