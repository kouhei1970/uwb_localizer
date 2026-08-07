"""キャリブレーション・幾何評価・シミュレータの検証."""

from __future__ import annotations

import numpy as np
import pytest

import uwb_loc as ul
from uwb_loc.calibration import (
    align_to_reference,
    apply_gauge,
    estimate_antenna_delays,
    fit_range_bias,
    self_survey,
)


# --------------------------------------------------------------------- 遅延


def test_fit_range_bias():
    true = np.linspace(1.0, 10.0, 20)
    measured = (true - 0.4) / 1.02
    scale, offset = fit_range_bias(measured, true)
    assert scale == pytest.approx(1.02, abs=1e-6)
    assert offset == pytest.approx(0.4, abs=1e-6)


def test_estimate_antenna_delays_recovers_relative_offsets():
    """アンカー遅延は平均 0 に正規化されるので, 相対値が一致すればよい."""
    rng = np.random.default_rng(0)
    truth = {"A0": 0.10, "A1": 0.25, "A2": -0.05, "A3": 0.30}
    tag = 0.20
    ids, meas, dist = [], [], []
    for aid, d0 in truth.items():
        for d in (1.0, 2.5, 5.0, 8.0):
            ids.append(aid)
            dist.append(d)
            meas.append(d + d0 + tag + rng.normal(0.0, 0.01))

    out = estimate_antenna_delays(ids, np.array(meas), np.array(dist))
    centered = {k: v - np.mean(list(truth.values())) for k, v in truth.items()}
    for aid, expected in centered.items():
        assert out[aid] == pytest.approx(expected, abs=0.02)
    assert out["__tag__"] == pytest.approx(tag + np.mean(list(truth.values())), abs=0.02)


# --------------------------------------------------------------------- 自己測量


def test_self_survey_recovers_layout():
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    pts = np.array([a.position for a in anchors])
    dmat = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)

    est = self_survey(dmat, [a.id for a in anchors], dim=3)
    got = np.array([a.position for a in est])
    want = apply_gauge(pts.copy(), dim=3)
    assert np.allclose(got, want, atol=1e-3)


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_self_survey_with_noise_and_missing_links(seed):
    """雑音と欠測リンクがあっても配置 (相互距離) を復元できること.

    自己測量の出す座標系は任意なので, 実世界座標との比較は
    :func:`align_to_reference` で既知点に合わせてから行う — 現場でも
    「何台かだけ実測して残りは自己測量」という使い方をするため.
    """
    rng = np.random.default_rng(seed)
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    pts = np.array([a.position for a in anchors])
    dmat = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    noisy = dmat + rng.normal(0.0, 0.05, dmat.shape)
    noisy = (noisy + noisy.T) / 2
    np.fill_diagonal(noisy, 0.0)
    noisy[0, 5] = noisy[5, 0] = np.nan  # 遮蔽で測れなかったリンク

    est = self_survey(noisy, [a.id for a in anchors], dim=3)

    # 形が合っているか (相互距離で見る).
    got = np.array([a.position for a in est])
    got_d = np.linalg.norm(got[:, None, :] - got[None, :, :], axis=2)
    assert np.sqrt(np.mean((got_d - dmat) ** 2)) < 0.10

    # 既知の 4 台に合わせれば実座標に載る.
    ref = {a.id: a.position for a in anchors[:4]}
    aligned = np.array([a.position for a in align_to_reference(est, ref)])
    assert np.sqrt(np.mean(np.sum((aligned - pts) ** 2, axis=1))) < 0.20


def test_align_to_reference_is_exact_for_rigid_motion():
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    theta = 0.7
    rot = np.array([[np.cos(theta), -np.sin(theta), 0.0],
                    [np.sin(theta), np.cos(theta), 0.0],
                    [0.0, 0.0, 1.0]])
    moved = [ul.Anchor(a.id, rot @ a.position + np.array([3.0, -1.0, 0.5])) for a in anchors]
    back = align_to_reference(moved, {a.id: a.position for a in anchors[:4]})
    assert np.allclose([a.position for a in back], [a.position for a in anchors], atol=1e-9)


def test_align_to_reference_needs_enough_points():
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    with pytest.raises(ValueError):
        align_to_reference(anchors, {anchors[0].id: anchors[0].position})


def test_self_survey_rejects_too_few_anchors():
    with pytest.raises(ValueError):
        self_survey(np.zeros((3, 3)), dim=3)


# --------------------------------------------------------------------- 幾何


def test_gdop_worse_when_anchors_clustered():
    good = ul.room_anchors((8.0, 6.0, 2.6))
    clustered = ul.make_anchors(np.array([
        [0.2, 0.2, 2.4], [0.6, 0.2, 2.4], [0.2, 0.6, 2.4], [0.4, 0.4, 0.3]]))
    assert ul.gdop_at([4, 3, 1.2], good) < ul.gdop_at([4, 3, 1.2], clustered)


def test_anchor_condition_detects_coplanar():
    flat = ul.room_anchors((8.0, 6.0, 2.6), n_low=0)
    assert ul.anchor_condition(flat)["coplanar"] is True
    assert ul.anchor_condition(ul.room_anchors((8.0, 6.0, 2.6)))["coplanar"] is False


def test_crlb_improves_with_better_ranging():
    anchors_a = ul.room_anchors((8.0, 6.0, 2.6))
    anchors_b = [ul.Anchor(a.id, a.position, sigma0=0.01) for a in anchors_a]
    assert ul.crlb_at([4, 3, 1.2], anchors_b) < ul.crlb_at([4, 3, 1.2], anchors_a)


def test_gdop_is_infinite_with_too_few_anchors():
    assert not np.isfinite(ul.gdop_at([4, 3, 1.2], ul.room_anchors((8.0, 6.0, 2.6))[:2]))


# --------------------------------------------------------------------- シミュレータ


def test_simulator_is_reproducible():
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    out = []
    for _ in range(2):
        hal = ul.SimulatedHal(anchors, ul.trajectory.figure8([4, 3, 1.2]), seed=42)
        _, _, batches = hal.generate(3.0)
        out.append([m.value for b in batches for m in b.measurements])
    assert out[0] == out[1]


def test_nlos_bias_is_one_sided():
    """NLOS は距離を伸ばす側にしか出ないこと (アルゴリズムの前提)."""
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    hal = ul.SimulatedHal(
        anchors, ul.trajectory.static([4, 3, 1.2]),
        ul.ErrorModel(sigma0=0.0, sigma_per_m=0.0, nlos_prob=0.5, loss_rate=0.0),
        seed=3,
    )
    _, _, batches = hal.generate(20.0)
    nlos = [m.value - m.raw["d_true"] for b in batches for m in b.measurements
            if m.raw["nlos_truth"]]
    assert len(nlos) > 50
    assert min(nlos) >= 0.0


def test_anchor_position_error_is_hidden_from_solver():
    """設置座標誤差は測位側に見えない (公称座標が返る)."""
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    hal = ul.SimulatedHal(
        anchors, ul.trajectory.static([4, 3, 1.2]),
        ul.ErrorModel(anchor_position_error=0.2), seed=1,
    )
    assert np.allclose([a.position for a in hal.anchors], [a.position for a in anchors])
    assert not np.allclose(hal.true_positions["A0"], anchors[0].position)


def test_pipeline_runs_end_to_end():
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    hal = ul.SimulatedHal(anchors, ul.trajectory.circle([4, 3, 1.2], 2.0), seed=0)
    pipe = ul.Pipeline(hal, level="Lv3", sigma_a=0.5)
    fixes = list(pipe.run(max_epochs=50))
    assert len(fixes) == 50
    assert sum(f.ok for f in fixes) >= 48
    assert pipe.positions().shape == (50, 3)


def test_metrics_handle_failed_fixes():
    truth = np.zeros((5, 3))
    est = np.zeros((5, 3))
    est[2] = np.nan
    stats = ul.error_stats(truth, est)
    assert stats["availability"] == pytest.approx(0.8)
    assert stats["rmse_3d"] == pytest.approx(0.0)
