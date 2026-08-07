"""UWB 測位のいちばん短い例 — ハードなしで Lv0-Lv3 を比べる.

    python examples/uwb_01_quickstart.py
"""

from __future__ import annotations

import numpy as np

import uwb_loc as ul


def main() -> None:
    # 部屋の四隅に上下 2 段でアンカーを置く. 3 次元で解くには
    # アンカーが同一平面に並んでいないことが本質的に効く.
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    cond = ul.anchor_condition(anchors)
    print(f"アンカー {int(cond['n'])} 台  同一平面={cond['coplanar']}")

    # 実機の HAL と同じインターフェイス. 差し替えても下は変わらない.
    hal = ul.SimulatedHal(
        anchors,
        ul.trajectory.figure8([4.0, 3.0, 1.2]),
        ul.ErrorModel(sigma0=0.08, nlos_prob=0.2, nlos_bias_mean=0.8, loss_rate=0.03),
        rate_hz=10.0,
        seed=0,
    )
    _, truth, batches = hal.generate(40.0)
    truth = np.array(truth)

    print(f"{len(batches)} エポック / 1 エポックあたり平均 "
          f"{np.mean([len(b) for b in batches]):.1f} 本\n")

    # 同じ観測列を各レベルに通す — アルゴリズムの差だけが出る.
    print(f"{'Lv':4s} {'測位率':>7s} {'RMSE3D':>8s} {'CEP50':>7s} {'CEP95':>7s} {'最大':>7s}")
    for level in ("Lv0", "Lv1", "Lv2", "Lv3"):
        fixes = ul.run_offline(batches, anchors, level=level)
        s = ul.error_stats(truth, np.array([f.position for f in fixes]))
        print(f"{level:4s} {s['availability']*100:6.1f}% {s['rmse_3d']:8.3f} "
              f"{s['cep50']:7.3f} {s['cep95']:7.3f} {s['max_3d']:7.3f}")

    crlb = ul.crlb_at(truth.mean(axis=0), anchors)
    print(f"\n理論下限 (CRLB, 軌道中心) {crlb:.3f} m")
    print("  1 エポック分の観測だけで解く限りこれより良くはならない。")
    print("  Lv3 は時間方向の情報も使うので下回りうる。")


if __name__ == "__main__":
    main()
