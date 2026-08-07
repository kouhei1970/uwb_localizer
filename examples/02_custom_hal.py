"""自前の UWB 用 HAL を書く例.

チップごとに実装するのは ``anchors`` と ``poll`` の 2 つだけ.
ここではファームウェアがカンマ区切りで距離を吐いてくる架空のチップを想定し,
その行を ``Measurement`` に変換している.

    python examples/uwb_02_custom_hal.py
"""

from __future__ import annotations

import numpy as np

import uwb_loc as ul


class FakeChipHal(ul.UwbHal):
    """架空チップの HAL.

    実機では ``_read_lines`` がシリアル/SPI からの読み出しになる.
    ここでは動かして見せるためにシミュレータから作った行を返している.
    """

    name = "fakechip"

    def __init__(self, anchors: list[ul.Anchor], lines: list[str]) -> None:
        self._anchors = anchors
        self._lines = list(lines)

    @property
    def anchors(self) -> list[ul.Anchor]:
        return self._anchors

    @property
    def is_open(self) -> bool:
        return bool(self._lines)

    def poll(self, timeout: float = 0.0) -> list[ul.MeasurementBatch]:
        """溜まっている行を観測に変換して返す (ブロックしない)."""
        if not self._lines:
            return []
        line = self._lines.pop(0)

        # 例: "12.345,A0:3.214:-79.4:-81.2,A1:2.887:-86.0:-95.9"
        parts = line.strip().split(",")
        t = float(parts[0])
        ms = []
        for token in parts[1:]:
            aid, dist, rx, fp = token.split(":")
            # 受信電力と first path 電力の差から見通し尤度を作る.
            # チップ固有の指標は必ずここで 0-1 に正規化して渡す.
            delta_db = float(rx) - float(fp)
            quality = float(np.clip(1.0 - delta_db / 10.0, 0.0, 1.0))
            ms.append(
                ul.Measurement(
                    anchor_id=aid,
                    value=float(dist),          # アンテナ遅延は Anchor 側で引く
                    t=t,
                    quality=quality,
                    raw={"rx_power": float(rx), "fp_power": float(fp)},
                )
            )
        return [ul.MeasurementBatch(t=t, measurements=ms)]


def _fake_firmware_output(anchors: list[ul.Anchor], n: int = 300) -> list[str]:
    """シミュレータの観測を「チップの生出力」形式の文字列にする."""
    hal = ul.SimulatedHal(anchors, ul.trajectory.circle([4.0, 3.0, 1.2], 2.0), seed=3)
    _, _, batches = hal.generate(n / 10.0)
    lines = []
    for b in batches:
        tokens = [f"{b.t:.3f}"]
        for m in b.measurements:
            # NLOS ほど first path が弱くなる, という体で電力値を作る
            rx = -79.0
            fp = rx - (12.0 if m.raw["nlos_truth"] else 2.0)
            tokens.append(f"{m.anchor_id}:{m.value:.3f}:{rx:.1f}:{fp:.1f}")
        lines.append(",".join(tokens))
    return lines


def main() -> None:
    anchors = ul.room_anchors((8.0, 6.0, 2.6))
    hal = FakeChipHal(anchors, _fake_firmware_output(anchors))

    # ここから先は実機でもシミュレータでも同じコード.
    pipe = ul.Pipeline(hal, level="Lv3", sigma_a=1.0)
    fixes = list(pipe.run())

    ok = [f for f in fixes if f.ok]
    print(f"{len(fixes)} エポック中 {len(ok)} 回測位成功")
    print(f"推定 σ の中央値 {np.median([f.sigma for f in ok]):.3f} m")
    print(f"1 エポックあたり使用観測数 {np.mean([f.n_used for f in ok]):.1f} 本")

    last = fixes[-1]
    print(f"\n最後の推定: t={last.t:.2f}  p={np.round(last.position, 3)}  "
          f"±{last.sigma:.3f} m  GDOP={last.gdop:.2f}")


if __name__ == "__main__":
    main()
