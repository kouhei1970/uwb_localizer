"""UART から ID と距離が取れている人向け — つなぎ方 3 通りの最小形.

    python examples/03_minimal_integration.py

「測距値は既に手に入っている」状態からライブラリに渡すには、
実質 3 通りしかない。どれも短い。

A. HAL を使わない  — 自分でパース済みなら、これがいちばん短い (5 行)
B. JSON Lines      — ファームウェアが 1 行 print するだけ。Python は書かない
C. HAL クラス      — ストリームを自分で管理したいとき (20 行)

どれを選んでも測位側のコードは同じ。
"""

from __future__ import annotations

import io
import time

import numpy as np

import uwb_loc as ul

# アンカー座標だけは、どの方法でも必要 (巻き尺 or self_survey)
ANCHORS = [
    ul.Anchor("A0", [0.2, 0.2, 2.4]),
    ul.Anchor("A1", [7.8, 0.2, 2.4]),
    ul.Anchor("A2", [7.8, 5.8, 0.3]),
    ul.Anchor("A3", [0.2, 5.8, 0.3]),
    ul.Anchor("A4", [0.2, 3.0, 0.3]),
]

# このスクリプトを実機なしで動かすための「UART から読めたことにする」データ。
# 実際には serial.readline() の結果を自分でパースして得る想定。
TRUE_POS = np.array([3.0, 2.0, 1.2])


def fake_uart_readings(t: float) -> list[tuple[str, float]]:
    """(アンカー ID, 距離 [m]) のリスト — ここが読者の環境では実機になる."""
    rng = np.random.default_rng(int(t * 1000) % 10000)
    return [
        (a.id, float(np.linalg.norm(TRUE_POS - a.position) + rng.normal(0, 0.06)))
        for a in ANCHORS
    ]


# --------------------------------------------------------------------------- A


def method_a_no_hal() -> None:
    """A. HAL を使わない — 自分でパース済みならこれが最短.

    ``Measurement`` を並べて ``MeasurementBatch`` にして ``update()`` に渡すだけ。
    ストリームの管理 (スレッド、バッファ) を自分でやるなら、HAL は要らない。
    """
    print("--- A. HAL なし (自分でパース済み) ---")

    est = ul.make_estimator("Lv2", ANCHORS)          # ← 1

    for k in range(3):
        t = k * 0.1
        readings = fake_uart_readings(t)             # ← 自分の UART 読み出し

        batch = ul.MeasurementBatch(                 # ← 2
            t=t,
            measurements=[ul.Measurement(aid, dist, t=t) for aid, dist in readings],
        )
        fix = est.update(batch)                      # ← 3

        if fix.ok:
            print(f"  t={t:.1f}  {fix.position.round(3)}  "
                  f"±{fix.sigma:.3f} m  ({fix.n_used}/{fix.n_total} 本)")

    print(f"  真値 {TRUE_POS}  → 実質 3 行\n")


# --------------------------------------------------------------------------- B


def method_b_json_lines() -> None:
    """B. JSON Lines — ファームウェアが 1 行 print するだけ.

    マイコン側 (C):

        printf("{\\"t\\":%.3f,\\"meas\\":["
               "{\\"a\\":\\"A0\\",\\"d\\":%.3f},"
               "{\\"a\\":\\"A1\\",\\"d\\":%.3f}]}\\n", t, d0, d1);

    Python 側はストリームを渡すだけで、パースのコードは 1 行も書かない。
    """
    print("--- B. JSON Lines (ファームが print) ---")

    # 実機では JsonLinesHal.from_serial("/dev/ttyUSB0", 115200) になる。
    # ここではファームの出力を文字列で作って代用する。
    lines = []
    for k in range(3):
        t = k * 0.1
        meas = ",".join(f'{{"a":"{aid}","d":{d:.3f}}}'
                        for aid, d in fake_uart_readings(t))
        lines.append(f'{{"t":{t:.3f},"meas":[{meas}]}}')
    print(f"  ファームの出力: {lines[0][:64]}...")

    hal = ul.JsonLinesHal(io.StringIO("\n".join(lines) + "\n"), anchors=ANCHORS)
    for fix in ul.Pipeline(hal, level="Lv2").run():
        if fix.ok:
            print(f"  t={fix.t:.1f}  {fix.position.round(3)}  ±{fix.sigma:.3f} m")
    print("  → Python 側はストリームを渡すだけ\n")


# --------------------------------------------------------------------------- C


class MyUartHal(ul.UwbHal):
    """C. HAL クラス — 実装するのは anchors と poll の 2 つだけ."""

    name = "my-uart"

    def __init__(self, anchors: list[ul.Anchor]) -> None:
        self._anchors = anchors
        self._t = 0.0
        # 実機では: self._ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)

    @property
    def anchors(self) -> list[ul.Anchor]:
        return self._anchors

    def poll(self, timeout: float = 0.0) -> list[ul.MeasurementBatch]:
        """溜まっている観測を返す (ブロックしないこと)."""
        # 実機ではここで self._ser.readline() を読んでパースする。
        # 時刻は「測距が成立した時刻」を入れる (ホスト到着時刻ではない)。
        readings = fake_uart_readings(self._t)
        batch = ul.MeasurementBatch(
            t=self._t,
            measurements=[ul.Measurement(aid, d, t=self._t) for aid, d in readings],
        )
        self._t += 0.1
        return [batch]

    @property
    def is_open(self) -> bool:
        return self._t < 0.3          # 実機では通信が生きているか


def method_c_hal_class() -> None:
    print("--- C. HAL クラス (20 行) ---")
    for fix in ul.Pipeline(MyUartHal(ANCHORS), level="Lv2").run():
        if fix.ok:
            print(f"  t={fix.t:.1f}  {fix.position.round(3)}  ±{fix.sigma:.3f} m")
    print("  → 実装は anchors と poll の 2 つだけ\n")


# ---------------------------------------------------------------------------


def main() -> None:
    print(f"アンカー {len(ANCHORS)} 台 / タグの真の位置 {TRUE_POS}\n")
    method_a_no_hal()
    method_b_json_lines()
    method_c_hal_class()

    print("どの方法でも測位側のコードは同じ。違うのは観測の入り口だけ。")
    print("時刻を出せないなら Lv2 を使う (Lv0-Lv2 は時刻を使わない)。")


if __name__ == "__main__":
    main()
