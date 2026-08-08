"""REYAX RYUW122 用 HAL の検証.

実機が無いので、仕様書 (docs/datasheets/RYUW122_AT_Command_Guide.pdf) の
応答どおりに振る舞う模擬モジュールを相手にする。
"""

from __future__ import annotations

import queue
import threading

import numpy as np
import pytest

import uwb_loc as ul
from uwb_loc.hal.ryuw122 import Ryuw122Config, Ryuw122Hal, parse_anchor_rcv


class FakeModule:
    """RYUW122 の応答を仕様書どおりに真似るシリアルもどき.

    Parameters
    ----------
    distances_cm:
        TAG アドレス -> 距離 [cm]. 載っていないアドレスは無応答 (圏外)。
    rssi:
        RSSI を +ANCHOR_RCV に載せるか (AT+RSSI=1 のときの挙動)。
    """

    def __init__(self, distances_cm: dict[str, float], *, rssi: bool = True,
                 tag_payload: str = "HELLO") -> None:
        self.distances_cm = dict(distances_cm)
        self.rssi = rssi
        self.tag_payload = tag_payload
        self._out: queue.Queue[str] = queue.Queue()
        self.received: list[str] = []
        self.closed = False

    # --- ストリームとしての振る舞い
    def write(self, data: str) -> None:
        for line in data.replace("\r", "").split("\n"):
            line = line.strip()
            if line:
                self._handle(line)

    def flush(self) -> None:
        pass

    def readline(self) -> str:
        try:
            return self._out.get(timeout=0.05)
        except queue.Empty:
            return ""

    def close(self) -> None:
        self.closed = True

    # --- AT の解釈
    def _handle(self, cmd: str) -> None:
        self.received.append(cmd)
        if not cmd.startswith("AT"):
            self._out.put("+ERR=2\n")
            return
        if cmd.startswith("AT+ANCHOR_SEND="):
            body = cmd.split("=", 1)[1]
            addr, _n, _data = body.split(",", 2)
            self._out.put("+OK\n")
            d = self.distances_cm.get(addr)
            if d is None:
                return                      # 圏外 = 無応答
            line = (f"+ANCHOR_RCV={addr},{len(self.tag_payload)},"
                    f"{self.tag_payload},{d:.0f} cm")
            if self.rssi:
                line += ",-72"
            self._out.put(line + "\n")
            return
        if cmd.startswith("AT+MODE=") or cmd.startswith("AT+"):
            self._out.put("+OK\n")
            return
        self._out.put("+OK\n")


# --------------------------------------------------------------------- 解析


@pytest.mark.parametrize("line,addr,dist,rssi", [
    # 仕様書 16 節の例そのもの
    ("+ANCHOR_RCV=DAVID123,5,HELLO,40 cm", "DAVID123", 0.40, None),
    ("+ANCHOR_RCV=DAVID123,5,HELLO,40 cm,-72", "DAVID123", 0.40, -72.0),
    ("+ANCHOR_RCV=REYAX003,4,TEST,1234 cm", "REYAX003", 12.34, None),
    ("+ANCHOR_RCV=REYAX003,0,,250 cm", "REYAX003", 2.50, None),
])
def test_parse_anchor_rcv(line, addr, dist, rssi):
    got = parse_anchor_rcv(line)
    assert got is not None
    assert got[0] == addr
    assert got[1] == pytest.approx(dist)
    assert (got[2] is None) if rssi is None else got[2] == pytest.approx(rssi)


def test_parse_handles_comma_inside_payload():
    """データ本体にカンマが入っていても、長さで切り出すので壊れない."""
    got = parse_anchor_rcv("+ANCHOR_RCV=DAVID123,5,A,B,C,350 cm")
    assert got is not None
    assert got[0] == "DAVID123"
    assert got[1] == pytest.approx(3.50)


def test_parse_rejects_lines_without_distance():
    assert parse_anchor_rcv("+TAG_RCV=4,TEST") is None
    assert parse_anchor_rcv("+OK") is None
    assert parse_anchor_rcv("+ANCHOR_RCV=DAVID123,4,TEST") is None


def test_distance_is_converted_from_cm():
    """仕様書 16 節: 距離は cm。ライブラリは m で扱う."""
    fake = FakeModule({"TAG00001": 1234})
    hal = Ryuw122Hal(fake, ["TAG00001"])
    m = hal.range_once("TAG00001")
    assert m is not None
    assert m.value == pytest.approx(12.34)


# --------------------------------------------------------------------- 設定


def test_config_validates_the_spec_constraints():
    with pytest.raises(ValueError, match="8 バイト"):
        Ryuw122Config(network_id="SHORT")
    with pytest.raises(ValueError, match="32 文字"):
        Ryuw122Config(password="AABB")
    with pytest.raises(ValueError, match="channel"):
        Ryuw122Config(channel=7)
    with pytest.raises(ValueError, match="bandwidth"):
        Ryuw122Config(bandwidth=2)
    with pytest.raises(ValueError, match="power"):
        Ryuw122Config(power=9)
    with pytest.raises(ValueError, match="calibration"):
        Ryuw122Config(calibration_cm=500)


def test_setup_sends_mode_first_and_all_settings():
    """仕様書 2 節: 先に AT+MODE でモードを決めること."""
    fake = FakeModule({})
    cfg = Ryuw122Config(network_id="REYAX123", address="ANCHOR01",
                        password="F" * 32, channel=9, bandwidth=1,
                        power=5, calibration_cm=-11)
    hal = Ryuw122Hal(fake, ["TAG00001"], config=cfg)
    assert hal.setup() is True

    sent = fake.received
    assert sent[0] == "AT+MODE=1"                    # ANCHOR、かつ最初
    assert "AT+NETWORKID=REYAX123" in sent
    assert "AT+ADDRESS=ANCHOR01" in sent
    assert f"AT+CPIN={'F' * 32}" in sent
    assert "AT+CHANNEL=9" in sent
    assert "AT+BANDWIDTH=1" in sent
    assert "AT+CRFOP=5" in sent
    assert "AT+CAL=-11" in sent
    assert "AT+RSSI=1" in sent


def test_anchor_send_uses_payload_length():
    """仕様書 13 節: AT+ANCHOR_SEND=<addr>,<len>,<data>."""
    fake = FakeModule({"TAG00001": 100})
    hal = Ryuw122Hal(fake, ["TAG00001"], payload="RNGE")
    hal.range_once("TAG00001")
    assert "AT+ANCHOR_SEND=TAG00001,4,RNGE" in fake.received


def test_out_of_range_tag_times_out_without_blocking_forever():
    fake = FakeModule({})                    # どの TAG も無応答
    hal = Ryuw122Hal(fake, ["TAG00001"], timeout=0.15)
    assert hal.range_once("TAG00001") is None
    assert hal.n_timeout == 1


# --------------------------------------------------------------------- 測位


def test_end_to_end_positioning_with_fake_module():
    """模擬モジュールから位置が出るところまで通す."""
    truth = np.array([3.0, 2.0, 1.2])
    anchors = [
        ul.Anchor("TAG00001", [0.2, 0.2, 2.4]),
        ul.Anchor("TAG00002", [7.8, 0.2, 2.4]),
        ul.Anchor("TAG00003", [7.8, 5.8, 0.3]),
        ul.Anchor("TAG00004", [0.2, 5.8, 0.3]),
        ul.Anchor("TAG00005", [0.2, 3.0, 0.3]),
    ]
    dist_cm = {a.id: float(np.linalg.norm(truth - a.position)) * 100.0
               for a in anchors}

    fake = FakeModule(dist_cm)
    hal = Ryuw122Hal(fake, [a.id for a in anchors], anchors, timeout=0.3)
    hal.open()

    fixes = []
    for _ in range(40):
        for b in hal.poll(0.05):
            fixes.append(ul.make_estimator("Lv2", anchors).update(b))
        if len(fixes) >= 3:
            break
    hal.close()

    ok = [f for f in fixes if f.ok]
    assert ok, "位置が 1 つも出なかった"
    assert np.linalg.norm(ok[0].position - truth) < 0.05
    assert hal.n_ranged >= 5


def test_polls_every_tag_in_turn():
    """1 巡で全 TAG を呼ぶ (1 台ずつしか測距できないモジュールなので)."""
    tags = ["TAG00001", "TAG00002", "TAG00003", "TAG00004"]
    fake = FakeModule({t: 300 for t in tags})
    hal = Ryuw122Hal(fake, tags, timeout=0.2)
    hal.open()
    got = []
    for _ in range(40):
        got.extend(hal.poll(0.05))
        if got and len(got[0]) == len(tags):
            break
    hal.close()
    assert got
    assert {m.anchor_id for m in got[0].measurements} == set(tags)


def test_rssi_becomes_a_quality_value():
    fake = FakeModule({"TAG00001": 200}, rssi=True)
    hal = Ryuw122Hal(fake, ["TAG00001"])
    m = hal.range_once("TAG00001")
    assert m is not None and m.quality is not None
    assert 0.0 <= m.quality <= 1.0

    fake2 = FakeModule({"TAG00001": 200}, rssi=False)
    hal2 = Ryuw122Hal(fake2, ["TAG00001"])
    m2 = hal2.range_once("TAG00001")
    assert m2 is not None and m2.quality is None


def test_requires_at_least_one_tag():
    with pytest.raises(ValueError, match="TAG"):
        Ryuw122Hal(FakeModule({}), [])
