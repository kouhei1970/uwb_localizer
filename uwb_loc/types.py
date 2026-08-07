"""HAL と測位アルゴリズムの間でやり取りする共通データ型.

チップ固有の処理 (レジスタ操作, タイムスタンプ, 測距シーケンス) は HAL 側に
閉じ込め, ここで定義する型だけをライブラリに渡す. どの UWB を使っていても
``Measurement`` にしてしまえば下流のコードは共通になる.

規約
----
単位
    長さ [m], 角度 [rad], 時刻 [s]. 例外なし.
座標系
    右手系, z 軸上向き (ENU 相当). アンカー座標と推定位置は同じ系.
時刻
    単調増加する秒. 絶対時刻である必要はないが, 全観測で同一の基準を使う.
    「測距が成立した時刻」を入れる (ホスト到着時刻ではない).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np

__all__ = [
    "MeasKind",
    "Anchor",
    "Measurement",
    "MeasurementBatch",
    "Fix",
    "WIRE_VERSION",
]

#: JSON Lines ワイヤ仕様のバージョン (docs/UWB_PROTOCOL.md).
WIRE_VERSION = 1


class MeasKind(str, Enum):
    """観測量の種別.

    値は文字列にしてある (JSON にそのまま載る).
    """

    RANGE = "range"
    """タグ-アンカー間距離 [m]. TWR (SS/DS/AltDS) の結果."""

    TDOA = "tdoa"
    """距離差 [m]. ``anchor_id`` までの距離 - ``ref_anchor_id`` までの距離."""

    AZIMUTH = "azimuth"
    """アンカーから見たタグの方位角 [rad]. x 軸から反時計回り."""

    ELEVATION = "elevation"
    """アンカーから見たタグの仰角 [rad]. xy 平面から上向き正."""


@dataclass
class Anchor:
    """アンカー (固定局).

    Attributes
    ----------
    id:
        文字列 ID. HAL が返す観測の ``anchor_id`` と一致させる.
    position:
        設置座標 [m], shape (3,).
    enabled:
        False なら測位に使わない (UI から個別に落とすため).
    antenna_delay_m:
        アンテナ遅延に相当する距離オフセット [m]. 観測距離から差し引く.
        :mod:`uwb_loc.calibration` で推定できる.
    sigma0:
        測距ノイズの定数項 [m].
    sigma_per_m:
        測距ノイズの距離比例項 [m/m]. 実効的な標準偏差は
        ``sigma0 + sigma_per_m * d``.
    position_sigma:
        設置座標そのものの不確かさ [m]. 測位の重みに加算される.
    """

    id: str
    position: np.ndarray
    enabled: bool = True
    antenna_delay_m: float = 0.0
    sigma0: float = 0.10
    sigma_per_m: float = 0.0
    position_sigma: float = 0.0

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float).reshape(3)

    def range_sigma(self, distance: float) -> float:
        """距離 ``distance`` におけるモデル上の測距標準偏差 [m]."""
        var = (self.sigma0 + self.sigma_per_m * distance) ** 2
        return float(np.sqrt(var + self.position_sigma**2))

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "p": [float(v) for v in self.position],
            "enabled": self.enabled,
            "antenna_delay_m": self.antenna_delay_m,
            "sigma0": self.sigma0,
            "sigma_per_m": self.sigma_per_m,
            "position_sigma": self.position_sigma,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Anchor":
        return cls(
            id=str(d["id"]),
            position=np.asarray(d["p"], dtype=float),
            enabled=bool(d.get("enabled", True)),
            antenna_delay_m=float(d.get("antenna_delay_m", 0.0)),
            sigma0=float(d.get("sigma0", 0.10)),
            sigma_per_m=float(d.get("sigma_per_m", 0.0)),
            position_sigma=float(d.get("position_sigma", 0.0)),
        )


@dataclass
class Measurement:
    """観測 1 本.

    HAL はこれを作って返すだけでよい. ``sigma`` と ``quality`` は
    省略できるが, 入れておくほど NLOS に強くなる.

    Attributes
    ----------
    anchor_id:
        相手アンカーの ID.
    value:
        観測値. ``kind`` が RANGE/TDOA なら [m], AZIMUTH/ELEVATION なら [rad].
    kind:
        観測種別.
    t:
        測距が成立した時刻 [s].
    sigma:
        観測の 1σ. None ならアンカー設定のモデル値を使う.
    quality:
        0-1 の見通し (LOS) 尤度. 1 が完全な見通し. None なら 1 とみなす.
        チップ非依存にするため, HAL 側で正規化してから入れる.
    ref_anchor_id:
        TDOA の基準アンカー ID. TDOA 以外では None.
    tag_id:
        測位対象 (移動局) の ID.
    raw:
        チップ固有の生情報 (受信電力, first path index など). 診断用で
        アルゴリズムは参照しないが, ログに残すと NLOS 分類器を後から作れる.
    """

    anchor_id: str
    value: float
    kind: MeasKind = MeasKind.RANGE
    t: float = 0.0
    sigma: float | None = None
    quality: float | None = None
    ref_anchor_id: str | None = None
    tag_id: str = "tag0"
    raw: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "a": self.anchor_id,
            "type": self.kind.value,
            "v": float(self.value),
            "t": float(self.t),
        }
        if self.sigma is not None:
            d["sigma"] = float(self.sigma)
        if self.quality is not None:
            d["q"] = float(self.quality)
        if self.ref_anchor_id is not None:
            d["ref"] = self.ref_anchor_id
        if self.raw:
            d["raw"] = self.raw
        return d

    @classmethod
    def from_dict(cls, d: dict[str, Any], *, tag_id: str = "tag0", t: float = 0.0) -> "Measurement":
        # "d" (distance) は range の別名として受け付ける. 実機ファームウェアが
        # 距離を素直に "d" と書くことが多いため.
        value = d.get("v")
        if value is None:
            value = d.get("d")
        if value is None:
            raise ValueError(f"観測値 ('v' または 'd') がない: {d}")
        return cls(
            anchor_id=str(d["a"]),
            value=float(value),
            kind=MeasKind(d.get("type", "range")),
            t=float(d.get("t", t)),
            sigma=None if d.get("sigma") is None else float(d["sigma"]),
            quality=None if d.get("q") is None else float(d["q"]),
            ref_anchor_id=d.get("ref"),
            tag_id=str(d.get("tag", tag_id)),
            raw=dict(d.get("raw", {})),
        )


@dataclass
class MeasurementBatch:
    """同一エポックにまとめた観測.

    密結合 EKF は本来 1 本ずつ処理できるが, ログ・UI・スナップショット測位の
    都合でエポック単位に束ねられると扱いやすいので, これを基本単位にする.
    束ねられていなくても (1 本だけの batch でも) 動く.
    """

    t: float
    measurements: list[Measurement] = field(default_factory=list)
    tag_id: str = "tag0"

    def __len__(self) -> int:
        return len(self.measurements)

    def of_kind(self, kind: MeasKind) -> list[Measurement]:
        return [m for m in self.measurements if m.kind is kind]

    def to_dict(self) -> dict[str, Any]:
        return {
            "v": WIRE_VERSION,
            "type": "meas",
            "t": float(self.t),
            "tag": self.tag_id,
            "meas": [m.to_dict() for m in self.measurements],
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "MeasurementBatch":
        t = float(d.get("t", 0.0))
        tag = str(d.get("tag", "tag0"))
        return cls(
            t=t,
            tag_id=tag,
            measurements=[Measurement.from_dict(m, tag_id=tag, t=t) for m in d.get("meas", [])],
        )


@dataclass
class Fix:
    """測位結果.

    位置だけでなく共分散と品質指標を必ず返す. 運用時に「今の値を信じて
    よいか」を判断できないライブラリは使い物にならないため.

    Attributes
    ----------
    position:
        推定位置 [m], shape (3,).
    covariance:
        位置の共分散 [m^2], shape (3, 3).
    t:
        この推定が対応する時刻 [s].
    ok:
        測位が成立したか. False のとき ``position`` は前回値または NaN.
    n_used / n_total:
        測位に使った観測数 / 入力された観測数.
    residual_rms:
        採用した観測の残差 RMS [m].
    gdop:
        幾何精度劣化係数. 観測の単位ベクトルだけから決まる無次元量.
    excluded:
        外れ値として落としたアンカー ID.
    iterations:
        反復回数 (閉形式なら 0).
    level:
        どの忠実度レベルが出したか ("Lv0" 〜 "Lv3").
    velocity:
        速度推定 [m/s]. 追跡フィルタのみ. それ以外は None.
    ambiguous:
        解が一意に定まらなかったか. アンカーが同一平面に並んでいると,
        その平面に関する鏡像は測距値ではまったく区別できない (どちらも
        全アンカーからの距離が厳密に一致する). True のときは
        ``position`` が鏡像側である可能性が残る —
        ``SolveConfig(z_bounds=...)`` で片側に絞るか, ``dim=2`` で
        高さを固定すれば解消する.
    """

    position: np.ndarray
    covariance: np.ndarray
    t: float = 0.0
    ok: bool = True
    n_used: int = 0
    n_total: int = 0
    residual_rms: float = float("nan")
    gdop: float = float("nan")
    excluded: list[str] = field(default_factory=list)
    iterations: int = 0
    level: str = ""
    velocity: np.ndarray | None = None
    ambiguous: bool = False

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float).reshape(3)
        self.covariance = np.asarray(self.covariance, dtype=float).reshape(3, 3)

    @property
    def sigma(self) -> float:
        """位置誤差の代表値 (共分散のトレースの平方根) [m]."""
        return float(np.sqrt(max(np.trace(self.covariance), 0.0)))

    @classmethod
    def failed(cls, t: float = 0.0, n_total: int = 0, level: str = "") -> "Fix":
        return cls(
            position=np.full(3, np.nan),
            covariance=np.full((3, 3), np.nan),
            t=t,
            ok=False,
            n_total=n_total,
            level=level,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "t": self.t,
            "ok": self.ok,
            "p": [None if not np.isfinite(v) else float(v) for v in self.position],
            "sigma": None if not np.isfinite(self.sigma) else self.sigma,
            "n_used": self.n_used,
            "n_total": self.n_total,
            "residual_rms": None if not np.isfinite(self.residual_rms) else self.residual_rms,
            "gdop": None if not np.isfinite(self.gdop) else self.gdop,
            "excluded": list(self.excluded),
            "level": self.level,
            "ambiguous": bool(self.ambiguous),
            "v": None if self.velocity is None else [float(v) for v in self.velocity],
        }
