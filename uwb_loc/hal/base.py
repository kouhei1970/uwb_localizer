"""UWB HAL の共通インターフェイス.

チップごとに 1 クラス書けばライブラリ全体が動く. 実装すべきは

* :meth:`UwbHal.anchors` — アンカー一覧 (座標込み)
* :meth:`UwbHal.poll` — 溜まっている観測を返す

の 2 つだけ. 測距シーケンス, レジスタ操作, タイムスタンプの生成といった
チップ固有の話は全部この内側に閉じ込める.

書き方の指針
------------
1. ``poll`` は**ブロックしない**か, ``timeout`` で必ず戻ること.
2. 観測には**測距が成立した時刻**を入れる (ホストに届いた時刻ではない).
   密結合 EKF の予測ステップがこの時刻差で回るため, ここがずれると
   移動中の精度が直接落ちる.
3. 距離は**アンテナ遅延を引く前の生の値**を入れてよい.
   :class:`~uwb_loc.pipeline.Pipeline` が :attr:`Anchor.antenna_delay_m` で補正する.
   HAL 側で既に補正済みなら ``antenna_delay_m=0`` にしておく.
4. 受信電力や first path 情報が取れるなら ``quality`` (0-1) に正規化して入れる.
   生値は ``raw`` に置いておくと後から NLOS 分類器を作れる.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Iterator

from ..types import Anchor, MeasurementBatch

__all__ = ["UwbHal"]


class UwbHal(ABC):
    """UWB ハードウェア抽象化層."""

    #: 実装の識別名 (ログ・UI 表示用).
    name: str = "uwb"

    # ------------------------------------------------------------------ 必須

    @property
    @abstractmethod
    def anchors(self) -> list[Anchor]:
        """アンカー一覧.

        座標が未知の場合は空リストを返してよい.
        :func:`uwb_loc.calibration.self_survey` で相互測距から推定できる.
        """

    @abstractmethod
    def poll(self, timeout: float = 0.0) -> list[MeasurementBatch]:
        """溜まっている観測を返す.

        Parameters
        ----------
        timeout:
            最大待ち時間 [s]. 0 なら即座に (今あるものだけ返して) 戻る.

        Returns
        -------
        list[MeasurementBatch]
            観測がなければ空リスト. 例外は投げない (通信エラーは
            :meth:`is_open` を False にして表現する).
        """

    # ------------------------------------------------------------------ 任意

    def open(self) -> None:
        """デバイスを開く. 既定は何もしない."""

    def close(self) -> None:
        """デバイスを閉じる. 既定は何もしない."""

    @property
    def is_open(self) -> bool:
        """通信が生きているか. 切れたら False を返す."""
        return True

    def stream(self, timeout: float = 1.0) -> Iterator[MeasurementBatch]:
        """観測を延々と流すジェネレータ.

        :meth:`is_open` が False になるまで回り続ける.
        """
        while self.is_open:
            batches = self.poll(timeout)
            if not batches:
                continue
            yield from batches

    # -------------------------------------------------------------- with 構文

    def __enter__(self) -> "UwbHal":
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def __repr__(self) -> str:  # pragma: no cover - 表示のみ
        return f"<{type(self).__name__} name={self.name!r} anchors={len(self.anchors)}>"
