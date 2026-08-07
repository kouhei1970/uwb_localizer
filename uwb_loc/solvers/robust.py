"""ロバスト化 — 屋内 UWB の精度はここで決まる.

見通し外 (NLOS) では電波が回り込む分だけ距離が伸びるので, 誤差は
**必ず正側に偏る**. ガウス仮定の最小二乗はこの偏りに極端に弱い.
本モジュールはその対策を段階的に用意する.

1. :func:`physical_gate` — 物理的にありえない観測を落とす (ほぼ無料)
2. :func:`robust_weights` — Huber/Tukey の M 推定. 外れ値を捨てずに重みを
   下げるので, アンカー本数が少ないときに「捨てすぎて解けない」を避けられる
3. 片側損失 — 正側の残差だけ厳しく見る. NLOS が正バイアスだという物理を
   そのまま重みに落とし込んだもの
4. :func:`ransac_ranges` — アンカーが十分あるとき, NLOS が過半でも効く最後の砦
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..types import Anchor, MeasKind, Measurement

__all__ = ["RobustLoss", "robust_weights", "physical_gate", "ransac_ranges"]


@dataclass
class RobustLoss:
    """ロバスト損失の設定.

    Attributes
    ----------
    kind:
        ``"none"`` (通常の最小二乗) / ``"huber"`` / ``"tukey"``.
        既定は Huber. Tukey は外れ値を完全に切るので効きは強いが,
        初期値が悪いと正しい観測まで切ってしまう.
    k:
        しきい値 (正規化残差の単位). Huber の 1.345 はガウス誤差に対して
        95% の効率を保つ標準的な値.
    one_sided:
        True なら**正側の残差 (= 測距が伸びた側) だけ**しきい値を
        ``k_pos_scale`` 倍に絞る. NLOS の物理と一致するので,
        LOS 側の情報を捨てずに NLOS だけ抑えられる.
    k_pos_scale:
        片側損失で正側に適用する係数 (< 1 で厳しくなる).
    """

    kind: str = "huber"
    k: float = 1.345
    one_sided: bool = True
    k_pos_scale: float = 0.6

    def thresholds(self, residual: np.ndarray) -> np.ndarray:
        """残差ごとのしきい値 (片側損失を反映)."""
        k = np.full(residual.shape, self.k, dtype=float)
        if self.one_sided:
            k[residual > 0.0] *= self.k_pos_scale
        return k


def robust_weights(residual: np.ndarray, sigma: np.ndarray, loss: RobustLoss) -> np.ndarray:
    """残差から重みの倍率 (0-1) を作る.

    Parameters
    ----------
    residual:
        残差 ``観測値 - 予測値`` [m], shape (n,). 距離観測では
        NLOS のとき正になる.
    sigma:
        各観測の 1σ, shape (n,).
    loss:
        損失設定.

    Returns
    -------
    np.ndarray
        重みの倍率, shape (n,). ``1/sigma^2`` に掛けて使う.
    """
    residual = np.asarray(residual, dtype=float)
    sigma = np.asarray(sigma, dtype=float)
    if loss.kind == "none":
        return np.ones_like(residual)

    # スケールは σ そのものではなく残差の MAD も見る. アンテナ遅延の
    # 取り残しなど全体が一様にずれている場合に効きすぎるのを防ぐ.
    scale = np.maximum(sigma, 1e-6)
    u = residual / scale
    k = loss.thresholds(residual)

    if loss.kind == "huber":
        return np.where(np.abs(u) <= k, 1.0, k / np.maximum(np.abs(u), 1e-9))
    if loss.kind == "tukey":
        inside = np.abs(u) <= k
        w = np.zeros_like(u)
        w[inside] = (1.0 - (u[inside] / k[inside]) ** 2) ** 2
        return w
    raise ValueError(f"未知の損失: {loss.kind}")


def physical_gate(
    meas: list[Measurement],
    anchors: dict[str, Anchor],
    *,
    max_range: float = 200.0,
    slack: float = 1.0,
) -> tuple[list[Measurement], list[str]]:
    """物理的にありえない観測を落とす.

    位置を解く前にできるチェックなので, 計算量ゼロに近いのに効く.

    * 負の距離, 到達しえない距離
    * 三角不等式違反 — アンカー間の距離が分かっているので,
      2 本の測距値の和がアンカー間距離より短い / 差がアンカー間距離より長い
      という組み合わせは (誤差 ``slack`` を見ても) ありえない

    Returns
    -------
    (残った観測, 落としたアンカー ID)
    """
    kept: list[Measurement] = []
    dropped: list[str] = []

    ranges: list[Measurement] = []
    for m in meas:
        if m.kind is not MeasKind.RANGE:
            kept.append(m)
            continue
        if not np.isfinite(m.value) or m.value < -slack or m.value > max_range:
            dropped.append(m.anchor_id)
            continue
        ranges.append(m)

    # 三角不等式: 各観測が他の観測と何回矛盾するかを数え, 多いものを落とす.
    n = len(ranges)
    if n >= 3:
        conflicts = np.zeros(n, dtype=int)
        for i in range(n):
            ai = anchors.get(ranges[i].anchor_id)
            for j in range(i + 1, n):
                aj = anchors.get(ranges[j].anchor_id)
                if ai is None or aj is None:
                    continue
                dij = float(np.linalg.norm(ai.position - aj.position))
                ri, rj = ranges[i].value, ranges[j].value
                if ri + rj + slack < dij or abs(ri - rj) - slack > dij:
                    conflicts[i] += 1
                    conflicts[j] += 1
        # 全体の過半と矛盾するものだけ落とす (1 本ずつの小競り合いは残す).
        limit = max(1, (n - 1) // 2)
        for i, m in enumerate(ranges):
            if conflicts[i] > limit:
                dropped.append(m.anchor_id)
            else:
                kept.append(m)
    else:
        kept.extend(ranges)

    return kept, dropped


def ransac_ranges(
    anchor_pos: np.ndarray,
    ranges: np.ndarray,
    sigma: np.ndarray,
    solve,
    *,
    dim: int = 3,
    threshold: float = 3.0,
    max_trials: int = 64,
    rng: np.random.Generator | None = None,
) -> np.ndarray | None:
    """RANSAC でインライア集合を選ぶ.

    最小構成のアンカーで仮の位置を作り, 残差がしきい値以内に収まる観測が
    最も多い組を採る. NLOS が過半を占めるような悪条件で, M 推定が
    引っぱられてしまう場合の保険.

    Parameters
    ----------
    anchor_pos:
        アンカー座標, shape (n, dim).
    ranges:
        測距値 [m], shape (n,).
    sigma:
        1σ, shape (n,).
    solve:
        ``solve(anchor_pos, ranges, weights) -> 位置 or None`` の閉形式ソルバ.
    threshold:
        インライア判定のしきい値 (σ の倍数).
    max_trials:
        試行回数.

    Returns
    -------
    np.ndarray | None
        インライアの真偽値, shape (n,). アンカーが足りなければ None.
    """
    n = len(ranges)
    minimal = dim + 1
    if n < minimal + 2:  # 冗長がないなら RANSAC の意味がない
        return None
    rng = rng or np.random.default_rng(0)

    best_mask: np.ndarray | None = None
    best_count = 0
    for _ in range(max_trials):
        sample = rng.choice(n, size=minimal, replace=False)
        p = solve(anchor_pos[sample], ranges[sample], 1.0 / np.maximum(sigma[sample], 1e-6) ** 2)
        if p is None:
            continue
        pred = np.linalg.norm(anchor_pos - p, axis=1)
        mask = np.abs(ranges - pred) <= threshold * np.maximum(sigma, 1e-6)
        count = int(mask.sum())
        if count > best_count:
            best_count, best_mask = count, mask
            if count == n:
                break

    if best_mask is None or best_count < minimal:
        return None
    return best_mask
