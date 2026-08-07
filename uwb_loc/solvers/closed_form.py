"""閉形式ソルバ — 反復も初期値も要らない解.

反復ソルバ (WNLS/EKF) の初期値を作るのが主な役目だが, Lv0 として単体でも
使える. どれも 1 回の線形代数で終わるので, 組込みへの移植が容易.

* :func:`lls_trilateration` — 基準アンカー差分による線形最小二乗 (Lv0)
* :func:`beck_gtrs` — Beck の厳密解. 球面最小二乗の**大域最適解**を
  1 変数の二分法だけで求める. 初期値の良し悪しで発散する心配がない
* :func:`chan_tdoa` — TDoA 用の 2 段階 WLS (1 段目)
"""

from __future__ import annotations

import numpy as np

__all__ = ["lls_trilateration", "beck_gtrs", "chan_tdoa"]

_EPS = 1e-12


def _weights(w: np.ndarray | None, n: int) -> np.ndarray:
    if w is None:
        return np.ones(n)
    w = np.asarray(w, dtype=float).reshape(n)
    return np.where(np.isfinite(w) & (w > 0.0), w, _EPS)


def lls_trilateration(
    anchors: np.ndarray,
    ranges: np.ndarray,
    weights: np.ndarray | None = None,
    *,
    ref: int | None = None,
) -> np.ndarray | None:
    """基準アンカー差分による線形最小二乗 (Lv0).

    ``||p||^2`` が差分で消えるので線形方程式になり, 擬似逆行列 1 発で解ける.
    反復も初期値も不要で最速だが, 基準アンカーの選び方で精度が変わり,
    統計的には最適でない (差分により観測誤差に相関が入るため). 初期値供給と
    動作確認向け.

    Parameters
    ----------
    anchors:
        アンカー座標, shape (n, d). d は 2 か 3.
    ranges:
        測距値 [m], shape (n,).
    weights:
        各観測の重み (``1/sigma^2`` 相当), shape (n,).
    ref:
        基準アンカーの添字. None なら測距値が最小のもの (通常いちばん
        S/N が良い) を選ぶ.

    Returns
    -------
    np.ndarray | None
        位置, shape (d,). 解けなければ None.
    """
    anchors = np.atleast_2d(np.asarray(anchors, dtype=float))
    ranges = np.asarray(ranges, dtype=float).reshape(-1)
    n, d = anchors.shape
    if n < d + 1:
        return None
    w = _weights(weights, n)

    if ref is None:
        ref = int(np.argmin(ranges))
    idx = [i for i in range(n) if i != ref]

    a_ref, r_ref = anchors[ref], ranges[ref]
    mat = 2.0 * (a_ref - anchors[idx])
    rhs = (
        ranges[idx] ** 2
        - r_ref**2
        - np.einsum("ij,ij->i", anchors[idx], anchors[idx])
        + float(a_ref @ a_ref)
    )
    # 差分をとった行の重みは 2 本の観測の合成. 調和平均で近似する.
    sw = np.sqrt(w[idx] * w[ref] / (w[idx] + w[ref]))
    try:
        sol, *_ = np.linalg.lstsq(mat * sw[:, None], rhs * sw, rcond=None)
    except np.linalg.LinAlgError:  # pragma: no cover - lstsq はまず落ちない
        return None
    return None if not np.all(np.isfinite(sol)) else sol


def beck_gtrs(
    anchors: np.ndarray,
    ranges: np.ndarray,
    weights: np.ndarray | None = None,
    *,
    max_bisect: int = 200,
) -> np.ndarray | None:
    """Beck の厳密解 (Generalized Trust Region Subproblem).

    ``y = [p; ||p||^2]`` と置くと測距方程式は ``A y = b`` の線形形になり,
    ``y`` の 4 (2D なら 3) 番目の要素が ``||p||^2`` である,
    という 2 次制約が 1 本つく. これは GTRS になり,
    **1 変数 λ の単調減少な永年方程式を二分法で解くだけで大域最適解が出る**.

    LLS と違って差分をとらないので偏りが小さく, Gauss-Newton と違って
    初期値依存の発散がない. 既定の初期解にはこれを使う.

    Parameters
    ----------
    anchors:
        アンカー座標, shape (n, d).
    ranges:
        測距値 [m], shape (n,).
    weights:
        各観測の重み (``1/sigma^2`` 相当), shape (n,).

    Returns
    -------
    np.ndarray | None
        位置, shape (d,). 退化して解けなければ None.
    """
    anchors = np.atleast_2d(np.asarray(anchors, dtype=float))
    ranges = np.asarray(ranges, dtype=float).reshape(-1)
    n, d = anchors.shape
    if n < d + 1:
        return None
    w = _weights(weights, n)

    # A y = b,  y = [p; ||p||^2]
    mat = np.hstack([-2.0 * anchors, np.ones((n, 1))])
    rhs = ranges**2 - np.einsum("ij,ij->i", anchors, anchors)

    dmat = np.zeros((d + 1, d + 1))
    dmat[:d, :d] = np.eye(d)
    fvec = np.zeros(d + 1)
    fvec[d] = -0.5

    gmat = mat.T @ (w[:, None] * mat)
    hvec = mat.T @ (w * rhs)

    def solve_y(lam: float) -> np.ndarray | None:
        try:
            return np.linalg.solve(gmat + lam * dmat, hvec - lam * fvec)
        except np.linalg.LinAlgError:
            return None

    def phi(lam: float) -> float:
        y = solve_y(lam)
        if y is None:
            return np.nan
        return float(y @ dmat @ y + 2.0 * fvec @ y)

    # λ の下限は一般化固有値 (D, G) の最大値の逆数の符号反転.
    # この右側で φ は狭義単調減少になる.
    try:
        eig = np.linalg.eigvals(np.linalg.solve(gmat, dmat))
    except np.linalg.LinAlgError:
        return None
    eig = eig.real[np.isfinite(eig.real)]
    gamma = float(np.max(eig)) if eig.size else 0.0
    if gamma <= _EPS:
        return None
    lam_lo = -1.0 / gamma

    # 開区間なので下限からわずかに内側に入る. φ(lam_lo+) は +∞.
    span = max(abs(lam_lo), 1.0)
    lo = lam_lo + 1e-9 * span
    for _ in range(60):
        v = phi(lo)
        if np.isfinite(v) and v > 0.0:
            break
        lo = lam_lo + (lo - lam_lo) * 10.0
    else:
        return None

    hi = lo + span
    for _ in range(200):
        v = phi(hi)
        if np.isfinite(v) and v < 0.0:
            break
        hi = lo + (hi - lo) * 2.0
    else:
        return None

    for _ in range(max_bisect):
        mid = 0.5 * (lo + hi)
        v = phi(mid)
        if not np.isfinite(v):
            return None
        if v > 0.0:
            lo = mid
        else:
            hi = mid
        if hi - lo < 1e-14 * max(1.0, abs(hi)):
            break

    y = solve_y(0.5 * (lo + hi))
    if y is None or not np.all(np.isfinite(y[:d])):
        return None
    return y[:d]


def chan_tdoa(
    anchors: np.ndarray,
    tdoa: np.ndarray,
    ref_index: int = 0,
    weights: np.ndarray | None = None,
) -> np.ndarray | None:
    """TDoA の Chan 法 (2 段階 WLS の 1 段目).

    基準アンカーまでの距離 ``d_ref`` を補助未知数に加えると線形方程式になる,
    という GPS 由来の手当て. 初期値不要の閉形式で, ここから Gauss-Newton で
    仕上げれば実用精度になる.

    2 段目 (``d_ref = ||p - a_ref||`` の拘束を使った補正) は入れていない.
    1 段目だけでも反復ソルバの初期値としては十分で, 冗長観測があるときの
    最終精度は後段の WNLS が担保するため.

    Parameters
    ----------
    anchors:
        アンカー座標, shape (n, d). ``ref_index`` 行を基準とする.
    tdoa:
        距離差 [m], shape (n,). ``i`` 番目は ``||p-a_i|| - ||p-a_ref||``.
        ``ref_index`` の要素は無視される (0 のはず).
    ref_index:
        基準アンカーの添字.
    weights:
        各観測の重み, shape (n,).

    Returns
    -------
    np.ndarray | None
        位置, shape (d,). 観測が足りなければ None.
    """
    anchors = np.atleast_2d(np.asarray(anchors, dtype=float))
    tdoa = np.asarray(tdoa, dtype=float).reshape(-1)
    n, d = anchors.shape
    idx = [i for i in range(n) if i != ref_index]
    if len(idx) < d + 1:  # 未知数は位置 d + 基準距離 1
        return None
    w = _weights(weights, n)[idx]

    a_ref = anchors[ref_index]
    k_ref = float(a_ref @ a_ref)
    r = tdoa[idx]

    # 2(a_i - a_ref)·p + 2 r_i d_ref = K_i - K_ref - r_i^2
    mat = np.hstack([2.0 * (anchors[idx] - a_ref), 2.0 * r[:, None]])
    rhs = np.einsum("ij,ij->i", anchors[idx], anchors[idx]) - k_ref - r**2

    sw = np.sqrt(w)
    try:
        sol, *_ = np.linalg.lstsq(mat * sw[:, None], rhs * sw, rcond=None)
    except np.linalg.LinAlgError:  # pragma: no cover
        return None
    if not np.all(np.isfinite(sol)):
        return None
    return sol[:d]
