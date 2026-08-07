"""幾何評価 — アンカー配置の良し悪しを測る.

測位精度は測距精度だけでは決まらない. 同じ 10 cm の測距誤差でも,
アンカーがタグから見て一方向に固まっていれば位置誤差は何倍にも増える.
その増幅率が GDOP で, 配置を決める段階で計算できる.

* :func:`gdop_from_jacobian` — 測位結果に付ける GDOP
* :func:`gdop_at` / :func:`gdop_map` — 設置前の配置検討用
* :func:`crlb_at` — 測距 σ を与えたときの位置誤差の下限 [m]
* :func:`anchor_plane` / :func:`mirror_point` — 同一平面配置で生じる鏡像解の扱い
"""

from __future__ import annotations

import numpy as np

from .types import Anchor

__all__ = [
    "gdop_from_jacobian",
    "gdop_at",
    "gdop_map",
    "crlb_at",
    "anchor_condition",
    "anchor_plane",
    "mirror_point",
]

_EPS = 1e-12


def gdop_from_jacobian(jac: np.ndarray, mask: np.ndarray | None = None) -> float:
    """ヤコビアンから GDOP を計算する.

    行を単位ベクトルに正規化してから ``sqrt(trace((H^T H)^-1))`` を取る.
    測距観測の行はもともと単位ベクトルなので, これは教科書どおりの GDOP に
    一致する. 角度観測が混ざっていてもスケールが揃うので比較可能な数字になる.
    """
    jac = np.atleast_2d(np.asarray(jac, dtype=float))
    if jac.size == 0:
        return float("nan")
    norm = np.linalg.norm(jac, axis=1)
    good = norm > _EPS
    if good.sum() == 0:
        return float("nan")
    hmat = jac[good] / norm[good, None]
    if mask is not None:
        hmat = hmat[:, mask]
    gram = hmat.T @ hmat
    # 観測が足りない / 一直線に並ぶと特異になる. 逆行列が数値的に暴れて
    # トレースが負に出ることもあるので, そこも幾何が退化しているとみなす.
    if np.linalg.matrix_rank(gram, tol=1e-9) < gram.shape[0]:
        return float("inf")
    try:
        tr = float(np.trace(np.linalg.inv(gram)))
    except np.linalg.LinAlgError:
        return float("inf")
    return float(np.sqrt(tr)) if tr > 0.0 else float("inf")


def gdop_at(point: np.ndarray, anchors: list[Anchor], *, dim: int = 3) -> float:
    """ある点における GDOP.

    Parameters
    ----------
    point:
        評価点 [m], shape (3,).
    anchors:
        アンカー一覧 (``enabled`` が False のものは除く).
    dim:
        2 なら xy のみで評価する (高さ既知の運用).
    """
    point = np.asarray(point, dtype=float).reshape(3)
    pts = np.array([a.position for a in anchors if a.enabled])
    if len(pts) < dim + 1:
        return float("inf")
    dv = point - pts
    norm = np.linalg.norm(dv, axis=1)
    good = norm > _EPS
    if good.sum() < dim + 1:
        return float("inf")
    hmat = dv[good] / norm[good, None]
    mask = np.array([True, True, dim == 3])
    return gdop_from_jacobian(hmat, mask)


def gdop_map(
    anchors: list[Anchor],
    bounds: tuple[tuple[float, float], tuple[float, float]],
    z: float = 1.0,
    *,
    nx: int = 40,
    ny: int = 40,
    dim: int = 3,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """水平面を格子で切って GDOP を評価する.

    「その部屋のどこで精度が落ちるか」を設置前に見るためのもの.
    UI ではこれをヒートマップにする.

    Returns
    -------
    (x, y, g)
        ``x`` shape (nx,), ``y`` shape (ny,), ``g`` shape (ny, nx).
    """
    (x0, x1), (y0, y1) = bounds
    x = np.linspace(x0, x1, nx)
    y = np.linspace(y0, y1, ny)
    g = np.empty((ny, nx))
    for j, yy in enumerate(y):
        for i, xx in enumerate(x):
            g[j, i] = gdop_at(np.array([xx, yy, z]), anchors, dim=dim)
    return x, y, g


def crlb_at(point: np.ndarray, anchors: list[Anchor], *, dim: int = 3) -> float:
    """クラメール・ラオ下限 (位置誤差の理論下限) [m].

    各アンカーのノイズモデル (``sigma0`` と ``sigma_per_m``) を使って
    フィッシャー情報行列を組み, その逆行列のトレースの平方根を返す.
    **どんなアルゴリズムを使ってもこれより良くはならない**ので,
    実測 RMSE と並べればアルゴリズムの改善余地が分かる.
    """
    point = np.asarray(point, dtype=float).reshape(3)
    active = [a for a in anchors if a.enabled]
    if len(active) < dim + 1:
        return float("inf")

    mask = np.array([True, True, dim == 3])
    fim = np.zeros((int(mask.sum()), int(mask.sum())))
    used = 0
    for a in active:
        dv = point - a.position
        d = float(np.linalg.norm(dv))
        if d < _EPS:
            continue
        u = (dv / d)[mask]
        sigma = a.range_sigma(d)
        fim += np.outer(u, u) / max(sigma, 1e-6) ** 2
        used += 1
    if used < dim + 1:
        return float("inf")
    try:
        return float(np.sqrt(np.trace(np.linalg.inv(fim))))
    except np.linalg.LinAlgError:
        return float("inf")


def anchor_condition(anchors: list[Anchor]) -> dict[str, float | bool]:
    """アンカー配置の素性を調べる.

    3 次元測位では**アンカーが同一平面に並んでいないこと**が本質的に効く.
    天井の 4 隅に貼っただけの配置は平面配置なので, 高さがほとんど
    観測できない. 実際に測る前にここで気づけるようにしておく.

    Returns
    -------
    dict
        ``n`` アンカー数, ``coplanar`` 同一平面か, ``planarity`` 平面からの
        広がり [m] (第 3 主成分の標準偏差), ``spread`` 全体の広がり [m].
    """
    pts = np.array([a.position for a in anchors if a.enabled], dtype=float)
    n = len(pts)
    out: dict[str, float | bool] = {"n": float(n)}
    if n < 3:
        out.update(coplanar=True, planarity=0.0, spread=0.0)
        return out
    centered = pts - pts.mean(axis=0)
    sv = np.linalg.svd(centered, compute_uv=False) / np.sqrt(n)
    planarity = float(sv[2]) if len(sv) > 2 else 0.0
    spread = float(sv[0])
    out.update(
        coplanar=bool(planarity < 0.05 * max(spread, 1e-9)),
        planarity=planarity,
        spread=spread,
    )
    return out


def anchor_plane(
    anchors: list[Anchor], *, tol: float = 0.05
) -> tuple[np.ndarray, float] | None:
    """アンカーが同一平面上に並んでいるなら, その平面を返す.

    測距値だけからは **平面に関する鏡映が区別できない**. アンカーが平面
    ``n·x = c`` 上にあるとき, 任意の点 ``p`` とその鏡像 ``p'`` は
    すべてのアンカーからの距離が厳密に等しくなるためで, これはアルゴリズムの
    出来不出来ではなく問題そのものが持つ多義性である.

    天井の 4 隅だけに貼った構成がまさにこれに当たるので, その場合は
    ``SolveConfig(z_bounds=...)`` で片側に絞るか, ``dim=2`` で高さを
    固定して解く必要がある.

    Parameters
    ----------
    anchors:
        アンカー一覧 (``enabled`` が False のものは無視).
    tol:
        同一平面とみなす許容量. 全体の広がりに対する相対値.

    Returns
    -------
    tuple[np.ndarray, float] | None
        ``(単位法線, オフセット)``. 同一平面でなければ None.
        アンカーが 3 台未満のときも None (平面が決まらない).
    """
    pts = np.array([a.position for a in anchors if a.enabled], dtype=float)
    if len(pts) < 3:
        return None
    center = pts.mean(axis=0)
    centered = pts - center
    # 最小特異値の方向が平面の法線. その方向の広がりが十分小さければ同一平面.
    _, sv, vt = np.linalg.svd(centered)
    spread = float(sv[0]) / np.sqrt(len(pts))
    if spread < _EPS:
        return None
    if float(sv[-1]) / np.sqrt(len(pts)) > tol * spread:
        return None
    normal = vt[-1] / np.linalg.norm(vt[-1])
    return normal, float(normal @ center)


def mirror_point(p: np.ndarray, normal: np.ndarray, offset: float) -> np.ndarray:
    """平面 ``normal·x = offset`` に関して点を鏡映する."""
    p = np.asarray(p, dtype=float)
    normal = np.asarray(normal, dtype=float)
    return p - 2.0 * (float(normal @ p) - offset) * normal
