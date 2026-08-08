"""キャリブレーションと設営支援.

実運用で精度が出ない原因は, アルゴリズムよりも**アンテナ遅延の未補正**と
**アンカー座標の測り間違い**であることが多い. どちらもここで潰せる.

* :func:`fit_range_bias` — 既知距離から ``r_true = a*r_meas + b`` を当てる
* :func:`estimate_antenna_delays` — アンカーごとの遅延をまとめて推定
* :func:`self_survey` — アンカー間の相互測距だけからアンカー配置を推定.
  **巻き尺で測る作業がなくなる**ので, これがいちばん効く
"""

from __future__ import annotations

import warnings

import numpy as np

from .types import Anchor

__all__ = [
    "fit_range_bias",
    "estimate_antenna_delays",
    "self_survey",
    "apply_gauge",
    "align_to_reference",
]

_EPS = 1e-12


def fit_range_bias(measured: np.ndarray, true: np.ndarray) -> tuple[float, float]:
    """距離バイアスの 1 次モデルを当てる.

    ``true ≈ scale * measured + offset``.

    Returns
    -------
    (scale, offset)
        ``scale`` は 1 に近いはず. 大きく外れるならクロック関連を疑う.
    """
    measured = np.asarray(measured, dtype=float).reshape(-1)
    true = np.asarray(true, dtype=float).reshape(-1)
    if measured.size < 2:
        raise ValueError("2 点以上必要")
    mat = np.column_stack([measured, np.ones_like(measured)])
    sol, *_ = np.linalg.lstsq(mat, true, rcond=None)
    return float(sol[0]), float(sol[1])


def estimate_antenna_delays(
    anchor_ids: list[str],
    measured: np.ndarray,
    true_distance: np.ndarray,
    *,
    tag_delay: bool = True,
) -> dict[str, float]:
    """アンテナ遅延をアンカーごとに推定する [m].

    測距値は ``r = d + delay_anchor + delay_tag`` と書けるので, 既知距離での
    測定を集めれば線形最小二乗で分離できる. ただしアンカー遅延とタグ遅延は
    定数分の不定性があるため, **アンカー遅延の平均を 0 に固定**して解く
    (残りはタグ側に寄る).

    Parameters
    ----------
    anchor_ids:
        各測定の相手アンカー ID, 長さ n.
    measured:
        測距値 [m], shape (n,).
    true_distance:
        既知の真の距離 [m], shape (n,).
    tag_delay:
        タグ側の共通遅延も未知数にするか.

    Returns
    -------
    dict
        アンカー ID -> 遅延 [m]. ``tag_delay=True`` なら ``"__tag__"`` に
        タグ側の遅延が入る. そのまま :attr:`Anchor.antenna_delay_m` に入れられる.
    """
    measured = np.asarray(measured, dtype=float).reshape(-1)
    true_distance = np.asarray(true_distance, dtype=float).reshape(-1)
    uniq = sorted(set(anchor_ids))
    n, k = len(measured), len(uniq)
    if n < k:
        raise ValueError("測定数がアンカー数より少ない")

    col = {a: i for i, a in enumerate(uniq)}
    ncol = k + (1 if tag_delay else 0)
    mat = np.zeros((n + 1, ncol))
    rhs = np.zeros(n + 1)
    for i, (aid, m, d) in enumerate(zip(anchor_ids, measured, true_distance)):
        mat[i, col[aid]] = 1.0
        if tag_delay:
            mat[i, k] = 1.0
        rhs[i] = m - d
    # 不定性を潰す拘束: アンカー遅延の平均 = 0.
    mat[n, :k] = 1.0
    rhs[n] = 0.0

    sol, *_ = np.linalg.lstsq(mat, rhs, rcond=None)
    out = {a: float(sol[col[a]]) for a in uniq}
    if tag_delay:
        out["__tag__"] = float(sol[k])
    return out


# --------------------------------------------------------------------------- 自己測量


def apply_gauge(pts: np.ndarray, *, dim: int = 3) -> np.ndarray:
    """座標系の不定性を潰す.

    相互距離だけからは回転・並進・鏡像が決まらない (6 自由度 + 鏡像) ので,
    点そのものから基底を作って固定する.

    * 0 番を原点
    * 1 番の方向を第 1 軸 (+x)
    * 1 番と一直線でない最初の点で第 2 軸 (+y) を決める
    * (3 次元なら) 平面から外れた最初の点が z>0 側になるようにする

    軸を「適当な固定ベクトル」から作ってはいけない. 第 1 軸まわりの回転が
    残ってしまい, 同じ配置でも入力座標系によって別の答えになる.
    """
    pts = np.array(pts, dtype=float)
    n = len(pts)
    if n == 0:
        return pts
    pts = pts - pts[0]
    if n < 2 or dim < 2:
        return pts

    v = pts[1]
    nv = float(np.linalg.norm(v))
    if nv <= _EPS:
        return pts
    e1 = v / nv
    scale = max(nv, 1.0)

    # 第 2 軸: e1 に直交する成分が**最大**の点から作る. 「最初に見つかった点」
    # で決めると, その成分がほぼ 0 の点に当たったとき雑音で向きが決まってしまう.
    e2 = None
    if n > 2:
        perp = pts[2:] - np.outer(pts[2:] @ e1, e1)
        k = int(np.argmax(np.linalg.norm(perp, axis=1)))
        w = perp[k]
        if float(np.linalg.norm(w)) > 1e-6 * scale:
            e2 = w / float(np.linalg.norm(w))
    if e2 is None:  # 全点が一直線. 残りの向きは決めようがないので任意に取る.
        tmp = np.zeros(dim)
        tmp[0] = 1.0
        if abs(float(tmp @ e1)) > 0.9:
            tmp[:] = 0.0
            tmp[1] = 1.0
        w = tmp - float(tmp @ e1) * e1
        e2 = w / float(np.linalg.norm(w))

    basis = [e1, e2]
    if dim == 3:
        basis.append(np.cross(e1, e2))
    pts = pts @ np.array(basis).T

    # 鏡像: 平面から十分離れた**最初の**点を z>0 側に置く. しきい値を
    # 全体の広がりに対する相対値にしてあるのは, 「最大の点」で決めると
    # 対称な配置 (箱型など) で複数の点が並んだときに雑音で選ばれ方が変わり,
    # 同じ配置なのに鏡像が反転してしまうため.
    if dim == 3 and n > 2:
        zmax = float(np.max(np.abs(pts[:, 2])))
        if zmax > 1e-9:
            for k in range(2, n):
                if abs(pts[k][2]) > 0.05 * zmax:
                    if pts[k][2] < 0.0:
                        pts[:, 2] *= -1.0
                    break
    return pts


def _warn_if_reference_cannot_fix_reflection(dst_points: np.ndarray) -> None:
    """既知点が鏡像を決められる配置か見て, 危なければ警告する.

    3 点は必ず 1 つの平面に乗るので, その平面に関して折り返した配置も
    同じだけよく合ってしまう. 4 点以上あっても同一平面なら同じこと.
    黙って裏返るより, ここで気づける方がよい.
    """
    n = len(dst_points)
    if n < 4:
        warnings.warn(
            f"align_to_reference: 既知点が {n} 点しかありません。3 次元では"
            "同一平面に乗らない 4 点以上ないと鏡像 (裏返り) が決まらず、"
            "推定が丸ごと反対側になることがあります。",
            stacklevel=3)
        return
    centered = dst_points - dst_points.mean(axis=0)
    sv = np.linalg.svd(centered, compute_uv=False)
    if sv[0] > _EPS and sv[-1] / sv[0] < 0.05:
        warnings.warn(
            "align_to_reference: 既知点がほぼ同一平面に並んでいます。"
            "鏡像 (裏返り) が決まらず、推定が丸ごと反対側になることがあります。"
            "高さの違う点を基準に加えてください。",
            stacklevel=3)


def align_to_reference(
    anchors: list[Anchor],
    reference: dict[str, np.ndarray],
    *,
    allow_reflection: bool = True,
) -> list[Anchor]:
    """自己測量の結果を実世界の座標系に合わせる.

    :func:`self_survey` が出す配置は**形は正しいが向きと原点は任意**
    (相互距離からは回転・並進・鏡像が決まらない). 実際の運用では
    「何台かだけ巻き尺で測って, 残りは自己測量に任せる」のが現実的なので,
    その既知点に重ねる変換 (Kabsch 法) を求めて全体に適用する.

    Parameters
    ----------
    anchors:
        :func:`self_survey` の出力.
    reference:
        アンカー ID -> 実測座標 [m]. **3 次元なら同一平面に乗らない 4 点以上**
        必要. 3 点だと鏡像 (どちら向きに折り返しているか) が決まらず,
        推定が丸ごと裏返ることがある —— 3 点はどちらの向きでも同じだけ
        よく合ってしまうため. 足りないときは警告を出す.
    allow_reflection:
        鏡像反転を許すか. 相互測距だけからは鏡像が決まらないので, 通常は True.

    Returns
    -------
    list[Anchor]
        実世界座標に載せ替えたアンカー一覧.
    """
    by_id = {a.id: a for a in anchors}
    ids = [k for k in reference if k in by_id]
    if len(ids) < 3:
        raise ValueError("既知点が 3 点以上必要")

    src = np.array([by_id[i].position for i in ids], dtype=float)
    _warn_if_reference_cannot_fix_reflection(dst_points=np.array(
        [np.asarray(reference[i], dtype=float).reshape(3) for i in ids]))
    dst = np.array([np.asarray(reference[i], dtype=float).reshape(3) for i in ids])
    src_c, dst_c = src.mean(axis=0), dst.mean(axis=0)

    umat, _, vt = np.linalg.svd((src - src_c).T @ (dst - dst_c))
    rot = (umat @ vt).T
    if not allow_reflection and np.linalg.det(rot) < 0.0:
        vt[-1] *= -1.0
        rot = (umat @ vt).T

    return [
        Anchor(
            id=a.id,
            position=rot @ (a.position - src_c) + dst_c,
            enabled=a.enabled,
            antenna_delay_m=a.antenna_delay_m,
            sigma0=a.sigma0,
            sigma_per_m=a.sigma_per_m,
            position_sigma=a.position_sigma,
        )
        for a in anchors
    ]


def self_survey(
    distances: np.ndarray,
    ids: list[str] | None = None,
    *,
    dim: int = 3,
    max_iter: int = 100,
    weights: np.ndarray | None = None,
) -> list[Anchor]:
    """アンカー間の相互測距からアンカー配置を推定する.

    古典的 MDS (多次元尺度構成法) で初期配置を作り, 距離残差の
    Gauss-Newton で仕上げる. 座標系は :func:`apply_gauge` の慣例で固定する.

    3 次元で意味のある解を得るには, **アンカーが同一平面に並んでいないこと**が
    必要. 平面配置しかない現場では ``dim=2`` で解いて高さは実測する.

    Parameters
    ----------
    distances:
        アンカー間距離行列 [m], shape (n, n). 対称で対角 0. 欠測は NaN.
    ids:
        アンカー ID. None なら ``A0, A1, ...``.
    dim:
        2 か 3.
    weights:
        距離ごとの重み, shape (n, n). None なら等重み.

    Returns
    -------
    list[Anchor]
        推定座標を持つアンカー一覧.
    """
    dmat = np.array(distances, dtype=float)
    n = len(dmat)
    if dmat.shape != (n, n):
        raise ValueError("距離行列は正方行列")
    if n < dim + 1:
        raise ValueError(f"{dim} 次元には最低 {dim + 1} 台必要")
    ids = ids or [f"A{i}" for i in range(n)]

    # 対称化 (片側しか測れていないリンクはその値を採る).
    both = np.stack([dmat, dmat.T])
    ok = np.isfinite(both)
    cnt = ok.sum(axis=0)
    dsym = np.where(cnt > 0, np.where(ok, both, 0.0).sum(axis=0) / np.maximum(cnt, 1), np.nan)
    # 欠測リンク (遮蔽で測れなかった組) は, 既知リンクを辿った**最短経路長**で
    # 埋める. 平均値などで埋めると MDS が別の形に落ち, 後段の Gauss-Newton では
    # 抜け出せなくなる (欠測ペアは残差に入らないので直す力が働かない).
    dfill = np.where(np.isfinite(dsym), dsym, np.inf)
    np.fill_diagonal(dfill, 0.0)
    for k in range(n):
        dfill = np.minimum(dfill, dfill[:, k, None] + dfill[None, k, :])
    finite = dfill[np.isfinite(dfill)]
    dfill = np.where(np.isfinite(dfill), dfill, float(np.mean(finite)) if finite.size else 1.0)
    np.fill_diagonal(dfill, 0.0)

    # 古典的 MDS: B = -1/2 J D^2 J の固有分解.
    jmat = np.eye(n) - np.ones((n, n)) / n
    bmat = -0.5 * jmat @ (dfill**2) @ jmat
    evals, evecs = np.linalg.eigh(bmat)
    order = np.argsort(evals)[::-1][:dim]
    pts = evecs[:, order] * np.sqrt(np.maximum(evals[order], 0.0))

    # Gauss-Newton で距離残差を詰める.
    pairs = [(i, j) for i in range(n) for j in range(i + 1, n) if np.isfinite(dsym[i, j])]
    if pairs:
        w = np.ones(len(pairs)) if weights is None else np.array(
            [weights[i, j] for i, j in pairs], dtype=float
        )
        target = np.array([dsym[i, j] for i, j in pairs])
        lam = 1e-6
        x = pts.reshape(-1).copy()

        def residual(vec: np.ndarray) -> np.ndarray:
            p = vec.reshape(n, dim)
            return np.array([np.linalg.norm(p[i] - p[j]) for i, j in pairs]) - target

        cost = float(np.sum(w * residual(x) ** 2))
        for _ in range(max_iter):
            p = x.reshape(n, dim)
            res = residual(x)
            jac = np.zeros((len(pairs), n * dim))
            for k, (i, j) in enumerate(pairs):
                dv = p[i] - p[j]
                d = float(np.linalg.norm(dv))
                if d < _EPS:
                    continue
                u = dv / d
                jac[k, i * dim : (i + 1) * dim] = u
                jac[k, j * dim : (j + 1) * dim] = -u
            hmat = jac.T @ (w[:, None] * jac)
            grad = jac.T @ (w * res)
            # 座標系の自由度ぶん H は特異なので, LM 減衰で吸収する.
            try:
                step = np.linalg.solve(hmat + lam * np.eye(n * dim), grad)
            except np.linalg.LinAlgError:  # pragma: no cover
                break
            cand = x - step
            cost_new = float(np.sum(w * residual(cand) ** 2))
            if cost_new <= cost:
                x, cost = cand, cost_new
                lam = max(lam * 0.5, 1e-9)
                if float(np.linalg.norm(step)) < 1e-9:
                    break
            else:
                lam *= 4.0
                if lam > 1e8:
                    break
        pts = x.reshape(n, dim)

    pts = apply_gauge(pts, dim=dim)
    out = np.zeros((n, 3))
    out[:, :dim] = pts
    return [Anchor(id=ids[i], position=out[i]) for i in range(n)]
