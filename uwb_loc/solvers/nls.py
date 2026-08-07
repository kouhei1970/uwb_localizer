"""反復ソルバ — Lv0 / Lv1 / Lv2 のスナップショット測位.

* :class:`Lv0Trilateration` — LLS 閉形式のみ. 反復なし
* :class:`Lv1WeightedNLS` — Beck 初期解 + Gauss-Newton(LM) + χ² ゲート
* :class:`Lv2RobustNLS` — Lv1 + Huber-IRLS + 片側損失 (+RANSAC)

いずれも 1 エポックの観測だけで解くステートレスな推定器 (直前の推定を
初期値に使うことはある). 時系列の平滑化は Lv3 の EKF が担当する.

scipy には依存しない. 正規方程式と LM 減衰を自前で書いてあるので,
そのまま C に移せる (行列は最大 3x3, 一般化固有値も 4x4 まで).
"""

from __future__ import annotations

import numpy as np

from ..geometry import gdop_from_jacobian
from ..types import Fix, MeasKind, Measurement, MeasurementBatch
from .base import PositionEstimator, SolveConfig
from .closed_form import beck_gtrs, chan_tdoa, lls_trilateration
from .robust import RobustLoss, physical_gate, ransac_ranges, robust_weights

__all__ = ["Lv0Trilateration", "Lv1WeightedNLS", "Lv2RobustNLS", "NlsResult", "solve_nls"]


class NlsResult:
    """反復ソルバの生の結果."""

    def __init__(
        self,
        position: np.ndarray,
        covariance: np.ndarray,
        *,
        ok: bool,
        iterations: int,
        residual_rms: float,
        weights: np.ndarray,
        gdop: float,
    ) -> None:
        self.position = position
        self.covariance = covariance
        self.ok = ok
        self.iterations = iterations
        self.residual_rms = residual_rms
        self.weights = weights
        self.gdop = gdop


def solve_nls(
    model,
    meas: list[Measurement],
    p0: np.ndarray,
    config: SolveConfig,
    *,
    loss: RobustLoss | None = None,
) -> NlsResult:
    """重み付き非線形最小二乗 (Gauss-Newton + Levenberg-Marquardt 減衰).

    ``J^T W J dp = J^T W e`` を解いて位置を更新する. ``W`` は
    ``1/sigma^2`` にロバスト損失の倍率を掛けたもの (IRLS: 反復のたびに
    残差から重みを作り直す).

    LM の減衰を入れてあるのは, アンカー配置が悪くて ``J^T W J`` がほとんど
    特異になる状況 (アンカーが一直線, タグがアンカー平面上) でも
    発散せずに止まるようにするため.
    """
    mask = config.free_mask
    nfree = int(mask.sum())
    p = config.project(np.asarray(p0, dtype=float))
    lam = 1e-6
    weights = np.ones(len(meas))
    iterations = 0

    def cost(pos: np.ndarray) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
        e, jac, sig = model.assemble(pos, meas)
        w = 1.0 / np.maximum(sig, 1e-6) ** 2
        if loss is not None:
            w = w * robust_weights(e, sig, loss)
        return float(np.sum(w * e**2)), e, jac, w

    c, e, jac, w = cost(p)
    if len(meas) == 0:
        return NlsResult(p, np.full((3, 3), np.nan), ok=False, iterations=0,
                         residual_rms=float("nan"), weights=weights, gdop=float("nan"))

    for iterations in range(1, config.max_iter + 1):
        jm = jac[:, mask]
        hmat = jm.T @ (w[:, None] * jm)
        grad = jm.T @ (w * e)
        try:
            step = np.linalg.solve(hmat + lam * np.eye(nfree) * max(np.trace(hmat) / nfree, 1e-9),
                                   grad)
        except np.linalg.LinAlgError:
            break

        cand = p.copy()
        cand[mask] += step
        cand = config.project(cand)
        c_new, e_new, jac_new, w_new = cost(cand)

        if c_new <= c:
            p, c, e, jac, w = cand, c_new, e_new, jac_new, w_new
            lam = max(lam * 0.3, 1e-9)
            if float(np.linalg.norm(step)) < config.tol:
                break
        else:
            lam *= 4.0
            if lam > 1e8:
                break

    jm = jac[:, mask]
    hmat = jm.T @ (w[:, None] * jm)
    cov = np.full((3, 3), np.nan)
    ok = True
    try:
        cov_free = np.linalg.inv(hmat)
        idx = np.where(mask)[0]
        cov = np.zeros((3, 3))
        cov[np.ix_(idx, idx)] = cov_free
    except np.linalg.LinAlgError:
        ok = False

    wsum = float(np.sum(w))
    rms = float(np.sqrt(np.sum(w * e**2) / wsum)) if wsum > 0 else float("nan")
    return NlsResult(
        p,
        cov,
        ok=ok and bool(np.all(np.isfinite(p))),
        iterations=iterations,
        residual_rms=rms,
        weights=w,
        gdop=gdop_from_jacobian(jac, mask),
    )


# --------------------------------------------------------------------------- 初期解


def initial_guess(model, meas: list[Measurement], config: SolveConfig) -> np.ndarray | None:
    """閉形式で初期位置を作る.

    距離観測があれば Beck の厳密解, なければ TDoA の Chan 法を使う.
    2 次元で解く設定のときは, 高さ差の分を測距値から抜いてから
    2 次元の問題として閉形式にかける.
    """
    ranges = [m for m in meas if m.kind is MeasKind.RANGE]
    dim = config.dim

    if len(ranges) >= dim + 1:
        pos = np.array([model.anchors[m.anchor_id].position for m in ranges])
        r = np.array([model.corrected_value(m) for m in ranges])
        sig = np.array([model.sigma(m, max(v, 0.1)) for m, v in zip(ranges, r)])
        w = 1.0 / np.maximum(sig, 1e-6) ** 2

        if dim == 2:
            dz = config.z_fixed - pos[:, 2]
            r2 = np.maximum(r**2 - dz**2, 1e-4)
            p2 = beck_gtrs(pos[:, :2], np.sqrt(r2), w)
            if p2 is None:
                p2 = lls_trilateration(pos[:, :2], np.sqrt(r2), w)
            if p2 is not None:
                return np.array([p2[0], p2[1], config.z_fixed])
        else:
            p3 = beck_gtrs(pos, r, w)
            if p3 is None:
                p3 = lls_trilateration(pos, r, w)
            if p3 is not None:
                return config.project(p3)

    tdoa = [m for m in meas if m.kind is MeasKind.TDOA and m.ref_anchor_id]
    if tdoa:
        ref_id = tdoa[0].ref_anchor_id
        ids = [ref_id] + [m.anchor_id for m in tdoa]
        pos = np.array([model.anchors[i].position for i in ids])
        d = np.array([0.0] + [model.corrected_value(m) for m in tdoa])
        sig = np.array([1.0] + [model.sigma(m, 10.0) for m in tdoa])
        w = 1.0 / np.maximum(sig, 1e-6) ** 2
        cols = slice(0, 2) if dim == 2 else slice(0, 3)
        p = chan_tdoa(pos[:, cols], d, 0, w)
        if p is not None:
            out = np.zeros(3)
            out[cols] = p
            if dim == 2:
                out[2] = config.z_fixed
            return config.project(out)

    # 最後の手段: 見えているアンカーの重心.
    ids = {m.anchor_id for m in meas} | {m.ref_anchor_id for m in meas if m.ref_anchor_id}
    pts = [model.anchors[i].position for i in ids if i in model.anchors]
    if not pts:
        return None
    return config.project(np.mean(pts, axis=0))


# --------------------------------------------------------------------------- Lv0


class Lv0Trilateration(PositionEstimator):
    """Lv0 — 線形最小二乗による三辺測量.

    反復も初期値も不要で最速. 精度は Lv1 以上に劣るが, 配線・座標系・
    単位の確認にはこれがいちばん使いやすい (おかしければ即座に破綻する).
    """

    level = "Lv0"

    def update(self, batch: MeasurementBatch) -> Fix:
        meas = [m for m in self._usable(batch) if m.kind is MeasKind.RANGE]
        n_total = len(batch)
        if len(meas) < self.config.dim + 1:
            return Fix.failed(batch.t, n_total, self.level)

        pos = np.array([self.model.anchors[m.anchor_id].position for m in meas])
        r = np.array([self.model.corrected_value(m) for m in meas])
        sig = np.array([self.model.sigma(m, max(v, 0.1)) for m, v in zip(meas, r)])
        w = 1.0 / np.maximum(sig, 1e-6) ** 2

        if self.config.dim == 2:
            dz = self.config.z_fixed - pos[:, 2]
            p2 = lls_trilateration(pos[:, :2], np.sqrt(np.maximum(r**2 - dz**2, 1e-4)), w)
            p = None if p2 is None else np.array([p2[0], p2[1], self.config.z_fixed])
        else:
            p = lls_trilateration(pos, r, w)

        if p is None:
            return Fix.failed(batch.t, n_total, self.level)
        p = self.config.project(p)

        e, jac, _ = self.model.assemble(p, meas)
        cov = _cov_from_jacobian(jac, sig, self.config.free_mask)
        return Fix(
            position=p,
            covariance=cov,
            t=batch.t,
            ok=True,
            n_used=len(meas),
            n_total=n_total,
            residual_rms=float(np.sqrt(np.mean(e**2))),
            gdop=gdop_from_jacobian(jac, self.config.free_mask),
            level=self.level,
        )


def _cov_from_jacobian(jac: np.ndarray, sigma: np.ndarray, mask: np.ndarray) -> np.ndarray:
    jm = jac[:, mask]
    w = 1.0 / np.maximum(sigma, 1e-6) ** 2
    try:
        inv = np.linalg.inv(jm.T @ (w[:, None] * jm))
    except np.linalg.LinAlgError:
        return np.full((3, 3), np.nan)
    idx = np.where(mask)[0]
    cov = np.zeros((3, 3))
    cov[np.ix_(idx, idx)] = inv
    return cov


# --------------------------------------------------------------------------- Lv1 / Lv2


class Lv1WeightedNLS(PositionEstimator):
    """Lv1 — 重み付き非線形最小二乗 + χ² ゲート.

    観測誤差がガウスなら最尤推定になる. 見通しの良い環境ではこれで十分.

    Parameters
    ----------
    chi2_threshold:
        標準化残差のしきい値 (σ の倍数). これを超えた観測を落として
        1 度だけ解き直す. None なら外れ値除去なし.
    """

    level = "Lv1"

    def __init__(
        self,
        anchors,
        config: SolveConfig | None = None,
        *,
        chi2_threshold: float | None = 3.5,
        loss: RobustLoss | None = None,
        use_physical_gate: bool = True,
        use_ransac: bool = False,
        ransac_trigger: float = 3.0,
        warm_start: bool = True,
    ) -> None:
        super().__init__(anchors, config)
        self.chi2_threshold = chi2_threshold
        self.loss = loss
        self.use_physical_gate = use_physical_gate
        self.use_ransac = use_ransac
        self.ransac_trigger = ransac_trigger
        self.warm_start = warm_start
        self._last: np.ndarray | None = None

    def reset(self) -> None:
        self._last = None

    # ------------------------------------------------------------------

    def update(self, batch: MeasurementBatch) -> Fix:
        n_total = len(batch)
        meas = self._usable(batch)
        excluded: list[str] = []

        if self.use_physical_gate:
            meas, dropped = physical_gate(meas, self.model.anchors)
            excluded.extend(dropped)

        if len(meas) < self.config.dim + 1:
            return Fix.failed(batch.t, n_total, self.level)

        p0 = self._last if (self.warm_start and self._last is not None) else None
        if p0 is None:
            p0 = initial_guess(self.model, meas, self.config)
        if p0 is None:
            return Fix.failed(batch.t, n_total, self.level)

        res, meas, gated = self._solve_gated(meas, p0)
        excluded.extend(gated)

        # RANSAC は**保険**として, 通常の解き方が明らかに失敗したときだけ走らせる.
        # 常時走らせると, 最小構成 (4 本) から作る仮解自体の誤差でまともな観測まで
        # 落としてしまい, かえって悪化する.
        if self.use_ransac and self._badly_fitted(res, meas):
            kept, dropped = self._ransac_filter(meas)
            if dropped and len(kept) >= self.config.dim + 1:
                p0b = initial_guess(self.model, kept, self.config) or res.position
                alt, kept2, gated2 = self._solve_gated(kept, p0b)
                if alt.ok and self._normalized_residual(alt, kept2) < self._normalized_residual(
                    res, meas
                ):
                    res, meas = alt, kept2
                    excluded.extend(dropped)
                    excluded.extend(gated2)

        if not res.ok:
            # 初期値からやり直しても駄目なら失敗として返す.
            p0b = initial_guess(self.model, meas, self.config)
            if p0b is not None:
                res = solve_nls(self.model, meas, p0b, self.config, loss=self.loss)
            if not res.ok:
                self._last = None
                return Fix.failed(batch.t, n_total, self.level)

        self._last = res.position.copy()
        return Fix(
            position=res.position,
            covariance=res.covariance,
            t=batch.t,
            ok=True,
            n_used=len(meas),
            n_total=n_total,
            residual_rms=res.residual_rms,
            gdop=res.gdop,
            excluded=excluded,
            iterations=res.iterations,
            level=self.level,
        )

    # ------------------------------------------------------------------

    def _solve_gated(
        self, meas: list[Measurement], p0: np.ndarray
    ) -> tuple[NlsResult, list[Measurement], list[str]]:
        """解いてから χ² ゲートで外れ値を落とし, 一度だけ解き直す."""
        res = solve_nls(self.model, meas, p0, self.config, loss=self.loss)
        excluded: list[str] = []
        if self.chi2_threshold is not None and len(meas) > self.config.dim + 1:
            e, _, sig = self.model.assemble(res.position, meas)
            bad = np.abs(e) > self.chi2_threshold * np.maximum(sig, 1e-6)
            if bad.any() and int((~bad).sum()) >= self.config.dim + 1:
                excluded = [m.anchor_id for m, b in zip(meas, bad) if b]
                meas = [m for m, b in zip(meas, bad) if not b]
                res = solve_nls(self.model, meas, res.position, self.config, loss=self.loss)
        return res, meas, excluded

    def _normalized_residual(self, res: NlsResult, meas: list[Measurement]) -> float:
        """σ で規格化した残差 RMS. 1 前後なら誤差モデルどおり."""
        if not meas or not res.ok:
            return float("inf")
        e, _, sig = self.model.assemble(res.position, meas)
        return float(np.sqrt(np.mean((e / np.maximum(sig, 1e-6)) ** 2)))

    def _badly_fitted(self, res: NlsResult, meas: list[Measurement]) -> bool:
        return self._normalized_residual(res, meas) > self.ransac_trigger

    def _ransac_filter(self, meas: list[Measurement]) -> tuple[list[Measurement], list[str]]:
        ranges = [m for m in meas if m.kind is MeasKind.RANGE]
        others = [m for m in meas if m.kind is not MeasKind.RANGE]
        if len(ranges) < self.config.dim + 3:
            return meas, []

        pos = np.array([self.model.anchors[m.anchor_id].position for m in ranges])
        r = np.array([self.model.corrected_value(m) for m in ranges])
        sig = np.array([self.model.sigma(m, max(v, 0.1)) for m, v in zip(ranges, r)])

        if self.config.dim == 2:
            dz = self.config.z_fixed - pos[:, 2]
            pos2, r2 = pos[:, :2], np.sqrt(np.maximum(r**2 - dz**2, 1e-4))
        else:
            pos2, r2 = pos, r

        # しきい値を σ の 4 倍と広めに取るのは, 最小構成から作った仮解自体が
        # 誤差を持つため. 狭くすると正しい観測まで外れ値扱いになる.
        mask = ransac_ranges(pos2, r2, sig, beck_gtrs, dim=self.config.dim, threshold=4.0)
        if mask is None:
            return meas, []
        kept = [m for m, keep in zip(ranges, mask) if keep]
        dropped = [m.anchor_id for m, keep in zip(ranges, mask) if not keep]
        return kept + others, dropped


class Lv2RobustNLS(Lv1WeightedNLS):
    """Lv2 — Beck 初期解 + Huber-IRLS + 片側損失 (+RANSAC).

    NLOS のある屋内での既定. Lv1 との違いは重みの作り方だけで,
    構造は同じ.
    """

    level = "Lv2"

    def __init__(self, anchors, config: SolveConfig | None = None, **kw) -> None:
        kw.setdefault("loss", RobustLoss(kind="huber", one_sided=True))
        kw.setdefault("use_ransac", True)
        kw.setdefault("chi2_threshold", 4.0)
        # ロバスト重みが効くので, 前回値ではなく毎回閉形式から解き直す.
        # 直前の推定に引きずられて外れ値に固着するのを避けるため.
        kw.setdefault("warm_start", False)
        super().__init__(anchors, config, **kw)
