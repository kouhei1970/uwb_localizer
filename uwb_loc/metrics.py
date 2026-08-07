"""評価指標 — 「で, 何 cm 出るのか」を答えるための集計.

RMSE だけ見ていると, たまに出る大外れ (NLOS で 2 m 飛ぶ, など) を
見落とす. 実運用で効くのは分布の裾なので, CEP50/CEP95 と
可用性 (測位できた割合) を必ず併記する.
"""

from __future__ import annotations

import numpy as np

__all__ = ["error_series", "error_stats", "error_cdf"]


def error_series(truth: np.ndarray, est: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """各時刻の 3 次元誤差と水平誤差 [m].

    ``est`` に NaN (測位失敗) が含まれていてもよい. その行は NaN のまま返す.
    """
    truth = np.atleast_2d(np.asarray(truth, dtype=float))
    est = np.atleast_2d(np.asarray(est, dtype=float))
    if truth.shape != est.shape:
        raise ValueError(f"形が違う: truth={truth.shape}, est={est.shape}")
    d = est - truth
    return np.linalg.norm(d, axis=1), np.linalg.norm(d[:, :2], axis=1)


def error_stats(truth: np.ndarray, est: np.ndarray) -> dict[str, float]:
    """誤差統計をまとめて返す.

    Returns
    -------
    dict
        ``availability`` 測位成功率, ``rmse_3d`` / ``rmse_2d`` / 軸別 RMSE,
        ``cep50`` / ``cep95`` (水平誤差の中央値と 95 パーセンタイル),
        ``p95_3d``, ``max_3d``, ``bias_*`` (系統誤差).
    """
    truth = np.atleast_2d(np.asarray(truth, dtype=float))
    est = np.atleast_2d(np.asarray(est, dtype=float))
    n = len(truth)
    ok = np.all(np.isfinite(est), axis=1)
    out: dict[str, float] = {"n": float(n), "availability": float(ok.sum()) / n if n else 0.0}
    if ok.sum() == 0:
        for k in ("rmse_3d", "rmse_2d", "rmse_x", "rmse_y", "rmse_z",
                  "cep50", "cep95", "p95_3d", "max_3d", "bias_x", "bias_y", "bias_z"):
            out[k] = float("nan")
        return out

    d = est[ok] - truth[ok]
    e3, e2 = np.linalg.norm(d, axis=1), np.linalg.norm(d[:, :2], axis=1)
    out.update(
        rmse_3d=float(np.sqrt(np.mean(e3**2))),
        rmse_2d=float(np.sqrt(np.mean(e2**2))),
        rmse_x=float(np.sqrt(np.mean(d[:, 0] ** 2))),
        rmse_y=float(np.sqrt(np.mean(d[:, 1] ** 2))),
        rmse_z=float(np.sqrt(np.mean(d[:, 2] ** 2))),
        cep50=float(np.percentile(e2, 50)),
        cep95=float(np.percentile(e2, 95)),
        p95_3d=float(np.percentile(e3, 95)),
        max_3d=float(np.max(e3)),
        bias_x=float(np.mean(d[:, 0])),
        bias_y=float(np.mean(d[:, 1])),
        bias_z=float(np.mean(d[:, 2])),
    )
    return out


def error_cdf(errors: np.ndarray, n: int = 100) -> tuple[np.ndarray, np.ndarray]:
    """誤差の累積分布.

    Returns
    -------
    (x, p)
        ``x`` 誤差 [m], ``p`` その値以下になる確率.
    """
    e = np.asarray(errors, dtype=float)
    e = np.sort(e[np.isfinite(e)])
    if e.size == 0:
        return np.zeros(0), np.zeros(0)
    idx = np.unique(np.linspace(0, e.size - 1, min(n, e.size)).astype(int))
    return e[idx], (idx + 1) / e.size
