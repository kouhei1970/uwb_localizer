/* 01 — いちばん短い形。距離 → 位置。
 *
 *   make examples && ./examples/01_snapshot
 *
 * マイコンでもこの形のまま動く。malloc も printf も、測位そのものには要らない
 * (ここでは結果を見せるために printf を使っているだけ)。
 */
#include "uwb_loc.h"

#include <stdio.h>

/* 部屋に固定したアンカー。座標は m、右手系で z が上。
 * **高さをばらしてある**のが大事 (平らに並べると鏡像解が出る)。 */
static const uwb_anchor anchors[6] = {
    /* id      x     y     z    有効 遅延  sigma0 sigma/m */
    {"A0", {0.2, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A1", {7.8, 0.2, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A2", {7.8, 5.8, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A3", {0.2, 5.8, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A4", {4.0, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A5", {4.0, 5.8, 0.3}, 1, 0.0, 0.08, 0.0}
};

int main(void)
{
    uwb_config cfg;
    uwb_meas   meas[6];
    uwb_fix    fix;
    int i;

    /* 1. 設定を既定値で埋めてから、必要なところだけ変える */
    uwb_config_init(&cfg, anchors, 6);

    /* 2. 測距値を詰める。anchor はアンカー配列の添字。
     *    sigma を 0 にするとアンカーの sigma0 から作られる。
     *    quality は「不明」を -1 で表す。 */
    {
        /* タグが (4.0, 3.0, 1.2) にいるときの距離に、数 cm の誤差を乗せたもの */
        static const double d[6] = {4.9003, 4.7852, 4.9103, 4.7952, 3.0663, 2.9111};
        for (i = 0; i < 6; ++i) {
            meas[i].anchor  = i;
            meas[i].value   = (uwb_real)d[i];
            meas[i].sigma   = (uwb_real)0;
            meas[i].quality = (uwb_real)-1;
        }
    }

    /* 3. 解く。Lv2 が屋内の既定 */
    if (!uwb_solve_lv2(&cfg, meas, 6, &fix) || !fix.ok) {
        printf("測位できなかった (届いた %d 本)\n", fix.n_total);
        return 1;
    }

    printf("位置    %.3f, %.3f, %.3f  [m]\n",
           (double)fix.p[0], (double)fix.p[1], (double)fix.p[2]);
    printf("不確かさ %.3f m   GDOP %.2f   残差 %.3f m\n",
           (double)fix.sigma, (double)fix.gdop, (double)fix.residual_rms);
    printf("使った本数 %d / %d\n", fix.n_used, fix.n_total);
    if (fix.excluded) {
        printf("外れ値として落とした観測: ");
        for (i = 0; i < 6; ++i)
            if (fix.excluded & (1UL << i)) printf("%s ", anchors[i].id);
        printf("\n");
    }
    if (fix.ambiguous)
        printf("警告: 鏡像解かもしれない (高さが信用できない)\n");

    /* 配置の良し悪しは置く前に分かる */
    {
        uwb_real center[3] = {(uwb_real)4.0, (uwb_real)3.0, (uwb_real)1.2};
        printf("\n部屋の中央の GDOP %.2f / CRLB %.3f m\n",
               (double)uwb_gdop_at(&cfg, center), (double)uwb_crlb_at(&cfg, center));
    }
    return 0;
}
