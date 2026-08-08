/* 02 — 動くタグを追う (Lv3 EKF)。
 *
 *   make examples && ./examples/02_tracking
 *
 * 密結合 EKF なので、**1 エポックに 1 本しか測距が無くても更新できる**。
 * 順繰りに 1 台ずつ測距するモジュール (RYUW122 など) と相性が良い。
 * ただし立ち上げだけは球面 1 枚では位置が決まらないので、貯まるのを待つ。
 */
#include "uwb_loc.h"

#include <math.h>
#include <stdio.h>

static const uwb_anchor anchors[6] = {
    {"A0", {0.2, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A1", {7.8, 0.2, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A2", {7.8, 5.8, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A3", {0.2, 5.8, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A4", {4.0, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A5", {4.0, 5.8, 0.3}, 1, 0.0, 0.08, 0.0}
};

/* 決定的な擬似乱数 (実機では要らない。ここは観測を作るためだけ) */
static unsigned long seed = 2024UL;
static double urand(void)
{
    seed = seed * 1103515245UL + 12345UL;
    return (double)((seed >> 16) & 0x7fff) / 32767.0;
}
static double noise(double s)
{
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return s * sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

int main(void)
{
    uwb_config cfg;
    uwb_ekf    ekf;
    uwb_fix    fix;
    int step;
    double err_sum = 0.0;
    int err_n = 0;

    uwb_config_init(&cfg, anchors, 6);

    /* sigma_a は「どれくらい機敏に動くと思うか」。
     * 大きいと追従が速くなるがノイズを拾い、小さいと滑らかだが遅れる。
     * 歩行者や台車なら 0.2-0.5、機敏なドローンなら 2-5 が目安。 */
    uwb_ekf_init(&ekf, &cfg, UWB_MOTION_CV, (uwb_real)0.5);

    printf("  t [s]     推定位置 [m]              真値との差\n");
    for (step = 0; step < 200; ++step) {
        uwb_real t = (uwb_real)(step * 0.1);
        double truth[3];
        uwb_meas one;
        int a = step % 6;                 /* 1 台ずつ順に測距する想定 */

        truth[0] = 4.0 + 2.0 * cos(step * 0.03);
        truth[1] = 3.0 + 2.0 * sin(step * 0.03);
        truth[2] = 1.2;

        {
            double dx = truth[0] - (double)anchors[a].p[0];
            double dy = truth[1] - (double)anchors[a].p[1];
            double dz = truth[2] - (double)anchors[a].p[2];
            one.anchor  = a;
            one.value   = (uwb_real)(sqrt(dx * dx + dy * dy + dz * dz) + noise(0.06));
            one.sigma   = (uwb_real)0;
            one.quality = (uwb_real)-1;
        }

        /* 1 本だけ渡す。立ち上がるまでは ok=0 が返る */
        uwb_ekf_update(&ekf, t, &one, 1, &fix);

        if (fix.ok) {
            double dx = (double)fix.p[0] - truth[0];
            double dy = (double)fix.p[1] - truth[1];
            double dz = (double)fix.p[2] - truth[2];
            double e = sqrt(dx * dx + dy * dy + dz * dz);
            if (step > 30) { err_sum += e; ++err_n; }
            if (step % 40 == 0)
                printf("  %5.1f    %6.2f %6.2f %6.2f        %.3f m\n",
                       (double)t, (double)fix.p[0], (double)fix.p[1],
                       (double)fix.p[2], e);
        } else if (step % 40 == 0) {
            printf("  %5.1f    (立ち上げ中 — 測距が貯まるのを待っている)\n", (double)t);
        }
    }

    printf("\n1 エポック 1 本だけで追従。落ち着いてからの平均誤差 %.3f m (%d 回)\n",
           err_sum / (err_n ? err_n : 1), err_n);
    printf("速度も出る: %.2f %.2f %.2f [m/s]\n",
           (double)fix.v[0], (double)fix.v[1], (double)fix.v[2]);
    return 0;
}
