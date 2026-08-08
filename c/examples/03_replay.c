/* 03 — 観測列を読んで測位し、CSV を吐く (ホスト用の突き合わせ道具)
 *
 * **マイコン用ではない。** Python 版と C 版に同じ観測を食わせて `Fix` を
 * 突き合わせるための入り口。移植の照合は、式を読み比べるよりこれが早い。
 *
 *   ./examples/03_replay input.txt Lv2 > c_fixes.csv
 *
 * 入力は 1 行 1 レコードの素朴な形。JSON を C で読む必要はない
 * (ライブラリ本体に文字列処理を持ち込みたくないので、あえてこの形)。
 *
 *   A <id> <x> <y> <z> <delay> <sigma0> <sigma_per_m>   アンカー 1 台
 *   M <anchor_index> <value> <sigma> <quality>          観測 1 本
 *   E <t>                                               エポック終わり -> 測位
 *
 * quality は「不明」を -1 で表す。
 */
#include "uwb_loc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    static uwb_anchor anchors[UWB_MAX_ANCHORS];
    static char line[4096];
    uwb_config cfg;
    uwb_ekf ekf;
    uwb_meas meas[UWB_MAX_MEAS];
    uwb_fix fix;
    FILE *f;
    const char *level = (argc >= 3) ? argv[2] : "Lv2";
    int n_anchors = 0, n = 0, use_ekf, started = 0;

    if (argc < 2) {
        fprintf(stderr, "使い方: %s input.txt [Lv0|Lv1|Lv2|Lv3]\n", argv[0]);
        return 2;
    }
    use_ekf = (strcmp(level, "Lv3") == 0);

    f = fopen(argv[1], "r");
    if (!f) { fprintf(stderr, "開けない: %s\n", argv[1]); return 2; }

    printf("t,ok,x,y,z,sigma,n_used,n_total,residual_rms,gdop,ambiguous\n");

    while (fgets(line, (int)sizeof(line), f)) {
        if (line[0] == 'A') {
            uwb_anchor *a;
            char id[UWB_ID_LEN];
            double x, y, z, d, s0, sp;
            if (n_anchors >= UWB_MAX_ANCHORS) continue;
            if (sscanf(line + 1, "%14s %lf %lf %lf %lf %lf %lf",
                       id, &x, &y, &z, &d, &s0, &sp) != 7) continue;
            a = &anchors[n_anchors++];
            memset(a->id, 0, UWB_ID_LEN);
            memcpy(a->id, id, strlen(id) < UWB_ID_LEN ? strlen(id) : UWB_ID_LEN - 1);
            a->p[0] = (uwb_real)x; a->p[1] = (uwb_real)y; a->p[2] = (uwb_real)z;
            a->enabled = 1;
            a->antenna_delay_m = (uwb_real)d;
            a->sigma0 = (uwb_real)s0;
            a->sigma_per_m = (uwb_real)sp;
        } else if (line[0] == 'M') {
            int idx;
            double v, s, q;
            if (n >= UWB_MAX_MEAS) continue;
            if (sscanf(line + 1, "%d %lf %lf %lf", &idx, &v, &s, &q) != 4) continue;
            meas[n].anchor = idx;
            meas[n].value = (uwb_real)v;
            meas[n].sigma = (uwb_real)s;
            meas[n].quality = (uwb_real)q;
            ++n;
        } else if (line[0] == 'E') {
            double t = 0.0;
            if (sscanf(line + 1, "%lf", &t) != 1) { n = 0; continue; }
            if (!started) {
                uwb_config_init(&cfg, anchors, n_anchors);
                if (use_ekf) uwb_ekf_init(&ekf, &cfg, UWB_MOTION_CV, (uwb_real)1.0);
                started = 1;
            }
            if (use_ekf)                              uwb_ekf_update(&ekf, (uwb_real)t, meas, n, &fix);
            else if (strcmp(level, "Lv0") == 0)       uwb_solve_lv0(&cfg, meas, n, &fix);
            else if (strcmp(level, "Lv1") == 0)       uwb_solve_lv1(&cfg, meas, n, &fix);
            else                                      uwb_solve_lv2(&cfg, meas, n, &fix);

            printf("%.9f,%d,%.12g,%.12g,%.12g,%.12g,%d,%d,%.12g,%.12g,%d\n",
                   t, fix.ok, (double)fix.p[0], (double)fix.p[1], (double)fix.p[2],
                   (double)fix.sigma, fix.n_used, fix.n_total,
                   (double)fix.residual_rms, (double)fix.gdop, fix.ambiguous);
            n = 0;
        }
    }
    fclose(f);
    return 0;
}
