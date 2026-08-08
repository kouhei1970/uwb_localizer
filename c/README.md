# uwb_loc C 版 — マイコンで測位する

> **この文書**: C 版の使い方と移植のしかた。アルゴリズムの中身は
> [../docs/UWB_ALGORITHMS.md](../docs/UWB_ALGORITHMS.md)、
> Python 版の使い方は [../docs/TUTORIAL.md](../docs/TUTORIAL.md)。
> → [ドキュメント一覧](../docs/README.md)

Python 版の**測位部分だけ**を C99 に移したもの。マイコンに載せることを
前提にしている。

- **malloc を呼ばない。** 作業領域はすべて呼び出し側の構造体かスタック
- **依存は libm の `sqrt` だけ。** printf も文字列処理もライブラリ本体には無い
- **C99。** コンパイラ拡張を使っていない
- **Python 版と数値が一致する** — 同じ入力で 1e-11 m まで合うことを毎回確認している

---

## 1 分で動かす

```bash
cd c
make            # libuwbloc.a ができる
make test       # 53 件のテスト
make examples   # サンプル 3 本
./examples/01_snapshot
```

```c
#include "uwb_loc.h"

/* 座標が分かっている固定点。高さをばらすこと (平らに並べると鏡像解が出る) */
static const uwb_anchor anchors[6] = {
    /* id      x     y     z    有効 遅延 sigma0 sigma/m */
    {"A0", {0.2, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A1", {7.8, 0.2, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A2", {7.8, 5.8, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A3", {0.2, 5.8, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A4", {4.0, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A5", {4.0, 5.8, 0.3}, 1, 0.0, 0.08, 0.0}
};

uwb_config cfg;
uwb_config_init(&cfg, anchors, 6);          /* 既定値で埋める */

uwb_meas meas[6];
for (int i = 0; i < 6; ++i) {
    meas[i].anchor  = i;                    /* アンカー配列の添字 */
    meas[i].value   = range_m[i];           /* 測距値 [m] */
    meas[i].sigma   = 0;                    /* 0 ならアンカーの設定から作る */
    meas[i].quality = -1;                   /* 不明は -1 */
}

uwb_fix fix;
if (uwb_solve_lv2(&cfg, meas, 6, &fix) && fix.ok) {
    use(fix.p[0], fix.p[1], fix.p[2]);
}
```

**`fix.ok` を必ず見ること。** 0 のとき `fix.p` は意味を持たない。

---

## 何が入っていて、何が入っていないか

| | C 版 | 備考 |
|---|---|---|
| Lv0 三辺測量 (閉形式) | **あり** | `uwb_solve_lv0` |
| Lv1 重み付き非線形最小二乗 | **あり** | `uwb_solve_lv1` |
| Lv2 Beck + Huber ロバスト化 | **あり** | `uwb_solve_lv2` — **既定はこれ** |
| Lv3 密結合 EKF | **あり** | `uwb_ekf_*` |
| GDOP / CRLB | **あり** | `uwb_gdop_at` / `uwb_crlb_at` |
| 同一平面の検出・鏡像解の処理 | **あり** | `uwb_anchors_coplanar`、`fix.ambiguous` |
| アンテナ遅延・品質値の反映 | **あり** | `uwb_anchor.antenna_delay_m` / `uwb_meas.quality` |
| RANSAC | **無い** | 組込みには重い。後述の「Python 版との違い」 |
| HAL (シリアル・BLE など) | **無い** | 距離を作るのはあなたのファーム |
| シミュレータ・UI・CLI | **無い** | ホスト側 (Python 版) の仕事 |
| 自己測量 (相互測距 → 座標) | **無い** | 現場で 1 回やるだけなのでホストで |

**設計方針**: マイコンに載せるのは「距離 → 位置」だけ。設営・評価・記録は
PC でやる方が速いし、その方がマイコンの ROM/RAM を食わない。

---

## 大きさ (実測)

`-Os`、x86-64 でのビルド結果。**ARM は手元に無いので測っていない**が、
Cortex-M ならこれより小さくなるのが普通。

| | 既定 (double, 16 台/32 本) | 節約 (float, 8 台/8 本) |
|---|---|---|
| ROM (text) | 21.5 KB | 20.6 KB |
| `uwb_config` | 112 B | 64 B |
| `uwb_fix` | 176 B | 104 B |
| `uwb_ekf` (Lv3 を使うときだけ) | 2096 B | 592 B |
| **最大スタック** (Lv2 の呼び出し連鎖) | **6.6 KB** | **1.5 KB** |

スタックが効くので、RAM が厳しいなら上限を絞る:

```c
/* コンパイル時に渡す */
-DUWB_MAX_ANCHORS=8 -DUWB_MAX_MEAS=8 -DUWB_ID_LEN=1 -DUWB_USE_FLOAT
```

- `UWB_MAX_ANCHORS` / `UWB_MAX_MEAS` — スタックとバッファに直接効く
- `UWB_ID_LEN=1` — 文字列 ID を使わない (添字だけで済ませる) なら
- `UWB_USE_FLOAT` — **まず double で動かしてから**切り替えること。
  float でも全テストは通るが、有効桁が 7 桁しかないので Beck 法の二分法と
  共分散が荒くなる

`make size` でセクションごとのサイズが出る。

---

## Python 版との違い

移植の忠実さは `tools/crossval.py` が毎回確かめている。**5 シナリオ ×
Lv0〜Lv3 の 20 通りで、位置が 1e-11 m 以内で一致**する。

```bash
make -C c examples
python tools/crossval.py            # 既定は 1e-6 m 判定
python tools/crossval.py --tol 1e-9 # もっと厳しく
```

```
シナリオ                Lv     一致       最大差 [m]     測位数
静止・無雑音・8台          Lv0     OK      2.92e-11      60
...
8の字・NLOS 30%・8台    Lv2     OK      7.88e-12      60
円・最小構成 4台          Lv3     OK      7.52e-12      17
すべて一致 (許容 1e-09 m)
```

一致させるために、突き合わせでは Python 側の設定を C に合わせている。
**既定のまま使うと違いが出るのは次の 2 点だけ**:

| | Python 版 | C 版 | なぜ |
|---|---|---|---|
| **RANSAC** | Lv2 の保険として持つ | **持たない** | 最小構成の仮解を何度も解き直すので、マイコンには重い。NLOS が過半という状況でしか効かない |
| **warm start** | Lv1 は前回位置を初期値にする | **毎回初期値から解く** | スナップショット解を状態なし (純関数) にしたかった。呼ぶ側が状態を持たなくて済む |

RANSAC が要るほど NLOS が多い現場なら、**アンカーを増やす方が確実**
(→ [../README.md の「何台買えばいいか」](../README.md))。

そのほか、`chi2_k` の既定は Python と揃えてある (Lv1 は 3.5、Lv2 は 4.0)。
`uwb_config.chi2_k` に正の値を入れれば上書きできる。

---

## 移植のしかた

### 必要な線形代数は 3 つだけ

全部 `src/uwb_linalg.c` に入っている (200 行ほど)。差し替えたければここだけ見る。

| 用途 | 大きさ | 実装 |
|---|---|---|
| 正規方程式 `(JᵀWJ)Δ = JᵀWe` | 3×3 | 部分ピボット付き LU |
| Beck 法の λ 下限 | 4×4 | コレスキー + 対称 Jacobi 固有値 |
| EKF の共分散 | 6×6 (CA なら 9×9) | **逆行列は不要**。逐次スカラー更新 |

EKF の更新はイノベーション分散 `S` がスカラーなので、**行列の逆行列を
一度も計算しない**。除算だけで済む。

### 実機に載せる手順

1. **まず PC で通す。** `make test` と `python tools/crossval.py` が通ることを確認
2. **`src/` と `include/` をコピー**してビルドに足す。ファイルは 5 つだけ
   ```
   include/uwb_loc.h
   src/uwb_linalg.{c,h}  src/uwb_internal.h
   src/uwb_model.c  src/uwb_closed_form.c  src/uwb_nls.c  src/uwb_ekf.c
   ```
3. **上限を絞る** (`UWB_MAX_ANCHORS` など)。スタックが足りるか確かめる
4. **ファームの測距値を `uwb_meas` に詰める。** ここだけがあなたの仕事
5. **合っているか確かめる。** 同じ観測列を PC の `03_replay` にも食わせて
   `Fix` を突き合わせる ↓

### 実機と PC を突き合わせる

`examples/03_replay` は素朴な行形式を読む。マイコンから測距値をダンプして
同じ形式にすれば、PC 側の答えと比べられる。

```
A A0 0.2 0.2 2.4 0.0 0.08 0.0     アンカー 1 台 (id x y z 遅延 sigma0 sigma/m)
M 0 4.8703 0.0 -1.0               観測 1 本 (添字 距離 sigma 品質)
M 1 4.8052 0.0 -1.0
E 0.0                             エポック終わり → 測位して 1 行出力
```

```bash
./examples/03_replay dump.txt Lv2
```

**式を読み比べるより、同じ入力で同じ数字が出るかを見る方が早い。**

---

## API

全部 `include/uwb_loc.h` にコメント付きで書いてある。よく使うものだけ:

### 設定

```c
void uwb_config_init(uwb_config *cfg, const uwb_anchor *anchors, int n_anchors);
int  uwb_anchor_index(const uwb_config *cfg, const char *id);   /* ID → 添字 */
```

`uwb_config_init` のあとに変えられる主なもの:

| 項目 | 既定 | 意味 |
|---|---|---|
| `dim` | 3 | 2 にすると高さを `z_fixed` に固定して解く |
| `use_z_bounds` / `z_min` / `z_max` | 無効 | 高さの範囲。**同一平面配置では鏡像解を片側に決める手がかりになる** |
| `huber_k` / `k_pos_scale` / `one_sided` | 1.345 / 0.6 / 1 | ロバスト損失。片側なのは NLOS が距離を伸ばす側にしか出ないから |
| `chi2_k` | 負 (レベル別既定) | `|残差| > chi2_k·σ` を外す。0 で無効 |
| `physical_gate` | 1 | 距離の妥当性と三角不等式で足切り |
| `max_iter` / `tol` | 30 / 1e-4 | Gauss-Newton の打ち切り |

### 測位 (スナップショット)

```c
int uwb_solve_lv0(const uwb_config*, const uwb_meas*, int n, uwb_fix *out);
int uwb_solve_lv1(const uwb_config*, const uwb_meas*, int n, uwb_fix *out);
int uwb_solve_lv2(const uwb_config*, const uwb_meas*, int n, uwb_fix *out);  /* 既定 */
```

どれも**状態を持たない**。戻り値と `fix.ok` は同じ意味。

### 追跡 (Lv3 EKF)

```c
uwb_ekf ekf;
uwb_ekf_init(&ekf, &cfg, UWB_MOTION_CV, 0.5);   /* sigma_a = 機敏さ */
...
uwb_ekf_update(&ekf, t_sec, meas, n, &fix);     /* 1 本だけでも呼べる */
```

- `UWB_MOTION_CV` (等速) / `UWB_MOTION_CA` (等加速度、機敏な動き向け)
- `sigma_a` — 歩行者や台車なら 0.2〜0.5、ドローンなら 2〜5 が目安。
  大きいと追従が速いがノイズを拾い、小さいと滑らかだが遅れる
- **立ち上げだけは `dim+2` 本たまるまで待つ** (測距 1 本は球面 1 枚でしかない)。
  貯めている間は `fix.ok` が 0
- `t` は単調増加の秒。巻き戻った観測は捨てる

### 配置の評価

```c
uwb_real uwb_gdop_at(const uwb_config*, const uwb_real *point);   /* 5 超で配置が悪い */
uwb_real uwb_crlb_at(const uwb_config*, const uwb_real *point);   /* 誤差の理論下限 [m] */
int uwb_anchors_coplanar(const uwb_config*, uwb_real *normal, uwb_real *offset);
```

### 結果 (`uwb_fix`)

| | 意味 |
|---|---|
| `ok` | **必ず見る。** 0 なら `p` は無意味 |
| `p[3]` / `v[3]` | 位置 [m] / 速度 [m/s] (Lv3 のみ) |
| `sigma` | 位置誤差の代表値 [m]。大きいなら信用しない |
| `cov[9]` | 位置の共分散 (行優先 3×3) |
| `n_used` / `n_total` | 使った本数 / 渡した本数 |
| `excluded` | 落とした観測のビットマスク (観測の添字)。**同じ本が出続けるなら座標か設置を疑う** |
| `gdop` / `residual_rms` | 幾何の悪さ / 残差 [m] |
| `ambiguous` | 1 なら鏡像解かもしれない (高さが信用できない) |

---

## 気をつけること

- **`fix.ok` を見ずに `fix.p` を使わない。**
- **アンカーは平らに並べない。** 3 次元では鏡像解が原理的に選べなくなる。
  `uwb_anchors_coplanar` が 1 を返す配置なら、`use_z_bounds` を与えるか
  `dim = 2` にする
- **3 次元は最低 4 本、実用は 6 本。** 足りないと `ok = 0` になる
- **`uwb_config` の `anchors` は借用。** ライブラリはコピーしないので、
  呼び出し側が生かし続けること (const 配列を static に置くのが安全)
- **スレッドセーフではない。** `uwb_ekf` は状態を持つので、
  複数タグを追うならタグごとに 1 つ用意する。スナップショット解 (Lv0〜Lv2)
  は状態を持たないので、同じ `cfg` を複数スレッドから読むのは安全
- **`float` に落とす前に double で動かす。** 有効桁が 7 桁になるので、
  広い部屋 (数十 m) では共分散が荒くなる

---

## テスト

```bash
make test      # 53 件
make float     # 単精度で作り直して同じテスト
make strict    # -Werror を足して、移植先のコンパイラで詰まらないか見る
```

数値の一致だけでなく、**アルゴリズムが持つべき性質**を確かめている:

- 無雑音なら閉形式が真値を返す
- Beck 法は LLS より偏りが小さい (実測 0.154 m 対 0.164 m)
- NLOS が 1 本混じっても Lv2 は崩れない (実測 Lv1 1.18 m 対 Lv2 0.17 m)
- EKF は **1 エポック 1 本**でも立ち上がって追従する
- 同一平面配置で、真値と鏡像の距離が全アンカーで一致することを確かめ、
  `ambiguous` が立つ
- 一直線配置やゼロ距離でも NaN を返さず `ok = 0` で返る

`valgrind` と AddressSanitizer/UBSan でもエラーなしを確認済み。

---

## ライセンス

Python 版と同じ MIT ([../LICENSE](../LICENSE))。
