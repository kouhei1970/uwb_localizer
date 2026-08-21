# 性能最適化のベースライン (2026-08-20)

このリポジトリの C 実装 (`c/`) をこれから最適化する。最適化対象・見積もりは
別プロジェクト `m5stack_uwb/docs/PERF_ANALYSIS.md` にある
(Beck GTRS の二分法、EKF predict の Kronecker 構造利用、EKF update の
Joseph 形式の O(nx²) 化)。

このドキュメントは **最適化を一切していない、いまの状態**の記録。
以降の変更はすべてここに書いた数字との比較で正しさと効果を確かめる。

このベースライン取得の過程で実施した唯一の実コード変更 (`c/src/uwb_nls.c` の
`wsum` 未使用変数の削除、後述) を除き、**アルゴリズムは何も変えていない**。

## 環境

```
OS       : Darwin 25.5.0 (arm64, Apple M3 Ultra)
コンパイラ: Apple clang 21.0.0 (clang-2100.1.1.101)
Python   : 3.12.12 (pyenv)
```

Python 側の依存 (numpy はグローバルに入っていたが pytest は無かったので追加した):

```
python3 -m pip install --user pytest
python3 -m pip install -e ".[dev]"      # tests/test_docs.py の例題実行に uwb_loc パッケージが要る
```

`pip install -e ".[dev]"` をしないと `tests/test_docs.py::test_examples_run` が
`ModuleNotFoundError: No module named 'uwb_loc'` で落ちる (README の開発手順
どおりにインストールしていないだけで、C 側とは無関係)。

---

## 1. ベースライン結果 (`wsum` 削除前)

### 1.1 `cd c && make test` — C テスト

```
cd c
make clean && make test
```

結果: **53 件すべて通った**。ビルド時に既知の警告が 1 件出る
(`make strict` で失敗する原因そのもの、後述)。

```
src/uwb_nls.c:342:18: warning: variable 'wsum' set but not used [-Wunused-but-set-variable]
...
uwb_loc C 版のテスト (double)
----------------------------------------------------------
     LLS 平均誤差 0.164 m / Beck 平均誤差 0.154 m
     NLOS 1 本: Lv1 1.176 m / Lv2 0.165 m
     GDOP: 6 台立体 2.05 / 4 台平面 17.25
     EKF 平均誤差 0.075 m (179 エポック)
----------------------------------------------------------
OK  53 件すべて通った
```

### 1.2 `python -m pytest -q` — Python テスト

```
cd /Users/kouhei/tmp/github/uwb_localizer
python3 -m pytest -q
```

**178 件収集** (依頼メモには「177件のはず」とあったが実際は 178 件。
下記の内訳を見ると、`wsum` を直した後の件数 177 passed / 1 failed の方が
近い数字で、依頼時点の想定と一致している)。

`wsum` 削除前は **176 passed, 2 failed**:

| テスト | 結果 | 原因 |
|---|---|---|
| `tests/test_c_port.py::test_c_library_builds_without_warnings` | FAILED | `make`(既定、`-Werror`無し)の出力に `wsum` の warning 文字列が含まれる。**今回 3 章で直す対象そのもの** |
| `tests/test_calibration.py::test_self_survey_with_noise_and_missing_links[1]` | FAILED | **C 側と無関係の既存の失敗**。詳細は後述 |

他 176 件は pass。`test_docs.py::test_examples_run` は `pip install -e ".[dev]"`
実行後は pass (実行前は `uwb_loc` が import できず fail していた)。

#### 無関係な既存失敗: `test_self_survey_with_noise_and_missing_links[1]`

`tests/test_calibration.py:91` の `assert ... < 0.20` が
`7.93 m` で落ちる。`seed=2,3` は pass、`seed=1` だけ fail する
(3 回再実行しても同じ、`np.random.default_rng(1)` で決定的)。

warning にヒントがある:

```
UserWarning: align_to_reference: 既知点がほぼ同一平面に並んでいます。
鏡像 (裏返り) が決まらず、推定が丸ごと反対側になることがあります。
```

このテストの基準点 `ref = anchors[:4]` (`room_anchors` の先頭 4 台) が
`y=0.2` 平面にほぼ乗っており、`m5stack_uwb/docs/ANCHOR_PLACEMENT.md` に
書いた「同一平面だと鏡像が決まらない」現象そのものが `seed=1` のときだけ
裏返りとして表面化している。**C 実装ともバックポートとも無関係の、
既存の (おそらく元から時々失敗する) Python 側テストの脆さ**なので、
今回のタスクでは手を付けない。`wsum` を直した後も変わらず 1 件のまま。

### 1.3 `python tools/crossval.py` — Python 版 / C 版の数値突合

事前に C の実行ファイルが要る:

```
make -C c examples
```

**既定許容 (`--tol` 省略、1e-6 m):**

```
python3 tools/crossval.py
```

**厳格モード (`--tol 1e-9`, `tests/test_c_port.py::test_c_matches_python` と同じ):**

```
python3 tools/crossval.py --tol 1e-9
```

どちらも **20/20 シナリオ×レベルの組み合わせすべて一致**。実際の最大差は
以下の通り (5 シナリオ × Lv0-Lv3、`wsum` 削除前後で完全に同じ数字):

| シナリオ | Lv0 | Lv1 | Lv2 | Lv3 |
|---|---:|---:|---:|---:|
| 静止・無雑音・8台 | 2.92e-11 | 7.11e-12 | 7.02e-12 | 7.73e-12 |
| 静止・雑音あり・8台 | 2.52e-11 | 7.71e-12 | 7.71e-12 | 7.37e-12 |
| 8の字・雑音あり・8台 | 2.85e-11 | 7.04e-12 | 7.75e-12 | 7.91e-12 |
| 8の字・NLOS 30%・8台 | 8.36e-11 | 7.88e-12 | 7.88e-12 | 8.07e-12 |
| 円・最小構成 4台 | 3.41e-11 | 6.91e-12 | 6.91e-12 | 7.52e-12 |

単位は [m]。**最大でも 8.36e-11 m** (Lv0) で、既定許容 (1e-6) はもちろん
厳格モード (1e-9) にも 1〜2 桁の余裕がある。Lv0 が Lv1-3 より 1 桁大きいのは
反復をしない閉形式解の丸め誤差の蓄積具合の違いで、想定内。

→ **これ以降の最適化で、この表の数字 (特に厳格モードで 1e-9 を超えないこと)
が崩れたら実装ミスを疑うこと。**

### 1.4 `cd c && make strict` — 警告ゼロ確認

```
cd c
make strict
```

**想定通り失敗する:**

```
cc ... -Wall -Wextra -Werror -pedantic -Wshadow -Wconversion -Wno-sign-conversion -c src/uwb_nls.c -o src/uwb_nls.o
src/uwb_nls.c:342:18: error: variable 'wsum' set but not used [-Werror,-Wunused-but-set-variable]
  342 |         uwb_real wsum = (uwb_real)0, num = (uwb_real)0;
      |                  ^
1 error generated.
make[1]: *** [src/uwb_nls.o] Error 1
make: *** [strict] Error 2
```

依頼メモの記述 (`src/uwb_nls.c:342` の `wsum` set-but-unused) と完全に一致。

---

## 2. ベンチマークハーネス (`c/bench/`)

新規: `c/bench/bench_uwb.c`。`c/Makefile` に `bench` (ビルド+実行) と
`bench-build` (ビルドのみ、`strict` から使う) を追加した。

```
cd c
make bench           # ビルドして実行、表 + TSV (bench/bench_result.tsv) を出す
make bench-build      # コンパイルだけ (strict が使う)
```

### 設計のポイント

- **ホスト専用** (`clock_gettime(CLOCK_MONOTONIC)` を使う)。ライブラリ本体
  (`c/src/*`) には一切依存を持ち込んでいない
- 各関数を **10 万回ループ**し、区間全体を計って割った平均を使う
  (1 回だけでは分解能以下)
- 結果を `volatile double g_sink` に毎回足し込み、`-O2` でもループごと
  最適化で消えないようにしてある
- アンカーは `crossval.py` の `room` 配置と同じ 8 台
  (z を 0.3/2.4 で互い違いにしてあり、先頭 N 台 (N=4,5,6,8) を取っても
  同一平面にならない)。起動時に `uwb_anchors_coplanar()` でも確認している
- **測距値の摂動は乱数ではなく決定的な小さな揺らぎ (振幅 1 cm) にした。**
  理由は 4 章の「気づいた点」を参照 (乱数だと Gauss-Newton の反復回数が
  構成によって大きく暴れ、アンカー台数依存を見たいベンチにノイズが乗った)
- `uwb_ekf_update` は **bootstrap (Lv2 スナップショットでの立ち上げ) を
  計測に混ぜない**よう、`uwb_ekf` の公開フィールドを直接触って
  「立ち上がった直後」の状態を作ってから計測する。毎回同じ時刻 (dt=0) で
  呼ぶので `uwb_ekf_predict` は内部で即 return し、Joseph 形式の更新
  そのものだけが計測される
- `uwb_ekf_predict` はアンカーを見ないので、CV/CA モデルだけを振っている
  (アンカー台数には依存しない)

### 結果 (2026-08-20、`wsum` 削除後、10万回/条件)

`us/回` = 1 回あたりのマイクロ秒。相対比は **同じ関数名の中で最初の行
(最小のアンカー台数、predict なら CV) を基準の 1.00x** にしたもの。

| 関数 | 条件 | us/回 | 相対比 |
|---|---|---:|---:|
| uwb_solve_lv0 | N=4 | 0.385 | 1.00x |
| uwb_solve_lv0 | N=5 | 0.493 | 1.28x |
| uwb_solve_lv0 | N=6 | 0.483 | 1.25x |
| uwb_solve_lv0 | N=8 | 0.730 | 1.89x |
| uwb_solve_lv1 | N=4 | 4.557 | 1.00x |
| uwb_solve_lv1 | N=5 | 4.823 | 1.06x |
| uwb_solve_lv1 | N=6 | 5.016 | 1.10x |
| uwb_solve_lv1 | N=8 | 5.436 | 1.19x |
| uwb_solve_lv2 | N=4 | 4.555 | 1.00x |
| uwb_solve_lv2 | N=5 | 4.823 | 1.06x |
| uwb_solve_lv2 | N=6 | 5.027 | 1.10x |
| uwb_solve_lv2 | N=8 | 5.454 | 1.20x |
| uwb_beck_gtrs | N=4 | 4.254 | 1.00x |
| uwb_beck_gtrs | N=5 | 4.100 | 0.96x |
| uwb_beck_gtrs | N=6 | 4.327 | 1.02x |
| uwb_beck_gtrs | N=8 | 4.343 | 1.02x |
| uwb_ekf_predict | CV (nx=6) | 0.225 | 1.00x |
| uwb_ekf_predict | CA (nx=9) | 0.458 | 2.04x |
| uwb_ekf_update | CV(nx=6), N=4 | 1.118 | 1.00x |
| uwb_ekf_update | CV(nx=6), N=5 | 1.378 | 1.23x |
| uwb_ekf_update | CV(nx=6), N=6 | 1.645 | 1.47x |
| uwb_ekf_update | CV(nx=6), N=8 | 2.151 | 1.92x |
| uwb_ekf_update | CA(nx=9), N=4 | 2.047 | 1.83x |
| uwb_ekf_update | CA(nx=9), N=5 | 2.530 | 2.26x |
| uwb_ekf_update | CA(nx=9), N=6 | 3.020 | 2.70x |
| uwb_ekf_update | CA(nx=9), N=8 | 3.973 | 3.55x |

同じ内容が `c/bench/bench_result.tsv` (TSV、`.gitignore` 対象) に出る。
最適化の前後でこのファイルを diff すればよい。実行のたびに壁時計時間は
数%〜十数%ぶれる (マシン負荷次第) が、台数依存・モデル依存の傾向は
複数回の再実行で安定して同じ形になることを確認済み。

### 読み取れること (台数依存・モデル依存)

- **Lv0/Lv1/Lv2 はアンカー台数にほぼ線形〜緩やかに増加。** N=4→N=8 で
  Lv0 は約 1.9 倍、Lv1/Lv2 は約 1.2 倍。Gauss-Newton の 1 反復あたりの
  コストが O(N) で効いている形で、想定通り
- **`uwb_beck_gtrs` はアンカー台数にほぼ依存しない (0.96x〜1.02x)。**
  PERF_ANALYSIS.md が指摘する「二分法の λ 評価は 4x4 (=d+1) の固定サイズで
  行われ、アンカー台数は G/h/f を組み立てる O(N) の部分にしか効かない」
  という構造の予測と綺麗に一致する。**最適化 (コレスキー+固有ベクトル
  使い回し) の効果は台数によらずほぼ一定の倍率で出るはず**、という見立てを
  裏付ける実測
- **`uwb_ekf_update` は台数にほぼ線形 (CV で N=4→8 は約 1.9 倍、
  CA で約 1.9 倍)。** 1 測距ごとに O(nx²) の Joseph 更新をしているので
  想定通り
- **`uwb_ekf_predict` は CV→CA で 2.04 倍。** PERF_ANALYSIS.md の
  乗算回数見積もり (CA 1458 / CV 432 = 3.4 倍) より小さい。4 章で詳述

---

## 3. `wsum` の削除 (今回の唯一の実コード変更)

`c/src/uwb_nls.c` の `uwb_solve_lv0` 内、Lv0 専用ブロック
(**Lv1/Lv2 側の `solve_nls` にある同名の `wsum` とは別スコープ、
そちらは重み付き RMS の計算に使っていて必要なので触っていない**):

```diff
         nls_result r;
-        uwb_real wsum = (uwb_real)0, num = (uwb_real)0;
+        uwb_real num = (uwb_real)0;
         uwb_real jm[UWB_MAX_MEAS * 3], hmat[9], inv[9];
```

```diff
             r.w[i] = (uwb_real)1 / (sg * sg);
-            wsum += r.w[i];
             num += e * e;
```

Lv0 の残差 RMS はコード中コメント通り「Python 版と同じく重み無しの RMS」
(`r.residual_rms = sqrt(num / set.n)`, 365 行目付近) で計算しており、
`wsum` はどこからも読まれない set-but-unused な変数だった。削除しても
計算結果は変わらない。

### 3.1 再検証

**`cd c && make strict`** → 通った (ライブラリ・テスト・`bench-build` すべて
警告ゼロ):

```
cc ... -Wall -Wextra -Werror -pedantic ... -c src/uwb_nls.c -o src/uwb_nls.o
(警告なし)
...
cc ... bench/bench_uwb.c libuwbloc.a -lm -o bench/bench_uwb
(警告なし)
```

**`cd c && make test`** → 53 件すべて通った。出力は削除前と**完全に同一**
(浮動小数の表示桁まで一致):

```
     LLS 平均誤差 0.164 m / Beck 平均誤差 0.154 m
     NLOS 1 本: Lv1 1.176 m / Lv2 0.165 m
     GDOP: 6 台立体 2.05 / 4 台平面 17.25
     EKF 平均誤差 0.075 m (179 エポック)
OK  53 件すべて通った
```

**`python tools/crossval.py` / `--tol 1e-9`** → 1.3 節の表と**寸分違わず
同じ数字** (最大差 8.36e-11 m のまま、20/20 一致)。`wsum` はどこにも
読まれていなかったので当然だが、実際に数値で確認した。

**`python -m pytest -q`** → **177 passed, 1 failed**。

- 直った: `test_c_library_builds_without_warnings` (`make`(既定) の
  出力に warning が出なくなった)
- 変わらず失敗: `test_self_survey_with_noise_and_missing_links[1]`
  (1.2 節で説明した、C 側と無関係の既存の失敗)

依頼メモの「177件のはず」は、この `wsum` 削除後の状態
(177 passed / 1 failed = 178 収集) を指していたと考えられる。

---

## 4. 気づいた点・PERF_ANALYSIS.md との食い違いの兆候

1. **ベンチの入力を乱数にすると、台数依存が埋もれる。**
   最初 `sigma0=0.08` m 相当の正規乱数で測距値を作ったところ、
   `uwb_solve_lv2` の Gauss-Newton 反復回数が N=4 で 24 回、N=8 で 2 回
   (!) と大きく暴れ、1 回あたりの時間が台数と無関係に最大 4 倍近くぶれた
   (`uwb_solve_lv2 N=6` が `N=4` の 3.9 倍、など)。原因は
   `solve_gated()` の chi2 ゲート再解 (外れ値を切って GN をもう一度回す)
   と、LM 減衰スケジュールが初期値・幾何条件によって大きく反復回数を
   変えること。**乱数のシードが анchors 台数ごとに違う (RNG の状態が
   通し番号で進む) と、「台数依存」を見ているつもりが実は「たまたま
   どの反復回数を引いたか」を見ていることになる。**
   → ベンチは決定的な小さい摂動 (振幅 1 cm、chi2 ゲート閾値の 1/30 以下)
   に変更し、GN が 1-2 反復で安定して収束するようにした。今後、同種の
   ベンチを書くときは同じ罠に注意。

2. **`uwb_beck_gtrs` の台数依存の無さは PERF_ANALYSIS.md の構造分析と
   一致した (良い意味で予想通り)。** 二分法が 4x4 (d+1) の固定サイズの
   系を解いているだけで、アンカー台数は前処理 (G,h,f の組み立て) にしか
   効かないため。最適化 (コレスキー + 固有ベクトル使い回し) の効果は
   台数に依らずほぼ一定倍で出ると期待してよさそう。

3. **`uwb_ekf_predict` の CV→CA 比が見積もりより小さい。**
   PERF_ANALYSIS.md の乗算回数ベースの見積もりは CA/CV = 1458/432 ≈ **3.4倍**
   だが、実測は **2.04倍**。nx=6/9 はどちらもごく小さい行列で、
   0.2〜0.5 マイクロ秒という短時間の呼び出しなので、関数呼び出し・
   `transition()` でのクロネッカー積展開・ループの固定オーバーヘッドが
   相対的に無視できず、単純な乗算回数の比ほど差が付かなかった可能性が高い。
   **構造最適化 (O(nx²) 化) をしても、nx がこの程度小さいうちは
   「乗算回数の比」がそのまま「実行時間の比」にはならないかもしれない**、
   という点は最適化後の効果測定で意識しておくとよい (ESP32-S3 は
   double がソフトエミュレーションなので乗算 1 回のコストがホストより
   ずっと重く、この「固定オーバーヘッドに埋もれる」効果はホストより
   小さいと予想されるが、実機で確認する価値はある)。

4. **`uwb_ekf_update` は台数依存もモデル依存もおおむね素直に線形**
   (CV: N=4→8 で 1.92倍、CA: N=4→8 で 1.94倍 [CA/CV 同士の比]、
   CA(N=4)/CV(N=4) で 1.83倍)。ここは PERF_ANALYSIS.md が
   「Joseph 形式の O(nx³) → O(nx²) 化で 5.7 倍」と見積もっている
   最大のターゲットで、今回の実測はその見積もりを検証する前段の
   ベースラインとして素直に使えそう。

5. **macOS の `-std=c99` は `_POSIX_C_SOURCE` を定義すると
   `_DARWIN_C_SOURCE` 無しに `snprintf` まで見えなくなる。**
   ベンチで `clock_gettime`/`CLOCK_MONOTONIC` を使うために
   `_POSIX_C_SOURCE 199309L` を定義したところ、`stdio.h` の `snprintf`
   が `-Wimplicit-function-declaration` で落ちた。`_DARWIN_C_SOURCE` を
   併記して解決した (Linux では単に無視される、はずだが Linux での
   ビルド確認はできていない — 未検証項目として残す)。

---

## 5. 作成・変更したファイル

| ファイル | 内容 |
|---|---|
| `c/src/uwb_nls.c` | `wsum` の宣言 1 行と加算 1 行を削除 (唯一の実コード変更) |
| `c/bench/bench_uwb.c` | 新規。ベンチマークハーネス本体 |
| `c/Makefile` | `bench` / `bench-build` ターゲットを追加、`strict`/`clean` に反映 |
| `.gitignore` | `*.tsv`、`c/bench/bench_uwb`、`c/**/*.dSYM` を追加 |
| `PERF_BASELINE.md` | 本ファイル (新規) |

`git commit` はしていない (ユーザレビュー待ち)。

---

## 6. 最適化後の最終結果 (2026-08-20)

ベースライン取得後に実施した最適化の結果。**アルゴリズムは変えていない**（同じ数式を、
構造を使って少ない演算で評価する）。数値の同一性は crossval で全工程を通じて確認した。

### 累積効果（同一環境、10万回/条件、us/回）

| 関数 | ベースライン | 最終 | **倍率** |
|---|---:|---:|---:|
| `uwb_beck_gtrs` N=4 | 4.543 | **0.792** | **5.7x** |
| `uwb_beck_gtrs` N=8 | 4.657 | **0.860** | **5.4x** |
| `uwb_solve_lv2` N=4 | 4.945 | **1.154** | **4.3x** |
| `uwb_solve_lv2` N=8 | 5.871 | **1.877** | **3.1x** |
| `uwb_solve_lv1` N=4 | 4.803 | ≒Lv2 と同等 | 約4x |
| `uwb_ekf_update` CA(nx=9) N=8 | 4.177 | **0.736** | **5.7x** |
| `uwb_ekf_update` CA(nx=9) N=4 | 2.149 | **0.420** | **5.1x** |
| `uwb_ekf_predict` CA(nx=9) | 0.472 | **0.111** | **4.3x** |
| `uwb_ekf_predict` CV(nx=6) | 0.240 | **0.059** | **4.1x** |
| `uwb_solve_lv0` | — | 変化なし | 1.0 |

### スタック使用量（double 版）
`uwb_ekf_update` で `tmp[81]+m1[81]` を、`uwb_ekf_predict` で `f[81]+q[81]+tmp[81]` を
それぞれ廃止し、**合計で約 3.1KB 削減**。
README のスタック上限 6.6KB に対して半分近く。組込みでは速度以上に効く。

### 数値の同一性
| 検証 | 結果 |
|---|---|
| C テスト | **77件**（元53 + 固有ベクトル検証24を追加）すべて通過 |
| `make strict` | 通る |
| `make float` | 通る（77件） |
| `crossval --tol 1e-9` | **worst 8.36e-11 m。全工程を通じて不変** |
| pytest | 177 passed / 1 failed（§1.2 の既存の1件のみ） |

### 実施した最適化
| # | 内容 |
|---|---|
| A | EKF update の Joseph 形式を O(nx³)→O(nx²)。`I - K hᵀ` のランク1構造を展開し作業行列を全廃 |
| B | EKF predict の `F P Fᵀ` を O(nx³)→O(nx²)。`F = f1 ⊗ I_nd` が単位上三角であることを利用 |
| C | Beck 二分法の λ ごとの 4x4 LU を廃止。`G = LLᵀ` と `S = L⁻¹DL⁻ᵀ` の固有分解で φ をスカラー化 |
| D | Beck の根探索を安全策付きニュートン法（rtsafe 型）に。反復 56回 → 3〜5回 |
| E | 収束判定を φ の丸め誤差の床で打ち切り。区間幅判定を真の相対許容に修正 |

### 【重要】E の実装中に発見した潜在バグ
最終的な `y` の復元が `beck_recover_y(&c, 0.5*(lo+hi), y)` になっていた。
**rtsafe は毎回 `lo`/`hi` の片方しか `x` に寄せない**ので、収束を早めると
`0.5*(lo+hi)` が遠い側の境界に引っ張られて不正確になる。
`UWB_USE_FLOAT` ビルドで `test_exact_when_noiseless` に **0.0945 m の誤差**が出ていた。
→ 最後に評価した `x` を使うよう修正。
純粋二分法では区間が十分詰まってから抜けるので顕在化せず、
**早期打ち切りを入れて初めて露呈するタイプ**だった。

### 採用しなかった最適化: Jacobi の要素スキップ閾値
`uwb_sym_eig` の 4x4 が Beck の 72% を占めるため、古典的な Jacobi の
しきい値処理（Numerical Recipes `jacobi` 相当）を実装したが、**効果がゼロだったので取り下げた**。

- 実測 0.5689us → 0.5694us（同じ行列、誤差範囲）
- 原因: Beck が扱う `S = L⁻¹DL⁻ᵀ` (D = diag(1,1,1,0)) は
  **6つの非対角ペアが均等に収束し、ちょうど4スイープで終わる**。
  「後半スイープに小さい要素が残る」パターンがこの行列には無い。
  2000回のランダム試行でも**スキップ0回**
- 3x3（`uwb_anchors_coplanar`）ではスイープが 3→4 に増えてむしろ悪化

**教科書的な最適化でも、対象行列の性質次第では全く効かない。必ず測ること。**

### 残っている最大のコスト
`uwb_sym_eig` 4x4 = **0.569us**（Beck 0.79us の 72%）。
`uwb_cholesky` 4x4 (0.025us) の **23倍**。削るなら:
- 固有分解を廃止する設計変更（ニュートン各反復で `(G+λD)` のコレスキー直接解、
  λ_max は power 法。φ' は `-2‖L⁻¹w‖²`, w = Dy+f で前進代入1回）
- または Jacobi を Householder 三重対角化 + QL に置き換える

いずれも**ターゲット実機で測ってから判断すべき**。
ESP32-S3 は単精度 FPU のみで `double` も `sqrt` もソフトウェアなので、
ホストで最適な設計が実機で最適とは限らない。
