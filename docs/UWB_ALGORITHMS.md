# UWB 測位アルゴリズム — 導出と実装の詳細

> **この文書**: 中で何をしているか (式の導出と実装の対応、C 移植)。**なぜ**その手法にしたかは [DESIGN.md](DESIGN.md)。 → [ドキュメント一覧](README.md)

`uwb_loc` が実際に何を計算しているかを式で追える形にしたもの。
選定の経緯は [DESIGN.md](DESIGN.md)、使い方は [TUTORIAL.md](TUTORIAL.md)。

記号

| 記号 | 意味 |
|---|---|
| $p \in \mathbb{R}^d$ | タグの位置 ($d=2$ または $3$) |
| $a_i$ | アンカー $i$ の座標 |
| $d_i = \lVert p - a_i \rVert$ | 真の距離 |
| $z_i$ | 観測値 |
| $e_i = z_i - h_i(p)$ | 残差 |
| $u_i = (p - a_i)/d_i$ | アンカー $i$ からタグへの単位ベクトル |
| $\sigma_i$ | 観測 $i$ の標準偏差、$w_i = 1/\sigma_i^2$ |

---

## 1. 観測モデル — すべての土台

実装: `uwb_loc/model.py`

どの観測種別も $(\text{残差}, \text{ヤコビアン } \partial h/\partial p, \sigma)$ の 3 つ組に
落とす。下流のソルバ（WNLS も EKF も）は種別を意識しない。

### 距離 (TWR)

$$h_i(p) = \lVert p - a_i \rVert, \qquad \frac{\partial h_i}{\partial p} = \frac{p - a_i}{\lVert p - a_i\rVert} = u_i$$

**ヤコビアンが単位ベクトルになる**のがこの問題の性質を決めている。幾何行列

$$H = \begin{bmatrix} u_1^\top \\ \vdots \\ u_n^\top \end{bmatrix}$$

は距離ではなく**方向だけ**で決まる。だから精度の幾何依存性 (GDOP) が
「アンカーがどの方向から見えるか」だけの関数になる。

### 距離差 (TDoA)

$$h_i(p) = \lVert p - a_i \rVert - \lVert p - a_\text{ref} \rVert, \qquad \frac{\partial h_i}{\partial p} = u_i - u_\text{ref}$$

ヤコビアンのノルムは高々 2 だが、タグが遠ざかって $u_i \to u_\text{ref}$ になると
**0 に近づく**。TDoA がアンカーの外側で急激に悪化するのはこれが理由で、
アルゴリズムを変えても直らない。

### 方位角・仰角 (AoA/PDoA)

$\Delta = p - a_i$、$\rho = \sqrt{\Delta_x^2+\Delta_y^2}$、$r^2 = \rho^2 + \Delta_z^2$ として

$$h = \operatorname{atan2}(\Delta_y, \Delta_x), \qquad \frac{\partial h}{\partial p} = \left(-\frac{\Delta_y}{\rho^2},\ \frac{\Delta_x}{\rho^2},\ 0\right)$$

$$h = \operatorname{atan2}(\Delta_z, \rho), \qquad \frac{\partial h}{\partial p} = \left(-\frac{\Delta_z \Delta_x}{r^2\rho},\ -\frac{\Delta_z \Delta_y}{r^2\rho},\ \frac{\rho}{r^2}\right)$$

方位角のヤコビアンのノルムは $1/\rho$。つまり同じ角度誤差 $\sigma_\theta$ でも位置誤差は
$\rho\,\sigma_\theta$ に比例して増える。重み $1/\sigma_\theta^2$ を掛けるだけで
「遠いアンカーの角度情報は効きにくい」という正しい振る舞いが自動的に出る。
角度残差は $(-\pi, \pi]$ に畳む。

### 残差の符号を $e = z - h$ に統一する理由

NLOS は電波の回り込みなので**距離を伸ばす側にしか出ない**。
この符号なら NLOS の残差は必ず $e_i > 0$ になり、
片側損失 (§5) がそのまま書ける。

### 品質値の使い方

HAL が返す $q \in [0,1]$ (見通し尤度) を

$$\sigma_\text{eff} = \sigma \cdot \bigl(1 + 3(1-q)\bigr)$$

で反映する。$q=1$ で等倍、$q=0$ で 4 倍 → 重みは 1/16 に落ちる。
**観測を捨てるのではなく重みを下げる**のは、アンカー本数が少ないときに
「捨てすぎて解けない」を避けるため。

---

## 2. Lv0 — 線形最小二乗 (LLS)

実装: `uwb_loc/solvers/closed_form.py:lls_trilateration`

測距方程式を展開すると

$$\lVert p \rVert^2 - 2a_i^\top p + \lVert a_i \rVert^2 = r_i^2$$

非線形なのは $\lVert p \rVert^2$ だけで、しかも**全式に共通**。だから基準アンカー $j$ の式を
引くと消える。

$$2(a_j - a_i)^\top p = r_i^2 - r_j^2 - \lVert a_i\rVert^2 + \lVert a_j\rVert^2$$

$p$ について線形なので擬似逆行列 1 発。反復も初期値も要らない。

### なぜこれで満足してはいけないか

3 つの問題がある。

1. **二乗による偏り**。$r_i = d_i + n_i$ なので $r_i^2 = d_i^2 + 2d_i n_i + n_i^2$。
   最後の項の期待値 $\sigma^2$ は差分をとっても消えず、系統誤差として残る。
2. **重みが正しくない**。雑音項が $2d_i n_i$ の形で入るので、実効的な分散が
   $4d_i^2\sigma_i^2$ になる。遠いアンカーほど不利で、$1/\sigma_i^2$ の重みは最適でない。
3. **行が相関する**。全行が基準アンカーの $r_j, n_j$ を共有するので、
   最小二乗の前提 (独立誤差) が崩れる。基準の選び方で答えが変わるのもこのため。

実装では測距値が最小のアンカー (通常いちばん S/N が良い) を基準に選び、
差分行の重みは 2 本の調和平均で近似している。それでも上の 3 つは残るので、
**動作確認と初期値供給専用**と割り切っている。

---

## 3. Beck の厳密解 (GTRS) — Lv2 の初期解

実装: `uwb_loc/solvers/closed_form.py:beck_gtrs`

LLS は $\lVert p\rVert^2$ を**消した**。Beck 法は逆に**変数に昇格させる**。

$$g = \lVert p \rVert^2, \qquad y = \begin{bmatrix} p \\ g\end{bmatrix} \in \mathbb{R}^{d+1}$$

すると測距方程式は完全に線形になる。

$$\underbrace{\begin{bmatrix} -2a_i^\top & 1\end{bmatrix}}_{A \text{ の第 } i \text{ 行}} y = \underbrace{r_i^2 - \lVert a_i\rVert^2}_{b_i} \qquad \Longrightarrow \qquad Ay = b$$

もちろん $y$ は自由ではなく、$g = \lVert p\rVert^2$ という拘束が 1 本つく。これは
2 次形式で書ける。

$$y^\top D y + 2f^\top y = 0, \qquad D = \begin{bmatrix} I_d & 0 \\ 0 & 0\end{bmatrix},\quad f = \begin{bmatrix} 0 \\ -1/2\end{bmatrix}$$

（検算: $y^\top D y = \lVert p\rVert^2$、$2f^\top y = -g$、和が 0。）

つまり解くべきは

$$\min_y\ \lVert W^{1/2}(Ay - b)\rVert^2 \quad \text{s.t.}\quad y^\top Dy + 2f^\top y = 0$$

**2 次目的関数 + 2 次拘束 1 本** = Generalized Trust Region Subproblem (GTRS)。
非凸だが、GTRS には**双対ギャップがない**ことが知られていて、
大域最適解が 1 個のラグランジュ乗数だけで書ける。

### 永年方程式

$$\mathcal{L}(y,\lambda) = (Ay-b)^\top W (Ay-b) + \lambda\,(y^\top Dy + 2f^\top y)$$

$$\frac{\partial \mathcal{L}}{\partial y} = 0 \ \Longrightarrow\ (A^\top W A + \lambda D)\,y = A^\top W b - \lambda f$$

$$\boxed{\ y(\lambda) = (A^\top W A + \lambda D)^{-1}(A^\top W b - \lambda f)\ }$$

あとは拘束を満たす $\lambda$ を探すだけ。

$$\varphi(\lambda) = y(\lambda)^\top D\,y(\lambda) + 2f^\top y(\lambda) = 0$$

### なぜ二分法で必ず解けるのか

$G = A^\top W A$ とする。$G + \lambda D \succ 0$ が成り立つ範囲を調べる。
一般化固有値 $Dv = \gamma\,Gv$ に対し

$$G + \lambda D \succ 0 \iff 1 + \lambda\gamma > 0 \ \ (\forall \gamma)$$

$D$ は半正定値 (階数 $d$) なので $\gamma \ge 0$。したがって条件は最大固有値 $\gamma_1$ が
決める。

$$\lambda > -1/\gamma_1$$

この区間 $I = (-1/\gamma_1, \infty)$ の上で

- $G + \lambda D \succ 0$ ⟺ **2 階の最小条件が満たされる** (最大点や鞍点ではない)
- $\varphi$ は**狭義単調減少**
- $\lambda \to -1/\gamma_1^{+}$ で $\varphi \to +\infty$

なので根は一意で、**二分法で必ず捕まる**。初期値も反復の発散もない。

実装 (`beck_gtrs`) は

1. $\gamma_1 = \max\operatorname{eig}(G^{-1}D)$ から $\lambda_\text{lo} = -1/\gamma_1$
2. $\lambda_\text{lo}$ のわずか内側から始めて $\varphi > 0$ を確認
3. $\varphi < 0$ になるまで上限を倍々に広げる
4. 200 回の二分法

行列は最大 4×4 なので、これ全部で数十マイクロ秒。

### LLS との違い

差分をとらないので**相関も基準アンカーの選択も入らない**。
$r_i^2$ の二乗バイアスは残るので厳密な最尤推定ではないが、
**大域最適**であることが保証されるので Gauss-Newton の初期値として理想的。

テスト `test_beck_is_global_and_beats_lls_with_noise` で、
ランダム 200 点の平均誤差が LLS より小さいことを確認している。

---

## 4. Lv1 — 重み付き非線形最小二乗

実装: `uwb_loc/solvers/nls.py:solve_nls`

測距誤差がガウスなら、最尤推定はこれ。

$$\hat{p} = \arg\min_p \sum_i w_i \bigl(z_i - h_i(p)\bigr)^2, \qquad w_i = 1/\sigma_i^2$$

### Gauss-Newton

$e(p + \Delta) \approx e - J\Delta$ と 1 次近似して代入すると、
$\Delta$ について線形最小二乗になる。正規方程式は

$$\boxed{\ (J^\top W J)\,\Delta = J^\top W e, \qquad p \leftarrow p + \Delta\ }$$

$J$ の行は §1 のヤコビアン、$W = \operatorname{diag}(w_i)$。

### Levenberg-Marquardt 減衰

$J^\top W J$ が特異に近づく状況が実際にある。

- アンカーがほぼ一直線に並んでいる
- タグがちょうどアンカー平面上にいる (3D で解いているとき)
- 見えているアンカーが 4 本ぎりぎり

そのまま解くと $\Delta$ が発散するので、減衰項を入れる。

$$\left(J^\top W J + \lambda\,\frac{\operatorname{tr}(J^\top W J)}{n_\text{free}} I\right)\Delta = J^\top W e$$

コストが下がったら $\lambda \leftarrow 0.3\lambda$、上がったら $\lambda \leftarrow 4\lambda$ で
更新を破棄。$\lambda$ が大きいときは最急降下法に、小さいときは Gauss-Newton に
近づく。**解が退化していても止まる**ことが保証される。

### 共分散

最適点でのフィッシャー情報行列は $J^\top W J$ なので

$$\operatorname{Cov}(\hat{p}) \approx (J^\top W J)^{-1}$$

これを `Fix.covariance` として返す。運用時に「今の値を信じてよいか」を
判断するための数字で、位置だけ返すライブラリは現場で困る。

### 次元拘束

$d=2$ のときは $J$ の $z$ 列を落として 2×2 の系にする。
$z$ は既知として固定するので、共分散も 2×2 部分だけが埋まる。

---

## 5. Lv2 — ロバスト化

実装: `uwb_loc/solvers/robust.py`, `uwb_loc/solvers/nls.py`

屋内 UWB の精度を決めるのは測距ノイズではなく NLOS。順に効く手を重ねる。

### 5.1 物理ゲート (ほぼ無料)

位置を解く前にできるチェック。

- 負の距離、到達不能な距離
- **三角不等式**: アンカー間距離 $D_{ij}$ は既知なので、
  $r_i + r_j < D_{ij}$ や $|r_i - r_j| > D_{ij}$ はありえない

各観測が他の何本と矛盾するかを数え、**過半と矛盾するものだけ**落とす。
1 本ずつの小競り合いで健全な観測を落とさないため。

### 5.2 M 推定 (Huber) と IRLS

二乗損失の代わりにフーバー損失を使う。$u = e/\sigma$ として

$$\rho(u) = \begin{cases} \tfrac{1}{2}u^2 & |u| \le k \\ k|u| - \tfrac{1}{2}k^2 & |u| > k\end{cases}$$

勾配は $\psi(u) = \rho'(u)$ で、これを $\psi(u) = \mathcal{W}(u)\cdot u$ と書き直すと

$$\mathcal{W}(u) = \frac{\psi(u)}{u} = \begin{cases} 1 & |u| \le k \\ k/|u| & |u| > k\end{cases}$$

**勾配が「重み $\mathcal{W}$ 付きの最小二乗の勾配」と同じ形になる**。だから
「重みを残差から作り直しては解く」を繰り返せばよい (IRLS: Iteratively
Reweighted Least Squares)。実装では反復のたびに $w_i \leftarrow \mathcal{W}(u_i)/\sigma_i^2$
を作り直している。

外れ値の重みが $\propto 1/|u|$ に落ちるので、**影響関数 $\psi$ が有界**になる。
これがロバストであることの定義そのもの。

$k = 1.345$ は、純ガウス誤差に対して効率 95% を保つ標準的な値。

Tukey の biweight ($\mathcal{W} = (1-(u/k)^2)^2$、$|u|>k$ で 0) も用意してあるが
既定にしていない。$\psi$ が再下降するので、初期値が悪いと正しい観測まで
完全に切って別の解に落ち着く。実測でも Huber より悪かった (§8)。

### 5.3 片側損失 — NLOS の物理を重みに入れる

NLOS は距離を**伸ばす側にしか出ない**。$e = z - h$ の符号なら $e > 0$ 側。
そこでしきい値を符号で変える。

$$k_i = \begin{cases} 0.6\,k & e_i > 0 \quad (\text{NLOS の疑いあり}) \\ k & e_i \le 0 \quad (\text{単なる雑音}) \end{cases}$$

「短すぎる側」は雑音でしか起きないので効率を落とさず、
「長すぎる側」だけ厳しく見る。実測で RMSE 0.350 → 0.297 m (§8)。

### 5.4 χ² ゲート

収束後の標準化残差 $u_i = e_i/\sigma_i$ は、モデルが正しければ概ね $N(0,1)$。
$|u_i| > 4$ は $6\times10^{-5}$ の事象なので、外れ値とみなして落とし、
**1 度だけ**解き直す。残る本数が $d+1$ を下回るなら実行しない。

### 5.5 RANSAC — 常時ではなく保険として

最小構成 ($d+1$ 本) から仮解を作り、$4\sigma$ 以内に収まる観測が最多の組を採る。

**ここが実装で最も試行錯誤した箇所。** 提案どおり常時走らせたら Lv1 より
悪化した (RMSE 0.297 → 0.401 m)。理由は明快で、3 次元の最小構成は 4 本、
そこから作る仮解**自体**が数十 cm の誤差を持つ。その仮解を基準に $3\sigma$ で
インライア判定すると、健全な観測が大量に外れる。

対策は 3 つ。

1. **起動条件をつける**。規格化残差 $\sqrt{\overline{u^2}}$ が 3 を超えたときだけ走らせる
   (モデルが正しければ 1 前後になるはずの量)
2. **しきい値を広げる** ($3\sigma \to 4\sigma$)。仮解自身の誤差を見込む
3. **改善したときだけ採用する**。RANSAC 後の規格化残差が元より小さくならなければ
   元の解を使う

結果として、まともな条件では RANSAC は**一度も起動しない**。それでよい。
これは精度を上げる部品ではなく、M 推定が引きずられるほど壊れた状況
(NLOS が過半を占める、アンカーが 1 台動かされた) のための保険である。

### 5.6 温め直さない

Lv2 は前回の推定を初期値に使わない (`warm_start=False`)。
ロバスト重みは残差から作るので、前回値に引きずられると
**外れ値に固着**しうる。毎回 Beck の閉形式から解き直しても、
それが大域最適解なので収束性は落ちない。

---

## 6. Lv3 — 密結合 EKF

実装: `uwb_loc/solvers/ekf.py`

### 6.1 なぜ「密結合」か

スナップショット位置をフィルタに入れる (疎結合) のではなく、
**測距値そのものを観測としてカルマンフィルタに直接入れる**。

- アンカーが 3 本未満しか見えないエポックでも**更新できる**。
  2 本では三辺測量は不可能だが、各測距は依然として 1 次元の拘束であり、
  その方向の共分散を縮める。残りは事前分布 (運動モデル) が埋める。
  疎結合ならこのエポックは丸ごと捨てになる
- TWR はアンカーを順にポーリングするので観測は**もともと非同期**に届く。
  「1 スキャン = 1 エポック」に束ねる必要がない
- 幾何が悪い方向の情報だけを部分的に取り込める

テスト `test_ekf_tracks_with_fewer_than_three_anchors` がこれを検証している。

### 6.2 状態と予測

状態は軸ごとの積分器の連鎖。CV なら $x = [p; v]$、CA なら $x = [p; v; a]$。

$$F_1[i][j] = \frac{\Delta t^{\,j-i}}{(j-i)!} \quad (j \ge i), \qquad F = F_1 \otimes I_d$$

クロネッカー積で軸に広げる (軸は独立と仮定)。

プロセスノイズは**最上位の微分に連続時間の白色雑音が乗る**モデルで統一する。
$Q = \int_0^{\Delta t} F(\tau)\,G\sigma^2 G^\top F(\tau)^\top\,d\tau$ を解いて

$$Q_1^{\text{CV}} = \sigma_a^2\begin{bmatrix} \Delta t^3/3 & \Delta t^2/2 \\ \Delta t^2/2 & \Delta t\end{bmatrix}, \qquad
Q_1^{\text{CA}} = \sigma_j^2\begin{bmatrix} \Delta t^5/20 & \Delta t^4/8 & \Delta t^3/6 \\ \Delta t^4/8 & \Delta t^3/3 & \Delta t^2/2 \\ \Delta t^3/6 & \Delta t^2/2 & \Delta t\end{bmatrix}$$

離散版 (1 ステップ中は加速度一定と見なす $\Gamma\Gamma^\top$ 形、$\Delta t^4/4$ が出るもの) と
**混ぜないこと**。`sigma_a` の意味がモードによって変わってしまう。

### 6.3 更新 — 1 本ずつスカラーで

観測は位置にしか依存しないので $H = [\partial h/\partial p,\ 0,\ \ldots]$ (1×$n_x$)。

$$\nu = z - h(\hat{x}), \qquad S = HPH^\top + \sigma^2 \quad (\text{スカラー})$$

$$K = \frac{PH^\top}{S}, \qquad \hat{x} \leftarrow \hat{x} + K\nu$$

$S$ がスカラーなので**行列の逆行列が一切要らない**。これは C 移植で効く。

共分散は Joseph 形で更新する。

$$P \leftarrow (I - KH)P(I - KH)^\top + K\sigma^2K^\top$$

最適 $K$ に対しては $(I-KH)P$ と代数的に等価だが、丸め誤差が乗っても
**対称性と半正定値性が壊れにくい**。スカラー更新を 1 エポックに何本も繰り返すので
この性質が要る。さらに $P \leftarrow (P + P^\top)/2$ で対称化する。

1 本ずつ順に処理する副次効果として、2 本目は**1 本目で更新済みの位置**で
線形化される。反復 EKF に近い効果がただで手に入る。

### 6.4 イノベーションゲートと「棺桶化」対策

$|\nu| > \gamma\sqrt{S}$ ($\gamma$ は既定 3) の観測はそのエポックで使わない。
これが NLOS 対策の本体で、$S$ が「予測の不確かさ + 観測の不確かさ」を
正しく含むので、**フィルタが自信を持っているときほど厳しく**判定される。

ただしこれには既知の失敗モードがある。何かの拍子に状態が間違うと、
**正しい観測がすべて外れ値に見えて全部弾かれ、二度と直らない**。
対策として、全観測を弾いたエポックを数え、`max_rejects` 回連続したら
状態を捨てて Lv2 のスナップショット測位からやり直す。
テスト `test_ekf_recovers_after_teleport` で検証している。

そのほか

- 立ち上げ (bootstrap) は Lv2 の解と共分散を使う。速度は大きな事前分散から始める
- 観測が `max_dt` 以上途切れたら予測が意味を失うので再初期化
- 時刻が巻き戻った観測 (遅れて届いたもの) は捨てる

---

## 7. TDoA の Chan 法

実装: `uwb_loc/solvers/closed_form.py:chan_tdoa`

基準アンカーまでの距離 $d_\text{ref}$ を**補助未知数に加える**と線形になる、
という GPS 由来の手当て。$r_i = d_i - d_\text{ref}$ (観測) として

$$d_i = d_\text{ref} + r_i \ \Longrightarrow\ \lVert p\rVert^2 - 2a_i^\top p + K_i = d_\text{ref}^2 + 2d_\text{ref}r_i + r_i^2$$

基準アンカーの式 $\lVert p\rVert^2 - 2a_\text{ref}^\top p + K_\text{ref} = d_\text{ref}^2$ を引くと
$\lVert p\rVert^2$ と $d_\text{ref}^2$ が両方消える。

$$\boxed{\ 2(a_i - a_\text{ref})^\top p + 2r_i\,d_\text{ref} = K_i - K_\text{ref} - r_i^2\ }$$

未知数 $[p;\,d_\text{ref}]$ について線形。$n-1 \ge d+1$ 本あれば WLS で解ける。

2 段目 ($d_\text{ref} = \lVert p - a_\text{ref}\rVert$ の拘束を使った補正) は入れていない。
初期値として使う分には 1 段目で十分で、最終精度は後段の WNLS が担保するため。

---

## 8. GDOP と CRLB — 配置を数字で評価する

実装: `uwb_loc/geometry.py`

### フィッシャー情報行列

ガウス誤差のとき、位置に関するフィッシャー情報は

$$\mathcal{F} = \sum_i \frac{1}{\sigma_i^2}\,u_i u_i^\top$$

クラメール・ラオ下限は $\operatorname{Cov}(\hat{p}) \succeq \mathcal{F}^{-1}$。
`crlb_at` は $\sqrt{\operatorname{tr}(\mathcal{F}^{-1})}$ を返す。
**どんなアルゴリズムを使ってもこれより良くはならない**ので、
実測 RMSE と並べれば改善余地が分かる。

ただしこれは**1 エポック分の観測で解く場合の下限**。Lv3 は時間方向の情報も
使うので下回りうる (実測で CRLB 0.178 m に対し Lv3 は 0.116 m)。

### GDOP

全 $\sigma_i$ を 1 とおいた (= 行を単位ベクトルに正規化した) ときの

$$\text{GDOP} = \sqrt{\operatorname{tr}\bigl((H^\top H)^{-1}\bigr)}$$

$\sigma_p \approx \text{GDOP} \times \sigma_\text{range}$ という読み方をする、
純粋な幾何の増幅率。実装では $H^\top H$ の階数を先に見て、
退化していれば $\infty$ を返す (2 本しか見えないときなど、逆行列が数値的に暴れて
トレースが負になることがある)。

### 同一平面の検出と鏡像解

アンカー座標を中心化して特異値分解し、第 3 特異値 $/\sqrt{n}$ が
平面からの広がりになる。これが最大特異値の 5% 未満なら同一平面と判定する。

**3 次元測位では、アンカーが同一平面に並んでいないことが本質的に効く。**
理由は 2 つある。

**(1) 高さが観測しにくい。** 天井 4 隅だけの配置では $z$ の情報がほとんど入らず、
しかもその誤差が水平方向にも回り込む。実測で RMSE(z) 0.92 m、
`dim=2` で高さを固定すると水平 RMSE まで 0.374 → 0.271 m に改善した。

**(2) 鏡像解が生じる。** これは (1) より厄介で、精度の劣化ではなく
**問題そのものの多義性**である。アンカーが平面 $n^\top x = c$ 上にあるとき、
任意の点 $p$ とその鏡像

$$p' = p - 2\,(n^\top p - c)\,n$$

は**すべてのアンカーからの距離が厳密に等しい**。

$$\lVert p' - a_i\rVert = \lVert p - a_i\rVert \quad (\forall i)$$

残差はどちらも完全に同じなので、**測距値だけからはどちらか選べない**。
どんなアルゴリズムを使っても解決しない。

実装 (`solvers/base.py:resolve_mirror`) は、距離以外の情報を強い順に使う。

1. **`SolveConfig(z_bounds=...)`** — 「タグは天井より下」のような事前知識。
   片方だけが範囲に入るならそれを採る。**これが本来の解決策**
2. **直前に採った側** — 平面のどちら側にいたかという**上下の二択だけ**を
   引き継ぐ。位置の初期値としては使わないので、Lv2 が前回値から
   温め直さない方針 (外れ値への固着を避けるため) と衝突しない
3. **どちらも無い** — 決めようがないので `Fix.ambiguous = True` を立てて
   そのまま返す

鏡映は直交変換なので、位置を移すときは共分散も $RCR^\top$
($R = I - 2nn^\top$) で移す。残差 RMS と GDOP は鏡映で不変。
**移した先で解き直してはいけない** — 退化した幾何では最適点が平面をまたいで
戻ってしまい、せっかく寄せた側から逃げる。

これを入れる前は、Lv2 が毎エポック閉形式から解き直すために側が反転し、
300 エポック中 24 回天井の上に飛んで RMSE 0.89 m だった (同条件の Lv1 は
warm start が偶然側を保つので 0.28 m で、レベルの順序が逆転していた)。
側を固定した後は反転がゼロになる。

ただし **2 の手当ては「安定させる」だけで「正しくする」ものではない**。
最初のエポックで反対側を掴むと、その track はまるごと鏡像側に張り付く
(水平は正しく、高さだけ折り返る)。実測でも 3 seed 中 1 回それが起きた。
だから同一平面 + 3D + `z_bounds` 無しの構成では**構築時に警告を出す**。
黙って 1/2 の確率で高さが折り返るのがいちばん危ないため。


---

## 9. アンカーの自己測量

実装: `uwb_loc/calibration.py:self_survey`

アンカー同士で相互測距できるなら、巻き尺は要らない。

### 古典的 MDS (Torgerson)

距離行列 $D$ から座標を復元する。二重中心化行列

$$B = -\tfrac{1}{2}\,J D^{\circ 2} J, \qquad J = I - \tfrac{1}{n}\mathbf{1}\mathbf{1}^\top$$

は、中心化した座標 $X$ のグラム行列 $XX^\top$ に一致する。
（$D^2_{ij} = \lVert x_i\rVert^2 + \lVert x_j\rVert^2 - 2x_i^\top x_j$ の
行・列方向の項が二重中心化で消えるため。）

固有分解 $B = U\Lambda U^\top$ の上位 $d$ 個を取って $X = U_d\Lambda_d^{1/2}$。

### 欠測リンクの埋め方 — 実装で見つかった落とし穴

遮蔽で測れなかったリンクを**平均値で埋めると壊れる**。MDS は大域的な
スペクトル法なので、1 箇所の大きな誤りが埋め込み全体を歪める。しかも
後段の Gauss-Newton は欠測ペアを残差に含めないので、**直す力が働かない**。

対策として、既知リンクを辿った**最短経路長** (Floyd–Warshall) で埋める。
凸に近い配置なら良い近似になり、MDS を正しい盆地に入れられる。

### Gauss-Newton による仕上げ

$$\min_X \sum_{(i,j)\in\text{既知}} w_{ij}\bigl(\lVert x_i - x_j\rVert - d_{ij}\bigr)^2$$

ペア $(i,j)$ のヤコビアンは $\partial/\partial x_i = u_{ij}$、$\partial/\partial x_j = -u_{ij}$。
座標系の自由度 (3 次元で並進 3 + 回転 3 = 6) の分だけ $H$ は必ず特異になるが、
LM 減衰がそれを吸収する。

### 座標系の固定 (ゲージ)

相互距離からは回転・並進・鏡像が決まらないので、点そのものから基底を作る。

1. 点 0 を原点
2. 点 1 の方向を第 1 軸
3. 第 1 軸への直交成分が**最大**の点で第 2 軸
4. 平面から十分離れた**最初の**点が $z>0$ 側になるよう鏡像を固定

3 で「最初の点」ではなく「最大の点」を使うのは、直交成分がほぼ 0 の点に
当たったとき雑音で向きが決まってしまうため。逆に 4 で「最大」ではなく
「最初」なのは、箱型のような対称配置では複数の点の $|z|$ が並び、
雑音で選ばれ方が変わって**鏡像が反転する**ため。どちらも実装中に
実際に踏んだ罠で、テストで固定してある。

なお第 2 軸を「適当な固定ベクトル」から作ってはいけない。第 1 軸まわりの
回転が残り、同じ配置でも入力座標系によって別の答えになる。

### 実座標への位置合わせ

自己測量の座標系は任意なので、実測した 3 台以上に重ねる (Kabsch 法)。

$$M = X_\text{src}^\top X_\text{dst} = U\Sigma V^\top \ \Longrightarrow\ R = VU^\top$$

`align_to_reference` がこれ。全台を巻き尺で測る必要はなく、
高さの違う 4 台を実測すれば、残りは自己測量に任せられる。

---

## 10. アンテナ遅延の推定

実装: `uwb_loc/calibration.py:estimate_antenna_delays`

測距値は $r = d + \delta_\text{anchor} + \delta_\text{tag}$ と書ける。既知距離での測定を
集めれば線形最小二乗で分離できる…が、$\delta_\text{anchor}$ と $\delta_\text{tag}$ には
**定数分の不定性がある** (全アンカーの遅延を $+c$、タグを $-c$ しても同じ)。

そこで**アンカー遅延の平均を 0 に固定**する拘束を 1 行足して解く。
残りはタグ側に寄る。

未補正のアンテナ遅延は数十 cm の系統誤差になる。距離バイアスの 1 次モデル
$d_\text{true} \approx \alpha\,r_\text{meas} + \beta$ は `fit_range_bias` で当てられる。

---

## 11. 実測での比較

同一の観測列 (8 台立体配置、$\sigma_0 = 8$ cm、10 Hz、8 の字軌道、5 seed 平均)。

| NLOS 率 | Lv0 | Lv1 | Lv2 | Lv3 |
|---|---|---|---|---|
| 0 % | 0.193 | 0.177 | 0.177 | **0.116** |
| 15 % | 0.607 | 0.376 | 0.311 | **0.172** |
| 35 % | 1.049 | 0.799 | 0.637 | **0.328** |

(RMSE 3D [m])

見通しが良ければ Lv1 と Lv2 に差はない。**ロバスト化が効くのは NLOS があるとき
だけ**で、しかも NLOS 率が上がるほど差が開く。

Lv2 の内訳 (NLOS 15%、4 seed 平均、RMSE 3D [m])。

| 構成 | RMSE | CEP95 |
|---|---|---|
| Lv1 (ロバスト化なし) | 0.356 | 0.251 |
| + Huber (両側) | 0.350 | 0.199 |
| + Huber + 片側損失 = **Lv2 既定** | **0.297** | **0.186** |
| 参考: 無条件に $3\sigma$ で RANSAC 事前間引き | 0.401 | 0.227 |
| 参考: Huber を Tukey に変更 | 0.474 | 0.337 |

効いているのは Huber ではなく**片側損失**の方 (0.350 → 0.297)。
NLOS が正側にしか出ないという物理を入れて初めて差が出る、と読める。

最後から 2 行目が §5.5 で述べた失敗で、これが常時 RANSAC をやめた理由。
条件付きにした現在の実装は、この条件 (NLOS 15%、8 台) では**一度も起動しない** —
規格化残差が 3 を超えないため。それが意図した挙動で、
RANSAC は本当に壊れたときのためだけに置いてある。

---

## 12. C への移植

必要になる線形代数は次の 3 つだけ。

| 用途 | 大きさ | 備考 |
|---|---|---|
| 正規方程式 $(J^\top WJ)\Delta = J^\top We$ | 3×3 | LU 分解で十分 |
| Beck 法の $\lambda$ 下限 (一般化固有値) | 4×4 | 実装が面倒なら十分小さい負値から二分法で探しても実用上同じ |
| EKF の共分散 | 6×6 | 逐次スカラー更新なので**逆行列は不要**、スカラー除算だけ |

移植の順序は

1. `model.py` の残差とヤコビアンをそのまま写す
2. `solve_nls` の Gauss-Newton ループ (LM 減衰込み)
3. `Lv3TightlyCoupledEKF` の predict / update

動作の照合は、同じ JSON Lines を Python 版と C 版に食わせて `Fix` を
突き合わせるのが早い。
