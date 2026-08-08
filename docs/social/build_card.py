"""SNS 用のカード画像 (OG image) を生成する.

    pip install playwright        # ブラウザ本体は同梱済みのものを使う
    python docs/social/build_card.py

出力

    docs/social/og-card.png       1280 x 640  リポジトリの Social preview 用
    docs/social/og-card.html      その元 HTML (デバッグ用)

絵は**実際にライブラリを走らせた出力**そのもの。飾りではない.

* アンカー配置は ``room_anchors()`` の既定 (四隅に上下 2 段)
* 灰色の線が真の軌跡、色付きが Lv0 と Lv3 の推定 — シミュレータが吐いた
  観測列を ``run_offline()`` に通した結果をそのまま描いている
* 細い円は、ある 1 エポックで実際に得られた測距値。**わざと綺麗に
  1 点で交わらない**のが本題で、この食い違いを最小二乗とフィルタで
  詰めるのがこのライブラリの仕事

数値 (CEP50 など) も同じ実行から取るので、宣伝と中身がずれない.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

import numpy as np

import uwb_loc as ul

HERE = Path(__file__).resolve().parent
W, H = 1280, 640

ROOM = (8.0, 6.0, 2.6)
SEED = 7
DURATION = 24.0
EPOCH = 150         # 測距円を描くエポック (タグが中央寄りに来る時刻)
TAG_Z = 1.2

# 図の描画領域 (カード右側)
ART_X, ART_Y, ART_W, ART_H = 700, 62, 540, 516


# --------------------------------------------------------------------------- 計算


def run():
    """実際にシミュレータと測位器を回して、描く材料を作る."""
    anchors = ul.room_anchors(ROOM)
    hal = ul.SimulatedHal(
        anchors,
        ul.trajectory.figure8([ROOM[0] / 2, ROOM[1] / 2, TAG_Z]),
        ul.ErrorModel(nlos_prob=0.15),
        rate_hz=10.0,
        seed=SEED,
    )
    _, truth, batches = hal.generate(DURATION)
    truth = np.array(truth)
    tracks = {
        lv: np.array([f.position for f in ul.run_offline(batches, anchors, level=lv)])
        for lv in ("Lv0", "Lv3")
    }
    stats = {lv: ul.error_stats(truth, p) for lv, p in tracks.items()}
    return anchors, truth, tracks, stats, batches[EPOCH]


# --------------------------------------------------------------------------- 作図


class Frame:
    """部屋の座標 [m] を絵の座標 [px] に写す (縦横比を保つ)."""

    def __init__(self) -> None:
        s = min(ART_W / ROOM[0], ART_H / ROOM[1])
        self.s = s
        self.ox = ART_X + (ART_W - s * ROOM[0]) / 2
        self.oy = ART_Y + (ART_H + s * ROOM[1]) / 2

    def x(self, v: float) -> float:
        return self.ox + v * self.s

    def y(self, v: float) -> float:
        return self.oy - v * self.s


def path_of(fr: Frame, pts: np.ndarray) -> str:
    """xy 平面に落とした軌跡を SVG path にする (NaN で切る)."""
    out, pen = [], False
    for p in pts:
        if not np.all(np.isfinite(p)):
            pen = False
            continue
        cmd = "L" if pen else "M"
        out.append(f"{cmd} {fr.x(p[0]):.1f} {fr.y(p[1]):.1f}")
        pen = True
    return " ".join(out)


def build_svg(anchors, truth, tracks, batch) -> str:
    fr = Frame()

    # 部屋の枠と方眼
    grid = "".join(
        f'<line x1="{fr.x(v):.1f}" y1="{fr.y(0):.1f}" '
        f'x2="{fr.x(v):.1f}" y2="{fr.y(ROOM[1]):.1f}" '
        f'stroke="#1d2c3d" stroke-width="1"/>'
        for v in np.arange(1, ROOM[0])
    ) + "".join(
        f'<line x1="{fr.x(0):.1f}" y1="{fr.y(v):.1f}" '
        f'x2="{fr.x(ROOM[0]):.1f}" y2="{fr.y(v):.1f}" '
        f'stroke="#1d2c3d" stroke-width="1"/>'
        for v in np.arange(1, ROOM[1])
    )
    rect = (f'x="{fr.x(0):.1f}" y="{fr.y(ROOM[1]):.1f}" '
            f'width="{ROOM[0]*fr.s:.1f}" height="{ROOM[1]*fr.s:.1f}"')
    room = f'<rect {rect} fill="none" stroke="#2b4157" stroke-width="1.5"/>'
    # 測距円は半径が数 m あってカード全体にはみ出すので、部屋の中だけに切る。
    # 部屋の中を走る円弧だけが残り、それが交わる所がタグ、という絵になる。
    clip = f'<defs><clipPath id="room"><rect {rect}/></clipPath></defs>' 

    # 測距円: 実際の測距値を水平距離に直したもの。
    # 高さの差ぶんを抜くので、平面図として幾何的に正しい半径になる。
    by_id = {a.id: a for a in anchors}
    circles = []
    for m in sorted(batch.measurements, key=lambda m: m.value)[:5]:
        a = by_id.get(m.anchor_id)
        if a is None:
            continue
        horiz2 = m.value**2 - (a.position[2] - TAG_Z) ** 2
        if horiz2 <= 0:
            continue
        circles.append(
            f'<circle cx="{fr.x(a.position[0]):.1f}" cy="{fr.y(a.position[1]):.1f}" '
            f'r="{np.sqrt(horiz2)*fr.s:.1f}" fill="none" stroke="#4aa3ff" '
            f'stroke-width="1.3" stroke-opacity="0.55"/>'
        )

    # 軌跡: 真値 → Lv0 → Lv3 の順に重ねる
    paths = (
        f'<path d="{path_of(fr, truth)}" fill="none" stroke="#8b97a6" '
        f'stroke-width="3" stroke-opacity="0.75" stroke-linejoin="round"/>'
        f'<path d="{path_of(fr, tracks["Lv0"])}" fill="none" stroke="#f2994a" '
        f'stroke-width="1.3" stroke-opacity="0.62" stroke-linejoin="round"/>'
        f'<path d="{path_of(fr, tracks["Lv3"])}" fill="none" stroke="#c084fc" '
        f'stroke-width="2" stroke-opacity="0.95" stroke-linejoin="round"/>'
    )

    # アンカー (上下 2 段は平面図で重なるので、下段は少し小さく描く)
    marks = "".join(
        f'<rect x="{fr.x(a.position[0])-5:.1f}" y="{fr.y(a.position[1])-5:.1f}" '
        f'width="10" height="10" fill="#4aa3ff" stroke="#0b1119" stroke-width="2"/>'
        if a.position[2] > ROOM[2] / 2 else
        f'<rect x="{fr.x(a.position[0])-3.5:.1f}" y="{fr.y(a.position[1])-3.5:.1f}" '
        f'width="7" height="7" fill="none" stroke="#4aa3ff" stroke-width="1.6" '
        f'stroke-opacity="0.7"/>'
        for a in anchors
    )

    # いま推定している位置
    tag = tracks["Lv3"][EPOCH]
    tag_mark = ""
    if np.all(np.isfinite(tag)):
        # 円弧が集まる所がタグ、というのがこの絵の主題なので、
        # 交点だけは軌跡に埋もれないよう暗いハローを敷いてから描く。
        cx, cy = fr.x(tag[0]), fr.y(tag[1])
        tag_mark = (
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="26" fill="#080d12" '
            f'fill-opacity="0.55"/>'
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="21" fill="none" '
            f'stroke="#c084fc" stroke-width="1.4" stroke-opacity="0.65"/>'
            f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="7" fill="#c084fc" '
            f'stroke="#080d12" stroke-width="2"/>'
        )

    return (f'<svg width="{W}" height="{H}" viewBox="0 0 {W} {H}">{clip}'
            f'{grid}{room}'
            f'<g clip-path="url(#room)">{"".join(circles)}</g>'
            f"{paths}{marks}{tag_mark}</svg>")


# --------------------------------------------------------------------------- HTML

CARD = """<!doctype html>
<html><head><meta charset="utf-8"><style>
  * {{ margin:0; padding:0; box-sizing:border-box; }}
  body {{ width:{W}px; height:{H}px; overflow:hidden;
    background:
      radial-gradient(980px 620px at 76% 48%, #10202e 0%, transparent 64%),
      linear-gradient(158deg, #080d12 0%, #0b141c 55%, #080e14 100%);
    color:#e7eef5;
    font-family:"Liberation Sans","DejaVu Sans","Noto Sans CJK JP",
      "IPAGothic","Hiragino Sans",sans-serif;
    position:relative; }}
  .art {{ position:absolute; inset:0; }}
  .wrap {{ position:relative; padding:58px 62px; height:100%;
    display:flex; flex-direction:column; justify-content:space-between;
    width:700px; }}
  .eyebrow {{ font-family:"DejaVu Sans Mono","Liberation Mono",monospace;
    font-size:18px; letter-spacing:.28em; text-transform:uppercase;
    color:#5f93c9; }}
  .rule {{ height:4px; width:92px; margin-top:22px;
    background:repeating-linear-gradient(90deg,#4aa3ff 0 3px,transparent 3px 8px); }}
  h1 {{ font-size:60px; line-height:1.18; font-weight:800;
    letter-spacing:-.022em; margin-top:28px; }}
  h1 .hl {{ color:#7cc0ff; }}
  .sub {{ font-size:22px; line-height:1.6; color:#9fb0c2;
    max-width:560px; margin-top:22px; }}
  .stats {{ display:flex; gap:44px; align-items:flex-end; }}
  .stat .v {{ font-family:"DejaVu Sans Mono","Liberation Mono",monospace;
    font-size:36px; font-weight:700; color:#fff; letter-spacing:-.02em; }}
  .stat .v span {{ font-size:16px; color:#7c8b9d; margin-left:4px; }}
  .stat .k {{ font-size:14px; white-space:nowrap; color:#7c8b9d; margin-top:5px; }}
  .repo {{ position:absolute; right:44px; bottom:34px;
    font-family:"DejaVu Sans Mono","Liberation Mono",monospace;
    font-size:19px; color:#5f93c9; }}
  .legend {{ position:absolute; right:44px; top:34px; display:flex; gap:18px;
    font-size:14px; color:#8595a8; align-items:center; }}
  .legend i {{ display:inline-block; width:16px; height:3px; margin-right:7px;
    vertical-align:3px; border-radius:2px; }}
</style></head><body>
  <div class="art">{svg}</div>
  <div class="legend">
    <span><i style="background:#8b97a6"></i>真値</span>
    <span><i style="background:#f2994a"></i>Lv0</span>
    <span><i style="background:#c084fc"></i>Lv3</span>
  </div>
  <div class="wrap">
    <div>
      <div class="eyebrow">Range-only Localization</div>
      <div class="rule"></div>
      <h1>距離だけで、<br><span class="hl">位置</span>を割り出す</h1>
      <div class="sub">{sub}</div>
    </div>
    <div class="stats">{stats}</div>
  </div>
  <div class="repo">github.com/kouhei1970/uwb_localizer</div>
</body></html>"""

SUB = ("チップに依存しない UWB 屋内測位ライブラリ。"
       "HAL から測距値をもらい、位置・共分散・品質指標を返す。")


def main() -> None:
    anchors, truth, tracks, stats, batch = run()
    svg = build_svg(anchors, truth, tracks, batch)

    # 数値は同じ実行から取る (宣伝と中身をずらさない)
    stat_items = [
        ("4", "段階のアルゴリズム", ""),
        (f"{stats['Lv3']['cep50']*100:.0f}", "CEP50 / NLOS 15% の屋内", "cm"),
        ("1", "依存パッケージ (numpy)", ""),
    ]
    stats_html = "\n".join(
        f'<div class="stat"><div class="v">{v}<span>{u}</span></div>'
        f'<div class="k">{k}</div></div>' for v, k, u in stat_items)

    html = CARD.format(W=W, H=H, svg=svg, sub=SUB, stats=stats_html)
    src = HERE / "og-card.html"
    src.write_text(html, encoding="utf-8")

    from playwright.sync_api import sync_playwright

    out = HERE / "og-card.png"
    # 同梱の Chromium を明示的に指す (playwright のビルド番号と一致しないため)
    exe = next((str(p) for p in (
        Path("/opt/pw-browsers/chromium-1194/chrome-linux/chrome"),
        Path("/opt/pw-browsers/chromium/chrome-linux/chrome"),
    ) if p.is_file()), None)
    with sync_playwright() as pw:
        b = pw.chromium.launch(executable_path=exe)
        pg = b.new_page(viewport={"width": W, "height": H}, device_scale_factor=1)
        pg.goto(src.as_uri())
        pg.wait_for_timeout(400)
        pg.screenshot(path=str(out))
        b.close()
    print(f"-> {out}  ({W}x{H})")
    print(f"   Lv0 CEP50 {stats['Lv0']['cep50']*100:.1f} cm / "
          f"Lv3 CEP50 {stats['Lv3']['cep50']*100:.1f} cm")


if __name__ == "__main__":
    main()
