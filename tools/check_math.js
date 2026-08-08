/* Markdown 中の数式を KaTeX で実際にレンダリングして、通らないものを報告する。
 *
 *   npm install katex
 *   node tools/check_math.js docs/*.md
 *
 * tests/test_docs.py にも同じ趣旨の検査があるが、あちらは禁止コマンドの
 * 文字列一致だけ (依存を増やしたくないため)。こちらは**実際に描画してみる**
 * ので、書き間違いや対応の取れていない括弧まで見つかる。
 *
 * GitHub は KaTeX を使い、さらに **マクロを定義できるコマンドを禁止**している
 * (\def \newcommand \operatorname など)。strict:true だとその手のものが
 * 警告になるので、GitHub と同じように弾けるかを見る。
 */
const fs = require('fs');

let katex;
try {
  katex = require('katex');
} catch (e) {
  console.error('katex が見つかりません。次で入れてください:\n  npm install katex');
  console.error('(数式を書き換えないなら不要です。tests/test_docs.py が');
  console.error(' 禁止コマンドだけは依存なしで見ています)');
  process.exit(2);
}

// GitHub が禁止しているマクロ (エラーメッセージ
// "The following macros are not allowed: ..." に出るもの)
const BANNED = [
  '\\def', '\\gdef', '\\edef', '\\xdef', '\\let', '\\futurelet',
  '\\newcommand', '\\renewcommand', '\\providecommand',
  '\\global', '\\operatorname', '\\includegraphics',
];

const files = process.argv.slice(2);
let bad = 0, total = 0;

for (const file of files) {
  const lines = fs.readFileSync(file, 'utf8').split('\n');
  let inCode = false, inBlock = false, blockBuf = [], blockStart = 0;

  const check = (src, lineNo, display) => {
    total++;
    for (const m of BANNED) {
      // \operatorname* も拾う。単語境界で見る
      const re = new RegExp('\\' + m + '(?![a-zA-Z])');
      if (re.test(src)) {
        console.log(`${file}:${lineNo}  GitHub が禁止しているマクロ ${m}`);
        console.log(`    ${src.slice(0, 90)}`);
        bad++;
        return;
      }
    }
    try {
      katex.renderToString(src, { displayMode: display, throwOnError: true, strict: 'error' });
    } catch (e) {
      console.log(`${file}:${lineNo}  KaTeX エラー: ${String(e.message).slice(0, 120)}`);
      console.log(`    ${src.slice(0, 90)}`);
      bad++;
    }
  };

  lines.forEach((line, i) => {
    const lineNo = i + 1;
    if (line.trim().startsWith('```')) { inCode = !inCode; return; }
    if (inCode) return;

    if (line.trim() === '$$') {
      if (inBlock) { check(blockBuf.join('\n'), blockStart, true); blockBuf = []; }
      else { blockStart = lineNo + 1; }
      inBlock = !inBlock;
      return;
    }
    if (inBlock) { blockBuf.push(line); return; }

    // インラインコード (`...`) の中の $ は数式ではない。
    // 「`$` という文字」を説明している箇所を拾わないよう、同じ長さの空白に潰す。
    line = line.replace(/`[^`]*`/g, (m) => ' '.repeat(m.length));

    // インライン $...$
    const re = /\$([^$\n]+)\$/g;
    let m;
    while ((m = re.exec(line)) !== null) check(m[1], lineNo, false);
  });
}

console.log(`\n数式 ${total} 個中 ${bad} 個が GitHub で表示できない`);
process.exit(bad === 0 ? 0 : 1);
