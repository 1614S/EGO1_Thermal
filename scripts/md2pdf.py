#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
md2pdf.py —— 把设计报告 markdown 转成提交用的 PDF

为什么要有：初赛要求交「设计报告.pdf」（核心文档），而仓库里只有 .md。
手工复制到 Word 再导出，一是慢、二是每改一次都要重来。

实现：
    markdown -> HTML（内嵌 CSS，A4 排版、中文字体、表格/代码块样式）
    -> Edge 无头模式 --print-to-pdf

依赖：
    pip install markdown
    Edge / Chrome（脚本自动探测常见安装路径）

用法：
    python scripts/md2pdf.py docs/07_设计报告_初稿.md -o docs/设计报告.pdf
    python scripts/md2pdf.py docs/07_设计报告_初稿.md -o out.pdf --title "XXX 设计报告"

注意：
    - 图片按 markdown 里的相对路径解析，所以 HTML 会生成在 md 同目录下；
    - 生成的 .html 是中间产物，默认保留（想删加 --clean）。
"""

import argparse
import os
import subprocess
import sys
import tempfile

CSS = """
@page {
    size: A4;
    margin: 18mm 16mm 18mm 16mm;
}
body {
    font-family: "Microsoft YaHei", "PingFang SC", "Source Han Sans SC", sans-serif;
    font-size: 10.5pt;
    line-height: 1.65;
    color: #1a1a1a;
    margin: 0;
}
h1 { font-size: 20pt; margin: 0 0 8pt 0; padding-bottom: 6pt; border-bottom: 2px solid #333; }
h2 { font-size: 14pt; margin: 18pt 0 8pt 0; padding-left: 6pt; border-left: 4px solid #2c6cb0; }
h3 { font-size: 12pt; margin: 12pt 0 6pt 0; color: #24486e; }
h4 { font-size: 11pt; margin: 10pt 0 4pt 0; }
p  { margin: 5pt 0; text-align: justify; }
table {
    border-collapse: collapse;
    width: 100%;
    margin: 8pt 0;
    font-size: 9.5pt;
    page-break-inside: avoid;
}
th, td { border: 1px solid #bbb; padding: 3pt 5pt; vertical-align: top; }
th { background: #eef3f8; font-weight: 600; }
tr:nth-child(even) td { background: #fafbfc; }
code {
    font-family: Consolas, "Courier New", monospace;
    font-size: 9pt;
    background: #f4f4f4;
    padding: 0 2pt;
    border-radius: 2pt;
}
pre {
    background: #f7f7f7;
    border: 1px solid #e0e0e0;
    border-left: 3px solid #2c6cb0;
    padding: 6pt 8pt;
    overflow-x: auto;
    page-break-inside: avoid;
}
pre code { background: none; font-size: 8.5pt; }
blockquote {
    margin: 8pt 0;
    padding: 4pt 10pt;
    border-left: 3px solid #f0a020;
    background: #fffaf0;
    color: #444;
}
img { max-width: 100%; page-break-inside: avoid; }
hr { border: none; border-top: 1px solid #ddd; margin: 14pt 0; }
ul, ol { margin: 5pt 0 5pt 18pt; padding: 0; }
li { margin: 2pt 0; }
"""


def find_browser():
    cands = [
        r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
        r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
        r"C:\Program Files\Google\Chrome\Application\chrome.exe",
        r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    ]
    for c in cands:
        if os.path.exists(c):
            return c
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("md", help="输入 markdown")
    ap.add_argument("-o", "--out", required=True, help="输出 PDF")
    ap.add_argument("--title", default="", help="覆盖 <title>")
    ap.add_argument("--clean", action="store_true", help="删除中间 HTML")
    args = ap.parse_args()

    try:
        import markdown
    except ImportError:
        print("缺 markdown：pip install markdown", file=sys.stderr)
        return 3

    md_path = os.path.abspath(args.md)
    if not os.path.exists(md_path):
        print("找不到 %s" % md_path, file=sys.stderr)
        return 2
    src_dir = os.path.dirname(md_path)
    text = open(md_path, encoding="utf-8").read()

    html_body = markdown.markdown(
        text,
        extensions=["tables", "fenced_code", "toc", "sane_lists", "nl2br"],
    )
    title = args.title or os.path.splitext(os.path.basename(md_path))[0]
    html = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>%s</title><style>%s</style></head><body>%s</body></html>""" % (title, CSS, html_body)

    html_path = os.path.splitext(md_path)[0] + ".html"
    open(html_path, "w", encoding="utf-8").write(html)
    print("HTML -> %s" % html_path)

    browser = find_browser()
    if browser is None:
        print("没找到 Edge/Chrome，HTML 已生成，可手动打印成 PDF", file=sys.stderr)
        return 4

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    cmd = [
        browser,
        "--headless=new",
        "--disable-gpu",
        "--no-pdf-header-footer",
        "--print-to-pdf=%s" % out_path,
        "file:///" + html_path.replace("\\", "/"),
    ]
    print("调用浏览器打印……")
    r = subprocess.run(cmd, capture_output=True, timeout=180)
    if not os.path.exists(out_path) or os.path.getsize(out_path) < 1000:
        print("PDF 生成失败（exit=%d）" % r.returncode, file=sys.stderr)
        print(r.stderr.decode("utf-8", "ignore")[-800:], file=sys.stderr)
        return 5

    size = os.path.getsize(out_path)
    print("PDF  -> %s  (%.0f KB)" % (out_path, size / 1024.0))
    if args.clean:
        os.remove(html_path)
        print("已删除中间 HTML")
    return 0


if __name__ == "__main__":
    sys.exit(main())
