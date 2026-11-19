"""Remplace la feuille KaTeX distante par une feuille inline (polices woff2 en base64). Usage: python inline_web.py in.html out.html katex_css_dir"""
import sys, re, base64, os
src, dst, kd = sys.argv[1:4]
css = open(os.path.join(kd, "katex.min.css"), encoding="utf-8").read()
def sub(m):
    p = os.path.join(kd, m.group(1))
    return "url(data:font/woff2;base64," + base64.b64encode(open(p, "rb").read()).decode() + ")"
css = re.sub(r"url\((fonts/[^)]+?\.woff2)\)\s*format\(\"woff2\"\)", lambda m: sub(m) + ' format("woff2")', css)
css = re.sub(r",url\(fonts/[^)]+\)\s*format\(\"(?:woff|truetype)\"\)", "", css)
h = open(src, encoding="utf-8").read()
h = re.sub(r'<link rel="stylesheet" href="https://cdnjs[^>]*>', lambda m: "<style>" + css + "</style>", h, 1)
assert "cdnjs" not in h and "fonts/KaTeX" not in h.replace("data:font", ""), "reste des urls externes"
open(dst, "w", encoding="utf-8").write(h); print(len(h) // 1024, "Ko")
