import sys
from playwright.sync_api import sync_playwright
src, dst = sys.argv[1], sys.argv[2]
with sync_playwright() as p:
    b = p.chromium.launch(executable_path="/opt/pw-browsers/chromium" if False else None)
    pg = b.new_page()
    pg.goto("file://" + src); pg.wait_for_timeout(1500)
    pg.evaluate("document.documentElement.setAttribute('data-print','1')")
    pg.pdf(path=dst, format="A4", print_background=True, margin=dict(top="18mm", bottom="18mm", left="16mm", right="16mm"),
           display_header_footer=True, header_template="<span></span>",
           footer_template='<div style="width:100%;font-size:8px;color:#888;padding:0 16mm;display:flex;justify-content:space-between"><span>Gr2 — Cohérence simulateur–optimiseur et guide d’utilisation</span><span class="pageNumber"></span></div>')
    b.close()
