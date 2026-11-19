import asyncio, sys
from playwright.async_api import async_playwright
async def main():
    async with async_playwright() as p:
        b=await p.chromium.launch(executable_path='/opt/pw-browsers/chromium') if False else await p.chromium.launch()
        pg=await b.new_page(viewport={'width':1300,'height':900})
        errs=[]; pg.on('console',lambda m: errs.append(m.text) if m.type=='error' else None); pg.on('pageerror',lambda e: errs.append(str(e)+' '+str(e.stack)[:600]))
        await pg.goto('file:///home/claude/gr2_dim/pattes/pattes_gr2.html'); await pg.wait_for_timeout(1500)
        await pg.screenshot(path='/tmp/p1.png',full_page=True)
        print(errs)
        pg2=await b.new_page(viewport={'width':390,'height':800}); await pg2.goto('file:///home/claude/gr2_dim/pattes/pattes_gr2.html'); await pg2.wait_for_timeout(1000)
        print(await pg2.evaluate('document.documentElement.scrollWidth'))
        await b.close()
asyncio.run(main())
