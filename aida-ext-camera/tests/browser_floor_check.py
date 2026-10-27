"""Browser smoke test for step 2; leaves real observations and intrinsics intact."""
import argparse
from pathlib import Path
from playwright.sync_api import sync_playwright

parser = argparse.ArgumentParser()
parser.add_argument("--base", default="http://127.0.0.1:8080")
parser.add_argument("--camera", action="store_true", help="Also connect the USB camera and verify the stream")
args = parser.parse_args()

with sync_playwright() as p:
    browser = p.chromium.launch(executable_path="/usr/bin/chromium", args=["--no-sandbox"])
    page = browser.new_page(viewport={"width": 1440, "height": 1100})
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    original = page.request.get(args.base+"/api/state").json()
    page.goto(args.base+"/floor.html", wait_until="domcontentloaded")
    page.wait_for_function("document.querySelector('#floor-intrinsics').textContent.includes('px')")
    assert page.locator("#floor-capture").is_disabled()
    if args.camera:
        if "Отключить" not in page.locator("#floor-connect").inner_text():
            page.locator("#floor-connect").click()
        page.wait_for_function("document.querySelector('#floor-connection').classList.contains('on')", timeout=20000)
        page.wait_for_function("document.querySelector('#floor-preview').naturalWidth > 0", timeout=20000)
    Path("artifacts/floor").mkdir(parents=True, exist_ok=True)
    page.screenshot(path="artifacts/floor/desktop.png", full_page=True)
    page.set_viewport_size({"width": 390, "height": 844})
    page.screenshot(path="artifacts/floor/mobile.png", full_page=True)
    assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
    current = page.request.get(args.base+"/api/state").json()
    assert current["result"] == original["result"]
    assert current["samples"] == original["samples"]
    assert not errors, errors
    browser.close()
    print("Floor page, capture guard, desktop/mobile, preserved intrinsics: OK")
