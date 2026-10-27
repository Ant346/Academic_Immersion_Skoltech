"""Run against the local application from the QA Docker image."""
from pathlib import Path
from playwright.sync_api import sync_playwright

with sync_playwright() as playwright:
    browser = playwright.chromium.launch(executable_path="/usr/bin/chromium", args=["--no-sandbox"])
    page = browser.new_page(viewport={"width": 1440, "height": 1100}, device_scale_factor=1)
    errors = []
    page.on("pageerror", lambda error: errors.append(str(error)))
    page.goto("http://127.0.0.1:8080", wait_until="networkidle")
    page.wait_for_function("document.querySelector('#camera-name').value === '/dev/camera'")
    assert "Jabra" in page.locator("#camera-name").inner_text()
    assert page.locator("#capture").is_disabled()
    assert not page.locator("#confirmed").is_checked()
    Path("data/qa").mkdir(parents=True, exist_ok=True)
    page.screenshot(path="data/qa/desktop.png", full_page=True)
    page.locator("#connect").click()
    page.wait_for_function("document.querySelector('#connection').classList.contains('on')", timeout=20000)
    page.wait_for_function("document.querySelector('#preview').naturalWidth > 0", timeout=20000)
    assert "1920 × 1080" in page.locator("#camera-actual").inner_text()
    assert page.locator("#capture").is_disabled()  # dimensions/optics not confirmed
    page.locator("#connect").click()
    page.wait_for_function("!document.querySelector('#connection').classList.contains('on')", timeout=20000)
    # Verify result rendering using an intercepted response, without writing
    # fictitious calibration parameters to the user's real session.
    fixture = page.request.get("http://127.0.0.1:8080/api/state").json()
    fixture["result"] = {"rms_px": .42, "image_width": 1920, "image_height": 1080,
                         "frame_count": 25, "camera_matrix": [[1100, 0, 960], [0, 1100, 540], [0, 0, 1]],
                         "distortion_coefficients": [-.1, .02, 0, 0, .01],
                         "warnings": ["Пример предупреждения для проверки интерфейса."]}
    page.route("**/api/state", lambda route: route.fulfill(json=fixture))
    page.wait_for_function("document.querySelector('#rms').textContent === '0.420'")
    assert page.locator("#result").is_visible()
    assert "1100.000" in page.locator("#matrix").inner_text()
    assert page.locator("#distortion > div").count() == 5
    assert page.locator("a[href='/api/export/yaml']").is_visible()
    page.unroute("**/api/state")
    page.wait_for_function("document.querySelector('#result').hidden")
    page.set_viewport_size({"width": 390, "height": 844})
    page.screenshot(path="data/qa/mobile.png", full_page=True)
    assert page.evaluate("document.documentElement.scrollWidth <= innerWidth"), "Mobile layout overflows"
    assert not errors, errors
    browser.close()
    print("Browser: USB connection, preview, capture guard, desktop/mobile layouts, JS errors: OK")
