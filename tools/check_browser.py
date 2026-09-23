"""Exercise compatibility fallbacks and the live GPU path in Chromium.

Requires Python Playwright and Chromium. Run: python tools/check_browser.py
Pass --gpu to require the real WebGPU simulation to produce a frame as well.
"""
import argparse
import json
import mimetypes
from pathlib import Path
from urllib.parse import urlsplit
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
URL = 'https://tront.xyz/vibesand/'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gpu', action='store_true')
    parser.add_argument('--channel', help='Use an installed browser, for example chrome')
    args = parser.parse_args()
    output = ROOT / 'artifacts'
    output.mkdir(exist_ok=True)
    cases = {
        'no-webgpu': 'undefined',
        'no-adapter': '{ requestAdapter: async () => null }',
        'adapter-rejected': '{ requestAdapter: async () => { throw new Error("Driver unavailable"); } }',
        'device-rejected': '{ requestAdapter: async () => ({ requestDevice: async () => { throw new Error("Device unavailable"); } }) }',
        'no-device': '{ requestAdapter: async () => ({ requestDevice: async () => null }) }',
    }
    results = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True, channel=args.channel,
            args=['--enable-unsafe-webgpu', '--ignore-gpu-blocklist', '--use-angle=d3d11'])

        def route(request):
            parsed = urlsplit(request.request.url)
            if parsed.netloc == 'tront.xyz' and parsed.path.startswith('/vibesand/'):
                path = ROOT / (parsed.path[len('/vibesand/'):] or 'index.html')
                if path.resolve().is_relative_to(ROOT) and path.is_file():
                    request.fulfill(status=200, body=path.read_bytes(), content_type=mimetypes.guess_type(path.name)[0] or 'application/octet-stream')
                    return
            # Analytics and unrelated outbound requests are unnecessary for this check.
            request.fulfill(status=204)

        for name, gpu in list(cases.items()) + [('javascript-disabled', None), ('mobile-no-webgpu', 'undefined')]:
            context = browser.new_context(java_script_enabled=name != 'javascript-disabled',
                viewport={'width': 390 if name.startswith('mobile') else 1280, 'height': 900})
            context.route('**/*', route)
            if gpu is not None:
                context.add_init_script('Object.defineProperty(navigator, "gpu", { configurable: true, value: ' + gpu + ' });')
            page = context.new_page()
            errors = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.goto(URL, wait_until='load')
            if name != 'javascript-disabled':
                page.locator('#compatibilityNotice').wait_for(state='visible')
            assert page.locator('#descriptionArea').is_visible()
            assert page.locator('h1').inner_text().startswith('VibeSand')
            assert 'millions of particles' in page.locator('#descriptionArea').inner_text()
            assert page.locator('.demo-preview img').evaluate('(image) => image.complete && image.naturalWidth > 0')
            assert page.locator('link[rel="canonical"]').get_attribute('href') == URL
            assert page.locator('a[href="https://github.com/TrentSterling/vibesand"]').count() == 1
            assert not errors, errors
            assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
            page.screenshot(path=str(output / (name + '.png')))
            results.append({'case': name, 'passed': True, 'pageErrors': errors,
                            'descriptionCharacters': len(page.locator('#descriptionArea').inner_text())})
            (output / 'browser-results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
            print(json.dumps(results[-1]), flush=True)
            context.close()
        if args.gpu:
            context = browser.new_context(viewport={'width': 1280, 'height': 900})
            context.route('**/*', route)
            page = context.new_page()
            errors = []
            messages = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.on('console', lambda message: messages.append(message.text[:1000]))
            page.goto(URL, wait_until='load')
            try:
                page.wait_for_function('document.querySelector("#fps").textContent.length > 0', timeout=20000)
            except Exception:
                page.screenshot(path=str(output / 'gpu-failure.png'))
                failure = {'case': 'real-webgpu', 'passed': False, 'pageErrors': errors, 'console': messages,
                           'notice': page.locator('#compatibilityNotice').inner_text()}
                results.append(failure)
                (output / 'browser-results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
                print(json.dumps(failure), flush=True)
                raise
            assert page.locator('#webgpu-canvas').is_visible()
            assert not page.locator('#compatibilityNotice').is_visible()
            assert page.locator('#voxelCount').inner_text() == '2,097,152'
            assert not errors, errors
            page.screenshot(path=str(output / 'gpu-running.png'))
            results.append({'case': 'real-webgpu', 'passed': True, 'voxels': page.locator('#voxelCount').inner_text(),
                            'fps': page.locator('#fps').inner_text(), 'pageErrors': errors})
            context.close()
        browser.close()
    (output / 'browser-results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    print(json.dumps(results), flush=True)


if __name__ == '__main__':
    main()
