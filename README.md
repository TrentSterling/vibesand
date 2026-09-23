# vibesand

WebGPU voxel sandbox - 2M particles at 60fps

[![vibesand screenshot](https://tront.xyz/img/chrome_doQGkdV5Qr.png)](https://tront.xyz/vibesand/)

---

[Try it live ->](https://tront.xyz/vibesand/)

Built by [Trent (Tront) Sterling](https://tront.xyz) · [Games Portfolio](https://tront.xyz/games) · [Discord](https://tront.xyz/discord/)

## Browser checks

`python tools/check_browser.py` checks missing WebGPU, missing/rejected adapters and devices, JavaScript-disabled browsing, and the mobile fallback. It requires Python Playwright and its Chromium browser.

`python tools/check_browser.py --gpu --channel chrome` also checks that the real simulation produces frames in installed Chrome. Screenshots and results are written to `artifacts/`.

When WebGPU cannot start, the page keeps the project description, links and preview visible. The compatibility notice only stops the interactive simulation; it does not replace the document. The preview is the existing project screenshot from the portfolio.
