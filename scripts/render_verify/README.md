# Windows / low-end-GPU render verification (on a Mac)

The 4WIS portable package is a **localhost web app** — `start.bat` / `start.command`
just open the built frontend in the machine's **default browser**. So a Windows
"compatibility" or "flicker" report is almost always about *how the WebGL/Canvas
scene renders on that machine's browser + GPU*, not about the Python backend.

Macs render through **ANGLE → Metal**: 24-bit depth, hardware-fast. Many Windows
machines don't:

| Windows situation | WebGL backend | Symptom it causes |
| --- | --- | --- |
| VM / RDP / no GPU / blocklisted driver | **SwiftShader** (software) | very low FPS → 抖动/卡顿; context loss → black flashes |
| Older iGPU, some D3D9 fallbacks | 16-bit depth buffer | z-fighting shimmer on the vehicle's stacked surfaces |
| Modern Chrome/Edge + real GPU | ANGLE → D3D11, 24-bit | usually fine (matches the Mac) |

This folder reproduces the first two locally.

## Tier 1 — force Chrome onto a weak backend (fast, do this first)

```bash
scripts/render_verify/launch.sh software   # SwiftShader — the weak-Windows path
scripts/render_verify/launch.sh gl         # ANGLE desktop-GL backend
scripts/render_verify/launch.sh default    # hardware (Metal) baseline, for comparison
scripts/render_verify/launch.sh probe      # depth/FPS/backend probe page only
```

It starts a backend on `:8017` serving `frontend/dist` (build it first with
`npm run build`), then opens **Google Chrome** in an isolated profile forced onto
the chosen WebGL backend. Confirm the backend at `chrome://gpu` (WebGL row should
say *SwiftShader* in `software` mode). Drive the car, switch 2D/3D, and watch the
FPS + whether the viewport flickers. Closing the window stops everything.

`webgl_probe.html` is a standalone readout: **renderer string, DEPTH_BITS, granted
antialias, live FPS**, plus a 2 cm-at-12 m coplanar-quad z-fight test (solid
magenta = seam resolved; speckle = z-fighting). Open it directly, or via
`launch.sh probe`.

Verified working on this Mac: `launch.sh software` reports
`ANGLE (Google, Vulkan 1.3.0 (SwiftShader Device ...), SwiftShader driver)
[SOFTWARE FALLBACK]`.

> SwiftShader on Apple Silicon still exposes a **24-bit** depth buffer, so the
> pure z-fighting artifact needs a real 16-bit Windows box to see — but the
> software path faithfully reproduces the **performance cliff** (the biggest
> driver of the 抖动/卡顿 + context-loss flashing reports).

## Tier 2 — actual Windows (full fidelity, for sign-off)

For a faithful depth-precision + driver test, run the real `windows-x64` package
on Windows:

- **Any Windows PC / colleague's machine**: unzip, run `start.bat`, then in the
  opened browser visit `chrome://gpu` (or `edge://gpu`) and copy the "Graphics
  Feature Status" + WebGL renderer line. That single readout tells us hardware vs
  software and the depth path.
- **Windows 11 VM on Apple Silicon** (Parallels / UTM free): the x64 package runs
  under Windows-on-ARM x64 emulation; the VM's virtual GPU is a reasonable
  low/mid-tier stand-in. Heavier to set up — use only if no real Windows box is
  available.

## What to capture from a failing machine

1. `chrome://gpu` → "Graphics Feature Status" block + the WebGL/WebGL2 renderer line.
2. The probe page's top readout (renderer / DEPTH_BITS / FPS).
3. Whether the flicker is in **2D** (default view) or only after switching to **3D**.

That trio pins the root cause to the right layer without guesswork.
