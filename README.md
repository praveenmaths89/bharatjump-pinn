# BharatJump PINN — Smartphone IMU + Physics-Informed vGRF Platform

1. **Web App (100% Client-Side PWA)**:
   - Open `bharatjump_pinn_standalone.html` directly on any phone or desktop browser, or serve `/workspace/jump-app/` over HTTPS (`npx serve` / GitHub Pages / Cloudflare Pages) for live iOS/Android `DeviceMotionEvent` 6-axis IMU capture.
   - Includes a **Simulate Demo Jump (Chest IMU)** selector with 4 Indian athlete presets (32 kg sub-junior to 74 kg senior national) and live comparison between:
     - `PINN Neural + Exact Impulse Anchor`
     - `PINN Neural Only (Unanchored)`
     - `Raw Chest IMU (1 + a_z/g)`
   - Includes **Export CSV (IMU+vGRF)** and **Export Trial JSON** for recording paired smartphone + VALD ForceDecks datasets.

2. **PyTorch PINN Training Pipeline (`train_vgrf_pinn.py`)**:
   - Architecture: `CMJForcePINN` (1D-ResNet + BiGRU + differentiable `ImpulseAnchoringLayer`).
   - Enforces Newton's impulse-momentum theorem:
     $$\int_{t_0}^{t_{\text{takeoff}}} (\text{vGRF}(t) - 1)g\,dt = v_{\text{takeoff}} = \frac{1}{2}g\,t_{\text{flight}}$$
   - Run synthetic verification:
     ```bash
     python3 train_vgrf_pinn.py --demo
     ```
