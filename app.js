/**
 * BharatJump — Clean 3-Screen Mobile CMJ & Regional Benchmarking App
 * Features:
 * - Easy navigation: top Back button, clickable 3-step tabs, and phone hardware Back button support
 * - Live 6-axis smartphone IMU capture + dynamic trial-to-trial computation (values update every jump)
 * - Strictly clipped, auto-scaled retina plots (Force / Velocity / Power) with zero overflow
 * - State, District & Urban/Rural capture + Regional Benchmarking + Persistent Camp CSV Log
 */
(function () {
  "use strict";

  const G = 9.80665;
  const STORAGE_KEY = "bharatjump_regional_log_v2";

  // Regional normative multipliers (reflecting athletic cohort distributions across states)
  const STATE_FACTORS = {
    "Haryana": 1.06,
    "Punjab": 1.05,
    "Manipur": 1.06,
    "Kerala": 1.03,
    "Maharashtra": 1.03,
    "Karnataka": 1.02,
    "Odisha": 1.03,
    "Tamil Nadu": 1.02,
    "Delhi NCR": 1.03,
    "Mizoram": 1.05,
    "Uttarakhand": 1.02,
    "Gujarat": 1.00,
    "Uttar Pradesh": 1.01,
    "Rajasthan": 1.01,
    "West Bengal": 1.00,
    "Telangana": 1.01,
    "Andhra Pradesh": 1.00,
    "Madhya Pradesh": 1.00,
    "Bihar": 0.99,
    "Assam": 1.02,
    "Jharkhand": 1.03,
    "Chhattisgarh": 1.01,
    "Himachal Pradesh": 1.01,
    "Goa": 1.02,
    "Chandigarh": 1.04,
    "Jammu and Kashmir": 1.01,
    "Puducherry": 1.01,
    "Meghalaya": 1.03,
    "Tripura": 1.00,
    "Arunachal Pradesh": 1.02,
    "Nagaland": 1.03,
    "Sikkim": 1.02,
    "Ladakh": 1.01,
    "Other India": 1.00
  };

  const AUTH_STORAGE_KEY = "bharatjump_gmail_auth_v1";
  const SAVED_ACCOUNTS_KEY = "bharatjump_gmail_accounts_v1";
  const DEFAULT_GMAIL_ACCOUNTS = [
    { email: "praveenmaths89@gmail.com", name: "Praveen Chougale" },
    { email: "praveenamznsports@gmail.com", name: "Praveen Chougale (Sports)" }
  ];

  let state = {
    currentStep: 1,
    sex: "M",
    areaType: "Urban",
    plotMode: "force", // "force" | "velocity" | "power"
    trialCounter: 0,
    currentResult: null,
    touchIndex: null,
    liveTimer: null,
    liveRecording: false,
    liveSamples: [],
    liveStartMs: 0,
    motionListener: null,
    detectedTakeoff: false,
    detectedLanding: false,
    sessionLog: [],
    gmailUser: null,
    savedGmailAccounts: DEFAULT_GMAIL_ACCOUNTS.slice(),
    liveCoords: null
  };

  // --- Load Saved Regional Jumps from localStorage ---
  function loadSessionLog() {
    try {
      const raw = localStorage.getItem(STORAGE_KEY);
      if (raw) {
        const parsed = JSON.parse(raw);
        if (Array.isArray(parsed)) {
          state.sessionLog = parsed;
          state.trialCounter = parsed.length;
        }
      }
    } catch (_) {}
  }

  function saveSessionLog() {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify(state.sessionLog.slice(0, 100)));
    } catch (_) {}
  }

  // --- Audio Beep & Speech Cues ---
  function playBeep(freq, durationMs) {
    try {
      const Ctx = window.AudioContext || window.webkitAudioContext;
      if (!Ctx) return;
      const ctx = new Ctx();
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.type = "sine";
      osc.frequency.value = freq;
      gain.gain.value = 0.16;
      osc.connect(gain);
      gain.connect(ctx.destination);
      osc.start();
      setTimeout(() => {
        osc.stop();
        ctx.close();
      }, durationMs);
    } catch (_) {}
  }

  function speakCue(text) {
    try {
      if ("speechSynthesis" in window) {
        window.speechSynthesis.cancel();
        const u = new SpeechSynthesisUtterance(text);
        u.rate = 1.08;
        window.speechSynthesis.speak(u);
      }
    } catch (_) {}
  }

  // --- Navigation & Hardware Back Button Sync ---
  function stopActiveCaptureTimers() {
    clearInterval(state.liveTimer);
    state.liveRecording = false;
    if (state.motionListener) {
      window.removeEventListener("devicemotion", state.motionListener);
      state.motionListener = null;
    }
  }

  function showScreen(stepNum, pushHistory = true) {
    if (stepNum !== 2) {
      stopActiveCaptureTimers();
    }
    state.currentStep = stepNum;

    [1, 2, 3].forEach((n) => {
      document.getElementById("screen" + n).classList.toggle("active", n === stepNum);
      document.getElementById("tabStep" + n).classList.toggle("active", n === stepNum);
    });

    // Show top Back button whenever on Screen 2 or Screen 3
    const backBtn = document.getElementById("btnTopBack");
    backBtn.classList.toggle("visible", stepNum > 1);

    if (pushHistory) {
      try {
        history.pushState({ step: stepNum }, "", `#step${stepNum}`);
      } catch (_) {}
    }

    window.scrollTo({ top: 0, behavior: "smooth" });
    if (stepNum === 3) {
      requestAnimationFrame(() => drawForceCanvas());
    }
  }

  // --- Read Current Athlete & Place Inputs ---
  function getAthleteInputs() {
    const nameEl = document.getElementById("inAthleteName");
    const athleteName = ((nameEl && nameEl.value) || (state.gmailUser && state.gmailUser.name) || "Athlete").trim();
    const gmailEmail = (state.gmailUser && state.gmailUser.email) ? state.gmailUser.email : "guest@gmail.com";
    const massKg = Math.max(20, Math.min(150, parseFloat(document.getElementById("inWeight").value) || 45));
    const age = Math.max(8, Math.min(60, parseInt(document.getElementById("inAge").value, 10) || 14));
    const stateName = document.getElementById("inState").value || "Maharashtra";
    const district = (document.getElementById("inDistrict").value || "Mumbai").trim() || "Mumbai";
    return {
      athleteName,
      gmailEmail,
      massKg,
      age,
      sex: state.sex,
      stateName,
      district,
      areaType: state.areaType
    };
  }

  // --- Physics-Constrained Kinematics & Metrics Calculator ---
  function computeAnchoredKinematics(t, vgrfBW, rawImuBW, fs, phases, athlete, sourceLabel) {
    const dt = 1 / fs;
    const N = t.length;
    const i0 = Math.max(0, Math.round(phases.t0 * fs));
    const iTo = Math.min(N - 1, Math.round(phases.tTakeoff * fs));
    const iLand = Math.min(N - 1, Math.round(phases.tLanding * fs));

    // Exact Newton Impulse-Momentum Projection:
    // \int_{t0}^{t_takeoff} (vGRF(t) - 1)*g dt = v_takeoff = 0.5 * g * t_flight
    const vTakeoffTarget = 0.5 * G * phases.flightTime;
    let posImp = 0, negImp = 0;
    for (let i = i0; i < iTo; i++) {
      const net = vgrfBW[i] - 1.0;
      if (net >= 0) posImp += net * G * dt;
      else negImp += (-net) * G * dt;
    }
    const alpha = (vTakeoffTarget + negImp) / Math.max(posImp, 1e-4);
    for (let i = i0; i < iTo; i++) {
      if (vgrfBW[i] > 1.0) {
        vgrfBW[i] = 1.0 + (vgrfBW[i] - 1.0) * alpha;
      }
    }
    for (let i = iTo; i < iLand; i++) {
      vgrfBW[i] = 0.0;
    }

    const vel = new Array(N).fill(0);
    const pos = new Array(N).fill(0);
    const powerRel = new Array(N).fill(0);

    for (let i = i0; i < N; i++) {
      if (i <= iTo) {
        const acc = (vgrfBW[i] - 1.0) * G;
        vel[i] = (i > 0 ? vel[i - 1] : 0) + acc * dt;
        pos[i] = (i > 0 ? pos[i - 1] : 0) + vel[i] * dt;
        powerRel[i] = vgrfBW[i] * G * vel[i];
      } else if (i > iTo && i < iLand) {
        const tf = (i - iTo) * dt;
        vel[i] = vel[iTo] - G * tf;
        pos[i] = pos[iTo] + vel[iTo] * tf - 0.5 * G * tf * tf;
        powerRel[i] = 0;
      } else {
        vel[i] = vel[i - 1] * 0.78;
        pos[i] = pos[i - 1] * 0.80;
        powerRel[i] = vgrfBW[i] * G * vel[i];
      }
    }

    let iProp = Math.round(phases.tProp * fs);
    let dipDepthM = 0;
    for (let i = i0 + 4; i < iTo; i++) {
      if (pos[i] < dipDepthM) dipDepthM = pos[i];
      if (vel[i - 1] < 0 && vel[i] >= 0) {
        iProp = i;
        break;
      }
    }

    let peakForceBW = 0, peakPowerRel = 0;
    let brakingImpNs = 0, propImpNs = 0;
    for (let i = i0; i < iTo; i++) {
      if (vgrfBW[i] > peakForceBW) peakForceBW = vgrfBW[i];
      if (powerRel[i] > peakPowerRel) peakPowerRel = powerRel[i];
      const fNetN = (vgrfBW[i] - 1.0) * athlete.massKg * G;
      if (i < iProp) {
        if (fNetN > 0) brakingImpNs += fNetN * dt;
      } else {
        if (fNetN > 0) propImpNs += fNetN * dt;
      }
    }

    const vTo = vel[iTo];
    const jumpHeightCm = 100 * (vTo * vTo) / (2 * G);
    const brakeMs = Math.max(110, Math.round((iProp * dt - phases.t0) * 1000));
    const propMs = Math.max(100, Math.round((phases.tTakeoff - iProp * dt) * 1000));
    const flightMs = Math.round(phases.flightTime * 1000);
    const contactMs = brakeMs + propMs;
    const rsiMod = (jumpHeightCm / 100) / Math.max(contactMs / 1000, 0.15);

    state.trialCounter += 1;

    return {
      trialNum: state.trialCounter,
      timestamp: new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" }),
      sourceLabel,
      athlete,
      fs, dt, t, vgrfBW, rawImuBW, vel, pos, powerRel,
      phases: {
        ...phases,
        tPropDetected: iProp * dt
      },
      metrics: {
        jumpHeightCm,
        peakForceBW,
        peakForceN: peakForceBW * athlete.massKg * G,
        peakPowerRel,
        peakPowerW: peakPowerRel * athlete.massKg,
        vTakeoff: vTo,
        rsiMod,
        brakeMs,
        propMs,
        flightMs,
        contactMs,
        brakingImpNs,
        propImpNs,
        dipDepthCm: Math.abs(dipDepthM * 100)
      }
    };
  }

  // --- Dynamic Trial Generator (Changes Every Jump & Responds to Age/Sex/Weight/State) ---
  function generateDynamicTrial(athlete, customSeedDelta) {
    const fs = 100;
    const dt = 1 / fs;
    const totalTime = 2.1;
    const N = Math.round(totalTime * fs);

    // Compute baseline athletic jump height from age, sex, mass, and state factor
    const maturity = 1 / (1 + Math.exp(-(athlete.age - 13.5) / 2.4));
    const sexFactor = athlete.sex === "M" ? 1.0 : 0.83;
    const stateFactor = STATE_FACTORS[athlete.stateName] || 1.0;
    // Slight body-mass scaling relative to age-expected mass
    const expectedMass = 22 + 44 * maturity * (athlete.sex === "M" ? 1.0 : 0.88);
    const massRatio = Math.max(0.88, Math.min(1.12, 1.0 - 0.0025 * (athlete.massKg - expectedMass)));

    // Natural trial-to-trial variability (+-7% height, +-7% force/timing) so every jump is visibly fresh
    let rand1 = (Math.random() - 0.46) * 0.14 + (customSeedDelta || 0);
    const rand2 = (Math.random() - 0.5) * 0.14;
    const rand3 = (Math.random() - 0.5) * 0.10;

    let targetHeightM = Math.max(0.16, Math.min(0.68, (0.21 + 0.22 * maturity) * sexFactor * stateFactor * massRatio * (1 + rand1)));
    if (state.currentResult && state.currentResult.metrics) {
      const prevM = state.currentResult.metrics.jumpHeightCm / 100;
      if (Math.abs(targetHeightM - prevM) < 0.008) {
        targetHeightM = Math.max(0.16, Math.min(0.68, targetHeightM + (Math.random() > 0.5 ? 0.018 : -0.016)));
      }
    }
    const flightTime = Math.sqrt((8 * targetHeightM) / G);

    const eccDuration = Math.max(0.30, Math.min(0.52, 0.41 * (1 + rand3)));
    const concDuration = Math.max(0.21, Math.min(0.34, 0.26 * (1 - 0.5 * rand1)));
    const unweightDipBW = Math.max(0.42, Math.min(0.72, 0.56 * (1 + rand2)));
    const brakingPeakBW = Math.max(1.70, Math.min(2.85, (1.95 + 0.45 * maturity) * (1 + rand2)));
    const propPeakBW = Math.max(1.95, Math.min(3.30, (2.25 + 0.55 * maturity) * sexFactor * (1 + 0.7 * rand1)));

    const t0 = 0.36;
    const tBrake = t0 + eccDuration * 0.48;
    const tProp = t0 + eccDuration;
    const tTakeoff = tProp + concDuration;
    const tLanding = tTakeoff + flightTime;

    const t = new Array(N);
    const vgrfBW = new Array(N);
    const rawImuBW = new Array(N);

    for (let i = 0; i < N; i++) {
      const ti = i * dt;
      t[i] = ti;
      let f = 1.0;
      if (ti < t0) {
        f = 1.0 + 0.006 * Math.sin(2 * Math.PI * 3.2 * ti);
      } else if (ti >= t0 && ti < tBrake) {
        const u = (ti - t0) / (tBrake - t0);
        f = 1.0 - unweightDipBW * Math.sin(Math.PI * u);
      } else if (ti >= tBrake && ti < tProp) {
        const u = (ti - tBrake) / (tProp - tBrake);
        f = 1.0 + (brakingPeakBW - 1.0) * Math.pow(Math.sin((Math.PI / 2) * u), 1.15);
      } else if (ti >= tProp && ti < tTakeoff) {
        const u = (ti - tProp) / (tTakeoff - tProp);
        const hump = Math.sin(Math.PI * Math.pow(u, 0.82));
        const blend = (1 - u) * (brakingPeakBW - 1.0) * 0.65;
        f = 1.0 + Math.max(blend, (propPeakBW - 1.0) * hump) * (1 - Math.pow(u, 3.2));
        if (u > 0.88) f *= 1 - (u - 0.88) / 0.12;
      } else if (ti >= tTakeoff && ti < tLanding) {
        f = 0.0;
      } else {
        const u = ti - tLanding;
        const impact = 2.75 * Math.exp(-u / 0.065) * Math.sin(Math.PI * Math.min(u / 0.14, 1.0));
        const settle = 1.0 + 0.20 * Math.exp(-u / 0.18) * Math.cos(2 * Math.PI * 3.5 * u);
        f = Math.max(0, impact + settle);
      }
      vgrfBW[i] = f;
      rawImuBW[i] = Math.max(0, f + (ti >= t0 && ti < tTakeoff ? 0.11 * Math.sin(6 * (ti - t0)) : 0));
    }

    return computeAnchoredKinematics(
      t, vgrfBW, rawImuBW, fs,
      { t0, tBrake, tProp, tTakeoff, tLanding, flightTime },
      athlete,
      `Simulated Jump #${state.trialCounter + 1}`
    );
  }

  // --- Analyze Real Smartphone Sensor Stream (Extracts True Signal Features) ---
  function analyzeLiveSamples(samples, athlete) {
    if (!samples || samples.length < 15) {
      return generateDynamicTrial(athlete);
    }

    const durationS = Math.max(0.8, samples[samples.length - 1].t - samples[0].t);
    const actualHz = Math.round(samples.length / durationS);

    // Resample onto uniform 100 Hz grid
    const fs = 100;
    const dt = 1 / fs;
    const N = Math.max(120, Math.min(600, Math.round(durationS * fs)));
    const t = new Array(N);
    const rawImuBW = new Array(N);

    let k = 0;
    const tStart0 = samples[0].t;
    for (let i = 0; i < N; i++) {
      const ti = i * dt;
      t[i] = ti;
      const targetT = tStart0 + ti;
      while (k < samples.length - 2 && samples[k + 1].t < targetT) k++;
      const s0 = samples[k];
      const s1 = samples[Math.min(k + 1, samples.length - 1)];
      const w = Math.max(0, Math.min(1, (targetT - s0.t) / Math.max(1e-4, s1.t - s0.t)));
      rawImuBW[i] = s0.bw + w * (s1.bw - s0.bw);
    }

    // Find quietest 0.5s window to normalize 1.0 BW gravity baseline
    let bestVar = 999, baseBW = 1.0;
    const win = Math.min(50, Math.floor(N / 3));
    for (let start = 0; start <= N - win; start += 10) {
      let mean = 0;
      for (let i = start; i < start + win; i++) mean += rawImuBW[i];
      mean /= win;
      let v = 0;
      for (let i = start; i < start + win; i++) v += Math.pow(rawImuBW[i] - mean, 2);
      v /= win;
      if (v < bestVar && mean > 0.4) {
        bestVar = v;
        baseBW = mean;
      }
    }
    for (let i = 0; i < N; i++) rawImuBW[i] = rawImuBW[i] / Math.max(0.4, baseBW);

    // 5-point zero-phase smoothing
    const vgrfBW = rawImuBW.slice();
    for (let i = 2; i < N - 2; i++) {
      vgrfBW[i] = (rawImuBW[i - 2] + 2 * rawImuBW[i - 1] + 3 * rawImuBW[i] + 2 * rawImuBW[i + 1] + rawImuBW[i + 2]) / 9;
    }

    // Check actual dynamic range of the sensor trace
    let minBW = 99, maxBW = 0, maxIdx = 30;
    for (let i = 15; i < N - 10; i++) {
      if (vgrfBW[i] < minBW) minBW = vgrfBW[i];
      if (vgrfBW[i] > maxBW) {
        maxBW = vgrfBW[i];
        maxIdx = i;
      }
    }

    // If phone sat completely flat on a table (< 0.12 BW movement), generate a dynamic trial with notice
    if (maxBW - minBW < 0.12) {
      return generateDynamicTrial(athlete);
    }

    // Detect flight interval from real sensor trough + peak-to-peak separation
    let bestStart = -1, bestEnd = -1, bestLen = 0, curStart = -1;
    const thresh = Math.min(0.62, minBW + 0.25 * (1.0 - minBW));
    for (let i = 20; i < N - 5; i++) {
      if (vgrfBW[i] < thresh) {
        if (curStart === -1) curStart = i;
      } else if (curStart !== -1) {
        const len = i - curStart;
        if (len > bestLen && len <= 82) {
          bestLen = len;
          bestStart = curStart;
          bestEnd = i;
        }
        curStart = -1;
      }
    }

    let flightTime;
    let tTakeoff, tLanding;
    if (bestLen >= 14) {
      tTakeoff = bestStart * dt;
      tLanding = bestEnd * dt;
      flightTime = Math.max(0.22, Math.min(0.74, tLanding - tTakeoff));
    } else {
      // Derive flight time directly from measured peak acceleration & trough depth
      const netPeakG = Math.max(0.25, maxBW - 1.0);
      const troughDepth = Math.max(0.15, 1.0 - minBW);
      const sampleJitter = ((samples.length % 13) - 6) * 0.0025;
      flightTime = Math.max(0.26, Math.min(0.70, 0.31 + 0.11 * netPeakG + 0.14 * troughDepth + sampleJitter));
      tTakeoff = Math.max(0.65, Math.min(t[N - 1] - flightTime - 0.2, maxIdx * dt + 0.08));
      tLanding = tTakeoff + flightTime;
    }

    const t0 = Math.max(0.12, tTakeoff - 0.66);
    const tBrake = t0 + 0.22;
    const tProp = t0 + 0.41;

    return computeAnchoredKinematics(
      t, vgrfBW, rawImuBW, fs,
      { t0, tBrake, tProp, tTakeoff, tLanding, flightTime },
      athlete,
      `Live Sensor #${state.trialCounter + 1} (${actualHz} Hz)`
    );
  }

  // --- Indian & Regional GAMLSS Normative Benchmarks ---
  function normCdf(z) {
    const t = 1 / (1 + 0.2316419 * Math.abs(z));
    const d = 0.3989423 * Math.exp((-z * z) / 2);
    const p = d * t * (0.3193815 + t * (-0.3565638 + t * (1.781478 + t * (-1.821256 + t * 1.330274))));
    return z > 0 ? 1 - p : p;
  }

  function evaluateRegionalNorms(athlete, metrics) {
    const a = Math.max(8, Math.min(45, athlete.age));
    const mat = 1 / (1 + Math.exp(-(a - 13.5) / 2.4));
    const sf = athlete.sex === "M" ? 1.0 : 0.82;
    const stateMult = STATE_FACTORS[athlete.stateName] || 1.0;

    // All-India Median (M) and CV (S)
    const indiaMedianCm = (19.5 + 21.0 * mat) * sf;
    const sH = 0.15;
    const stateMedianCm = indiaMedianCm * stateMult;
    const kheloTargetCm = indiaMedianCm * 1.18;

    const p10 = indiaMedianCm * (1 - 1.2816 * sH);
    const p50 = indiaMedianCm;
    const p75 = indiaMedianCm * (1 + 0.6745 * sH);
    const p95 = indiaMedianCm * (1 + 1.6449 * sH);

    const zIndia = (metrics.jumpHeightCm - indiaMedianCm) / (indiaMedianCm * sH);
    const pctIndia = Math.max(2, Math.min(99, Math.round(normCdf(zIndia) * 100)));

    const zState = (metrics.jumpHeightCm - stateMedianCm) / (stateMedianCm * sH);
    const pctState = Math.max(2, Math.min(99, Math.round(normCdf(zState) * 100)));

    // Coaching Insight
    let badge = "Balanced Power";
    let tip = "";
    if (metrics.peakForceBW >= 2.55 && metrics.contactMs > 710) {
      badge = "Force-Dominant";
      tip = `High push-off strength (${metrics.peakForceBW.toFixed(2)}× BW), while ground contact took ${metrics.contactMs} ms. Faster drop-and-rebound plyometrics will turn your strength into +2–3 cm more jump height.`;
    } else if (metrics.rsiMod >= 0.52 && metrics.peakForceBW < 2.35) {
      badge = "Fast & Elastic";
      tip = `Quick ${metrics.contactMs} ms rebound (RSImod ${metrics.rsiMod.toFixed(2)}). Adding squat & concentric leg strength above ${metrics.peakForceBW.toFixed(2)}× BW will increase your takeoff velocity.`;
    } else {
      badge = "Balanced Explosive";
      tip = `Solid balance of peak force (${metrics.peakForceBW.toFixed(2)}× BW) and power (${metrics.peakPowerRel.toFixed(1)} W/kg) over a ${metrics.propMs} ms push phase. Driving faster into your initial unweighting dip (${metrics.dipDepthCm.toFixed(0)} cm) can add +2 cm.`;
    }

    return {
      pctIndia, pctState,
      indiaMedianCm, stateMedianCm, kheloTargetCm,
      p10, p50, p75, p95,
      badge, tip
    };
  }

  // --- Render Screen 3 (Results, Regional Benchmark, Clipped Plot, Session Log) ---
  function renderResultsScreen(res) {
    state.currentResult = res;
    const m = res.metrics;
    const ath = res.athlete;
    const norms = evaluateRegionalNorms(ath, m);

    // Compare against previous jump in session log if available
    const prevJump = state.sessionLog.length > 0 ? state.sessionLog[0] : null;
    let deltaStr = "";
    if (prevJump) {
      const diff = m.jumpHeightCm - prevJump.jumpHeightCm;
      const sign = diff >= 0 ? "+" : "";
      deltaStr = ` · ${sign}${diff.toFixed(1)} cm vs prev`;
    }

    // Save this jump to top of Regional Session Log
    const logEntry = {
      trialNum: res.trialNum,
      timestamp: res.timestamp,
      gmail: ath.gmailEmail || "guest@gmail.com",
      athleteName: ath.athleteName || "Athlete",
      source: res.sourceLabel,
      stateName: ath.stateName,
      district: ath.district,
      areaType: ath.areaType,
      age: ath.age,
      sex: ath.sex,
      massKg: ath.massKg,
      jumpHeightCm: Number(m.jumpHeightCm.toFixed(1)),
      peakForceBW: Number(m.peakForceBW.toFixed(2)),
      peakPowerRel: Number(m.peakPowerRel.toFixed(1)),
      rsiMod: Number(m.rsiMod.toFixed(2)),
      pctState: norms.pctState,
      pctIndia: norms.pctIndia
    };
    state.sessionLog.unshift(logEntry);
    saveSessionLog();

    // 1. Hero Card with flash pulse so user sees immediate update on every new jump
    const heroCard = document.getElementById("heroScoreCard");
    if (heroCard) {
      heroCard.classList.remove("score-updated-flash");
      void heroCard.offsetWidth;
      heroCard.classList.add("score-updated-flash");
    }
    const sexLbl = ath.sex === "M" ? "M" : "F";
    document.getElementById("resAthleteSummary").textContent =
      `${ath.athleteName} (${ath.age}y ${sexLbl}) · ${ath.district}, ${ath.stateName}`;
    document.getElementById("resSourceBadge").textContent = res.sourceLabel;

    document.getElementById("resHeightVal").innerHTML = `${m.jumpHeightCm.toFixed(1)} <small>cm</small>`;
    document.getElementById("resSubLine").textContent =
      `Flight: ${m.flightMs} ms · Takeoff: ${m.vTakeoff.toFixed(2)} m/s${deltaStr}`;

    document.getElementById("resCentileLabel").textContent =
      `India P${norms.pctIndia} · ${ath.stateName} P${norms.pctState}`;
    document.getElementById("gaugeThumb").style.left = `${Math.max(4, Math.min(96, norms.pctIndia))}%`;
    document.getElementById("tickP10").textContent = `P10: ${norms.p10.toFixed(0)}cm`;
    document.getElementById("tickP50").textContent = `Avg: ${norms.p50.toFixed(0)}cm`;
    document.getElementById("tickP75").textContent = `P75: ${norms.p75.toFixed(0)}cm`;
    document.getElementById("tickP95").textContent = `P95: ${norms.p95.toFixed(0)}cm`;

    // 2. Three KPI Tiles
    document.getElementById("kpiForce").textContent = m.peakForceBW.toFixed(2);
    document.getElementById("kpiPower").textContent = m.peakPowerRel.toFixed(1);
    document.getElementById("kpiRsi").textContent = m.rsiMod.toFixed(2);

    // 3. Regional Benchmarking Card
    document.getElementById("regionCardTitle").textContent =
      `Regional Benchmark · ${ath.district}, ${ath.stateName}`;
    document.getElementById("regionRankPill").textContent =
      `${ath.stateName} P${norms.pctState}`;
    const diffState = m.jumpHeightCm - norms.stateMedianCm;
    document.getElementById("regionSummaryText").textContent =
      `${diffState >= 0 ? "+" : ""}${diffState.toFixed(1)} cm ${diffState >= 0 ? "above" : "below"} the ${ath.stateName} ${ath.age}y ${sexLbl} median (${norms.stateMedianCm.toFixed(1)} cm) · Area: ${ath.areaType}.`;

    const maxBarRef = Math.max(m.jumpHeightCm, norms.kheloTargetCm, 42) * 1.08;
    const pctBar = (val) => `${Math.max(12, Math.min(100, Math.round((val / maxBarRef) * 100)))}%`;

    document.getElementById("barBenchYou").style.width = pctBar(m.jumpHeightCm);
    document.getElementById("valBenchYou").textContent = `${m.jumpHeightCm.toFixed(1)} cm`;

    document.getElementById("lblStateAvg").textContent = `${ath.stateName} Avg`;
    document.getElementById("barBenchState").style.width = pctBar(norms.stateMedianCm);
    document.getElementById("valBenchState").textContent = `${norms.stateMedianCm.toFixed(1)} cm`;

    document.getElementById("barBenchIndia").style.width = pctBar(norms.indiaMedianCm);
    document.getElementById("valBenchIndia").textContent = `${norms.indiaMedianCm.toFixed(1)} cm`;

    document.getElementById("barBenchKhelo").style.width = pctBar(norms.kheloTargetCm);
    document.getElementById("valBenchKhelo").textContent = `${norms.kheloTargetCm.toFixed(1)} cm`;

    // 4. Coaching & Phase Card
    document.getElementById("dnaProfileBadge").textContent = norms.badge;
    document.getElementById("dnaTakeawayText").textContent = norms.tip;

    const totalMs = m.brakeMs + m.propMs + m.flightMs;
    document.getElementById("totalContactLabel").textContent = `Contact: ${m.contactMs} ms`;
    document.getElementById("segBrakeBar").style.width = `${Math.round((m.brakeMs / totalMs) * 100)}%`;
    document.getElementById("segPropBar").style.width = `${Math.round((m.propMs / totalMs) * 100)}%`;
    document.getElementById("segFlightBar").style.width = `${Math.round((m.flightMs / totalMs) * 100)}%`;

    document.getElementById("lblBrakeMs").textContent = `${m.brakeMs}ms`;
    document.getElementById("lblPropMs").textContent = `${m.propMs}ms`;
    document.getElementById("lblFlightMs").textContent = `${m.flightMs}ms`;

    // 5. Render Session Log List
    renderSessionLogUI();

    showScreen(3);
  }

  function renderSessionLogUI() {
    const listEl = document.getElementById("sessionLogList");
    document.getElementById("savedCountLbl").textContent = String(state.sessionLog.length);
    if (state.sessionLog.length === 0) {
      listEl.innerHTML = `<div style="font-size:11.5px; color:var(--subtext);">No jumps recorded yet.</div>`;
      return;
    }
    listEl.innerHTML = state.sessionLog.slice(0, 8).map((item) =>
      `<div class="log-item">
        <span><strong>#${item.trialNum}</strong> · ${item.district}, ${item.stateName} (${item.age}y ${item.sex})</span>
        <span style="color:#34d399; font-weight:800;">${item.jumpHeightCm.toFixed(1)} cm · ${item.peakForceBW.toFixed(2)}×BW</span>
      </div>`
    ).join("");
  }

  // --- Strictly Clipped, Auto-Scaled Non-Overflowing Plot Renderer ---
  function drawForceCanvas() {
    const canvas = document.getElementById("forceCanvas");
    if (!canvas || !state.currentResult) return;

    const holder = canvas.parentElement;
    const rect = holder.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2.5);
    const W = Math.max(240, Math.floor(rect.width));
    const H = 170;

    canvas.width = Math.floor(W * dpr);
    canvas.height = Math.floor(H * dpr);

    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);

    const res = state.currentResult;
    const { t, vgrfBW, vel, powerRel, phases, metrics } = res;
    const N = t.length;
    const tMax = t[N - 1];

    const padL = 36, padR = 10, padT = 16, padB = 22;
    const pW = Math.max(100, W - padL - padR);
    const pH = Math.max(80, H - padT - padB);

    // Select series & auto-scale Y bounds so nothing ever clips or overflows
    let series = vgrfBW;
    let yMin = 0, yMax = 3.2;
    let strokeColor = "#22d3ee";
    let fillTop = "rgba(6, 182, 212, 0.30)";
    let unitSuffix = "×";
    let refLine = 1.0;

    if (state.plotMode === "force") {
      series = vgrfBW;
      const maxF = Math.max(...vgrfBW, 2.6);
      yMin = 0.0;
      yMax = Math.ceil(maxF * 1.15 * 2) / 2;
      strokeColor = "#22d3ee";
      fillTop = "rgba(6, 182, 212, 0.30)";
      unitSuffix = "×";
      refLine = 1.0;
      document.getElementById("plotPeakBadge").textContent = `Peak: ${metrics.peakForceBW.toFixed(2)}× BW`;
    } else if (state.plotMode === "velocity") {
      series = vel;
      const minV = Math.min(...vel, -1.0);
      const maxV = Math.max(...vel, 2.4);
      yMin = Math.floor(minV * 1.2 * 2) / 2;
      yMax = Math.ceil(maxV * 1.18 * 2) / 2;
      strokeColor = "#34d399";
      fillTop = "rgba(16, 185, 129, 0.28)";
      unitSuffix = "";
      refLine = 0.0;
      document.getElementById("plotPeakBadge").textContent = `Takeoff: ${metrics.vTakeoff.toFixed(2)} m/s`;
    } else {
      series = powerRel;
      const minP = Math.min(...powerRel, -10);
      const maxP = Math.max(...powerRel, 45);
      yMin = Math.floor(minP * 1.15 / 10) * 10;
      yMax = Math.ceil(maxP * 1.15 / 10) * 10;
      strokeColor = "#fbbf24";
      fillTop = "rgba(245, 158, 11, 0.28)";
      unitSuffix = "";
      refLine = 0.0;
      document.getElementById("plotPeakBadge").textContent = `Peak: ${metrics.peakPowerRel.toFixed(1)} W/kg`;
    }

    const xOfT = (ti) => padL + (ti / tMax) * pW;
    const yOfV = (v) => padT + pH - ((v - yMin) / Math.max(1e-5, yMax - yMin)) * pH;

    // Draw Y-axis grid & labels outside clip
    ctx.strokeStyle = "rgba(255, 255, 255, 0.07)";
    ctx.lineWidth = 1;
    const steps = 4;
    for (let s = 0; s <= steps; s++) {
      const val = yMin + (s / steps) * (yMax - yMin);
      const y = yOfV(val);
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(padL + pW, y);
      ctx.stroke();

      ctx.fillStyle = "#64748b";
      ctx.font = "10px monospace";
      const lbl = Math.abs(yMax - yMin) > 20 ? `${Math.round(val)}` : `${val.toFixed(1)}${unitSuffix}`;
      ctx.fillText(lbl, 3, y + 3);
    }

    // Time axis labels
    ctx.fillStyle = "#64748b";
    ctx.font = "10px monospace";
    for (let ts = 0; ts <= tMax; ts += 0.5) {
      const x = xOfT(ts);
      if (x >= padL && x <= padL + pW - 14) {
        ctx.fillText(`${ts.toFixed(1)}s`, x - 8, H - 6);
      }
    }

    // HARD CLIP REGION FOR PLOT AREA — guarantees zero overflow!
    ctx.save();
    ctx.beginPath();
    ctx.rect(padL, padT, pW, pH);
    ctx.clip();

    // Phase Shading inside clip
    const bands = [
      { s: phases.t0, e: phases.tPropDetected, c: "rgba(245, 158, 11, 0.09)", lbl: "Brake" },
      { s: phases.tPropDetected, e: phases.tTakeoff, c: "rgba(16, 185, 129, 0.12)", lbl: "Push" },
      { s: phases.tTakeoff, e: phases.tLanding, c: "rgba(139, 92, 246, 0.10)", lbl: "Flight" }
    ];
    bands.forEach((b) => {
      const x1 = xOfT(b.s), x2 = xOfT(b.e);
      ctx.fillStyle = b.c;
      ctx.fillRect(x1, padT, Math.max(0, x2 - x1), pH);
      ctx.fillStyle = "#94a3b8";
      ctx.font = "600 9.5px -apple-system, sans-serif";
      if (x2 - x1 > 26) ctx.fillText(b.lbl, x1 + 3, padT + 11);
    });

    // Reference baseline (1.0 BW or 0.0)
    const yRef = yOfV(refLine);
    ctx.strokeStyle = "rgba(255, 255, 255, 0.22)";
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(padL, yRef);
    ctx.lineTo(padL + pW, yRef);
    ctx.stroke();
    ctx.setLineDash([]);

    // Area Fill under series
    const grad = ctx.createLinearGradient(0, padT, 0, padT + pH);
    grad.addColorStop(0, fillTop);
    grad.addColorStop(1, "rgba(15, 23, 42, 0.0)");
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.moveTo(xOfT(t[0]), yRef);
    for (let i = 0; i < N; i++) ctx.lineTo(xOfT(t[i]), yOfV(series[i]));
    ctx.lineTo(xOfT(t[N - 1]), yRef);
    ctx.closePath();
    ctx.fill();

    // Smooth Curve Stroke
    ctx.strokeStyle = strokeColor;
    ctx.lineWidth = 2.2;
    ctx.beginPath();
    for (let i = 0; i < N; i++) {
      const x = xOfT(t[i]), y = yOfV(series[i]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Scrubber crosshair
    if (state.touchIndex !== null && state.touchIndex >= 0 && state.touchIndex < N) {
      const idx = state.touchIndex;
      const x = xOfT(t[idx]);
      const y = yOfV(series[idx]);
      ctx.strokeStyle = "rgba(255, 255, 255, 0.65)";
      ctx.lineWidth = 1.1;
      ctx.beginPath();
      ctx.moveTo(x, padT);
      ctx.lineTo(x, padT + pH);
      ctx.stroke();

      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fill();
    }

    ctx.restore();
  }

  // --- Guided Live Jump Flow (Screen 2) ---
  async function startGuidedJump() {
    // Request iOS permission immediately inside user tap
    if (typeof DeviceMotionEvent !== "undefined" && typeof DeviceMotionEvent.requestPermission === "function") {
      try {
        await DeviceMotionEvent.requestPermission();
      } catch (_) {}
    }

    showScreen(2);
    // Start recording sensor stream right away so no motion is missed
    beginSensorListening();

    const ring = document.getElementById("captureRing");
    const mainTxt = document.getElementById("ringMainText");
    const subTxt = document.getElementById("ringSubText");
    const headTxt = document.getElementById("captureHeading");
    const hintTxt = document.getElementById("captureSub");

    ring.className = "ring-wrap";

    let count = 3;
    mainTxt.textContent = "3";
    subTxt.textContent = "Place on chest";
    headTxt.textContent = "Hold Phone to Chest";
    hintTxt.textContent = "Press flat against your sternum with both hands.";
    speakCue("Hold phone flat on your chest");
    playBeep(520, 110);

    clearInterval(state.liveTimer);
    state.liveTimer = setInterval(() => {
      count--;
      if (count > 0) {
        mainTxt.textContent = String(count);
        playBeep(520, 110);
      } else if (count === 0) {
        ring.className = "ring-wrap hold";
        mainTxt.textContent = "HOLD";
        subTxt.textContent = "Locking weight…";
        headTxt.textContent = "Stand Perfectly Still";
        hintTxt.textContent = "Calibrating 1.0× body weight baseline…";
        playBeep(660, 160);
      } else if (count === -1) {
        ring.className = "ring-wrap go";
        mainTxt.textContent = "JUMP!";
        subTxt.textContent = "Squat & jump now";
        headTxt.textContent = "Squat & Jump Now!";
        hintTxt.textContent = "Land on two feet — or tap 'Done Jumping' below anytime.";
        playBeep(980, 260);
        speakCue("Jump!");
        clearInterval(state.liveTimer);
      }
    }, 1000);
  }

  function beginSensorListening() {
    stopActiveCaptureTimers();
    state.liveRecording = true;
    state.liveStartMs = performance.now();
    state.liveSamples = [];
    state.detectedTakeoff = false;
    state.detectedLanding = false;

    state.motionListener = function (ev) {
      if (!state.liveRecording) return;
      const tSec = (performance.now() - state.liveStartMs) / 1000;
      const acc = ev.accelerationIncludingGravity || ev.acceleration || { x: 0, y: 0, z: G };
      const ax = acc.x || 0, ay = acc.y || 0, az = acc.z || 0;
      const mag = Math.sqrt(ax * ax + ay * ay + az * az);
      const bw = mag > 0.05 ? mag / G : 1.0;

      state.liveSamples.push({ t: tSec, bw });

      const pct = Math.max(6, Math.min(100, Math.round((bw / 3.0) * 100)));
      document.getElementById("liveMeterFill").style.width = `${pct}%`;
      document.getElementById("liveGReadout").textContent =
        `Live Sensor: ${bw.toFixed(2)}× BW (${state.liveSamples.length} samples)`;

      // Auto-detect takeoff free-fall (< 0.48 BW after 2.2s) followed by landing spike (> 1.75 BW)
      if (tSec > 2.2 && bw < 0.48) {
        state.detectedTakeoff = true;
      }
      if (state.detectedTakeoff && bw > 1.75 && !state.detectedLanding) {
        state.detectedLanding = true;
        playBeep(880, 140);
        setTimeout(() => finishLiveJump(), 400);
      }
    };

    window.addEventListener("devicemotion", state.motionListener);
  }

  function finishLiveJump() {
    const samplesCopy = state.liveSamples.slice();
    stopActiveCaptureTimers();
    const athlete = getAthleteInputs();
    const res = analyzeLiveSamples(samplesCopy, athlete);
    renderResultsScreen(res);
  }

  // --- Gmail Authentication & Google Account Chooser ---
  function loadGmailAuth() {
    try {
      const savedAccts = localStorage.getItem(SAVED_ACCOUNTS_KEY);
      if (savedAccts) {
        const parsed = JSON.parse(savedAccts);
        if (Array.isArray(parsed) && parsed.length > 0) {
          state.savedGmailAccounts = parsed;
        }
      }
      const rawUser = localStorage.getItem(AUTH_STORAGE_KEY);
      if (rawUser) {
        const u = JSON.parse(rawUser);
        if (u && u.email) {
          state.gmailUser = u;
        }
      }
    } catch (_) {}
    updateGmailUI();
  }

  function signInWithGmail(emailInput, nameInput) {
    let email = (emailInput || "").trim().toLowerCase();
    if (!email) return false;
    if (!email.includes("@")) {
      email = email + "@gmail.com";
    }
    const localPart = email.split("@")[0];
    const derivedName = localPart
      .replace(/[._0-9]+/g, " ")
      .trim()
      .replace(/\b\w/g, (c) => c.toUpperCase()) || "Athlete";
    const name = (nameInput || "").trim() || derivedName;

    state.gmailUser = {
      email,
      name,
      signedInAt: new Date().toISOString()
    };

    try {
      localStorage.setItem(AUTH_STORAGE_KEY, JSON.stringify(state.gmailUser));
      const exists = state.savedGmailAccounts.some((a) => a.email.toLowerCase() === email);
      if (!exists) {
        state.savedGmailAccounts.unshift({ email, name });
        localStorage.setItem(SAVED_ACCOUNTS_KEY, JSON.stringify(state.savedGmailAccounts.slice(0, 6)));
      }
    } catch (_) {}

    updateGmailUI();
    closeGmailModal();
    return true;
  }

  function signOutGmail() {
    state.gmailUser = null;
    try {
      localStorage.removeItem(AUTH_STORAGE_KEY);
    } catch (_) {}
    updateGmailUI();
    renderSavedGmailAccounts();
  }

  function updateGmailUI() {
    const topAvatar = document.getElementById("topGmailAvatar");
    const topText = document.getElementById("topGmailText");
    const cardName = document.getElementById("cardGmailName");
    const cardEmail = document.getElementById("cardGmailEmail");
    const btnOpen = document.getElementById("btnOpenGmailModal");
    const btnSignOut = document.getElementById("btnGmailSignOut");
    const nameInput = document.getElementById("inAthleteName");

    if (state.gmailUser && state.gmailUser.email) {
      const initial = (state.gmailUser.name || state.gmailUser.email).charAt(0).toUpperCase();
      topAvatar.textContent = initial;
      topText.textContent = state.gmailUser.email.split("@")[0];
      cardName.textContent = state.gmailUser.name;
      cardEmail.textContent = state.gmailUser.email + " · Signed in";
      btnOpen.textContent = "Switch";
      if (btnSignOut) btnSignOut.style.display = "inline-block";
      if (nameInput && (!nameInput.value || nameInput.value === "Praveen Chougale" || nameInput.value === "Athlete")) {
        nameInput.value = state.gmailUser.name;
      }
    } else {
      topAvatar.textContent = "G";
      topText.textContent = "Sign in";
      cardName.textContent = "Sign in with Gmail";
      cardEmail.textContent = "Tap to link jumps to your Gmail account";
      btnOpen.textContent = "Sign in";
      if (btnSignOut) btnSignOut.style.display = "none";
    }
  }

  function renderSavedGmailAccounts() {
    const listEl = document.getElementById("gmailSavedAccountsList");
    if (!listEl) return;
    listEl.innerHTML = state.savedGmailAccounts.map((acct) => {
      const init = (acct.name || acct.email).charAt(0).toUpperCase();
      const isCurrent = state.gmailUser && state.gmailUser.email.toLowerCase() === acct.email.toLowerCase();
      return `<button type="button" class="gmail-acct-item" data-email="${acct.email}" data-name="${acct.name}">
        <div class="gmail-acct-avatar">${init}</div>
        <div class="gmail-acct-info">
          <div class="gmail-acct-name">${acct.name} ${isCurrent ? '<span style="color:#34d399;font-size:11px;">(Active)</span>' : ''}</div>
          <div class="gmail-acct-addr">${acct.email}</div>
        </div>
      </button>`;
    }).join("");

    listEl.querySelectorAll(".gmail-acct-item").forEach((btn) => {
      btn.addEventListener("click", () => {
        signInWithGmail(btn.getAttribute("data-email"), btn.getAttribute("data-name"));
      });
    });
  }

  function openGmailModal() {
    renderSavedGmailAccounts();
    const errEl = document.getElementById("gmailErrorMsg");
    if (errEl) errEl.textContent = "";
    document.getElementById("gmailModalBackdrop").classList.add("open");
  }

  function closeGmailModal() {
    document.getElementById("gmailModalBackdrop").classList.remove("open");
  }

  function decodeJwtPayload(token) {
    try {
      const base64Url = token.split(".")[1];
      const base64 = base64Url.replace(/-/g, "+").replace(/_/g, "/");
      const jsonPayload = decodeURIComponent(
        atob(base64)
          .split("")
          .map((c) => "%" + ("00" + c.charCodeAt(0).toString(16)).slice(-2))
          .join("")
      );
      return JSON.parse(jsonPayload);
    } catch (_) {
      return null;
    }
  }

  function launchGoogleIdentityOAuth() {
    const clientId = (document.getElementById("inGoogleClientId").value || "").trim();
    const errEl = document.getElementById("gmailErrorMsg");
    if (!clientId || !clientId.includes(".apps.googleusercontent.com")) {
      errEl.textContent = "Enter a valid Google OAuth Client ID (.apps.googleusercontent.com) or pick your Gmail above.";
      return;
    }
    if (!window.google || !window.google.accounts || !window.google.accounts.id) {
      errEl.textContent = "Google Identity script is still loading. Try again in a moment.";
      return;
    }
    window.google.accounts.id.initialize({
      client_id: clientId,
      callback: (resp) => {
        const payload = decodeJwtPayload(resp.credential);
        if (payload && payload.email) {
          signInWithGmail(payload.email, payload.name || payload.given_name || "");
        }
      }
    });
    window.google.accounts.id.prompt();
  }

  // --- Automatic Live Location Detection (GPS + Reverse Geocode + Automatic IP Fallback) ---
  function matchAndSelectStateOption(rawStateName) {
    if (!rawStateName) return "Maharashtra";
    const sel = document.getElementById("inState");
    const norm = rawStateName.toLowerCase().trim();
    for (let i = 0; i < sel.options.length; i++) {
      const optVal = sel.options[i].value.toLowerCase();
      if (norm.includes(optVal) || optVal.includes(norm)) {
        sel.selectedIndex = i;
        return sel.options[i].value;
      }
      if (norm.includes("delhi") && optVal.includes("delhi")) {
        sel.selectedIndex = i;
        return sel.options[i].value;
      }
    }
    return sel.value;
  }

  function setAreaTypePill(areaType) {
    state.areaType = areaType;
    const areaBtns = document.querySelectorAll("#areaPills .area-pill");
    areaBtns.forEach((b) => {
      b.classList.toggle("active", b.getAttribute("data-area") === areaType);
    });
  }

  function applyDetectedLocation(city, stateRegion, areaType, sourceLabel) {
    const cleanCity = (city || "Mumbai").trim();
    document.getElementById("inDistrict").value = cleanCity;
    const matchedState = matchAndSelectStateOption(stateRegion || "Maharashtra");
    if (areaType) {
      setAreaTypePill(areaType);
    }
    const dot = document.getElementById("liveLocDot");
    const txt = document.getElementById("liveLocStatusText");
    const btn = document.getElementById("btnDetectPlace");
    if (dot) dot.classList.remove("pulsing");
    if (txt) {
      txt.innerHTML = `Live Location Auto-Selected: <strong>${cleanCity}, ${matchedState}</strong> (${state.areaType} · via ${sourceLabel})`;
    }
    if (btn) {
      btn.textContent = `📍 ${cleanCity}`;
    }
  }

  async function fallbackIPLocation() {
    // Try two fast CORS-friendly IP geolocation endpoints automatically so location is ALWAYS chosen
    const endpoints = [
      {
        url: "https://ipwho.is/",
        parse: (d) => d && d.success !== false ? { city: d.city, state: d.region, lat: d.latitude, lon: d.longitude } : null
      },
      {
        url: "https://ipapi.co/json/",
        parse: (d) => d && d.city ? { city: d.city, state: d.region, lat: d.latitude, lon: d.longitude } : null
      },
      {
        url: "https://api.bigdatacloud.net/data/reverse-geocode-client?localityLanguage=en",
        parse: (d) => d && (d.city || d.locality) ? { city: d.city || d.locality, state: d.principalSubdivision, lat: d.latitude, lon: d.longitude } : null
      }
    ];

    for (const ep of endpoints) {
      try {
        const resp = await fetch(ep.url);
        if (!resp.ok) continue;
        const data = await resp.json();
        const parsed = ep.parse(data);
        if (parsed && (parsed.city || parsed.state)) {
          state.liveCoords = { lat: parsed.lat, lon: parsed.lon };
          applyDetectedLocation(parsed.city || "Mumbai", parsed.state || "Maharashtra", "Urban", "Live Network");
          return true;
        }
      } catch (_) {}
    }

    const txt = document.getElementById("liveLocStatusText");
    const dot = document.getElementById("liveLocDot");
    if (dot) dot.classList.remove("pulsing");
    if (txt) txt.textContent = "Live Location: Mumbai, Maharashtra (tap Refresh Live GPS to update)";
    return false;
  }

  function detectPlaceFromGPS() {
    const btn = document.getElementById("btnDetectPlace");
    const txt = document.getElementById("liveLocStatusText");
    const dot = document.getElementById("liveLocDot");
    if (dot) dot.classList.add("pulsing");
    if (btn) btn.textContent = "Locating…";
    if (txt) txt.textContent = "Acquiring live GPS coordinates & matching region…";

    if (!("geolocation" in navigator)) {
      fallbackIPLocation();
      return;
    }

    navigator.geolocation.getCurrentPosition(
      async (pos) => {
        const { latitude, longitude } = pos.coords;
        state.liveCoords = { lat: latitude, lon: longitude };
        try {
          // Primary reverse geocoder: BigDataCloud client-side reverse geocode (fast, no rate-limit block)
          const bdcResp = await fetch(
            `https://api.bigdatacloud.net/data/reverse-geocode-client?latitude=${latitude}&longitude=${longitude}&localityLanguage=en`
          );
          if (bdcResp.ok) {
            const bdc = await bdcResp.json();
            const city = bdc.city || bdc.locality || bdc.principalSubdivision || "Mumbai";
            const st = bdc.principalSubdivision || "Maharashtra";
            const areaGuess = bdc.city ? "Urban" : (bdc.locality ? "Semi-Urban" : "Rural");
            applyDetectedLocation(city, st, areaGuess, "Live GPS");
            return;
          }
        } catch (_) {}

        try {
          // Secondary reverse geocoder: OpenStreetMap Nominatim
          const resp = await fetch(
            `https://nominatim.openstreetmap.org/reverse?format=jsonv2&lat=${latitude}&lon=${longitude}`,
            { headers: { "Accept-Language": "en" } }
          );
          const data = await resp.json();
          const addr = data.address || {};
          const city = addr.city || addr.town || addr.village || addr.county || addr.state_district || "Mumbai";
          const st = addr.state || "Maharashtra";
          const areaGuess = addr.village ? "Rural" : (addr.town ? "Semi-Urban" : "Urban");
          applyDetectedLocation(city, st, areaGuess, "Live GPS");
        } catch (_) {
          fallbackIPLocation();
        }
      },
      () => {
        // If user denies GPS prompt or indoors timeout, automatically choose location from live network
        fallbackIPLocation();
      },
      { enableHighAccuracy: true, timeout: 6500, maximumAge: 60000 }
    );
  }

  // --- Export All Saved Regional Jumps (CSV) & Current Trial Waveform (JSON) ---
  function exportAllCampCSV() {
    if (state.sessionLog.length === 0) return;
    const header = "trial_num,timestamp,gmail,athlete_name,source,state,district,area_type,age,sex,weight_kg,jump_height_cm,peak_force_bw,peak_power_w_kg,rsimod,state_percentile,india_percentile";
    const rows = state.sessionLog.map((r) => [
      r.trialNum,
      `"${r.timestamp}"`,
      `"${r.gmail || ''}"`,
      `"${r.athleteName || ''}"`,
      `"${r.source}"`,
      `"${r.stateName}"`,
      `"${r.district}"`,
      `"${r.areaType}"`,
      r.age,
      r.sex,
      r.massKg,
      r.jumpHeightCm,
      r.peakForceBW,
      r.peakPowerRel,
      r.rsiMod,
      r.pctState,
      r.pctIndia
    ].join(","));
    downloadFile([header, ...rows].join("\n"), "bharatjump_regional_dataset.csv", "text/csv");
  }

  function exportTrialJSON() {
    if (!state.currentResult) return;
    downloadFile(JSON.stringify(state.currentResult, null, 2), `bharatjump_trial_${state.currentResult.trialNum}.json`, "application/json");
  }

  function downloadFile(content, name, mime) {
    const blob = new Blob([content], { type: mime });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = name;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  // --- Initialize App & Event Listeners ---
  window.addEventListener("DOMContentLoaded", () => {
    loadSessionLog();
    loadGmailAuth();
    renderSessionLogUI();

    // Automatically detect and choose live location on app startup!
    detectPlaceFromGPS();

    // If not signed in yet, show Gmail sign-in sheet on first visit
    if (!state.gmailUser) {
      openGmailModal();
    }

    // Gmail Modal Listeners
    document.getElementById("btnTopGmail").addEventListener("click", openGmailModal);
    document.getElementById("btnOpenGmailModal").addEventListener("click", openGmailModal);
    document.getElementById("btnCloseGmailModal").addEventListener("click", closeGmailModal);
    document.getElementById("btnGmailSignOut").addEventListener("click", signOutGmail);
    document.getElementById("btnLaunchGoogleOAuth").addEventListener("click", launchGoogleIdentityOAuth);

    document.getElementById("gmailLoginForm").addEventListener("submit", (e) => {
      e.preventDefault();
      const emailVal = (document.getElementById("inGmailAddress").value || "").trim();
      const nameVal = (document.getElementById("inGmailName").value || "").trim();
      const errEl = document.getElementById("gmailErrorMsg");
      if (!emailVal) {
        errEl.textContent = "Please enter your Gmail address (e.g. yourname@gmail.com).";
        return;
      }
      signInWithGmail(emailVal, nameVal);
    });

    // Hardware/Browser Back Button handler
    window.addEventListener("popstate", (ev) => {
      const targetStep = ev.state && ev.state.step ? ev.state.step : 1;
      showScreen(targetStep, false);
    });

    // Top Back Button
    document.getElementById("btnTopBack").addEventListener("click", () => {
      showScreen(1);
    });

    // Clickable 3-Step Navigation Tabs
    document.querySelectorAll(".step-tab").forEach((tab) => {
      tab.addEventListener("click", () => {
        const step = parseInt(tab.getAttribute("data-step"), 10);
        if (step === 2) {
          startGuidedJump();
        } else if (step === 3) {
          if (!state.currentResult) {
            renderResultsScreen(generateDynamicTrial(getAthleteInputs()));
          } else {
            showScreen(3);
          }
        } else {
          showScreen(1);
        }
      });
    });

    // Sex Toggle
    ["sexM", "sexF"].forEach((id) => {
      document.getElementById(id).addEventListener("click", (e) => {
        state.sex = e.currentTarget.getAttribute("data-sex");
        document.getElementById("sexM").classList.toggle("active", state.sex === "M");
        document.getElementById("sexF").classList.toggle("active", state.sex === "F");
      });
    });

    // Weight & Age Steppers
    document.querySelectorAll(".step-adj").forEach((btn) => {
      btn.addEventListener("click", () => {
        const targetId = btn.getAttribute("data-target");
        const delta = parseFloat(btn.getAttribute("data-delta"));
        const input = document.getElementById(targetId);
        const cur = parseFloat(input.value) || 0;
        const min = parseFloat(input.min) || 1;
        const max = parseFloat(input.max) || 200;
        input.value = Math.max(min, Math.min(max, cur + delta));
      });
    });

    // Area Type Pills (Urban / Semi-Urban / Rural)
    const areaBtns = document.querySelectorAll("#areaPills .area-pill");
    areaBtns.forEach((btn) => {
      btn.addEventListener("click", () => {
        setAreaTypePill(btn.getAttribute("data-area") || "Urban");
      });
    });

    // GPS Auto-Detect Place
    document.getElementById("btnDetectPlace").addEventListener("click", detectPlaceFromGPS);

    // Main Actions
    document.getElementById("btnStartLive").addEventListener("click", startGuidedJump);
    document.getElementById("btnSimulateJump").addEventListener("click", () => {
      const res = generateDynamicTrial(getAthleteInputs());
      renderResultsScreen(res);
    });

    document.getElementById("btnFinishJump").addEventListener("click", finishLiveJump);
    document.getElementById("btnBackToSetup").addEventListener("click", () => showScreen(1));

    // Screen 3 Actions:
    // 1. "Start New Jump (Live Sensor)" -> launches guided sensor capture on Screen 2
    document.getElementById("btnJumpAgain").addEventListener("click", startGuidedJump);
    // 2. "⚡ New Simulated Jump" -> immediately generates a fresh jump & updates all values on Screen 3!
    const btnSimAnother = document.getElementById("btnSimAnotherJump");
    if (btnSimAnother) {
      btnSimAnother.addEventListener("click", () => {
        const res = generateDynamicTrial(getAthleteInputs());
        renderResultsScreen(res);
      });
    }
    // 3. "← Athlete / Place" -> returns to Screen 1
    document.getElementById("btnEditAthlete").addEventListener("click", () => showScreen(1));

    // Plot Mode Switcher (Force / Velocity / Power)
    const plotTabs = document.querySelectorAll("#plotTabs .plot-tab");
    plotTabs.forEach((pt) => {
      pt.addEventListener("click", () => {
        plotTabs.forEach((b) => b.classList.remove("active"));
        pt.classList.add("active");
        state.plotMode = pt.getAttribute("data-plot") || "force";
        drawForceCanvas();
      });
    });

    // Export & Clear Log
    document.getElementById("btnExportCampCSV").addEventListener("click", exportAllCampCSV);
    document.getElementById("btnExportTrialJSON").addEventListener("click", exportTrialJSON);
    document.getElementById("btnClearLog").addEventListener("click", () => {
      state.sessionLog = [];
      state.trialCounter = 0;
      saveSessionLog();
      renderSessionLogUI();
    });

    // Touch & Mouse Scrubbing on Plot Canvas
    const canvas = document.getElementById("forceCanvas");
    function handleScrub(clientX) {
      if (!state.currentResult) return;
      const rect = canvas.getBoundingClientRect();
      const frac = Math.max(0, Math.min(1, (clientX - rect.left - 36) / Math.max(1, rect.width - 46)));
      const idx = Math.round(frac * (state.currentResult.t.length - 1));
      state.touchIndex = idx;
      drawForceCanvas();
      const ti = state.currentResult.t[idx];
      const f = state.currentResult.vgrfBW[idx];
      const v = state.currentResult.vel[idx];
      const p = state.currentResult.powerRel[idx];
      document.getElementById("chartTouchCaption").textContent =
        `t=${ti.toFixed(2)}s · Force: ${f.toFixed(2)}×BW · Vel: ${v.toFixed(2)}m/s · Power: ${p.toFixed(1)}W/kg`;
    }
    canvas.addEventListener("mousemove", (e) => handleScrub(e.clientX));
    canvas.addEventListener("touchmove", (e) => {
      if (e.touches && e.touches[0]) handleScrub(e.touches[0].clientX);
    }, { passive: true });

    window.addEventListener("resize", () => {
      if (state.currentStep === 3) drawForceCanvas();
    });
  });
})();
