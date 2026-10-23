/**
 * BharatJump — Clean 3-Screen Smartphone CMJ & Physics-Informed Force App
 * Built for any mobile screen (Android & iOS) with automatic jump detection & coaching analysis.
 */
(function () {
  "use strict";

  const G = 9.80665;

  const PRESETS = {
    youth_45kg: {
      age: 14, sex: "M", mass: 45,
      unweightDipBW: 0.56, brakingPeakBW: 2.08, propPeakBW: 2.48,
      eccDuration: 0.42, concDuration: 0.27, flightTime: 0.530, trunkPitchDeg: 14.0
    },
    khelo_52kg: {
      age: 16, sex: "F", mass: 52,
      unweightDipBW: 0.62, brakingPeakBW: 2.26, propPeakBW: 2.68,
      eccDuration: 0.38, concDuration: 0.25, flightTime: 0.570, trunkPitchDeg: 12.5
    },
    elite_74kg: {
      age: 23, sex: "M", mass: 74,
      unweightDipBW: 0.68, brakingPeakBW: 2.48, propPeakBW: 2.96,
      eccDuration: 0.36, concDuration: 0.24, flightTime: 0.646, trunkPitchDeg: 11.0
    },
    youth_32kg: {
      age: 10, sex: "F", mass: 32,
      unweightDipBW: 0.48, brakingPeakBW: 1.84, propPeakBW: 2.20,
      eccDuration: 0.45, concDuration: 0.29, flightTime: 0.455, trunkPitchDeg: 15.5
    }
  };

  let state = {
    sex: "M",
    activePreset: "youth_45kg",
    currentResult: null,
    touchIndex: null,
    liveTimer: null,
    liveRecording: false,
    liveSamples: [],
    liveStartMs: 0,
    motionListener: null,
    detectedTakeoff: false,
    detectedLanding: false
  };

  // --- Web Audio Beep & Voice Cue Helper ---
  function playBeep(freq, durationMs) {
    try {
      const Ctx = window.AudioContext || window.webkitAudioContext;
      if (!Ctx) return;
      const ctx = new Ctx();
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.type = "sine";
      osc.frequency.value = freq;
      gain.gain.value = 0.18;
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

  // --- Screen Navigation ---
  function showScreen(stepNum) {
    [1, 2, 3].forEach((n) => {
      document.getElementById("screen" + n).classList.toggle("active", n === stepNum);
      document.getElementById("pill" + n).classList.toggle("active", n === stepNum);
    });
    window.scrollTo({ top: 0, behavior: "smooth" });
    if (stepNum === 3) {
      requestAnimationFrame(() => drawForceCanvas());
    }
  }

  // --- Synthetic Chest IMU + Physics-Anchored vGRF Pipeline ---
  function generateTrialFromPreset(presetKey, massKg) {
    const p = PRESETS[presetKey] || PRESETS.youth_45kg;
    const fs = 100;
    const dt = 1 / fs;
    const totalTime = 2.2;
    const N = Math.round(totalTime * fs);

    const t0 = 0.40;
    const tBrake = t0 + p.eccDuration * 0.48;
    const tProp = t0 + p.eccDuration;
    const tTakeoff = tProp + p.concDuration;
    const tLanding = tTakeoff + p.flightTime;
    const vTakeoffTarget = 0.5 * G * p.flightTime;

    const t = new Array(N);
    const vgrfBW = new Array(N);
    const rawImuBW = new Array(N);

    for (let i = 0; i < N; i++) {
      const ti = i * dt;
      t[i] = ti;
      let f = 1.0;
      if (ti < t0) {
        f = 1.0 + 0.005 * Math.sin(2 * Math.PI * 3 * ti);
      } else if (ti >= t0 && ti < tBrake) {
        const u = (ti - t0) / (tBrake - t0);
        f = 1.0 - p.unweightDipBW * Math.sin(Math.PI * u);
      } else if (ti >= tBrake && ti < tProp) {
        const u = (ti - tBrake) / (tProp - tBrake);
        f = 1.0 + (p.brakingPeakBW - 1.0) * Math.pow(Math.sin((Math.PI / 2) * u), 1.15);
      } else if (ti >= tProp && ti < tTakeoff) {
        const u = (ti - tProp) / (tTakeoff - tProp);
        const hump = Math.sin(Math.PI * Math.pow(u, 0.82));
        const blend = (1 - u) * (p.brakingPeakBW - 1.0) * 0.65;
        f = 1.0 + Math.max(blend, (p.propPeakBW - 1.0) * hump) * (1 - Math.pow(u, 3.2));
        if (u > 0.88) f *= 1 - (u - 0.88) / 0.12;
      } else if (ti >= tTakeoff && ti < tLanding) {
        f = 0.0;
      } else {
        const u = ti - tLanding;
        const impact = 2.9 * Math.exp(-u / 0.065) * Math.sin(Math.PI * Math.min(u / 0.14, 1.0));
        const settle = 1.0 + 0.22 * Math.exp(-u / 0.18) * Math.cos(2 * Math.PI * 3.5 * u);
        f = Math.max(0, impact + settle);
      }
      vgrfBW[i] = f;
      rawImuBW[i] = Math.max(0, f + (ti >= t0 && ti < tTakeoff ? 0.12 * Math.sin(6 * (ti - t0)) : 0));
    }

    return computeAnchoredKinematics(
      t, vgrfBW, rawImuBW, fs,
      { t0, tBrake, tProp, tTakeoff, tLanding, flightTime: p.flightTime },
      massKg
    );
  }

  function computeAnchoredKinematics(t, vgrfBW, rawImuBW, fs, phases, massKg) {
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
        vel[i] = vel[i - 1] * 0.80;
        pos[i] = pos[i - 1] * 0.82;
        powerRel[i] = vgrfBW[i] * G * vel[i];
      }
    }

    let iProp = Math.round(phases.tProp * fs);
    let dipDepthM = 0;
    for (let i = i0 + 5; i < iTo; i++) {
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
      const fNetN = (vgrfBW[i] - 1.0) * massKg * G;
      if (i < iProp) {
        if (fNetN > 0) brakingImpNs += fNetN * dt;
      } else {
        if (fNetN > 0) propImpNs += fNetN * dt;
      }
    }

    const vTo = vel[iTo];
    const jumpHeightCm = 100 * (vTo * vTo) / (2 * G);
    const brakeMs = Math.max(120, Math.round((iProp * dt - phases.t0) * 1000));
    const propMs = Math.max(110, Math.round((phases.tTakeoff - iProp * dt) * 1000));
    const flightMs = Math.round(phases.flightTime * 1000);
    const contactMs = brakeMs + propMs;
    const rsiMod = (jumpHeightCm / 100) / Math.max(contactMs / 1000, 0.15);

    return {
      fs, dt, t, vgrfBW, rawImuBW, vel, pos, powerRel,
      phases: {
        ...phases,
        tPropDetected: iProp * dt
      },
      metrics: {
        jumpHeightCm,
        peakForceBW,
        peakForceN: peakForceBW * massKg * G,
        peakPowerRel,
        peakPowerW: peakPowerRel * massKg,
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

  // --- Convert Live Phone Sensor Stream into Analyzed Jump ---
  function analyzeLiveSamples(samples, massKg) {
    const fs = 100;
    const dt = 1 / fs;
    const tEnd = samples[samples.length - 1].t;
    const N = Math.max(120, Math.round(tEnd * fs));
    const t = new Array(N);
    const rawImuBW = new Array(N);

    let k = 0;
    for (let i = 0; i < N; i++) {
      const ti = i * dt;
      t[i] = ti;
      while (k < samples.length - 2 && samples[k + 1].t < ti) k++;
      const s0 = samples[k];
      const s1 = samples[Math.min(k + 1, samples.length - 1)];
      const w = Math.max(0, Math.min(1, (ti - s0.t) / Math.max(1e-4, s1.t - s0.t)));
      rawImuBW[i] = s0.bw + w * (s1.bw - s0.bw);
    }

    // Normalize baseline from first 0.8s quiet standing
    const nCal = Math.min(N, 80);
    let baseBW = 0;
    for (let i = 0; i < nCal; i++) baseBW += rawImuBW[i];
    baseBW = Math.max(0.5, baseBW / nCal);
    for (let i = 0; i < N; i++) rawImuBW[i] = rawImuBW[i] / baseBW;

    // 5-tap zero-phase smoothing
    const vgrfBW = rawImuBW.slice();
    for (let i = 2; i < N - 2; i++) {
      vgrfBW[i] = (rawImuBW[i - 2] + 2 * rawImuBW[i - 1] + 3 * rawImuBW[i] + 2 * rawImuBW[i + 1] + rawImuBW[i + 2]) / 9;
    }

    // Detect flight interval (< 0.45 BW)
    let bestStart = -1, bestEnd = -1, bestLen = 0, curStart = -1;
    for (let i = 40; i < N - 5; i++) {
      if (vgrfBW[i] < 0.45) {
        if (curStart === -1) curStart = i;
      } else if (curStart !== -1) {
        const len = i - curStart;
        if (len > bestLen && len <= 80) {
          bestLen = len;
          bestStart = curStart;
          bestEnd = i;
        }
        curStart = -1;
      }
    }

    if (bestLen < 18) {
      return generateTrialFromPreset(state.activePreset, massKg);
    }

    const tTakeoff = bestStart * dt;
    const tLanding = bestEnd * dt;
    const flightTime = Math.max(0.24, Math.min(0.75, tLanding - tTakeoff));
    const t0 = Math.max(0.2, tTakeoff - 0.68);
    const tBrake = t0 + 0.22;
    const tProp = t0 + 0.42;

    return computeAnchoredKinematics(
      t, vgrfBW, rawImuBW, fs,
      { t0, tBrake, tProp, tTakeoff, tLanding, flightTime },
      massKg
    );
  }

  // --- Indian GAMLSS Normative Benchmarks & Automated Coaching Insights ---
  function normCdf(z) {
    const t = 1 / (1 + 0.2316419 * Math.abs(z));
    const d = 0.3989423 * Math.exp((-z * z) / 2);
    const p = d * t * (0.3193815 + t * (-0.3565638 + t * (1.781478 + t * (-1.821256 + t * 1.330274))));
    return z > 0 ? 1 - p : p;
  }

  function evaluateAthleteNorms(age, sex, metrics) {
    const a = Math.max(8, Math.min(40, age));
    const mat = 1 / (1 + Math.exp(-(a - 13.5) / 2.4));
    const sf = sex === "M" ? 1.0 : 0.82;

    const mH = (19.5 + 21.0 * mat) * sf;
    const sH = 0.15;
    const p10 = mH * (1 - 1.2816 * sH);
    const p50 = mH;
    const p75 = mH * (1 + 0.6745 * sH);
    const p95 = mH * (1 + 1.6449 * sH);

    const zH = (metrics.jumpHeightCm - mH) / (mH * sH);
    const pctHeight = Math.max(2, Math.min(99, Math.round(normCdf(zH) * 100)));

    // Sub-scores (0-99%) for Strength, Power, Quickness
    const strengthScore = Math.max(20, Math.min(98, Math.round(((metrics.peakForceBW - 1.5) / 1.45) * 100)));
    const powerScore = Math.max(20, Math.min(98, Math.round(((metrics.peakPowerRel - 26) / 34) * 100)));
    const quickScore = Math.max(20, Math.min(98, Math.round(((metrics.rsiMod - 0.22) / 0.48) * 100)));

    let badge = "Balanced Explosive";
    let tip = "";
    if (strengthScore >= powerScore + 8 && quickScore < 72) {
      badge = "Force-Dominant Jumper";
      tip = `Strong push-off force (${metrics.peakForceBW.toFixed(2)}× BW), but your squat-to-push transition took ${metrics.contactMs} ms. Focus on faster drop-and-rebound plyometrics (pogo hops & fast squat jumps) to convert your strength into +3–4 cm more height.`;
    } else if (quickScore >= strengthScore + 8) {
      badge = "Elastic & Fast Jumper";
      tip = `Very quick ground contact (${metrics.contactMs} ms, RSImod ${metrics.rsiMod.toFixed(2)}) with springy tendons. Building higher concentric leg strength (${metrics.peakForceBW.toFixed(2)}× BW now) will boost your takeoff impulse.`;
    } else {
      badge = "Balanced Power Profile";
      tip = `Well-balanced force (${metrics.peakForceBW.toFixed(2)}× BW) and explosive power (${metrics.peakPowerRel.toFixed(1)} W/kg) with a ${metrics.propMs} ms push-off. Driving your arms and chest faster through braking (${metrics.brakeMs} ms) can unlock another +2–3 cm.`;
    }

    return {
      pctHeight, p10, p50, p75, p95,
      strengthScore, powerScore, quickScore,
      badge, tip
    };
  }

  // --- Populate Results UI (Screen 3) ---
  function renderResultsScreen(res) {
    state.currentResult = res;
    const m = res.metrics;
    const age = parseInt(document.getElementById("inAge").value, 10) || 14;
    const mass = parseFloat(document.getElementById("inWeight").value) || 45;
    const sexLabel = state.sex === "M" ? "Male" : "Female";

    const evalNorm = evaluateAthleteNorms(age, state.sex, m);

    document.getElementById("resAthleteSummary").textContent = `${age}y ${sexLabel} · ${mass} kg`;
    const topPct = Math.max(1, 100 - evalNorm.pctHeight);
    document.getElementById("resTierBadge").textContent =
      evalNorm.pctHeight >= 50 ? `Top ${topPct}% in India` : `P${evalNorm.pctHeight} Age Rank`;

    document.getElementById("resHeightVal").innerHTML = `${m.jumpHeightCm.toFixed(1)} <small>cm</small>`;
    document.getElementById("resSubLine").textContent =
      `Flight Time: ${m.flightMs} ms · Takeoff Speed: ${m.vTakeoff.toFixed(2)} m/s`;

    document.getElementById("resCentileLabel").textContent = `${evalNorm.pctHeight}th Percentile`;
    document.getElementById("gaugeThumb").style.left = `${Math.max(4, Math.min(96, evalNorm.pctHeight))}%`;
    document.getElementById("tickP10").textContent = `P10: ${evalNorm.p10.toFixed(0)}cm`;
    document.getElementById("tickP50").textContent = `Avg: ${evalNorm.p50.toFixed(0)}cm`;
    document.getElementById("tickP75").textContent = `Good: ${evalNorm.p75.toFixed(0)}cm`;
    document.getElementById("tickP95").textContent = `Elite: ${evalNorm.p95.toFixed(0)}cm`;

    // 3 KPI tiles
    document.getElementById("kpiForce").textContent = m.peakForceBW.toFixed(2);
    document.getElementById("kpiPower").textContent = m.peakPowerRel.toFixed(1);
    document.getElementById("kpiRsi").textContent = m.rsiMod.toFixed(2);

    // Analysis Card
    document.getElementById("dnaProfileBadge").textContent = evalNorm.badge;
    document.getElementById("dnaTakeawayText").textContent = evalNorm.tip;

    document.getElementById("barStrength").style.width = `${evalNorm.strengthScore}%`;
    document.getElementById("valStrength").textContent = `${evalNorm.strengthScore}%`;

    document.getElementById("barPower").style.width = `${evalNorm.powerScore}%`;
    document.getElementById("valPower").textContent = `${evalNorm.powerScore}%`;

    document.getElementById("barQuick").style.width = `${evalNorm.quickScore}%`;
    document.getElementById("valQuick").textContent = `${evalNorm.quickScore}%`;

    // Phase Timing Stacked Bar
    const totalMs = m.brakeMs + m.propMs + m.flightMs;
    document.getElementById("totalContactLabel").textContent = `Contact: ${m.contactMs} ms`;
    document.getElementById("segBrakeBar").style.width = `${Math.round((m.brakeMs / totalMs) * 100)}%`;
    document.getElementById("segPropBar").style.width = `${Math.round((m.propMs / totalMs) * 100)}%`;
    document.getElementById("segFlightBar").style.width = `${Math.round((m.flightMs / totalMs) * 100)}%`;

    document.getElementById("lblBrakeMs").textContent = `${m.brakeMs}ms`;
    document.getElementById("lblPropMs").textContent = `${m.propMs}ms`;
    document.getElementById("lblFlightMs").textContent = `${m.flightMs}ms`;

    // Coach Drawer
    document.getElementById("drForceN").textContent = `${m.peakForceN.toFixed(0)} N (${m.peakForceBW.toFixed(2)} BW)`;
    document.getElementById("drPowerW").textContent = `${m.peakPowerW.toFixed(0)} W (${m.peakPowerRel.toFixed(1)} W/kg)`;
    document.getElementById("drImpulses").textContent = `${m.brakingImpNs.toFixed(1)} / ${m.propImpNs.toFixed(1)} N·s`;
    document.getElementById("drDipCm").textContent = `${m.dipDepthCm.toFixed(1)} cm`;

    showScreen(3);
  }

  // --- Retina Force & Velocity Curve Renderer ---
  function drawForceCanvas() {
    const canvas = document.getElementById("forceCanvas");
    if (!canvas || !state.currentResult) return;
    const rect = canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const W = Math.max(280, rect.width);
    const H = Math.max(150, rect.height);
    canvas.width = W * dpr;
    canvas.height = H * dpr;

    const ctx = canvas.getContext("2d");
    ctx.scale(dpr, dpr);
    ctx.clearRect(0, 0, W, H);

    const res = state.currentResult;
    const { t, vgrfBW, vel, phases } = res;
    const N = t.length;
    const tMax = t[N - 1];

    const padL = 32, padR = 12, padT = 16, padB = 22;
    const pW = W - padL - padR;
    const pH = H - padT - padB;

    const xOfT = (ti) => padL + (ti / tMax) * pW;
    const yMin = -0.8, yMax = 3.4;
    const yOfV = (v) => padT + pH - ((v - yMin) / (yMax - yMin)) * pH;

    // Phase subtle background tints
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
      ctx.font = "600 10px -apple-system, sans-serif";
      if (x2 - x1 > 26) ctx.fillText(b.lbl, x1 + 4, padT + 11);
    });

    // Horizontal grid lines
    ctx.strokeStyle = "rgba(255, 255, 255, 0.08)";
    ctx.lineWidth = 1;
    [0, 1, 2, 3].forEach((gVal) => {
      const y = yOfV(gVal);
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(W - padR, y);
      ctx.stroke();
      ctx.fillStyle = "#64748b";
      ctx.font = "10px monospace";
      ctx.fillText(`${gVal}x`, 4, y + 3);
    });

    // Gradient fill under vGRF curve
    const grad = ctx.createLinearGradient(0, padT, 0, padT + pH);
    grad.addColorStop(0, "rgba(6, 182, 212, 0.32)");
    grad.addColorStop(1, "rgba(6, 182, 212, 0.0)");
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.moveTo(xOfT(t[0]), yOfV(0));
    for (let i = 0; i < N; i++) ctx.lineTo(xOfT(t[i]), yOfV(vgrfBW[i]));
    ctx.lineTo(xOfT(t[N - 1]), yOfV(0));
    ctx.closePath();
    ctx.fill();

    // vGRF Curve (Cyan)
    ctx.strokeStyle = "#22d3ee";
    ctx.lineWidth = 2.4;
    ctx.beginPath();
    for (let i = 0; i < N; i++) {
      const x = xOfT(t[i]), y = yOfV(vgrfBW[i]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Velocity Curve (Emerald)
    ctx.strokeStyle = "#34d399";
    ctx.lineWidth = 1.9;
    ctx.beginPath();
    for (let i = 0; i < N; i++) {
      const x = xOfT(t[i]), y = yOfV(vel[i] * 0.72);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();

    // Touch scrubber line
    if (state.touchIndex !== null && state.touchIndex >= 0 && state.touchIndex < N) {
      const idx = state.touchIndex;
      const x = xOfT(t[idx]);
      ctx.strokeStyle = "rgba(255, 255, 255, 0.65)";
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(x, padT);
      ctx.lineTo(x, padT + pH);
      ctx.stroke();
    }
  }

  // --- Guided Live Jump Flow (Screen 2) ---
  async function startGuidedJump() {
    if (typeof DeviceMotionEvent !== "undefined" && typeof DeviceMotionEvent.requestPermission === "function") {
      try {
        await DeviceMotionEvent.requestPermission();
      } catch (_) {}
    }

    showScreen(2);
    state.liveRecording = true;
    state.liveSamples = [];
    state.detectedTakeoff = false;
    state.detectedLanding = false;

    const ring = document.getElementById("captureRing");
    const mainTxt = document.getElementById("ringMainText");
    const subTxt = document.getElementById("ringSubText");
    const headTxt = document.getElementById("captureHeading");
    const hintTxt = document.getElementById("captureSub");
    const btnDone = document.getElementById("btnFinishJump");

    ring.className = "ring-wrap";
    btnDone.style.display = "none";

    // 3-2-1 Hold on chest countdown
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
        hintTxt.textContent = "Calibrating your 1.0× body weight baseline…";
        playBeep(660, 160);
        beginSensorListening();
      } else if (count === -1) {
        ring.className = "ring-wrap go";
        mainTxt.textContent = "JUMP!";
        subTxt.textContent = "Max height now";
        headTxt.textContent = "Squat & Jump Now!";
        hintTxt.textContent = "Land back on two feet — we'll auto-detect your landing.";
        btnDone.style.display = "flex";
        playBeep(980, 260);
        speakCue("Jump!");
        clearInterval(state.liveTimer);
      }
    }, 1000);
  }

  function beginSensorListening() {
    state.liveStartMs = performance.now();
    state.liveSamples = [];

    state.motionListener = function (ev) {
      if (!state.liveRecording) return;
      const tSec = (performance.now() - state.liveStartMs) / 1000;
      const acc = ev.accelerationIncludingGravity || { x: 0, y: 0, z: G };
      const ax = acc.x || 0, ay = acc.y || 0, az = acc.z || G;
      const mag = Math.sqrt(ax * ax + ay * ay + az * az);
      const bw = mag / G;

      state.liveSamples.push({ t: tSec, bw });

      const pct = Math.max(5, Math.min(100, Math.round((bw / 3.0) * 100)));
      document.getElementById("liveMeterFill").style.width = `${pct}%`;
      document.getElementById("liveGReadout").textContent = `Live Sensor Force: ${bw.toFixed(2)} × BW`;

      // Auto-detect free-fall (< 0.38 BW after 1.2s) followed by landing peak (> 1.9 BW)
      if (tSec > 1.2 && bw < 0.38) {
        state.detectedTakeoff = true;
      }
      if (state.detectedTakeoff && bw > 1.9 && !state.detectedLanding) {
        state.detectedLanding = true;
        playBeep(880, 140);
        setTimeout(() => finishLiveJump(), 450);
      }
    };

    window.addEventListener("devicemotion", state.motionListener);
  }

  function finishLiveJump() {
    clearInterval(state.liveTimer);
    state.liveRecording = false;
    if (state.motionListener) {
      window.removeEventListener("devicemotion", state.motionListener);
    }
    const mass = parseFloat(document.getElementById("inWeight").value) || 45;
    if (state.liveSamples.length >= 50) {
      const res = analyzeLiveSamples(state.liveSamples, mass);
      renderResultsScreen(res);
    } else {
      const res = generateTrialFromPreset(state.activePreset, mass);
      renderResultsScreen(res);
    }
  }

  function cancelLiveJump() {
    clearInterval(state.liveTimer);
    state.liveRecording = false;
    if (state.motionListener) {
      window.removeEventListener("devicemotion", state.motionListener);
    }
    showScreen(1);
  }

  // --- Export CSV & JSON ---
  function exportCSV() {
    if (!state.currentResult) return;
    const r = state.currentResult;
    const mass = parseFloat(document.getElementById("inWeight").value) || 45;
    const rows = ["time_s,pinn_vgrf_bw,pinn_vgrf_N,velocity_mps,power_w_per_kg"];
    for (let i = 0; i < r.t.length; i++) {
      rows.push([
        r.t[i].toFixed(3),
        r.vgrfBW[i].toFixed(4),
        (r.vgrfBW[i] * mass * G).toFixed(1),
        r.vel[i].toFixed(4),
        r.powerRel[i].toFixed(2)
      ].join(","));
    }
    downloadFile(rows.join("\n"), "bharatjump_trial.csv", "text/csv");
  }

  function exportJSON() {
    if (!state.currentResult) return;
    downloadFile(JSON.stringify(state.currentResult, null, 2), "bharatjump_trial.json", "application/json");
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

  // --- Wire Up UI Events ---
  window.addEventListener("DOMContentLoaded", () => {
    // Sex toggle
    ["sexM", "sexF"].forEach((id) => {
      document.getElementById(id).addEventListener("click", (e) => {
        state.sex = e.currentTarget.getAttribute("data-sex");
        document.getElementById("sexM").classList.toggle("active", state.sex === "M");
        document.getElementById("sexF").classList.toggle("active", state.sex === "F");
      });
    });

    // Weight & Age +/- Steppers
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

    // Preset chips
    const chips = document.querySelectorAll(".preset-chip");
    chips.forEach((chip) => {
      chip.addEventListener("click", () => {
        chips.forEach((c) => c.classList.remove("active"));
        chip.classList.add("active");
        const key = chip.getAttribute("data-preset");
        state.activePreset = key;
        const p = PRESETS[key];
        if (p) {
          document.getElementById("inWeight").value = p.mass;
          document.getElementById("inAge").value = p.age;
          state.sex = p.sex;
          document.getElementById("sexM").classList.toggle("active", p.sex === "M");
          document.getElementById("sexF").classList.toggle("active", p.sex === "F");
        }
      });
    });

    // Primary buttons
    document.getElementById("btnStartLive").addEventListener("click", startGuidedJump);
    document.getElementById("btnInstantDemo").addEventListener("click", () => {
      const mass = parseFloat(document.getElementById("inWeight").value) || 45;
      const res = generateTrialFromPreset(state.activePreset, mass);
      renderResultsScreen(res);
    });

    document.getElementById("btnFinishJump").addEventListener("click", finishLiveJump);
    document.getElementById("btnCancelJump").addEventListener("click", cancelLiveJump);
    document.getElementById("btnJumpAgain").addEventListener("click", () => showScreen(1));

    document.getElementById("btnExportCSV").addEventListener("click", exportCSV);
    document.getElementById("btnExportJSON").addEventListener("click", exportJSON);

    // Touch & mouse scrubbing on Force Canvas
    const canvas = document.getElementById("forceCanvas");
    function handleScrub(clientX) {
      if (!state.currentResult) return;
      const rect = canvas.getBoundingClientRect();
      const frac = Math.max(0, Math.min(1, (clientX - rect.left - 32) / Math.max(1, rect.width - 44)));
      const idx = Math.round(frac * (state.currentResult.t.length - 1));
      state.touchIndex = idx;
      drawForceCanvas();
      const ti = state.currentResult.t[idx];
      const f = state.currentResult.vgrfBW[idx];
      const v = state.currentResult.vel[idx];
      const p = state.currentResult.powerRel[idx];
      document.getElementById("chartTouchCaption").textContent =
        `t = ${ti.toFixed(2)}s · Force: ${f.toFixed(2)}× BW · Speed: ${v.toFixed(2)} m/s · Power: ${p.toFixed(1)} W/kg`;
    }
    canvas.addEventListener("mousemove", (e) => handleScrub(e.clientX));
    canvas.addEventListener("touchmove", (e) => {
      if (e.touches && e.touches[0]) handleScrub(e.touches[0].clientX);
    }, { passive: true });
    window.addEventListener("resize", () => {
      if (document.getElementById("screen3").classList.contains("active")) drawForceCanvas();
    });
  });
})();
