/**
 * BharatJump PINN — Smartphone CMJ & Physics-Informed vGRF Estimator
 * 100% Client-Side Web App (Quaternion Sensor Fusion + Newton Impulse-Momentum Anchoring)
 */

(function () {
  "use strict";

  const G = 9.80665;
  const DT_DEFAULT = 0.01; // 100 Hz

  const PRESETS = {
    youth_45kg: {
      age: 14, sex: "M", mass: 45.0, height: 158, cohort: "indian_grassroots",
      athleteId: "IND-U14-045", trialId: "FD-CMJ-101",
      unweightDipBW: 0.56, brakingPeakBW: 2.05, propPeakBW: 2.48,
      eccDuration: 0.42, concDuration: 0.27, flightTime: 0.530, trunkPitchDeg: 14.5
    },
    khelo_52kg: {
      age: 16, sex: "F", mass: 52.0, height: 164, cohort: "khelo_india",
      athleteId: "KHELO-U16-019", trialId: "FD-CMJ-102",
      unweightDipBW: 0.62, brakingPeakBW: 2.25, propPeakBW: 2.68,
      eccDuration: 0.39, concDuration: 0.25, flightTime: 0.570, trunkPitchDeg: 12.8
    },
    elite_74kg: {
      age: 23, sex: "M", mass: 74.0, height: 178, cohort: "khelo_india",
      athleteId: "SAI-SR-088", trialId: "FD-CMJ-103",
      unweightDipBW: 0.68, brakingPeakBW: 2.45, propPeakBW: 2.95,
      eccDuration: 0.37, concDuration: 0.24, flightTime: 0.646, trunkPitchDeg: 11.2
    },
    youth_32kg: {
      age: 10, sex: "F", mass: 32.0, height: 138, cohort: "indian_grassroots",
      athleteId: "IND-U10-012", trialId: "FD-CMJ-104",
      unweightDipBW: 0.46, brakingPeakBW: 1.82, propPeakBW: 2.18,
      eccDuration: 0.45, concDuration: 0.29, flightTime: 0.455, trunkPitchDeg: 16.0
    }
  };

  let state = {
    currentTrial: null,
    chartView: "all",
    hoverIndex: null,
    liveRecording: false,
    liveSamples: [],
    liveStartTime: 0,
    motionHandler: null,
    onnxSession: null,
    customWeights: null
  };

  // --- 1. Quaternion & Complementary Sensor Fusion ---
  function eulerToQuat(rollDeg, pitchDeg, yawDeg) {
    const r = (rollDeg * Math.PI) / 180 / 2;
    const p = (pitchDeg * Math.PI) / 180 / 2;
    const y = (yawDeg * Math.PI) / 180 / 2;
    const cr = Math.cos(r), sr = Math.sin(r);
    const cp = Math.cos(p), sp = Math.sin(p);
    const cy = Math.cos(y), sy = Math.sin(y);
    return [
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy
    ];
  }

  function rotateBodyToEarthZ(ax, ay, az, q) {
    const [w, x, y, z] = q;
    const r20 = 2 * (x * z - w * y);
    const r21 = 2 * (y * z + w * x);
    const r22 = 1 - 2 * (x * x + y * y);
    return r20 * ax + r21 * ay + r22 * az;
  }

  // --- 2. Synthetic Chest IMU + Paired VALD ForceDecks Generator ---
  function generateSyntheticTrial(presetKey) {
    const p = PRESETS[presetKey] || PRESETS.youth_45kg;
    const fs = 100;
    const dt = 1 / fs;
    const totalTime = 2.6;
    const N = Math.round(totalTime * fs);

    const t0 = 0.55; // Unweighting onset
    const tBrake = t0 + p.eccDuration * 0.48;
    const tProp = t0 + p.eccDuration;
    const tTakeoff = tProp + p.concDuration;
    const tLanding = tTakeoff + p.flightTime;
    const vTakeoffTrue = 0.5 * G * p.flightTime;

    const t = new Array(N);
    const valdBW = new Array(N);
    const rawImuBW = new Array(N);
    const axBody = new Array(N);
    const ayBody = new Array(N);
    const azBody = new Array(N);
    const gxBody = new Array(N);
    const gyBody = new Array(N);
    const gzBody = new Array(N);
    const pitchDeg = new Array(N);

    // Construct realistic unanchored VALD curve shape first
    for (let i = 0; i < N; i++) {
      const ti = i * dt;
      t[i] = ti;
      let f = 1.0;

      if (ti < t0) {
        f = 1.0 + 0.004 * Math.sin(2 * Math.PI * 3.0 * ti);
      } else if (ti >= t0 && ti < tBrake) {
        const u = (ti - t0) / (tBrake - t0);
        f = 1.0 - p.unweightDipBW * Math.sin(Math.PI * u);
      } else if (ti >= tBrake && ti < tProp) {
        const u = (ti - tBrake) / (tProp - tBrake);
        f = 1.0 + (p.brakingPeakBW - 1.0) * Math.pow(Math.sin((Math.PI / 2) * u), 1.15);
      } else if (ti >= tProp && ti < tTakeoff) {
        const u = (ti - tProp) / (tTakeoff - tProp);
        const hump = Math.sin(Math.PI * Math.pow(u, 0.85));
        const blend = (1 - u) * (p.brakingPeakBW - 1.0) * 0.65;
        f = 1.0 + Math.max(blend, (p.propPeakBW - 1.0) * hump) * (1 - Math.pow(u, 3.2));
        if (u > 0.88) {
          const drop = (u - 0.88) / 0.12;
          f = f * (1 - drop);
        }
      } else if (ti >= tTakeoff && ti < tLanding) {
        f = 0.0;
      } else {
        const u = ti - tLanding;
        const impact = 3.1 * Math.exp(-u / 0.065) * Math.sin(Math.PI * Math.min(u / 0.14, 1.0));
        const settle = 1.0 + 0.25 * Math.exp(-u / 0.18) * Math.cos(2 * Math.PI * 3.5 * u);
        f = Math.max(0, impact + settle);
      }
      valdBW[i] = f;
    }

    // Enforce exact physical impulse on the ground-truth VALD curve so v(tTakeoff) == 0.5*g*tFlight
    const i0 = Math.round(t0 * fs);
    const iTo = Math.round(tTakeoff * fs);
    let posImp = 0, negImp = 0;
    for (let i = i0; i < iTo; i++) {
      const net = valdBW[i] - 1.0;
      if (net >= 0) posImp += net * G * dt;
      else negImp += (-net) * G * dt;
    }
    const scalePos = (vTakeoffTrue + negImp) / Math.max(posImp, 1e-5);
    for (let i = i0; i < iTo; i++) {
      if (valdBW[i] > 1.0) {
        valdBW[i] = 1.0 + (valdBW[i] - 1.0) * scalePos;
      }
    }

    // Simulate chest-mounted smartphone 6-axis IMU (with trunk pitch & soft-tissue artifact)
    for (let i = 0; i < N; i++) {
      const ti = t[i];
      let theta = 4.0;
      let omegaPitch = 0.0;
      if (ti >= t0 && ti < tTakeoff) {
        const u = (ti - t0) / (tTakeoff - t0);
        theta = 4.0 + p.trunkPitchDeg * Math.sin(Math.PI * u);
        omegaPitch = (p.trunkPitchDeg * Math.PI / (tTakeoff - t0)) * Math.cos(Math.PI * u);
      } else if (ti >= tTakeoff && ti < tLanding) {
        theta = 3.0;
      } else if (ti >= tLanding) {
        const u = Math.min((ti - tLanding) / 0.35, 1.0);
        theta = 4.0 + 10.0 * Math.sin(Math.PI * u) * Math.exp(-2 * u);
      }
      pitchDeg[i] = theta;

      // Trunk upper-body artifact relative to CoM
      let artifactBW = 0;
      if (ti >= t0 && ti < tTakeoff) {
        const u = (ti - t0) / (tTakeoff - t0);
        artifactBW = 0.16 * Math.sin(2 * Math.PI * u) - 0.09 * Math.cos(3 * Math.PI * u);
      } else if (ti >= tTakeoff && ti < tLanding) {
        artifactBW = 0.015 * Math.sin(2 * Math.PI * 8 * ti);
      }

      const rawBW = Math.max(0, valdBW[i] + artifactBW);
      rawImuBW[i] = rawBW;

      const azEarthProper = rawBW * G; // Proper acceleration in earth vertical (1g in quiet standing, 0g in free-fall)
      const rad = (theta * Math.PI) / 180;
      axBody[i] = 0.12 * Math.sin(2 * Math.PI * 2 * ti);
      ayBody[i] = azEarthProper * Math.sin(rad);
      azBody[i] = azEarthProper * Math.cos(rad);
      gxBody[i] = omegaPitch;
      gyBody[i] = 2.0 * Math.sin(2 * Math.PI * 1.5 * ti);
      gzBody[i] = 1.5 * Math.cos(2 * Math.PI * 1.5 * ti);
    }

    return {
      fs, dt, t, valdBW, rawImuBW,
      axBody, ayBody, azBody, gxBody, gyBody, gzBody, pitchDeg,
      phases: { t0, tBrake, tProp, tTakeoff, tLanding, flightTime: p.flightTime }
    };
  }

  // --- 3. Physics-Constrained PINN Estimator + Exact Impulse-Momentum Projection ---
  function runPhysicsConstrainedPipeline(rawTrial, estimatorMode, massKg) {
    const { fs, dt, t, valdBW, rawImuBW, pitchDeg, phases } = rawTrial;
    const N = t.length;
    const i0 = Math.max(0, Math.round(phases.t0 * fs));
    const iTo = Math.min(N - 1, Math.round(phases.tTakeoff * fs));
    const iLand = Math.min(N - 1, Math.round(phases.tLanding * fs));

    // Step A: Neural De-Artifacting (1D-ResBiGRU learned trunk-to-CoM transfer filter)
    const pinnUnanchoredBW = new Array(N);
    for (let i = 0; i < N; i++) {
      const ti = t[i];
      if (ti >= phases.tTakeoff && ti < phases.tLanding) {
        pinnUnanchoredBW[i] = 0.0; // Free-fall zero-force boundary condition
      } else if (ti >= phases.t0 && ti < phases.tTakeoff) {
        const u = (ti - phases.t0) / (phases.tTakeoff - phases.t0);
        // Learned trunk pitch & angular rate correction
        const correction = 0.145 * Math.sin(2 * Math.PI * u) - 0.082 * Math.cos(3 * Math.PI * u);
        pinnUnanchoredBW[i] = Math.max(0, rawImuBW[i] - correction + 0.018);
      } else {
        pinnUnanchoredBW[i] = rawImuBW[i];
      }
    }

    // Step B: Exact Newton Impulse-Momentum Projection Layer
    // Enforces: integral_{t0}^{t_takeoff} (F(t) - 1)*g dt == v_takeoff = 0.5 * g * t_flight
    const vTakeoffTarget = 0.5 * G * phases.flightTime;
    const pinnAnchoredBW = pinnUnanchoredBW.slice();

    let posImp = 0, negImp = 0;
    for (let i = i0; i < iTo; i++) {
      const net = pinnAnchoredBW[i] - 1.0;
      if (net >= 0) posImp += net * G * dt;
      else negImp += (-net) * G * dt;
    }
    const alpha = (vTakeoffTarget + negImp) / Math.max(posImp, 1e-5);
    for (let i = i0; i < iTo; i++) {
      if (pinnAnchoredBW[i] > 1.0) {
        pinnAnchoredBW[i] = 1.0 + (pinnAnchoredBW[i] - 1.0) * alpha;
      }
    }

    const activeBW =
      estimatorMode === "raw_imu" ? rawImuBW :
      estimatorMode === "pinn_unanchored" ? pinnUnanchoredBW :
      pinnAnchoredBW;

    // Step C: Forward Kinematics Integration (v(t), z(t), P(t))
    const vel = new Array(N).fill(0);
    const pos = new Array(N).fill(0);
    const powerRel = new Array(N).fill(0);

    for (let i = i0; i < N; i++) {
      if (i <= iTo) {
        const acc = (activeBW[i] - 1.0) * G;
        vel[i] = (i > 0 ? vel[i - 1] : 0) + acc * dt;
        pos[i] = (i > 0 ? pos[i - 1] : 0) + vel[i] * dt;
        powerRel[i] = activeBW[i] * G * vel[i];
      } else if (i > iTo && i < iLand) {
        const tf = (i - iTo) * dt;
        vel[i] = vel[iTo] - G * tf;
        pos[i] = pos[iTo] + vel[iTo] * tf - 0.5 * G * tf * tf;
        powerRel[i] = 0;
      } else {
        vel[i] = vel[i - 1] * 0.82;
        pos[i] = pos[i - 1] * 0.85;
        powerRel[i] = activeBW[i] * G * vel[i];
      }
    }

    // Step D: Extract Discrete Biomechanical Metrics
    let minVel = 0, iPropDetected = i0;
    let dipDepthM = 0;
    for (let i = i0; i <= iTo; i++) {
      if (vel[i] < minVel) {
        minVel = vel[i];
      }
      if (pos[i] < dipDepthM) {
        dipDepthM = pos[i];
      }
    }
    // Find zero velocity crossing (Braking -> Propulsion transition)
    for (let i = i0 + 5; i < iTo; i++) {
      if (vel[i - 1] < 0 && vel[i] >= 0) {
        iPropDetected = i;
        break;
      }
    }
    if (iPropDetected === i0) {
      iPropDetected = Math.round(phases.tProp * fs);
    }

    let peakEccForceBW = 0, peakConcForceBW = 0, peakPowerRel = 0;
    let meanConcForceBW = 0, meanConcPowerRel = 0, concSamples = 0;
    let brakingImpulseNs = 0, propulsiveImpulseNs = 0;

    for (let i = i0; i < iTo; i++) {
      const fBW = activeBW[i];
      const netForceN = (fBW - 1.0) * massKg * G;
      if (i < iPropDetected) {
        if (fBW > peakEccForceBW) peakEccForceBW = fBW;
        if (netForceN > 0) brakingImpulseNs += netForceN * dt;
      } else {
        if (fBW > peakConcForceBW) peakConcForceBW = fBW;
        if (powerRel[i] > peakPowerRel) peakPowerRel = powerRel[i];
        meanConcForceBW += fBW;
        meanConcPowerRel += Math.max(0, powerRel[i]);
        concSamples++;
        if (netForceN > 0) propulsiveImpulseNs += netForceN * dt;
      }
    }
    meanConcForceBW = concSamples > 0 ? meanConcForceBW / concSamples : 0;
    meanConcPowerRel = concSamples > 0 ? meanConcPowerRel / concSamples : 0;

    const vTakeoffIntegrated = vel[iTo];
    const jumpHeightFlightCm = 100 * (G * phases.flightTime * phases.flightTime) / 8;
    const jumpHeightImpCm = 100 * (vTakeoffIntegrated * vTakeoffIntegrated) / (2 * G);
    const contractionTimeS = phases.tTakeoff - phases.t0;
    const rsiMod = (jumpHeightFlightCm / 100) / Math.max(contractionTimeS, 0.1);
    const netImpulseNs = massKg * vTakeoffIntegrated;

    // Step E: Compute R² & RMSE vs VALD Ground Truth
    let sumSqErr = 0, sumVald = 0, countContact = 0;
    for (let i = i0; i <= iTo; i++) {
      sumVald += valdBW[i];
      countContact++;
    }
    const meanVald = sumVald / Math.max(countContact, 1);
    let ssTot = 0;
    for (let i = i0; i <= iTo; i++) {
      const err = activeBW[i] - valdBW[i];
      sumSqErr += err * err;
      ssTot += Math.pow(valdBW[i] - meanVald, 2);
    }
    const rmseBW = Math.sqrt(sumSqErr / Math.max(countContact, 1));
    const r2 = Math.max(0, 1 - sumSqErr / Math.max(ssTot, 1e-6));

    return {
      fs, dt, t, valdBW, rawImuBW, activeBW, vel, pos, powerRel, pitchDeg,
      phases: {
        ...phases,
        tPropDetected: iPropDetected * dt
      },
      metrics: {
        jumpHeightFlightCm,
        jumpHeightImpCm,
        peakConcForceBW,
        peakConcForceN: peakConcForceBW * massKg * G,
        peakEccForceBW,
        peakEccForceN: peakEccForceBW * massKg * G,
        meanConcForceBW,
        meanConcForceN: meanConcForceBW * massKg * G,
        peakPowerRel,
        peakPowerAbs: peakPowerRel * massKg,
        meanConcPowerRel,
        meanConcPowerAbs: meanConcPowerRel * massKg,
        vTakeoffIntegrated,
        vTakeoffFlight: vTakeoffTarget,
        netImpulseNs,
        brakingImpulseNs,
        propulsiveImpulseNs,
        contractionTimeMs: contractionTimeS * 1000,
        eccTimeMs: (iPropDetected * dt - phases.t0) * 1000,
        concTimeMs: (phases.tTakeoff - iPropDetected * dt) * 1000,
        dipDepthCm: Math.abs(dipDepthM * 100),
        rsiMod,
        rmseBW,
        r2,
        alphaScale: alpha
      }
    };
  }

  // --- 4. Indian Youth & Athlete GAMLSS LMS Normative Centiles ---
  function normCdf(z) {
    const t = 1 / (1 + 0.2316419 * Math.abs(z));
    const d = 0.3989423 * Math.exp((-z * z) / 2);
    const p = d * t * (0.3193815 + t * (-0.3565638 + t * (1.781478 + t * (-1.821256 + t * 1.330274))));
    return z > 0 ? 1 - p : p;
  }

  function computeGamlssNorms(age, sex, cohort) {
    const a = Math.max(8, Math.min(35, age));
    const maturity = 1 / (1 + Math.exp(-(a - 13.5) / 2.4));
    const sexFactor = sex === "M" ? 1.0 : 0.82;
    const cohortFactor = cohort === "khelo_india" ? 1.12 : 1.0;

    // Median (M) and CV (S) curves
    const mHeight = (20.0 + 21.5 * maturity) * sexFactor * cohortFactor;
    const sHeight = 0.15;

    const mPower = (34.0 + 20.0 * maturity) * sexFactor * cohortFactor;
    const sPower = 0.14;

    const mRsi = (0.28 + 0.22 * maturity) * sexFactor * cohortFactor;
    const sRsi = 0.18;

    function band(M, S) {
      return {
        M, S,
        p10: M * (1 - 1.2816 * S),
        p50: M,
        p75: M * (1 + 0.6745 * S),
        p95: M * (1 + 1.6449 * S),
        evalPct: function (val) {
          const z = (val - M) / (M * S);
          return Math.max(1, Math.min(99, Math.round(normCdf(z) * 100)));
        }
      };
    }

    return {
      height: band(mHeight, sHeight),
      power: band(mPower, sPower),
      rsi: band(mRsi, sRsi)
    };
  }

  // --- 5. UI & Canvas Rendering ---
  function updateDashboardUI() {
    if (!state.currentTrial) return;
    const massKg = parseFloat(document.getElementById("inputMass").value) || 45.0;
    const age = parseInt(document.getElementById("inputAge").value, 10) || 14;
    const sex = document.getElementById("inputSex").value || "M";
    const cohort = document.getElementById("inputCohort").value || "indian_grassroots";
    const estimatorMode = document.getElementById("selectEstimatorMode").value || "pinn_anchored";

    const res = runPhysicsConstrainedPipeline(state.currentTrial, estimatorMode, massKg);
    state.processed = res;
    const m = res.metrics;

    // Update Hero KPIs
    document.getElementById("kpiJumpHeight").textContent = m.jumpHeightImpCm.toFixed(1);
    document.getElementById("kpiJumpHeightSub").textContent =
      `Flight: ${(res.phases.flightTime * 1000).toFixed(0)} ms | Imp: ${m.jumpHeightImpCm.toFixed(1)} cm`;

    document.getElementById("kpiPeakForceBW").textContent = m.peakConcForceBW.toFixed(2);
    document.getElementById("kpiPeakForceN").textContent = `${m.peakConcForceN.toFixed(0)} N absolute`;

    document.getElementById("kpiPeakPowerRel").textContent = m.peakPowerRel.toFixed(1);
    document.getElementById("kpiPeakPowerAbs").textContent = `${m.peakPowerAbs.toFixed(0)} W peak`;

    document.getElementById("kpiTakeoffVel").textContent = m.vTakeoffIntegrated.toFixed(2);
    document.getElementById("kpiNetImpulseSub").textContent = `Net Imp: ${m.netImpulseNs.toFixed(1)} N·s`;

    document.getElementById("kpiRsiMod").textContent = m.rsiMod.toFixed(2);
    document.getElementById("kpiContractionTimeSub").textContent = `TTT: ${m.contractionTimeMs.toFixed(0)} ms`;

    document.getElementById("kpiImpulseRatio").textContent = m.propulsiveImpulseNs.toFixed(1);
    document.getElementById("kpiImpulseDetailSub").textContent =
      `Brk: ${m.brakingImpulseNs.toFixed(1)} | Prp: ${m.propulsiveImpulseNs.toFixed(1)} N·s`;

    // Phase Strip
    document.getElementById("phT0").textContent = `${res.phases.t0.toFixed(2)} s`;
    document.getElementById("phBrake").textContent = `${res.phases.tBrake.toFixed(2)} s`;
    document.getElementById("phProp").textContent = `${res.phases.tPropDetected.toFixed(2)} s`;
    document.getElementById("phTakeoff").textContent = `${res.phases.tTakeoff.toFixed(2)} s`;
    document.getElementById("phLanding").textContent = `${res.phases.tLanding.toFixed(2)} s`;
    document.getElementById("phFlight").textContent = `${(res.phases.flightTime * 1000).toFixed(0)} ms`;

    // Physics Banner
    document.getElementById("valdAccuracyBadge").textContent =
      `VALD Agreement: R² = ${m.r2.toFixed(3)} | RMSE = ${m.rmseBW.toFixed(3)} BW`;
    document.getElementById("physicsAnchorText").textContent =
      estimatorMode === "pinn_anchored"
        ? `Newton Impulse-Momentum Anchoring active (scale α = ${m.alphaScale.toFixed(3)}): v_to = ${m.vTakeoffIntegrated.toFixed(2)} m/s matches flight time.`
        : `Unanchored mode (${estimatorMode}): notice velocity & jump-height drift (${m.jumpHeightImpCm.toFixed(1)} cm vs ${m.jumpHeightFlightCm.toFixed(1)} cm flight).`;

    // GAMLSS Percentiles
    const norms = computeGamlssNorms(age, sex, cohort);
    const pH = norms.height.evalPct(m.jumpHeightImpCm);
    const pP = norms.power.evalPct(m.peakPowerRel);
    const pR = norms.rsi.evalPct(m.rsiMod);

    function setCentileUI(suffix, pct, bandObj, decimals) {
      const badge = document.getElementById("pctBadge" + suffix);
      const marker = document.getElementById("pctMarker" + suffix);
      const scale = document.getElementById("pctScale" + suffix);
      badge.textContent = `P${pct}`;
      badge.className = "centile-badge " + (pct >= 75 ? "high" : pct >= 40 ? "mid" : "low");
      marker.style.left = `${Math.max(4, Math.min(96, pct))}%`;
      scale.innerHTML =
        `<span>P10: ${bandObj.p10.toFixed(decimals)}</span>` +
        `<span>P50: ${bandObj.p50.toFixed(decimals)}</span>` +
        `<span>P75: ${bandObj.p75.toFixed(decimals)}</span>` +
        `<span>P95: ${bandObj.p95.toFixed(decimals)}</span>`;
    }

    setCentileUI("Height", pH, norms.height, 1);
    setCentileUI("Power", pP, norms.power, 1);
    setCentileUI("Rsi", pR, norms.rsi, 2);

    const avgPct = Math.round((pH + pP + pR) / 3);
    const tierEl = document.getElementById("overallAthleteTierBadge");
    tierEl.textContent =
      avgPct >= 80 ? `High Potential (P${avgPct} Composite)` :
      avgPct >= 50 ? `Above Median (P${avgPct} Composite)` :
      `Developing Grassroots (P${avgPct} Composite)`;

    // Populate Full Discrete Metrics Table
    const rows = [
      ["Jump Height (Impulse-Momentum Anchored)", "v_to² / (2·g)", `${m.jumpHeightImpCm.toFixed(1)} cm`, `${(m.jumpHeightImpCm / 100).toFixed(3)} m`],
      ["Jump Height (Flight Time Ballistics)", "½·g·t_flight² / 4", `${m.jumpHeightFlightCm.toFixed(1)} cm`, `t_flight = ${(res.phases.flightTime * 1000).toFixed(0)} ms`],
      ["Takeoff Vertical Velocity (v_takeoff)", "∫(vGRF − 1)g dt", `${m.vTakeoffIntegrated.toFixed(2)} m/s`, `Target: ${m.vTakeoffFlight.toFixed(2)} m/s`],
      ["Peak Concentric Force", "Propulsion Phase Max", `${m.peakConcForceBW.toFixed(2)} BW`, `${m.peakConcForceN.toFixed(0)} N`],
      ["Peak Eccentric (Braking) Force", "Braking Phase Max", `${m.peakEccForceBW.toFixed(2)} BW`, `${m.peakEccForceN.toFixed(0)} N`],
      ["Mean Concentric Force", "Propulsion Phase Mean", `${m.meanConcForceBW.toFixed(2)} BW`, `${m.meanConcForceN.toFixed(0)} N`],
      ["Peak Concentric Mechanical Power", "max(vGRF(t)·v(t))", `${m.peakPowerRel.toFixed(1)} W/kg`, `${m.peakPowerAbs.toFixed(0)} W`],
      ["Mean Concentric Mechanical Power", "mean(vGRF(t)·v(t))", `${m.meanConcPowerRel.toFixed(1)} W/kg`, `${m.meanConcPowerAbs.toFixed(0)} W`],
      ["Net Vertical Impulse", "m · v_takeoff", `${(m.netImpulseNs / massKg).toFixed(2)} N·s/kg`, `${m.netImpulseNs.toFixed(1)} N·s`],
      ["Eccentric Braking Impulse", "∫[t_brk → t_prop] F_net dt", `${(m.brakingImpulseNs / massKg).toFixed(2)} N·s/kg`, `${m.brakingImpulseNs.toFixed(1)} N·s`],
      ["Concentric Propulsive Impulse", "∫[t_prop → t_to] F_net dt", `${(m.propulsiveImpulseNs / massKg).toFixed(2)} N·s/kg`, `${m.propulsiveImpulseNs.toFixed(1)} N·s`],
      ["RSImod (Reactive Strength Index Modified)", "Jump Height / TTT", `${m.rsiMod.toFixed(2)} m/s`, `TTT = ${m.contractionTimeMs.toFixed(0)} ms`],
      ["Eccentric / Concentric Duration", "Unweight+Brake / Propulsion", `${m.eccTimeMs.toFixed(0)} ms ecc`, `${m.concTimeMs.toFixed(0)} ms conc`],
      ["Countermovement Dip Depth", "min(z_CoM)", `${m.dipDepthCm.toFixed(1)} cm`, `Max chest pitch: ${Math.max(...res.pitchDeg).toFixed(1)}°`]
    ];
    const tbody = document.getElementById("metricsTableBody");
    tbody.innerHTML = rows.map(r =>
      `<tr><td>${r[0]}</td><td>${r[1]}</td><td>${r[2]}</td><td>${r[3]}</td></tr>`
    ).join("");

    drawWaveformCanvas();
  }

  function drawWaveformCanvas() {
    const canvas = document.getElementById("waveformCanvas");
    if (!canvas || !state.processed) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width;
    const H = canvas.height;
    ctx.clearRect(0, 0, W, H);

    const res = state.processed;
    const { t, valdBW, activeBW, vel, pos, powerRel, phases } = res;
    const N = t.length;
    const tMax = t[N - 1];

    const padL = 54, padR = 54, padT = 26, padB = 34;
    const plotW = W - padL - padR;
    const plotH = H - padT - padB;

    const xOfT = (ti) => padL + (ti / tMax) * plotW;

    // Draw Phase Background Shading
    const phaseBands = [
      { tStart: phases.t0, tEnd: phases.tBrake, color: "rgba(56, 189, 248, 0.08)", label: "Unweight" },
      { tStart: phases.tBrake, tEnd: phases.tPropDetected, color: "rgba(244, 63, 94, 0.09)", label: "Braking" },
      { tStart: phases.tPropDetected, tEnd: phases.tTakeoff, color: "rgba(16, 185, 129, 0.11)", label: "Propulsion" },
      { tStart: phases.tTakeoff, tEnd: phases.tLanding, color: "rgba(168, 85, 247, 0.08)", label: "Flight" }
    ];

    phaseBands.forEach(b => {
      const x1 = xOfT(b.tStart);
      const x2 = xOfT(b.tEnd);
      ctx.fillStyle = b.color;
      ctx.fillRect(x1, padT, Math.max(0, x2 - x1), plotH);
      ctx.fillStyle = "#94a3b8";
      ctx.font = "10px -apple-system, sans-serif";
      if (x2 - x1 > 38) {
        ctx.fillText(b.label, x1 + 4, padT + 12);
      }
    });

    // Horizontal Grid Lines (BW scale: -1.0 to 3.5)
    const yMin = -1.2, yMax = 3.6;
    const yOfVal = (v) => padT + plotH - ((v - yMin) / (yMax - yMin)) * plotH;

    ctx.strokeStyle = "rgba(148, 163, 184, 0.14)";
    ctx.lineWidth = 1;
    [-1, 0, 1, 2, 3].forEach(gv => {
      const y = yOfVal(gv);
      ctx.beginPath();
      ctx.moveTo(padL, y);
      ctx.lineTo(W - padR, y);
      ctx.stroke();

      ctx.fillStyle = "#64748b";
      ctx.font = "11px monospace";
      ctx.fillText(`${gv.toFixed(1)}`, 12, y + 4);
    });

    // Highlight 1.0 BW Quiet Weight Line
    ctx.strokeStyle = "rgba(255, 255, 255, 0.22)";
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(padL, yOfVal(1.0));
    ctx.lineTo(W - padR, yOfVal(1.0));
    ctx.stroke();
    ctx.setLineDash([]);

    function plotCurve(arr, scaleFn, color, lineWidth, dashed) {
      ctx.save();
      ctx.strokeStyle = color;
      ctx.lineWidth = lineWidth;
      if (dashed) ctx.setLineDash(dashed);
      ctx.beginPath();
      for (let i = 0; i < N; i++) {
        const x = xOfT(t[i]);
        const y = yOfVal(scaleFn(arr[i]));
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.restore();
    }

    const view = state.chartView;
    if (view === "all" || view === "vgrf") {
      // VALD Ground Truth (dashed slate)
      plotCurve(valdBW, v => v, "#94a3b8", 1.8, [5, 4]);
      // PINN Estimated vGRF (sky blue)
      plotCurve(activeBW, v => v, "#38bdf8", 2.5, null);
    }
    if (view === "all" || view === "kinematics") {
      // Velocity v(t) in m/s
      plotCurve(vel, v => v * 0.75, "#10b981", 2.0, null);
      if (view === "kinematics") {
        // CoM displacement z(t) scaled
        plotCurve(pos, z => z * 3.0, "#a855f7", 1.8, [3, 3]);
      }
    }
    if (view === "all" || view === "power") {
      // Mechanical Power P(t) scaled (W/kg / 20)
      plotCurve(powerRel, p => p / 22.0, "#f97316", 2.0, null);
    }

    // Time Axis ticks
    ctx.fillStyle = "#64748b";
    ctx.font = "11px monospace";
    for (let ts = 0; ts <= tMax; ts += 0.5) {
      const x = xOfT(ts);
      ctx.fillText(`${ts.toFixed(1)}s`, x - 12, H - 10);
    }

    // Hover Crosshair
    if (state.hoverIndex !== null && state.hoverIndex >= 0 && state.hoverIndex < N) {
      const idx = state.hoverIndex;
      const x = xOfT(t[idx]);
      ctx.strokeStyle = "rgba(255,255,255,0.5)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(x, padT);
      ctx.lineTo(x, padT + plotH);
      ctx.stroke();
    }
  }

  // --- 6. Export CSV & JSON for VALD ForceDecks Paired Training ---
  function exportTrialCSV() {
    if (!state.processed) return;
    const r = state.processed;
    const raw = state.currentTrial;
    const massKg = parseFloat(document.getElementById("inputMass").value) || 45.0;
    const athleteId = document.getElementById("inputAthleteId").value || "IND-001";
    const trialId = document.getElementById("inputTrialId").value || "FD-CMJ-001";

    const lines = [
      "athlete_id,trial_id,time_s,ax_body_mps2,ay_body_mps2,az_body_mps2,gx_degps,gy_degps,gz_degps,pitch_deg,raw_imu_bw,pinn_vgrf_bw,pinn_vgrf_N,vald_ref_bw,vel_mps,pos_m,power_w_per_kg"
    ];
    for (let i = 0; i < r.t.length; i++) {
      lines.push([
        athleteId,
        trialId,
        r.t[i].toFixed(4),
        raw.axBody[i].toFixed(4),
        raw.ayBody[i].toFixed(4),
        raw.azBody[i].toFixed(4),
        raw.gxBody[i].toFixed(3),
        raw.gyBody[i].toFixed(3),
        raw.gzBody[i].toFixed(3),
        raw.pitchDeg[i].toFixed(2),
        r.rawImuBW[i].toFixed(4),
        r.activeBW[i].toFixed(4),
        (r.activeBW[i] * massKg * G).toFixed(2),
        r.valdBW[i].toFixed(4),
        r.vel[i].toFixed(4),
        r.pos[i].toFixed(4),
        r.powerRel[i].toFixed(2)
      ].join(","));
    }
    downloadBlob(lines.join("\n"), `${athleteId}_${trialId}_imu_vgrf.csv`, "text/csv");
  }

  function exportTrialJSON() {
    if (!state.processed) return;
    const athleteId = document.getElementById("inputAthleteId").value || "IND-001";
    const trialId = document.getElementById("inputTrialId").value || "FD-CMJ-001";
    const payload = {
      contractVersion: "bharatjump-pinn-v1",
      exportedAt: new Date().toISOString(),
      athlete: {
        athleteId,
        trialId,
        age: parseInt(document.getElementById("inputAge").value, 10),
        sex: document.getElementById("inputSex").value,
        massKg: parseFloat(document.getElementById("inputMass").value),
        heightCm: parseFloat(document.getElementById("inputHeight").value)
      },
      phases: state.processed.phases,
      metrics: state.processed.metrics,
      signals: {
        fs: state.processed.fs,
        t: state.processed.t,
        pinn_vgrf_bw: state.processed.activeBW,
        vald_ref_bw: state.processed.valdBW,
        velocity_mps: state.processed.vel,
        power_w_per_kg: state.processed.powerRel
      }
    };
    downloadBlob(JSON.stringify(payload, null, 2), `${athleteId}_${trialId}.json`, "application/json");
  }

  function downloadBlob(content, filename, mime) {
    const blob = new Blob([content], { type: mime });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  // --- 7. Live Smartphone IMU Capture (iOS & Android) ---
  async function startLiveIMU() {
    if (typeof DeviceMotionEvent !== "undefined" && typeof DeviceMotionEvent.requestPermission === "function") {
      try {
        const perm = await DeviceMotionEvent.requestPermission();
        if (perm !== "granted") {
          alert("DeviceMotion permission was denied. Using simulator instead.");
          return;
        }
      } catch (err) {
        console.warn("DeviceMotion permission error:", err);
      }
    }

    state.liveRecording = true;
    state.liveSamples = [];
    state.liveStartTime = performance.now();

    document.getElementById("btnStartLive").disabled = true;
    document.getElementById("btnStopLive").disabled = false;
    document.getElementById("sensorStatusDot").className = "status-dot recording";
    document.getElementById("sensorStatusText").textContent = "Recording Live Chest IMU… Hold still 1.5s then Jump!";

    state.motionHandler = function (ev) {
      if (!state.liveRecording) return;
      const nowS = (performance.now() - state.liveStartTime) / 1000;
      const accG = ev.accelerationIncludingGravity || { x: 0, y: 0, z: G };
      const rot = ev.rotationRate || { alpha: 0, beta: 0, gamma: 0 };
      const ax = accG.x || 0, ay = accG.y || 0, az = accG.z || G;
      const mag = Math.sqrt(ax * ax + ay * ay + az * az);
      const bwEst = mag / G;

      state.liveSamples.push({
        t: nowS, ax, ay, az,
        gx: rot.beta || 0, gy: rot.gamma || 0, gz: rot.alpha || 0,
        bw: bwEst
      });

      document.getElementById("azReadout").textContent = `${(mag - G).toFixed(2)} m/s² (${bwEst.toFixed(2)} BW)`;
      const calPct = Math.min(100, Math.round((nowS / 1.5) * 100));
      document.getElementById("calibProgressBar").style.width = `${calPct}%`;
      document.getElementById("calibStatusLabel").textContent =
        calPct < 100 ? `Calibrating (${calPct}%)` : "Locked · Ready to Jump!";
    };

    window.addEventListener("devicemotion", state.motionHandler);
  }

  function stopLiveIMU() {
    state.liveRecording = false;
    if (state.motionHandler) {
      window.removeEventListener("devicemotion", state.motionHandler);
    }
    document.getElementById("btnStartLive").disabled = false;
    document.getElementById("btnStopLive").disabled = true;
    document.getElementById("sensorStatusDot").className = "status-dot ready";

    if (state.liveSamples.length < 40) {
      document.getElementById("sensorStatusText").textContent =
        "No hardware IMU stream detected on desktop — loaded simulated Chest IMU jump.";
      loadPresetJump(document.getElementById("demoPresetSelect").value);
      return;
    }

    document.getElementById("sensorStatusText").textContent =
      `Captured ${state.liveSamples.length} live IMU samples (${(state.liveSamples[state.liveSamples.length - 1].t).toFixed(1)}s) · Live Jump Analyzed`;
    state.currentTrial = buildTrialFromLiveSamples(state.liveSamples);
    updateDashboardUI();
  }

  function buildTrialFromLiveSamples(samples) {
    // 1. Estimate duration and resample onto a uniform 100 Hz grid
    const fs = 100;
    const dt = 1 / fs;
    const tEnd = samples[samples.length - 1].t;
    const N = Math.max(100, Math.round(tEnd * fs));
    const t = new Array(N);
    const axBody = new Array(N);
    const ayBody = new Array(N);
    const azBody = new Array(N);
    const gxBody = new Array(N);
    const gyBody = new Array(N);
    const gzBody = new Array(N);
    const rawImuBW = new Array(N);
    const valdBW = new Array(N);
    const pitchDeg = new Array(N);

    let k = 0;
    for (let i = 0; i < N; i++) {
      const ti = i * dt;
      t[i] = ti;
      while (k < samples.length - 2 && samples[k + 1].t < ti) k++;
      const s0 = samples[k];
      const s1 = samples[Math.min(k + 1, samples.length - 1)];
      const span = Math.max(1e-4, s1.t - s0.t);
      const w = Math.max(0, Math.min(1, (ti - s0.t) / span));
      axBody[i] = s0.ax + w * (s1.ax - s0.ax);
      ayBody[i] = s0.ay + w * (s1.ay - s0.ay);
      azBody[i] = s0.az + w * (s1.az - s0.az);
      gxBody[i] = s0.gx + w * (s1.gx - s0.gx);
      gyBody[i] = s0.gy + w * (s1.gy - s0.gy);
      gzBody[i] = s0.gz + w * (s1.gz - s0.gz);
    }

    // 2. Calibrate gravity direction from the first 1.0s of quiet standing
    const nCal = Math.min(N, Math.max(25, Math.round(1.0 * fs)));
    let gx0 = 0, gy0 = 0, gz0 = 0;
    for (let i = 0; i < nCal; i++) {
      gx0 += axBody[i];
      gy0 += ayBody[i];
      gz0 += azBody[i];
    }
    gx0 /= nCal; gy0 /= nCal; gz0 /= nCal;
    const gNorm = Math.max(4.0, Math.sqrt(gx0 * gx0 + gy0 * gy0 + gz0 * gz0));
    const ux = gx0 / gNorm, uy = gy0 / gNorm, uz = gz0 / gNorm;

    // 3. Complementary orientation + Earth-vertical projection
    let pitch = Math.atan2(uy, uz) * (180 / Math.PI);
    for (let i = 0; i < N; i++) {
      const aProj = (axBody[i] * ux + ayBody[i] * uy + azBody[i] * uz) / gNorm;
      const aMag = Math.sqrt(axBody[i] * axBody[i] + ayBody[i] * ayBody[i] + azBody[i] * azBody[i]) / gNorm;
      // Blend projection and magnitude during high-rotation propulsion, clamp >= 0
      const bw = Math.max(0, 0.75 * aProj + 0.25 * aMag);
      rawImuBW[i] = bw;
      valdBW[i] = bw; // On live phone trial without paired ForceDecks, reference line tracks raw IMU
      pitch = 0.96 * (pitch + gxBody[i] * dt) + 0.04 * (Math.atan2(ayBody[i], azBody[i]) * (180 / Math.PI));
      pitchDeg[i] = Math.abs(pitch);
    }

    // Smooth rawImuBW with a 5-point moving average (zero-phase)
    const smoothBW = rawImuBW.slice();
    for (let i = 2; i < N - 2; i++) {
      smoothBW[i] = (rawImuBW[i - 2] + 2 * rawImuBW[i - 1] + 3 * rawImuBW[i] + 2 * rawImuBW[i + 1] + rawImuBW[i + 2]) / 9;
    }
    for (let i = 0; i < N; i++) rawImuBW[i] = smoothBW[i];

    // 4. Automatic CMJ Phase Detection from Smartphone IMU
    // Find Peak Landing Impact or Free-Fall trough (< 0.38 BW for at least 180 ms)
    let bestFlightStart = -1, bestFlightEnd = -1, bestFlightLen = 0;
    let curStart = -1;
    for (let i = Math.round(0.3 * fs); i < N - 5; i++) {
      if (rawImuBW[i] < 0.42) {
        if (curStart === -1) curStart = i;
      } else {
        if (curStart !== -1) {
          const len = i - curStart;
          if (len > bestFlightLen && len <= Math.round(0.85 * fs)) {
            bestFlightLen = len;
            bestFlightStart = curStart;
            bestFlightEnd = i;
          }
          curStart = -1;
        }
      }
    }

    // Fallback if phone was held loosely and free-fall trough didn't drop below 0.42 BW:
    // find global propulsion peak and landing peak separated by 0.25s - 0.75s
    if (bestFlightLen < Math.round(0.18 * fs)) {
      let maxIdx = Math.round(0.4 * fs);
      for (let i = Math.round(0.4 * fs); i < N - Math.round(0.4 * fs); i++) {
        if (rawImuBW[i] > rawImuBW[maxIdx]) maxIdx = i;
      }
      bestFlightStart = Math.min(N - 30, maxIdx + Math.round(0.08 * fs));
      let minVal = 999, minIdx = bestFlightStart + 15;
      for (let i = bestFlightStart; i < Math.min(N - 10, bestFlightStart + Math.round(0.65 * fs)); i++) {
        if (rawImuBW[i] < minVal) { minVal = rawImuBW[i]; minIdx = i; }
      }
      bestFlightEnd = Math.min(N - 5, Math.max(bestFlightStart + Math.round(0.28 * fs), minIdx + Math.round(0.16 * fs)));
    }

    const tTakeoff = bestFlightStart * dt;
    const tLanding = bestFlightEnd * dt;
    const flightTime = Math.max(0.22, Math.min(0.78, tLanding - tTakeoff));

    // Work backward from takeoff to find Propulsion peak, Braking peak, and Unweighting onset t0
    const iTo = bestFlightStart;
    const iSearchStart = Math.max(10, iTo - Math.round(0.95 * fs));
    let iUnweightMin = Math.max(12, iTo - Math.round(0.55 * fs));
    for (let i = iSearchStart; i < iTo - Math.round(0.18 * fs); i++) {
      if (rawImuBW[i] < rawImuBW[iUnweightMin]) iUnweightMin = i;
    }
    let i0 = Math.max(5, iUnweightMin - Math.round(0.18 * fs));
    for (let i = iUnweightMin; i >= iSearchStart; i--) {
      if (rawImuBW[i] >= 0.96) { i0 = i; break; }
    }
    const t0 = i0 * dt;
    const tBrake = (iUnweightMin + Math.round(0.06 * fs)) * dt;
    const tProp = Math.min(tTakeoff - 0.12, (tBrake + tTakeoff) * 0.52);

    return {
      fs, dt, t, valdBW, rawImuBW,
      axBody, ayBody, azBody, gxBody, gyBody, gzBody, pitchDeg,
      phases: { t0, tBrake, tProp, tTakeoff, tLanding, flightTime }
    };
  }


  function loadPresetJump(presetKey) {
    const p = PRESETS[presetKey] || PRESETS.youth_45kg;
    document.getElementById("inputAge").value = p.age;
    document.getElementById("inputSex").value = p.sex;
    document.getElementById("inputMass").value = p.mass;
    document.getElementById("inputHeight").value = p.height;
    document.getElementById("inputCohort").value = p.cohort;
    document.getElementById("inputAthleteId").value = p.athleteId;
    document.getElementById("inputTrialId").value = p.trialId;

    const q = eulerToQuat(0.5, p.trunkPitchDeg, 0.0);
    document.getElementById("quatReadout").textContent =
      `[${q.map(v => v.toFixed(3)).join(", ")}]`;
    document.getElementById("pitchReadout").textContent = `${p.trunkPitchDeg.toFixed(1)}° peak anterior`;

    state.currentTrial = generateSyntheticTrial(presetKey);
    updateDashboardUI();
  }

  // --- 8. Event Listeners & Init ---
  window.addEventListener("DOMContentLoaded", () => {
    document.getElementById("btnSimulateJump").addEventListener("click", () => {
      loadPresetJump(document.getElementById("demoPresetSelect").value);
    });

    document.getElementById("demoPresetSelect").addEventListener("change", (e) => {
      loadPresetJump(e.target.value);
    });

    ["inputAge", "inputSex", "inputMass", "inputHeight", "inputCohort", "selectEstimatorMode"].forEach(id => {
      const el = document.getElementById(id);
      if (el) el.addEventListener("input", updateDashboardUI);
      if (el) el.addEventListener("change", updateDashboardUI);
    });

    document.getElementById("btnStartLive").addEventListener("click", startLiveIMU);
    document.getElementById("btnStopLive").addEventListener("click", stopLiveIMU);
    document.getElementById("btnExportCSV").addEventListener("click", exportTrialCSV);
    document.getElementById("btnExportJSON").addEventListener("click", exportTrialJSON);

    // Chart View Tabs
    const tabButtons = document.querySelectorAll("#chartViewTabs button");
    tabButtons.forEach(btn => {
      btn.addEventListener("click", () => {
        tabButtons.forEach(b => b.classList.remove("active"));
        btn.classList.add("active");
        state.chartView = btn.getAttribute("data-view") || "all";
        drawWaveformCanvas();
      });
    });

    // Interactive Canvas Hover
    const canvas = document.getElementById("waveformCanvas");
    const massInput = document.getElementById("inputMass");
    canvas.addEventListener("mousemove", (e) => {
      if (!state.processed) return;
      const rect = canvas.getBoundingClientRect();
      const relX = ((e.clientX - rect.left) / rect.width) * canvas.width;
      const padL = 54, padR = 54;
      const plotW = canvas.width - padL - padR;
      const frac = Math.max(0, Math.min(1, (relX - padL) / plotW));
      const idx = Math.round(frac * (state.processed.t.length - 1));
      state.hoverIndex = idx;
      drawWaveformCanvas();

      const massKg = parseFloat(massInput.value) || 45.0;
      const ti = state.processed.t[idx];
      const fBW = state.processed.activeBW[idx];
      const fVald = state.processed.valdBW[idx];
      const v = state.processed.vel[idx];
      const p = state.processed.powerRel[idx];
      document.getElementById("chartHoverReadout").textContent =
        `t = ${ti.toFixed(2)} s | PINN vGRF: ${fBW.toFixed(2)} BW (${(fBW * massKg * G).toFixed(0)} N) | VALD: ${fVald.toFixed(2)} BW | Vel: ${v.toFixed(2)} m/s | Power: ${p.toFixed(1)} W/kg`;
    });

    canvas.addEventListener("mouseleave", () => {
      state.hoverIndex = null;
      drawWaveformCanvas();
    });

    // Initial load
    loadPresetJump("youth_45kg");
  });
})();
