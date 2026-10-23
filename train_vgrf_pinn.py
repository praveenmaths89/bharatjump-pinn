#!/usr/bin/env python3
r"""
BharatJump PINN — Physics-Informed 1D-ResNet + BiGRU for Smartphone IMU -> vGRF(t)
===================================================================================
Trains a compact neural model on paired Chest Smartphone 6-axis IMU + VALD ForceDecks
ground-reaction force waveforms, enforcing Newton's Impulse-Momentum theorem:

    \int_{t_0}^{t_takeoff} (vGRF(t) - 1) * g dt = v_takeoff = 0.5 * g * t_flight

Usage:
    python3 train_vgrf_pinn.py --demo
r"""

import argparse
import json
import math
from pathlib import Path
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

G = 9.80665


class ResBlock1D(nn.Module):
    def __init__(self, channels: int, kernel_size: int = 5, dilation: int = 1):
        super().__init__()
        pad = (kernel_size - 1) * dilation // 2
        self.conv1 = nn.Conv1d(channels, channels, kernel_size, padding=pad, dilation=dilation)
        self.bn1 = nn.BatchNorm1d(channels)
        self.conv2 = nn.Conv1d(channels, channels, kernel_size, padding=pad, dilation=dilation)
        self.bn2 = nn.BatchNorm1d(channels)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        residual = x
        out = F.gelu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        return F.gelu(out + residual)


class ImpulseAnchoringLayer(nn.Module):
    """
    Differentiable Newton Impulse-Momentum Projection Layer:
    Scales the positive net-force region during contact [t0, t_takeoff] so that
    \int_{t0}^{t_takeoff} (F(t) - 1)*g dt == v_takeoff = 0.5 * g * t_flight,
    and clamps free-fall flight phase [t_takeoff, t_landing] to 0 BW.
    """

    def __init__(self, dt: float = 0.01):
        super().__init__()
        self.dt = dt

    def forward(
        self,
        f_raw_bw: torch.Tensor,
        contact_mask: torch.Tensor,
        flight_mask: torch.Tensor,
        v_takeoff_target: torch.Tensor,
    ) -> torch.Tensor:
        # Zero force during flight
        f = f_raw_bw * (1.0 - flight_mask)

        net = (f - 1.0) * contact_mask
        pos_net = F.relu(net)
        neg_net = F.relu(-net)

        pos_imp = torch.sum(pos_net * G * self.dt, dim=-1, keepdim=True)
        neg_imp = torch.sum(neg_net * G * self.dt, dim=-1, keepdim=True)

        alpha = (v_takeoff_target.unsqueeze(-1) + neg_imp) / torch.clamp(pos_imp, min=1e-4)
        f_anchored = torch.where(
            (contact_mask > 0.5) & (f > 1.0),
            1.0 + (f - 1.0) * alpha,
            f,
        )
        return F.relu(f_anchored)


class CMJForcePINN(nn.Module):
    """
    Compact 1D-ResNet + BiGRU + Differentiable Impulse-Anchoring Model
    Inputs:
      - x_imu: (B, T, 6) -> [a_earth_x, a_earth_y, a_earth_z, gx, gy, gz]
      - meta:  (B, 2)    -> [mass_kg / 100.0, height_cm / 200.0]
    Outputs:
      - f_anchored_bw: (B, T) continuous vGRF in Body Weights (BW)
      - f_unanchored_bw: (B, T) pre-projection neural prediction
    """

    def __init__(self, in_channels: int = 6, hidden: int = 48, dt: float = 0.01):
        super().__init__()
        self.stem = nn.Conv1d(in_channels + 2, hidden, kernel_size=5, padding=2)
        self.res1 = ResBlock1D(hidden, kernel_size=5, dilation=1)
        self.res2 = ResBlock1D(hidden, kernel_size=5, dilation=2)
        self.gru = nn.GRU(hidden, hidden, num_layers=1, batch_first=True, bidirectional=True)
        self.head = nn.Sequential(
            nn.Linear(hidden * 2, hidden),
            nn.GELU(),
            nn.Linear(hidden, 1),
        )
        self.anchor = ImpulseAnchoringLayer(dt=dt)

    def forward(
        self,
        x_imu: torch.Tensor,
        meta: torch.Tensor,
        contact_mask: torch.Tensor,
        flight_mask: torch.Tensor,
        v_takeoff_target: torch.Tensor,
    ):
        B, T, _ = x_imu.shape
        meta_seq = meta.unsqueeze(1).expand(B, T, 2)
        x = torch.cat([x_imu, meta_seq], dim=-1).transpose(1, 2)  # (B, C, T)
        h = F.gelu(self.stem(x))
        h = self.res2(self.res1(h)).transpose(1, 2)  # (B, T, H)
        h_gru, _ = self.gru(h)
        f_unanchored = F.softplus(self.head(h_gru).squeeze(-1))
        f_anchored = self.anchor(f_unanchored, contact_mask, flight_mask, v_takeoff_target)
        return f_anchored, f_unanchored


class PINNLoss(nn.Module):
    def __init__(self, dt: float = 0.01, w_imp: float = 0.5, w_peak: float = 0.25, w_smooth: float = 0.05):
        super().__init__()
        self.dt = dt
        self.w_imp = w_imp
        self.w_peak = w_peak
        self.w_smooth = w_smooth

    def forward(
        self,
        f_pred: torch.Tensor,
        f_unanchored: torch.Tensor,
        f_vald: torch.Tensor,
        contact_mask: torch.Tensor,
        v_takeoff_target: torch.Tensor,
    ):
        l_huber = F.huber_loss(f_pred, f_vald, delta=0.1)

        # Impulse-momentum loss on the unanchored head so the backbone learns physical impulse directly
        v_est = torch.sum((f_unanchored - 1.0) * contact_mask * G * self.dt, dim=-1)
        l_imp = F.mse_loss(v_est, v_takeoff_target)

        # Concentric peak force preservation loss
        peak_pred = torch.max(f_pred * contact_mask, dim=-1).values
        peak_true = torch.max(f_vald * contact_mask, dim=-1).values
        l_peak = F.mse_loss(peak_pred, peak_true)

        # Second-difference smoothness regularization
        d2_pred = f_pred[:, 2:] - 2 * f_pred[:, 1:-1] + f_pred[:, :-2]
        d2_true = f_vald[:, 2:] - 2 * f_vald[:, 1:-1] + f_vald[:, :-2]
        l_smooth = F.mse_loss(d2_pred, d2_true)

        total = l_huber + self.w_imp * l_imp + self.w_peak * l_peak + self.w_smooth * l_smooth
        return total, {
            "huber": float(l_huber.item()),
            "impulse": float(l_imp.item()),
            "peak": float(l_peak.item()),
            "smooth": float(l_smooth.item()),
        }


def make_synthetic_batch(batch_size: int = 24, seq_len: int = 250, dt: float = 0.01):
    t = np.arange(seq_len) * dt
    x_imu = np.zeros((batch_size, seq_len, 6), dtype=np.float32)
    meta = np.zeros((batch_size, 2), dtype=np.float32)
    f_vald = np.ones((batch_size, seq_len), dtype=np.float32)
    contact_mask = np.zeros((batch_size, seq_len), dtype=np.float32)
    flight_mask = np.zeros((batch_size, seq_len), dtype=np.float32)
    v_to = np.zeros((batch_size,), dtype=np.float32)

    rng = np.random.default_rng(42)
    for b in range(batch_size):
        mass = rng.uniform(30.0, 82.0)
        height = rng.uniform(135.0, 185.0)
        meta[b] = [mass / 100.0, height / 200.0]

        t0 = 0.50
        t_prop = 0.90
        t_to = 1.18
        t_flight = rng.uniform(0.44, 0.62)
        t_land = t_to + t_flight
        v_target = 0.5 * G * t_flight
        v_to[b] = v_target

        curve = np.ones(seq_len, dtype=np.float32)
        for i, ti in enumerate(t):
            if t0 <= ti < t_to:
                contact_mask[b, i] = 1.0
                u = (ti - t0) / (t_to - t0)
                curve[i] = 1.0 - 0.55 * math.sin(math.pi * min(u / 0.35, 1.0)) if u < 0.35 else 1.0 + 1.55 * math.sin(math.pi * (u - 0.35) / 0.65)
            elif t_to <= ti < t_land:
                flight_mask[b, i] = 1.0
                curve[i] = 0.0
            elif ti >= t_land:
                u = ti - t_land
                curve[i] = max(0.0, 1.0 + 1.8 * math.exp(-u / 0.08) * math.sin(math.pi * min(u / 0.15, 1.0)))

        # Enforce exact impulse on ground truth
        pos = np.maximum(0.0, (curve - 1.0) * contact_mask[b])
        neg = np.maximum(0.0, (1.0 - curve) * contact_mask[b])
        alpha = (v_target + np.sum(neg) * G * dt) / max(np.sum(pos) * G * dt, 1e-4)
        for i in range(seq_len):
            if contact_mask[b, i] > 0.5 and curve[i] > 1.0:
                curve[i] = 1.0 + (curve[i] - 1.0) * alpha
        f_vald[b] = curve

        # Chest IMU channels with trunk artifact
        u_contact = np.clip((t - t0) / (t_to - t0), 0.0, 1.0)
        artifact = (0.14 * np.sin(2 * np.pi * u_contact) - 0.08 * np.cos(3 * np.pi * u_contact)) * contact_mask[b]
        az_chest = (curve + artifact) * (1.0 - flight_mask[b])
        pitch_rad = 0.24 * np.sin(np.pi * u_contact) * contact_mask[b]

        x_imu[b, :, 0] = 0.05 * rng.standard_normal(seq_len)
        x_imu[b, :, 1] = az_chest * np.sin(pitch_rad)
        x_imu[b, :, 2] = az_chest * np.cos(pitch_rad)
        x_imu[b, :, 3] = np.gradient(pitch_rad, dt)
        x_imu[b, :, 4] = 0.03 * rng.standard_normal(seq_len)
        x_imu[b, :, 5] = 0.03 * rng.standard_normal(seq_len)

    return (
        torch.tensor(x_imu),
        torch.tensor(meta),
        torch.tensor(f_vald),
        torch.tensor(contact_mask),
        torch.tensor(flight_mask),
        torch.tensor(v_to),
    )


def run_demo():
    torch.manual_seed(42)
    x_imu, meta, f_vald, contact_mask, flight_mask, v_to = make_synthetic_batch(batch_size=32)

    model = CMJForcePINN(in_channels=6, hidden=32, dt=0.01)
    criterion = PINNLoss(dt=0.01)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-3, weight_decay=1e-4)

    print("Training BharatJump CMJForcePINN (1D-ResNet + BiGRU + Newton Impulse-Anchoring)...")
    for epoch in range(1, 41):
        optimizer.zero_grad()
        f_pred, f_unanchored = model(x_imu, meta, contact_mask, flight_mask, v_to)
        loss, parts = criterion(f_pred, f_unanchored, f_vald, contact_mask, v_to)
        loss.backward()
        optimizer.step()
        if epoch in (1, 10, 20, 30, 40):
            with torch.no_grad():
                rmse = torch.sqrt(torch.mean(((f_pred - f_vald) * contact_mask) ** 2)).item()
                v_integrated = torch.sum((f_pred - 1.0) * contact_mask * G * 0.01, dim=-1)
                imp_err = torch.max(torch.abs(v_integrated - v_to)).item()
            print(
                f"  Epoch {epoch:02d}/40 | Total Loss: {loss.item():.4f} | "
                f"Contact vGRF RMSE: {rmse:.4f} BW | Max Impulse Velocity Drift: {imp_err:.6f} m/s"
            )

    out_dir = Path("/workspace/jump-app")
    ckpt_path = out_dir / "cmj_pinn_demo_weights.pt"
    torch.save(model.state_dict(), ckpt_path)
    print(f"Saved PyTorch checkpoint -> {ckpt_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--demo", action="store_true", help="Run synthetic paired IMU+VALD training verification")
    args = parser.parse_args()
    run_demo()
