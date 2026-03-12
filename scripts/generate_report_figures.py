#!/usr/bin/env python3
"""
Generate trajectory_evaluation.png - 4-panel figure for AAE5303 Assignment 2
Matches teacher's example format closely.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ── 1. Load TUM format files ────────────────────────────────────────────────
def load_tum(filepath):
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            vals = list(map(float, line.split()))
            data.append(vals)
    return np.array(data)  # [t, x, y, z, qx, qy, qz, qw]

gt  = load_tum('/root/ORB_SLAM3/ground_truth.txt')
est = load_tum('/root/ORB_SLAM3/KeyFrameTrajectory.txt')

# ── 2. Timestamp association (t_max_diff = 0.1 s) ──────────────────────────
def associate(gt, est, max_diff=0.1):
    gt_idx, est_idx = [], []
    for i, t_est in enumerate(est[:, 0]):
        diffs = np.abs(gt[:, 0] - t_est)
        j = np.argmin(diffs)
        if diffs[j] <= max_diff:
            gt_idx.append(j)
            est_idx.append(i)
    return np.array(gt_idx), np.array(est_idx)

gi, ei = associate(gt, est)
gt_matched  = gt[gi,  1:4]
est_matched = est[ei, 1:4]

# ── 3. Sim(3) Umeyama alignment ─────────────────────────────────────────────
def umeyama_sim3(src, dst):
    n = src.shape[0]
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst
    var_src = np.mean(np.sum(src_c**2, axis=1))
    W = (dst_c.T @ src_c) / n
    U, D, Vt = np.linalg.svd(W)
    S = np.diag([1, 1, np.linalg.det(U @ Vt)])
    R = U @ S @ Vt
    scale = float(np.sum(D * np.diag(S)) / var_src)
    t = mu_dst - scale * R @ mu_src
    return scale, R, t

scale, R, t = umeyama_sim3(est_matched, gt_matched)
est_aligned = (scale * (R @ est_matched.T)).T + t

# ── 4. ATE errors ─────────────────────────────────────────────────────────
ate_errors = np.linalg.norm(est_aligned - gt_matched, axis=1)
ate_rmse   = float(np.sqrt(np.mean(ate_errors**2)))
ate_mean   = float(np.mean(ate_errors))
ate_median = float(np.median(ate_errors))

print(f"ATE RMSE   = {ate_rmse:.6f} m")
print(f"ATE Mean   = {ate_mean:.6f} m")
print(f"ATE Median = {ate_median:.6f} m")
print(f"Matched poses: {len(ate_errors)}")

# ── 5. Raw estimated trajectory - NO translation, keep original coords ──────
# This reveals the true scale mismatch (VO is in arbitrary scale/frame)
est_raw = est_matched  # raw ORB-SLAM3 coordinates as-is

# ── 6. Plot ─────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 10), facecolor='white')
fig.suptitle(f'Trajectory Comparison',
             fontsize=15, fontweight='bold', x=0.05, ha='left', y=0.98)

gs = gridspec.GridSpec(2, 2, hspace=0.35, wspace=0.30,
                       left=0.08, right=0.96, top=0.93, bottom=0.07)

# ── Panel 1 (top-left): BEFORE alignment ──────────────────────────────────
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(gt_matched[:, 0], gt_matched[:, 1], 'g-', lw=1.5,
         label='Ground Truth', zorder=2)
ax1.plot(est_raw[:, 0], est_raw[:, 1], 'r--', lw=1.2,
         label='VO (Unaligned)', zorder=3)
ax1.set_title(f'2D Trajectory - Before Alignment ( HKisland_GNSS03)', fontsize=10)
ax1.set_xlabel('X [m]'); ax1.set_ylabel('Y [m]')
ax1.legend(fontsize=9, loc='upper right')
ax1.grid(True, alpha=0.3)
ax1.set_aspect('equal', 'datalim')

# ── Panel 2 (top-right): AFTER Sim(3) alignment ────────────────────────────
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(gt_matched[:, 0], gt_matched[:, 1], 'g-', lw=1.5,
         label='Ground Truth', zorder=2)
ax2.plot(est_aligned[:, 0], est_aligned[:, 1], 'b-', lw=1.2,
         label='VO (Aligned)', zorder=3)
ax2.set_title(f'2D Trajectory - After Sim(3) Alignment ( HKisland_GNSS03)', fontsize=10)
ax2.set_xlabel('X [m]'); ax2.set_ylabel('Y [m]')
ax2.legend(fontsize=9, loc='upper right')
ax2.grid(True, alpha=0.3)
ax2.set_aspect('equal', 'datalim')

# ── Panel 3 (bottom-left): ATE histogram ──────────────────────────────────
ax3 = fig.add_subplot(gs[1, 0])
ax3.hist(ate_errors, bins=40, color='steelblue', edgecolor='white', alpha=0.85)
ax3.axvline(ate_mean,   color='red',    lw=1.8, linestyle='--',
            label=f'Mean: {ate_mean:.2f} m')
ax3.axvline(ate_median, color='orange', lw=1.8, linestyle='--',
            label=f'Median: {ate_median:.2f} m')
ax3.set_title('Absolute Trajectory Error Distribution', fontsize=10)
ax3.set_xlabel('ATE [m]')
ax3.set_ylabel('Frequency')
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3, axis='y')

# ── Panel 4 (bottom-right): ATE along trajectory ──────────────────────────
ax4 = fig.add_subplot(gs[1, 1])
pose_idx = np.arange(len(ate_errors))
ax4.fill_between(pose_idx, ate_errors, alpha=0.25, color='teal')
ax4.plot(pose_idx, ate_errors, color='steelblue', lw=1.2, zorder=3)
ax4.set_title('ATE Error Along Trajectory', fontsize=10)
ax4.set_xlabel('Matched Pose Index')
ax4.set_ylabel('ATE [m]')
ax4.grid(True, alpha=0.3)

# ── Save ─────────────────────────────────────────────────────────────────────
out = '/root/ORB_SLAM3/trajectory_evaluation.png'
fig.savefig(out, dpi=150, bbox_inches='tight', facecolor='white')
print(f'\nSaved: {out}')
