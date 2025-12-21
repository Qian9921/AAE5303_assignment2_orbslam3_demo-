#!/usr/bin/env python3
"""
AAE5303 Visual Odometry Accuracy Evaluation Script

Evaluates VO trajectory using 3 academic metrics:
1. ATE (Absolute Trajectory Error)
2. RPE (Relative Pose Error)
3. Scale Error

Usage:
    python3 evaluate_vo_accuracy.py \
        --groundtruth rtk_groundtruth.txt \
        --estimated KeyFrameTrajectory.txt \
        --output-dir evaluation_results
"""

import numpy as np
import argparse
import json
from datetime import date
import sys


def load_trajectory_tum(filename):
    """
    Load trajectory from TUM format file.
    Format: timestamp x y z qx qy qz qw
    """
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    orientations = data[:, 4:8]
    return timestamps, positions, orientations


def associate_trajectories(t_gt, t_est, max_time_diff=0.02):
    """Associate ground truth and estimated trajectories by timestamp."""
    matches = []
    for i, t_e in enumerate(t_est):
        time_diffs = np.abs(t_gt - t_e)
        min_idx = np.argmin(time_diffs)
        if time_diffs[min_idx] < max_time_diff:
            matches.append((min_idx, i))
    return matches


def align_trajectories_sim3(P_gt, P_est):
    """Align estimated trajectory to ground truth using Sim(3)."""
    centroid_gt = np.mean(P_gt, axis=0)
    centroid_est = np.mean(P_est, axis=0)
    
    P_gt_centered = P_gt - centroid_gt
    P_est_centered = P_est - centroid_est
    
    scale = np.sqrt(np.sum(P_gt_centered ** 2) / np.sum(P_est_centered ** 2))
    P_est_scaled = P_est_centered * scale
    
    H = P_est_scaled.T @ P_gt_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    t = centroid_gt - scale * (R @ centroid_est)
    P_aligned = scale * (P_est @ R.T) + t
    
    return P_aligned, scale, R, t


def compute_ate(P_gt, P_aligned):
    """Compute Absolute Trajectory Error (ATE)."""
    errors = np.linalg.norm(P_gt - P_aligned, axis=1)
    
    return {
        'rmse': np.sqrt(np.mean(errors ** 2)),
        'mean': np.mean(errors),
        'median': np.median(errors),
        'std': np.std(errors),
        'min': np.min(errors),
        'max': np.max(errors)
    }


def compute_rpe(P_gt, P_est, delta=10):
    """Compute Relative Pose Error (RPE)."""
    trans_errors = []
    
    for i in range(len(P_gt) - delta):
        gt_rel = P_gt[i + delta] - P_gt[i]
        est_rel = P_est[i + delta] - P_est[i]
        error = np.linalg.norm(gt_rel - est_rel)
        trans_errors.append(error)
    
    trans_errors = np.array(trans_errors)
    
    return {
        'trans_rmse': np.sqrt(np.mean(trans_errors ** 2)),
        'trans_mean': np.mean(trans_errors),
        'trans_median': np.median(trans_errors),
        'trans_std': np.std(trans_errors)
    }


def compute_scale_error(P_gt, P_est_unaligned):
    """Compute Scale Error."""
    gt_segments = np.diff(P_gt, axis=0)
    est_segments = np.diff(P_est_unaligned, axis=0)
    
    gt_length = np.sum(np.linalg.norm(gt_segments, axis=1))
    est_length = np.sum(np.linalg.norm(est_segments, axis=1))
    
    scale_ratio = est_length / gt_length
    scale_drift_percent = abs(1.0 - scale_ratio) * 100.0
    
    return {
        'scale_ratio': scale_ratio,
        'scale_drift_percent': scale_drift_percent
    }


def main():
    parser = argparse.ArgumentParser(
        description='Evaluate VO accuracy using ATE, RPE, and Scale Error'
    )
    parser.add_argument('--groundtruth', required=True)
    parser.add_argument('--estimated', required=True)
    parser.add_argument('--max-time-diff', type=float, default=0.02)
    parser.add_argument('--rpe-delta', type=int, default=10)
    parser.add_argument('--output-dir', default='.')
    
    args = parser.parse_args()
    
    print("="*60)
    print("VISUAL ODOMETRY TRAJECTORY EVALUATION")
    print("="*60)
    
    # Load trajectories
    t_gt, P_gt, _ = load_trajectory_tum(args.groundtruth)
    t_est, P_est, _ = load_trajectory_tum(args.estimated)
    
    print(f"Ground truth: {len(P_gt)} poses")
    print(f"Estimated:    {len(P_est)} poses")
    
    # Associate
    matches = associate_trajectories(t_gt, t_est, args.max_time_diff)
    print(f"Matched: {len(matches)} / {len(P_est)} poses ({100*len(matches)/len(P_est):.1f}%)")
    
    if len(matches) < 10:
        print("ERROR: Too few matched poses!")
        return 1
    
    gt_indices = [m[0] for m in matches]
    est_indices = [m[1] for m in matches]
    P_gt_matched = P_gt[gt_indices]
    P_est_matched = P_est[est_indices]
    P_est_unaligned = P_est_matched.copy()
    
    # Align
    P_aligned, scale, R, t = align_trajectories_sim3(P_gt_matched, P_est_matched)
    
    # Compute metrics
    ate = compute_ate(P_gt_matched, P_aligned)
    rpe = compute_rpe(P_gt_matched, P_aligned, args.rpe_delta)
    scale_error = compute_scale_error(P_gt_matched, P_est_unaligned)
    
    # Print results
    print("\n" + "="*60)
    print("RESULTS")
    print("="*60)
    print(f"\nATE (Absolute Trajectory Error):")
    print(f"  RMSE: {ate['rmse']:.4f} m")
    print(f"\nRPE (Relative Pose Error, delta={args.rpe_delta}):")
    print(f"  Trans RMSE: {rpe['trans_rmse']:.4f} m")
    print(f"\nScale Error:")
    print(f"  Scale Drift: {scale_error['scale_drift_percent']:.2f}%")
    
    print("\n" + "="*60)
    print("LEADERBOARD SUBMISSION FORMAT")
    print("="*60)
    
    submission = {
        "group_id": "YOUR_GROUP_ID",
        "group_name": "Your Group Name",
        "metrics": {
            "ate": round(ate['rmse'], 4),
            "rpe": round(rpe['trans_rmse'], 4),
            "scale_error": round(scale_error['scale_drift_percent'], 2)
        },
        "submission_date": str(date.today())
    }
    
    print(json.dumps(submission, indent=4))
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

