# 🏆 AAE5303 Visual Odometry - Leaderboard

## 📁 Evaluation Dataset

**HKisland_GNSS03** sequence from MARS-LVIG / UAVScenes Dataset

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |

---

## 📊 Evaluation Metrics

The leaderboard evaluates Visual Odometry submissions using three standard trajectory accuracy metrics. All metrics are computed by comparing **estimated trajectory** against **RTK ground truth**.

---

### 1. ATE (Absolute Trajectory Error) ↓

**Lower is better** | Unit: meters (m)

#### Definition

ATE measures the root mean square error (RMSE) of the estimated trajectory after optimal Sim(3) alignment to the ground truth. It represents the overall trajectory accuracy including cumulative drift.

#### Mathematical Formula

$$ATE_{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\|\mathbf{p}_{est}^i - \mathbf{p}_{gt}^i\|^2}$$

where:
- $\mathbf{p}_{est}^i$ = Aligned estimated position at time $i$
- $\mathbf{p}_{gt}^i$ = Ground truth position at time $i$
- $N$ = Number of matched poses

The alignment uses Sim(3) transformation (7-DOF: 3 translation + 3 rotation + 1 scale) to optimally align the estimated trajectory to ground truth before computing errors.

#### Reference Code

```python
import numpy as np

def compute_ate(P_gt: np.ndarray, P_aligned: np.ndarray) -> float:
    """
    Compute Absolute Trajectory Error (ATE) RMSE.
    
    Args:
        P_gt: Nx3 ground truth positions (after association)
        P_aligned: Nx3 aligned estimated positions
    
    Returns:
        ATE RMSE in meters
    """
    # Compute Euclidean distance errors
    errors = np.linalg.norm(P_gt - P_aligned, axis=1)
    
    # Return RMSE
    ate_rmse = np.sqrt(np.mean(errors ** 2))
    
    return ate_rmse
```

#### Reference

Sturm, J., et al. "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012.

---

### 2. RPE (Relative Pose Error) ↓

**Lower is better** | Unit: meters (m)

#### Definition

RPE measures the local consistency of the trajectory by comparing relative transformations between the estimated and ground truth trajectories over a fixed time interval (delta). It evaluates drift rate rather than cumulative error.

#### Mathematical Formula

$$RPE_{trans} = \sqrt{\frac{1}{M}\sum_{i=1}^{M}\|\Delta\mathbf{p}_{est}^i - \Delta\mathbf{p}_{gt}^i\|^2}$$

where:
- $\Delta\mathbf{p}^i = \mathbf{p}(t_i + \Delta) - \mathbf{p}(t_i)$ = Relative motion over interval $\Delta$
- $M$ = Number of relative pose pairs

#### Reference Code

```python
import numpy as np

def compute_rpe(P_gt: np.ndarray, P_est: np.ndarray, delta: int = 10) -> float:
    """
    Compute Relative Pose Error (RPE) translational RMSE.
    
    Args:
        P_gt: Nx3 ground truth positions
        P_est: Nx3 estimated positions (aligned)
        delta: Frame interval for relative motion computation
    
    Returns:
        RPE translational RMSE in meters
    """
    n = len(P_gt)
    trans_errors = []
    
    for i in range(n - delta):
        # Ground truth relative motion
        gt_rel = P_gt[i + delta] - P_gt[i]
        
        # Estimated relative motion
        est_rel = P_est[i + delta] - P_est[i]
        
        # Compute error
        error = np.linalg.norm(gt_rel - est_rel)
        trans_errors.append(error)
    
    # Return RMSE
    rpe_rmse = np.sqrt(np.mean(np.array(trans_errors) ** 2))
    
    return rpe_rmse
```

#### Reference

Geiger, A., et al. "Vision meets Robotics: The KITTI Dataset", IJRR 2013.

---

### 3. Scale Error ↓

**Lower is better** | Unit: percentage (%)

#### Definition

For monocular Visual Odometry, absolute scale is unobservable (scale ambiguity). Scale Error measures how accurately the VO system estimates the trajectory scale by comparing path lengths.

#### Mathematical Formula

$$Scale_{error} = |1 - \frac{L_{est}}{L_{gt}}| \times 100\%$$

where:
- $L_{est}$ = Total path length of estimated trajectory (before alignment)
- $L_{gt}$ = Total path length of ground truth trajectory

$$L = \sum_{i=1}^{N-1}\|\mathbf{p}^{i+1} - \mathbf{p}^i\|$$

#### Reference Code

```python
import numpy as np

def compute_scale_error(P_gt: np.ndarray, P_est_unaligned: np.ndarray) -> float:
    """
    Compute Scale Error for monocular VO.
    
    Args:
        P_gt: Nx3 ground truth positions
        P_est_unaligned: Nx3 estimated positions (before Sim3 alignment)
    
    Returns:
        Scale error in percentage
    """
    # Compute total path lengths
    gt_segments = np.diff(P_gt, axis=0)
    est_segments = np.diff(P_est_unaligned, axis=0)
    
    gt_length = np.sum(np.linalg.norm(gt_segments, axis=1))
    est_length = np.sum(np.linalg.norm(est_segments, axis=1))
    
    # Scale ratio
    scale_ratio = est_length / gt_length
    
    # Scale error (percentage)
    scale_error = abs(1.0 - scale_ratio) * 100.0
    
    return scale_error
```

#### Interpretation

| Scale Error | Interpretation |
|-------------|----------------|
| < 5% | Excellent scale estimation |
| 5-10% | Good scale estimation |
| 10-15% | Acceptable |
| > 15% | Poor scale estimation |

---

## 📦 Complete Evaluation Script

Use this script to compute all three metrics for your submission:

```python
#!/usr/bin/env python3
"""
AAE5303 Visual Odometry Leaderboard - Metrics Calculation Script
"""

import numpy as np
import json
from datetime import date

def load_trajectory_tum(filename: str):
    """
    Load trajectory from TUM format file.
    Format: timestamp x y z qx qy qz qw
    
    Returns:
        timestamps: Nx1 array
        positions: Nx3 array
    """
    data = np.loadtxt(filename)
    timestamps = data[:, 0]
    positions = data[:, 1:4]
    return timestamps, positions


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
    # Center point clouds
    centroid_gt = np.mean(P_gt, axis=0)
    centroid_est = np.mean(P_est, axis=0)
    
    P_gt_centered = P_gt - centroid_gt
    P_est_centered = P_est - centroid_est
    
    # Compute scale
    scale = np.sqrt(np.sum(P_gt_centered ** 2) / np.sum(P_est_centered ** 2))
    P_est_scaled = P_est_centered * scale
    
    # Compute rotation using SVD
    H = P_est_scaled.T @ P_gt_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Compute translation
    t = centroid_gt - scale * (R @ centroid_est)
    
    # Apply transformation
    P_aligned = scale * (P_est @ R.T) + t
    
    return P_aligned, scale


def compute_ate(P_gt, P_aligned):
    """Compute Absolute Trajectory Error RMSE."""
    errors = np.linalg.norm(P_gt - P_aligned, axis=1)
    return np.sqrt(np.mean(errors ** 2))


def compute_rpe(P_gt, P_est, delta=10):
    """Compute Relative Pose Error translational RMSE."""
    trans_errors = []
    for i in range(len(P_gt) - delta):
        gt_rel = P_gt[i + delta] - P_gt[i]
        est_rel = P_est[i + delta] - P_est[i]
        trans_errors.append(np.linalg.norm(gt_rel - est_rel))
    return np.sqrt(np.mean(np.array(trans_errors) ** 2))


def compute_scale_error(P_gt, P_est_unaligned):
    """Compute Scale Error percentage."""
    gt_length = np.sum(np.linalg.norm(np.diff(P_gt, axis=0), axis=1))
    est_length = np.sum(np.linalg.norm(np.diff(P_est_unaligned, axis=0), axis=1))
    return abs(1.0 - est_length / gt_length) * 100.0


def evaluate_trajectory(gt_file: str, est_file: str) -> dict:
    """
    Evaluate estimated trajectory against ground truth.
    
    Args:
        gt_file: Path to ground truth trajectory (TUM format)
        est_file: Path to estimated trajectory (TUM format)
    
    Returns:
        Dictionary with metrics
    """
    # Load trajectories
    t_gt, P_gt = load_trajectory_tum(gt_file)
    t_est, P_est = load_trajectory_tum(est_file)
    
    # Associate by timestamp
    matches = associate_trajectories(t_gt, t_est)
    
    gt_indices = [m[0] for m in matches]
    est_indices = [m[1] for m in matches]
    P_gt_matched = P_gt[gt_indices]
    P_est_matched = P_est[est_indices]
    P_est_unaligned = P_est_matched.copy()
    
    # Align using Sim(3)
    P_aligned, scale = align_trajectories_sim3(P_gt_matched, P_est_matched)
    
    # Compute metrics
    ate = compute_ate(P_gt_matched, P_aligned)
    rpe = compute_rpe(P_gt_matched, P_aligned)
    scale_error = compute_scale_error(P_gt_matched, P_est_unaligned)
    
    return {
        'ate': round(ate, 4),
        'rpe': round(rpe, 4),
        'scale_error': round(scale_error, 2)
    }


def generate_submission_json(group_id: str, group_name: str, metrics: dict, output_path: str):
    """Generate submission JSON file."""
    submission = {
        "group_id": group_id,
        "group_name": group_name,
        "metrics": metrics,
        "submission_date": str(date.today())
    }
    
    with open(output_path, 'w') as f:
        json.dump(submission, f, indent=4)
    
    print(f"Submission saved to: {output_path}")
    print(json.dumps(submission, indent=4))


# Example usage:
if __name__ == "__main__":
    # Calculate metrics
    metrics = evaluate_trajectory(
        gt_file="./rtk_groundtruth.txt",
        est_file="./KeyFrameTrajectory.txt"
    )
    
    # Generate submission JSON
    generate_submission_json(
        group_id="Group_01",
        group_name="Your Group Name",
        metrics=metrics,
        output_path="Group_01_leaderboard.json"
    )
```

---

## 📄 Submission Format

Submit a JSON file with the following format:

```json
{
    "group_id": "Group_01",
    "group_name": "Team Alpha",
    "metrics": {
        "ate": 2.5335,
        "rpe": 1.7309,
        "scale_error": 8.38
    },
    "submission_date": "2024-12-22"
}
```

Template file: [submission_template.json](./submission_template.json)

---

## 🌐 Leaderboard Website & Baseline

> **📢 The leaderboard submission website and baseline results will be announced later.**
>
> Students will be able to upload their JSON submission files to the website, which will automatically parse the metrics and display real-time rankings.

### Baseline Results (Reference)

| Metric | Baseline Value |
|--------|---------------|
| **ATE** | 2.5335 m |
| **RPE** | 1.7309 m |
| **Scale Error** | 8.38% |

*These baseline results are from the default ORB-SLAM3 configuration without optimization.*

