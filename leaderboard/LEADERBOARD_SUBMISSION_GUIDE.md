# AAE5303 - Leaderboard Submission Guide

## 📁 Evaluation Dataset

The evaluation dataset (bag / images / ground truth) is provided by the instructor.

The leaderboard expects an estimated trajectory in **TUM format** and computes metrics against the provided **ground truth**.

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |

All sequences used in this course are sourced from the MARS-LVIG dataset.

---

## 📊 Evaluation Metrics

| Metric | Direction | Unit | Description |
|--------|-----------|------|-------------|
| **ATE RMSE** | ↓ Lower is better | meters (m) | Global accuracy after Sim(3) alignment + scale correction |
| **RPE Trans Drift** | ↓ Lower is better | meters per meter (m/m) | Translation drift rate (distance-based RPE) |
| **RPE Rot Drift** | ↓ Lower is better | degrees per 100 meters (deg/100m) | Rotation drift rate (distance-based RPE) |
| **Completeness** | ↑ Higher is better | percent (%) | Matched poses / total ground-truth poses |

### Fixed Evaluation Parameters

To make submissions comparable, the leaderboard uses the following fixed parameters:

- **Trajectory format**: TUM (`t tx ty tz qx qy qz qw`)
- **Timestamp association**: `t_max_diff = 0.1 s`
- **Alignment**: Sim(3) with scale correction (`--align --correct_scale`)
- **RPE delta**: `delta = 10 m` (distance domain)

---

## 📄 JSON Submission Format

Submit your results using the following JSON format:

```json
{
  "group_name": "Team Alpha",
  "project_private_repo_url": "https://github.com/yourusername/project.git",
  "metrics": {
    "ate_rmse_m": 88.2281,
    "rpe_trans_drift_m_per_m": 2.04084,
    "rpe_rot_drift_deg_per_100m": 76.69911,
    "completeness_pct": 95.73
  }
}
```

### Field Descriptions

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| `group_name` | string | Your group name (as shown on the leaderboard) | `"Team Alpha"` |
| `project_private_repo_url` | string | Private Git repository URL for your project | `"https://github.com/yourusername/project.git"` |
| `metrics.ate_rmse_m` | number | ATE RMSE in meters | `88.2281` |
| `metrics.rpe_trans_drift_m_per_m` | number | Translation drift rate | `2.04084` |
| `metrics.rpe_rot_drift_deg_per_100m` | number | Rotation drift rate | `76.69911` |
| `metrics.completeness_pct` | number | Completeness percentage | `95.73` |

### File Naming Convention

`{GroupName}_leaderboard.json`

Example: `Team_Alpha_leaderboard.json`

---

## ✅ Trajectory Requirements (read this before evaluating)

- Use **`CameraTrajectory.txt`** (full-frame trajectory), not `KeyFrameTrajectory.txt`.
- Ensure your file is valid **TUM format**: `t tx ty tz qx qy qz qw` with timestamps in seconds.

For a practical “do this, avoid that” guide, see:

- `ORB_SLAM3_TIPS.md`

---

## 🔢 Metric Calculation

This section describes how to compute the metrics **locally** in a way that matches the leaderboard.

### 1. ATE RMSE (Sim(3) aligned, scale corrected)

```python
# After Sim(3) alignment
errors = np.linalg.norm(P_gt - P_aligned, axis=1)
ate = np.sqrt(np.mean(errors ** 2))  # RMSE in meters
```

Recommended `evo` command:

```bash
evo_ape tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 -va
```

### 2. RPE Translation Drift (m/m)

```python
# Over a distance interval of delta_d meters (delta_d = 10 m)
# evo reports mean RPE in meters over delta_d
rpe_trans_drift_m_per_m = rpe_trans_mean_m / delta_d
```

Recommended `evo` command:

```bash
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation trans_part -va
```

### 3. RPE Rotation Drift (deg/100m)

```python
# evo reports mean rotation angle error in degrees over delta_d
rpe_rot_drift_deg_per_100m = (rpe_rot_mean_deg / delta_d) * 100.0
```

Recommended `evo` command:

```bash
evo_rpe tum ground_truth.txt CameraTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation angle_deg -va
```

### 4. Completeness (%)

```python
completeness_pct = matched_poses / gt_poses * 100.0
```

Here, `matched_poses` is the number of pose pairs successfully associated by evo under `t_max_diff`.

---

## 🌐 Leaderboard Website & Baseline

### Website

Leaderboard URL: `https://qian9921.github.io/leaderboard_web/`

### Baseline (AMtown sequence)

The following baseline is computed on the AMtown sequence using the fixed evaluation protocol above:

- `ate_rmse_m`: **88.2281**
- `rpe_trans_drift_m_per_m`: **2.04084**
- `rpe_rot_drift_deg_per_100m`: **76.69911**
- `completeness_pct`: **95.73**

