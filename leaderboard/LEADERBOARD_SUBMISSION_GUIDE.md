# AAE5303 - Leaderboard Submission Guide

## 📁 Evaluation Dataset

**HKisland_GNSS03** sequence from MARS-LVIG / UAVScenes Dataset

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |

---

## 📊 Evaluation Metrics

| Metric | Direction | Unit | Description |
|--------|-----------|------|-------------|
| **ATE** | ↓ Lower is better | meters (m) | Absolute Trajectory Error - RMSE after Sim(3) alignment |
| **RPE** | ↓ Lower is better | meters (m) | Relative Pose Error - Local consistency over 10 frames |
| **Scale Error** | ↓ Lower is better | percentage (%) | Scale drift for monocular VO |

---

## 📄 JSON Submission Format

Submit your results using the following JSON format:

```json
{
    "group_id": "YOUR_GROUP_ID",
    "group_name": "Your Group Name",
    "metrics": {
        "ate": 2.5335,
        "rpe": 1.7309,
        "scale_error": 8.38
    },
    "submission_date": "YYYY-MM-DD"
}
```

### Field Descriptions

| Field | Type | Description | Example |
|-------|------|-------------|---------|
| `group_id` | string | Your group ID | `"Group_01"` |
| `group_name` | string | Your group name | `"Team Alpha"` |
| `metrics.ate` | number | ATE RMSE in meters | `2.5335` |
| `metrics.rpe` | number | RPE Trans RMSE in meters | `1.7309` |
| `metrics.scale_error` | number | Scale error in percentage | `8.38` |
| `submission_date` | string | Date (YYYY-MM-DD) | `"2024-12-22"` |

### File Naming Convention

`{GroupID}_leaderboard.json`

Example: `Group_01_leaderboard.json`

---

## 🔢 Metric Calculation

### 1. ATE (Absolute Trajectory Error)

```python
# After Sim(3) alignment
errors = np.linalg.norm(P_gt - P_aligned, axis=1)
ate = np.sqrt(np.mean(errors ** 2))  # RMSE in meters
```

### 2. RPE (Relative Pose Error)

```python
# Over 10-frame intervals
for i in range(len(P_gt) - 10):
    gt_rel = P_gt[i + 10] - P_gt[i]
    est_rel = P_est[i + 10] - P_est[i]
    error = np.linalg.norm(gt_rel - est_rel)
rpe = np.sqrt(np.mean(errors ** 2))  # RMSE in meters
```

### 3. Scale Error

```python
# Path length comparison (before alignment)
gt_length = sum of segment lengths in ground truth
est_length = sum of segment lengths in estimated trajectory
scale_error = abs(1 - est_length / gt_length) * 100  # Percentage
```

---

## 🌐 Leaderboard Website & Baseline

> **📢 The leaderboard submission website and baseline results will be announced later.**

