# 🏆 AAE5303 Visual Odometry – Leaderboard

This folder contains the **student-facing** leaderboard specification:

- What to submit
- Which metrics are used (four *parallel* metrics, no weighting)
- The fixed evaluation protocol (association + alignment), so results are comparable across teams

If you are looking for a “how to run ORB-SLAM3 without pitfalls” guide, see:

- `ORB_SLAM3_TIPS.md`

## 📌 What you submit

Submit **one JSON file per group**:

- File name: `{GroupName}_leaderboard.json`
- Format template: `submission_template.json`

The leaderboard will parse your JSON file and display rankings **separately for each metric**.

## 📊 Metrics (four parallel metrics)

All metrics are computed by comparing an **estimated TUM trajectory** against the **provided ground truth**, using a fixed evaluation protocol (alignment + association).

| Metric | Direction | Unit | Description |
|--------|-----------|------|-------------|
| **ATE RMSE** | ↓ | m | Global accuracy after Sim(3) alignment + scale correction |
| **RPE Trans Drift** | ↓ | m/m | Translation drift rate (distance-based RPE, delta = 10 m) |
| **RPE Rot Drift** | ↓ | deg/100m | Rotation drift rate (distance-based RPE, delta = 10 m) |
| **Completeness** | ↑ | % | Matched poses / total ground-truth poses |

### What each metric measures (intuition)

#### ATE RMSE (m)

Absolute Trajectory Error measures the **global** discrepancy between the estimated trajectory and the ground truth **after** applying a single best-fit Sim(3) transform (rotation + translation + scale).

- Good ATE → your *overall* trajectory shape is close to ground truth.
- Bad ATE → strong accumulated drift, wrong relocalization, or inconsistent tracking.

#### RPE translation drift (m/m)

Relative Pose Error (translation) is computed over a fixed distance interval (10 m). `evo` reports mean translation error in meters over that interval, which we normalize into a drift rate:

```text
RPE_trans_drift_m_per_m = RPE_trans_mean_m / 10
```

This metric emphasizes **local drift** rather than cumulative error.

#### RPE rotation drift (deg/100m)

Relative Pose Error (rotation) uses the rotation angle error in degrees over the same 10 m distance interval, normalized as:

```text
RPE_rot_drift_deg_per_100m = (RPE_rot_mean_deg / 10) * 100
```

Large values typically indicate unstable orientation estimates and/or poor feature geometry.

#### Completeness (%)

Completeness measures how much of the ground-truth trajectory can be evaluated:

```text
Completeness (%) = matched_poses / gt_poses * 100
```

This discourages submissions that only output a short “easy” segment.

### Fixed evaluation parameters

- **Trajectory format**: TUM (`t tx ty tz qx qy qz qw`)
- **Timestamp association**: `t_max_diff = 0.1 s`
- **Alignment**: Sim(3) with scale correction (`--align --correct_scale`)
- **RPE delta**: `delta = 10 m` (distance domain)

### Why Sim(3) alignment is required for monocular VO

Monocular VO cannot observe absolute metric scale. Without Sim(3) alignment, metrics would be dominated by an arbitrary scale factor. Using Sim(3) with scale correction makes the metrics reflect:

- Trajectory **shape** consistency
- Drift and tracking quality

rather than the unknown global scale.

## ✅ How to compute the same numbers locally

See `LEADERBOARD_SUBMISSION_GUIDE.md` for:

- The exact `evo` commands used by the leaderboard
- How to compute drift rates from evo outputs
- The JSON schema and an example submission

## 🧾 Submission example

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

## ❓ FAQ

### Q1: Can I submit `KeyFrameTrajectory.txt`?

No. Use `CameraTrajectory.txt` (full-frame trajectory). Keyframe-only trajectories distort completeness and drift-rate metrics.

### Q2: evo says “Found no matching timestamps”

Common causes:

- You evaluated with the wrong ground truth file.
- Your trajectory timestamps are not in seconds (e.g., frame indices).
- `t_max_diff` is too small (the leaderboard uses 0.1 s).

## 🌐 Website & Baseline

Leaderboard URL: `https://qian9921.github.io/leaderboard_web/`

Baseline (AMtown):

- **ATE RMSE**: 88.2281 m
- **RPE Trans Drift**: 2.04084 m/m
- **RPE Rot Drift**: 76.69911 deg/100m
- **Completeness**: 95.73 %

