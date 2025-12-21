# AAE5303 Assignment: Visual Odometry with ORB-SLAM3

<div align="center">

![ORB-SLAM3](https://img.shields.io/badge/SLAM-ORB--SLAM3-blue?style=for-the-badge)
![VO](https://img.shields.io/badge/Mode-Visual_Odometry-green?style=for-the-badge)
![Dataset](https://img.shields.io/badge/Dataset-HKisland__GNSS03-orange?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Baseline-success?style=for-the-badge)

**Monocular Visual Odometry Evaluation on UAV Aerial Imagery**

*Hong Kong Island GNSS Dataset - MARS-LVIG*

</div>

---

## 📋 Table of Contents

1. [Executive Summary](#-executive-summary)
2. [Introduction](#-introduction)
3. [Methodology](#-methodology)
4. [Dataset Description](#-dataset-description)
5. [Implementation Details](#-implementation-details)
6. [Results and Analysis](#-results-and-analysis)
7. [Visualizations](#-visualizations)
8. [Discussion](#-discussion)
9. [Conclusions](#-conclusions)
10. [References](#-references)
11. [Appendix](#-appendix)

---

## 📊 Executive Summary

This report presents the implementation and evaluation of **Monocular Visual Odometry (VO)** using the **ORB-SLAM3** framework on the **HKisland_GNSS03** UAV aerial imagery dataset. The project evaluates trajectory accuracy against RTK ground truth using three standard academic metrics.

### Key Results

| Metric | Value | Description |
|--------|-------|-------------|
| **ATE RMSE** | 2.5335 m | Absolute Trajectory Error |
| **RPE Trans RMSE** | 1.7309 m | Relative Pose Error (10 frames) |
| **Scale Error** | 8.38% | Monocular Scale Drift |
| **Keyframes** | 620 | Output keyframe count |
| **Matched Poses** | 328 / 619 (53.0%) | Ground truth association rate |
| **Trajectory Length** | ~1,900 m | Total distance traveled |

---

## 📖 Introduction

### Background

ORB-SLAM3 is a state-of-the-art visual SLAM system capable of performing:

- **Monocular Visual Odometry** (pure camera-based)
- **Stereo Visual Odometry**
- **Visual-Inertial Odometry** (with IMU fusion)
- **Multi-map SLAM** with relocalization

This assignment focuses on **Monocular VO mode**, which:

- Uses only camera images for pose estimation
- Cannot observe absolute scale (scale ambiguity)
- Relies on feature matching (ORB features) for tracking
- Is susceptible to drift without loop closure

### Objectives

1. Implement monocular Visual Odometry using ORB-SLAM3
2. Process UAV aerial imagery from the HKisland_GNSS03 dataset
3. Extract RTK (Real-Time Kinematic) GPS data as ground truth
4. Evaluate trajectory accuracy using three academic metrics
5. Document the complete workflow for reproducibility

### Scope

This assignment evaluates:
- **ATE (Absolute Trajectory Error)**: Global trajectory accuracy
- **RPE (Relative Pose Error)**: Local consistency and drift rate
- **Scale Error**: Monocular scale estimation accuracy

---

## 🔬 Methodology

### ORB-SLAM3 Visual Odometry Overview

ORB-SLAM3 performs visual odometry through the following pipeline:

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Input Image    │────▶│   ORB Feature   │────▶│   Feature       │
│  Sequence       │     │   Extraction    │     │   Matching      │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                         │
┌─────────────────┐     ┌─────────────────┐     ┌────────▼────────┐
│   Trajectory    │◀────│   Pose          │◀────│   Motion        │
│   Output        │     │   Estimation    │     │   Model         │
└─────────────────┘     └────────┬────────┘     └─────────────────┘
                                 │
                        ┌────────▼────────┐
                        │   Local Map     │
                        │   Optimization  │
                        └─────────────────┘
```

### Evaluation Metrics

#### 1. ATE (Absolute Trajectory Error)

Measures the RMSE of the aligned trajectory after Sim(3) alignment:

$$ATE_{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^{N}\|\mathbf{p}_{est}^i - \mathbf{p}_{gt}^i\|^2}$$

**Reference**: Sturm et al., "A Benchmark for the Evaluation of RGB-D SLAM Systems", IROS 2012

#### 2. RPE (Relative Pose Error)

Measures local consistency by comparing relative transformations:

$$RPE_{trans} = \|\Delta\mathbf{p}_{est} - \Delta\mathbf{p}_{gt}\|$$

where $\Delta\mathbf{p} = \mathbf{p}(t+\Delta) - \mathbf{p}(t)$

**Reference**: Geiger et al., "Vision meets Robotics: The KITTI Dataset", IJRR 2013

#### 3. Scale Error

For monocular VO, measures scale estimation accuracy:

$$Scale_{error} = |1 - \frac{L_{est}}{L_{gt}}| \times 100\%$$

where $L$ is the total trajectory path length.

### Trajectory Alignment

We use Sim(3) (7-DOF) alignment to optimally align estimated trajectory to ground truth:

- **3-DOF Translation**: Align trajectory origins
- **3-DOF Rotation**: Align trajectory orientations
- **1-DOF Scale**: Compensate for monocular scale ambiguity

---

## 📁 Dataset Description

### HKisland_GNSS03 Dataset

The dataset is from the **MARS-LVIG** UAV dataset, captured over Hong Kong Island.

| Property | Value |
|----------|-------|
| **Dataset Name** | HKisland_GNSS03 |
| **Source** | MARS-LVIG / UAVScenes |
| **Duration** | 390.78 seconds (~6.5 minutes) |
| **Total Images** | 3,833 frames |
| **Image Resolution** | 2448 × 2048 pixels |
| **Frame Rate** | ~10 Hz |
| **Trajectory Length** | ~1,900 meters |
| **Height Variation** | 0 - 90 meters |

### Data Sources

| Resource | Link |
|----------|------|
| MARS-LVIG Dataset | https://mars.hku.hk/dataset.html |
| UAVScenes GitHub | https://github.com/sijieaaa/UAVScenes |

### Ground Truth

RTK (Real-Time Kinematic) GPS provides centimeter-level positioning accuracy:

| Property | Value |
|----------|-------|
| **RTK Positions** | 1,955 poses |
| **Rate** | 5 Hz |
| **Accuracy** | ±2 cm (horizontal), ±5 cm (vertical) |
| **Coordinate System** | WGS84 → Local ENU |

---

## ⚙️ Implementation Details

### System Configuration

| Component | Specification |
|-----------|---------------|
| **Framework** | ORB-SLAM3 (C++) |
| **Mode** | Monocular Visual Odometry |
| **Vocabulary** | ORBvoc.txt (pre-trained) |
| **Operating System** | Linux (Ubuntu 22.04) |

### Camera Calibration

```yaml
Camera.type: "PinHole"
Camera.fx: 1444.43
Camera.fy: 1444.34
Camera.cx: 1179.50
Camera.cy: 1044.90

Camera.k1: -0.0560
Camera.k2: 0.1180
Camera.p1: 0.00122
Camera.p2: 0.00064
Camera.k3: -0.0627

Camera.width: 2448
Camera.height: 2048
Camera.fps: 10.0
```

### ORB Feature Extraction Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `nFeatures` | 1500 | Features per frame |
| `scaleFactor` | 1.2 | Pyramid scale factor |
| `nLevels` | 8 | Pyramid levels |
| `iniThFAST` | 20 | Initial FAST threshold |
| `minThFAST` | 7 | Minimum FAST threshold |

### Running Command

```bash
./Examples/Monocular/mono_tum \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/DJI_Camera.yaml \
    data/extracted_data
```

---

## 📈 Results and Analysis

### Evaluation Results

```
================================================================================
VISUAL ODOMETRY EVALUATION RESULTS
================================================================================

Ground Truth: RTK trajectory (1,955 poses)
Estimated:    ORB-SLAM3 keyframes (619 poses)
Matched Poses: 328 / 619 (53.0%)

METRIC 1: ATE (Absolute Trajectory Error)
────────────────────────────────────────
RMSE:   2.5335 m    ← Primary metric
Mean:   2.3019 m
Median: 2.0145 m
Std:    1.0581 m
Min:    0.9402 m
Max:    5.9473 m

METRIC 2: RPE (Relative Pose Error, delta=10 frames)
────────────────────────────────────────
Trans RMSE:   1.7309 m    ← Primary metric
Trans Mean:   1.1978 m
Trans Median: 0.9636 m
Trans Std:    1.2496 m

METRIC 3: Scale Error
────────────────────────────────────────
Scale Ratio: 0.9162
Scale Drift: 8.38%    ← Primary metric

================================================================================
```

### Trajectory Alignment Statistics

| Parameter | Value |
|-----------|-------|
| **Estimated Scale** | 1.0857 |
| **Translation** | [-0.486, 0.634, 0.909] m |
| **Association Rate** | 53.0% |

### Performance Analysis

| Metric | Value | Grade | Interpretation |
|--------|-------|-------|----------------|
| **ATE RMSE** | 2.53 m | C+ | Moderate accuracy; drift present |
| **RPE RMSE** | 1.73 m | C+ | Local tracking issues observed |
| **Scale Error** | 8.38% | B+ | Good scale estimation |

---

## 📊 Visualizations

### Trajectory Comparison

![Trajectory Evaluation](figures/trajectory_evaluation.png)

The visualization includes:
1. **Top-Left**: 2D trajectory before alignment (shows scale difference)
2. **Top-Right**: 2D trajectory after Sim(3) alignment
3. **Bottom-Left**: ATE error histogram distribution
4. **Bottom-Right**: ATE error along trajectory (temporal evolution)

---

## 💭 Discussion

### Strengths

1. **Good Scale Estimation**: 8.38% scale error is acceptable for monocular VO, demonstrating reasonable scale observability from camera motion.

2. **Complete Trajectory**: The system successfully processed the entire ~6.5-minute sequence without complete failure.

3. **Reasonable Drift Rate**: 0.13% relative trajectory error (ATE/distance) is within acceptable range for pure VO.

### Limitations

1. **Tracking Instability**: Frequent "Fail to track local map!" errors observed, leading to multiple map resets (2 maps created).

2. **Low Association Rate**: Only 53% of keyframes matched with ground truth, indicating significant tracking failures.

3. **No Loop Closure**: Pure VO mode without relocalization accumulates drift over long trajectories.

### Error Sources

1. **Fast UAV Motion**: Aggressive flight maneuvers cause motion blur and large inter-frame displacements.

2. **Feature Extraction**: Default ORB parameters (1500 features) may be insufficient for high-resolution images.

3. **Calibration Accuracy**: Camera intrinsics and distortion parameters affect pose estimation quality.

---

## 🎯 Conclusions

This assignment demonstrates monocular Visual Odometry implementation using ORB-SLAM3 on UAV aerial imagery. Key findings:

1. ✅ **System Operation**: ORB-SLAM3 successfully processes 3,833 images over 1.9 km trajectory
2. ✅ **Scale Estimation**: 8.38% scale error demonstrates good monocular scale estimation
3. ⚠️ **Tracking Stability**: Frequent tracking failures indicate need for parameter optimization
4. ⚠️ **Drift Accumulation**: 2.5m ATE shows typical monocular VO drift characteristics

### Recommendations for Improvement

| Priority | Action | Expected Improvement |
|----------|--------|---------------------|
| High | Increase `nFeatures` to 2000-2500 | 30-40% ATE reduction |
| High | Lower FAST thresholds (15/5) | 20-30% RPE reduction |
| Medium | Verify camera calibration | 15-25% overall improvement |
| Low | Enable IMU fusion (VIO mode) | 50-70% accuracy improvement |

---

## 📚 References

1. Campos, C., Elvira, R., Rodríguez, J. J. G., Montiel, J. M., & Tardós, J. D. (2021). **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**. *IEEE Transactions on Robotics*, 37(6), 1874-1890.

2. Sturm, J., Engelhard, N., Endres, F., Burgard, W., & Cremers, D. (2012). **A Benchmark for the Evaluation of RGB-D SLAM Systems**. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.

3. Geiger, A., Lenz, P., & Urtasun, R. (2012). **Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite**. *IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*.

4. MARS-LVIG Dataset: https://mars.hku.hk/dataset.html

5. ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3

---

## 📎 Appendix

### A. Repository Structure

```
AAE5303_assignment2_orbslam3_demo-/
├── README.md                    # This report
├── requirements.txt             # Python dependencies
├── figures/
│   └── trajectory_evaluation.png
├── output/
│   └── evaluation_report.json
├── scripts/
│   └── evaluate_vo_accuracy.py
├── docs/
│   └── camera_config.yaml
└── leaderboard/
    ├── README.md
    ├── LEADERBOARD_SUBMISSION_GUIDE.md
    └── submission_template.json
```

### B. Running Commands

```bash
# 1. Extract images from ROS bag
python3 extract_images_final.py HKisland_GNSS03.bag --output extracted_data

# 2. Run ORB-SLAM3 VO
./Examples/Monocular/mono_tum \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/DJI_Camera.yaml \
    data/extracted_data

# 3. Extract RTK ground truth
python3 extract_rtk_groundtruth.py HKisland_GNSS03.bag --output rtk_groundtruth.txt

# 4. Evaluate trajectory
python3 evaluate_vo_accuracy.py \
    --groundtruth rtk_groundtruth.txt \
    --estimated KeyFrameTrajectory.txt \
    --output-dir evaluation_results
```

### C. Output Trajectory Format (TUM)

```
# timestamp x y z qx qy qz qw
1698132964.499888 0.0000000 0.0000000 0.0000000 -0.0000000 -0.0000000 -0.0000000 1.0000000
1698132964.599976 -0.0198950 0.0163751 -0.0965251 -0.0048082 0.0122335 0.0013237 0.9999127
...
```

---

<div align="center">

**AAE5303 - Robust Control Technology in Low-Altitude Aerial Vehicle**

*Department of Aeronautical and Aviation Engineering*

*The Hong Kong Polytechnic University*

December 2024

</div>

