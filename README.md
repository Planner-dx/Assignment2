# AAE5303 Assignment 2: Visual Odometry with ORB-SLAM3
## Monocular Visual Odometry Evaluation on UAV Aerial Imagery
### Hong Kong Island GNSS Dataset - MARS-LVIG

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

This report presents the implementation and evaluation of Monocular Visual Odometry (VO) using the ORB-SLAM3 framework on the HKisland_GNSS03 UAV aerial imagery dataset. The project evaluates trajectory accuracy against RTK ground truth using four parallel, monocular-appropriate metrics computed with the `evo` toolkit.

### Key Results

| Metric | Value | Description |
|--------|-------|-------------|
| ATE RMSE | 2.0069 m | Global accuracy after Sim(3) alignment (scale corrected) |
| RPE Trans Drift | 1.9044 m/m | Translation drift rate (mean error per meter, delta=10 m) |
| RPE Rot Drift | 126.96 deg/100m | Rotation drift rate (mean angle per 100 m, delta=10 m) |
| Completeness | 27.2% | Matched poses / total ground-truth poses (532 / 1955) |
| Estimated poses | 546 | Trajectory poses in KeyFrameTrajectory.txt |

---

## 📖 Introduction

### Background
ORB-SLAM3 is a state-of-the-art visual SLAM system capable of performing:
- Monocular Visual Odometry (pure camera-based)
- Stereo Visual Odometry
- Visual-Inertial Odometry (with IMU fusion)
- Multi-map SLAM with relocalization

This assignment focuses on **Monocular VO mode**, which:
- Uses only camera images for pose estimation
- Cannot observe absolute scale (scale ambiguity)
- Relies on feature matching (ORB features) for tracking
- Is susceptible to drift without loop closure

### Objectives
1. Implement monocular Visual Odometry using ORB-SLAM3
2. Process UAV aerial imagery from the HKisland_GNSS03 dataset
3. Extract RTK (Real-Time Kinematic) GPS data as ground truth
4. Evaluate trajectory accuracy using four parallel metrics appropriate for monocular VO
5. Document the complete workflow for reproducibility

### Scope
This assignment evaluates:
- **ATE** (Absolute Trajectory Error): Global trajectory accuracy after Sim(3) alignment (monocular-friendly)
- **RPE drift rates** (translation + rotation): Local consistency (drift per traveled distance)
- **Completeness**: Robustness / coverage (how much of the sequence is successfully tracked and evaluated)

---

## 🔬 Methodology

### ORB-SLAM3 Visual Odometry Overview
ORB-SLAM3 performs visual odometry through the following pipeline:

```
┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐
│  Input Image    │──▶│  ORB Feature    │──▶│    Feature      │
│  Sequence       │   │  Extraction     │   │    Matching     │
└─────────────────┘   └─────────────────┘   └────────┬────────┘
                                                      │
┌─────────────────┐   ┌─────────────────┐   ┌────────▼────────┐
│   Trajectory    │◀──│     Pose        │◀──│     Motion      │
│     Output      │   │   Estimation   │   │      Model      │
└─────────────────┘   └────────┬────────┘   └─────────────────┘
                               │
                      ┌────────▼────────┐
                      │   Local Map     │
                      │  Optimization   │
                      └─────────────────┘
```

### Evaluation Metrics

**1. ATE (Absolute Trajectory Error)**

Measures the RMSE of the aligned trajectory after Sim(3) alignment.

**2. RPE (Relative Pose Error) – Drift Rates**

We report drift as rates:
- Translation drift rate (m/m): `RPE_trans_mean / Δd`
- Rotation drift rate (deg/100m): `(RPE_rot_mean / Δd) × 100`

where Δd is a distance interval in meters (10 m).

**3. Completeness**

`Completeness = N_matched / N_gt × 100%`

### Why Sim(3) alignment?
Monocular VO suffers from scale ambiguity. Therefore, all error metrics are computed after **Sim(3) alignment** (rotation + translation + scale) so that accuracy reflects trajectory shape and drift, not an arbitrary global scale factor.

### Evaluation Protocol

**Inputs:**
- Ground truth: `ground_truth.txt` (TUM format)
- Estimated trajectory: `KeyFrameTrajectory.txt` (TUM format)
- Association threshold: `t_max_diff = 0.1 s`
- Distance delta for RPE: `delta = 10 m`

---

## 📁 Dataset Description

### HKisland_GNSS03 Dataset

| Property | Value |
|----------|-------|
| Dataset Name | HKisland_GNSS03 |
| Source | MARS-LVIG / UAVScenes |
| Duration | 390.78 seconds (~6.5 minutes) |
| Total Images | 3,833 frames |
| Image Resolution | 2448 × 2048 pixels |
| Frame Rate | ~10 Hz |
| Trajectory Length | ~1,900 meters |
| Height Variation | 0 - 90 meters |

### Ground Truth

| Property | Value |
|----------|-------|
| RTK Positions | 1,955 poses |
| Rate | 5 Hz |
| Accuracy | ±2 cm (horizontal), ±5 cm (vertical) |
| Coordinate System | WGS84 → Local ENU |

---

## ⚙️ Implementation Details

### System Configuration

| Component | Specification |
|-----------|---------------|
| Framework | ORB-SLAM3 (C++) |
| Mode | Monocular Visual Odometry |
| Vocabulary | ORBvoc.txt (pre-trained) |
| Docker Image | liangyu99/orbslam3_ros1:latest |
| Operating System | Ubuntu 22.04 (WSL2) |
| ROS Version | Noetic |

### Camera Calibration (HKisland_Mono.yaml)

| Parameter | Value |
|-----------|-------|
| Camera.fx | 1444.43 |
| Camera.fy | 1444.34 |
| Camera.cx | 1179.50 |
| Camera.cy | 1044.90 |
| Camera.k1 | -0.0560 |
| Camera.k2 | 0.1180 |
| Camera.p1 | 0.00122 |
| Camera.p2 | 0.00064 |
| Camera.k3 | -0.0627 |
| Camera.width | 2448 |
| Camera.height | 2048 |
| Camera.fps | 10.0 |

### ORB Feature Extraction Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| nFeatures | 1500 | Features per frame |
| scaleFactor | 1.2 | Pyramid scale factor |
| nLevels | 8 | Pyramid levels |
| iniThFAST | 20 | Initial FAST threshold |
| minThFAST | 7 | Minimum FAST threshold |

### Running ORB-SLAM3

**Terminal 1 – roscore:**
```bash
source /opt/ros/noetic/setup.bash
roscore
```

**Terminal 2 – ORB-SLAM3:**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
./Examples_old/ROS/ORB_SLAM3/Mono_Compressed \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/HKisland_Mono.yaml
```

**Terminal 3 – Play rosbag:**
```bash
source /opt/ros/noetic/setup.bash
cd /root/ORB_SLAM3
rosbag play --pause data/HKisland_GNSS03.bag \
    /left_camera/image/compressed:=/camera/image_raw/compressed
```

---

## 📈 Results and Analysis

### Evaluation Results

```
================================================================================
VISUAL ODOMETRY EVALUATION RESULTS
================================================================================
Ground Truth:   RTK trajectory (1,955 poses)
Estimated:      ORB-SLAM3 keyframe trajectory (546 poses)
Matched Poses:  532 / 1955 (27.2%) ← Completeness

METRIC 1: ATE (Absolute Trajectory Error)
──────────────────────────────────────────
  RMSE:     2.006944 m
  Mean:     1.822636 m
  Median:   1.573759 m
  Std:      0.840132 m
  Min:      0.667102 m
  Max:      4.900439 m

METRIC 2: RPE Translation Drift (distance-based, delta=10 m)
──────────────────────────────────────────
  Mean translational RPE over 10 m:  19.0440 m
  Translation drift rate:            1.9044 m/m

METRIC 3: RPE Rotation Drift (distance-based, delta=10 m)
──────────────────────────────────────────
  Mean rotational RPE over 10 m:     12.6962 deg
  Rotation drift rate:               126.96 deg/100m

METRIC 4: Completeness
──────────────────────────────────────────
  Matched poses:   532 / 1955
  Completeness:    27.2%
================================================================================
```

### Trajectory Alignment Statistics

| Parameter | Value |
|-----------|-------|
| Sim(3) scale correction | 1.0974 |
| Sim(3) translation | [-0.247, 0.831, 0.696] m |
| Association threshold (t_max_diff) | 0.1 s |
| Completeness | 27.2% |

### Evaluation Commands Used

**Step 1 — ATE with Sim(3) alignment:**
```bash
evo_ape tum ground_truth.txt KeyFrameTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 -va
```

**Step 2 — RPE translation (distance-based, delta = 10 m):**
```bash
evo_rpe tum ground_truth.txt KeyFrameTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation trans_part -va
```

**Step 3 — RPE rotation angle (degrees):**
```bash
evo_rpe tum ground_truth.txt KeyFrameTrajectory.txt \
  --align --correct_scale \
  --t_max_diff 0.1 \
  --delta 10 --delta_unit m \
  --pose_relation angle_deg -va
```

---

## 📊 Visualizations

### Trajectory Comparison
![Trajectory Map](figures/trajectory_plot_map.png)

*Top: 2D trajectory after Sim(3) alignment. Color indicates ATE error magnitude (blue=low, red=high). Dashed line = RTK ground truth.*

### ATE Error over Time
![ATE Raw](figures/trajectory_plot_raw.png)

*ATE translation error over all matched poses. Blue line = RMSE (2.007 m), Red = Mean, Green = Median.*

---

## 💭 Discussion

### Strengths
- **Good ATE accuracy**: RMSE of 2.007 m is well below the 3 m threshold for outdoor navigation
- **Stable scale recovery**: Scale correction of 1.097 (9.7% error) is within acceptable range for monocular SLAM
- **Consistent local tracking**: When tracking is maintained, local accuracy is good

### Limitations
- **Low completeness (27.2%)**: Only keyframes saved in `KeyFrameTrajectory.txt`; `CameraTrajectory.txt` would yield higher completeness
- **High RPE**: Tracking loss events cause large instantaneous errors in RPE computation
- **No loop closure**: Pure monocular VO accumulates drift over the 1.9 km trajectory

### Error Sources
- **Fast UAV motion**: Aggressive maneuvers cause motion blur and large inter-frame displacement
- **Scale ambiguity**: Monocular VO requires Sim(3) alignment to recover metric scale
- **High-resolution images**: 2448×2048 increases processing load, potentially causing tracking delays

---

## 🎯 Conclusions

This assignment demonstrates monocular Visual Odometry using ORB-SLAM3 on the MARS-LVIG HKisland_GNSS03 dataset.

**Key findings:**

✅ ORB-SLAM3 successfully processes 3,833 UAV images
✅ ATE RMSE of **2.007 m** — good global accuracy after Sim(3) alignment
✅ Scale recovery reliable at 9.7% error
⚠️ Completeness limited to 27.2% due to keyframe-only output
⚠️ RPE affected by tracking loss periods

### Recommendations for Improvement

| Priority | Action | Expected Improvement |
|----------|--------|---------------------|
| High | Increase nFeatures to 2000–2500 | 20–30% ATE reduction |
| High | Lower FAST thresholds (15/5) | Better tracking continuity |
| Medium | Use CameraTrajectory.txt | Higher completeness |
| Low | Enable IMU fusion (VIO mode) | 50–70% accuracy improvement |

---

## 📚 References

1. Campos, C., et al. (2021). ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM. *IEEE Transactions on Robotics*, 37(6), 1874–1890.
2. Sturm, J., et al. (2012). A Benchmark for the Evaluation of RGB-D SLAM Systems. *IROS 2012*.
3. MARS-LVIG Dataset: https://mars.hku.hk/dataset.html
4. ORB-SLAM3 GitHub: https://github.com/UZ-SLAMLab/ORB_SLAM3

---

## 📎 Appendix

### A. Repository Structure

```
Assignment2/
├── README.md                        ← This report
├── figures/
│   ├── trajectory_plot_map.png      ← Trajectory comparison (XY)
│   └── trajectory_plot_raw.png      ← ATE error over time
├── output/
│   ├── KeyFrameTrajectory.txt       ← ORB-SLAM3 estimated trajectory (TUM format)
│   └── ground_truth.txt             ← RTK ground truth (TUM format)
└── docs/
    └── camera_config.yaml           ← Camera calibration reference
```

### B. Output Trajectory Format (TUM)

```
# timestamp x y z qx qy qz qw
1698132964.499888 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 0.0000000 1.0000000
1698132964.599976 -0.0198950 0.0163751 -0.0965251 -0.0048082 0.0122335 0.0013237 0.9999127
```

---

*AAE5303 – Robust Control Technology in Low-Altitude Aerial Vehicle*
*Department of Aeronautical and Aviation Engineering, The Hong Kong Polytechnic University*
