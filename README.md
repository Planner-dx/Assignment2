# AAE5303 Assignment 2 - ORB-SLAM3 Demo

## 1. Project Overview
This project demonstrates a basic ORB-SLAM3 visual odometry pipeline for AAE5303 Assignment 2. The goal is to run the provided ORB-SLAM3 demo in the course environment, adapt the configuration and workflow with Cursor, and evaluate the output trajectory against reference data.

## 2. Environment
- Host OS: Windows 11
- WSL: WSL2
- Linux: Ubuntu 22.04
- Container runtime: Docker Desktop
- Development tool: Cursor
- Course image: `liangyu99/orbslam3_ros1:latest`

## 3. Repository Structure
```text
.
├── README.md
├── docs/
├── figures/
├── output/
├── scripts/
└── results/
```

## 4. Dataset and Calibration
### Dataset
- Dataset name: `[fill in dataset name]`
- Sequence used: `[fill in sequence name]`
- Data source: `[fill in source / bag / extracted images]`

### Calibration / YAML
- YAML file used: `[fill in yaml file name]`
- Scene type: `[airport / island / other]`
- Reason for choosing this YAML: `[brief reason]`

## 5. Setup
### 5.1 Clone the repository
```bash
git clone https://github.com/Qian9921/AAE5303_assignment2_orbslam3_demo-.git
cd AAE5303_assignment2_orbslam3_demo-
```

### 5.2 Pull the course Docker image
```bash
docker pull liangyu99/orbslam3_ros1:latest
```

### 5.3 Start the container
```bash
docker run -it --name aae5303_orbslam3 liangyu99/orbslam3_ros1:latest bash
```

## 6. Workflow
### Step 1: Prepare input data
- Extract images from ROS bag or prepare the image sequence.
- Confirm the timestamp format and image folder path.

### Step 2: Select calibration file
- Choose the YAML file that matches the dataset scene.
- Verify camera parameters and path settings.

### Step 3: Run ORB-SLAM3 demo
```bash
[fill in actual command here]
```

### Step 4: Save outputs
- Trajectory output path: `[fill in path]`
- Log output path: `[fill in path]`
- Screenshot path: `[fill in path]`

### Step 5: Evaluate results
- Ground truth source: `[fill in source]`
- Evaluation script: `[fill in script / command]`
- Metrics used:
  - ATE RMSE
  - RPE translation drift
  - RPE rotation drift
  - Completeness

## 7. What I Changed
Describe the modifications you made for the assignment.

### Modification 1
- File changed: `[fill in file]`
- Change made: `[describe change]`
- Why: `[reason]`
- Effect: `[observed result]`

### Modification 2
- File changed: `[fill in file]`
- Change made: `[describe change]`
- Why: `[reason]`
- Effect: `[observed result]`

### Optional Modification 3
- File changed: `[fill in file]`
- Change made: `[describe change]`
- Why: `[reason]`
- Effect: `[observed result]`

## 8. Results
### Run Summary
- Tracking status: `[successful / unstable / failed partially]`
- Tested sequence: `[fill in sequence]`
- YAML used: `[fill in yaml]`

### Quantitative Results
| Metric | Value |
|---|---:|
| ATE RMSE | `[fill in]` |
| RPE Translation Drift | `[fill in]` |
| RPE Rotation Drift | `[fill in]` |
| Completeness | `[fill in]` |

### Qualitative Results
- Screenshot 1: `[insert or link]`
- Screenshot 2: `[insert or link]`
- Demo video: `[insert or link]`

## 9. Problems Encountered
### Problem 1
- Issue: `[describe issue]`
- Cause: `[possible cause]`
- Fix: `[what you did]`

### Problem 2
- Issue: `[describe issue]`
- Cause: `[possible cause]`
- Fix: `[what you did]`

## 10. Reproducibility
To reproduce this result:
1. Prepare the same Docker environment.
2. Use the same dataset and YAML file.
3. Run the commands in Section 6.
4. Compare the generated trajectory with the reference output.

## 11. Collaborators / Submission Notes
- Repository visibility: `[private / public]`
- Added collaborators required by the course: `[fill in names]`
- Submission date: `[fill in date]`

## 12. Acknowledgment
This project is based on the ORB-SLAM3 course demo environment and adapted for AAE5303 Assignment 2.

