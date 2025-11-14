# relocalization_toolbox

**A passive 2-D global relocalization framework for LiDAR-only mobile robots**  
Developed on **Ubuntu 20.04** with **ROS Noetic**.

---

## Overview

This work addresses the Kidnapped Robot Problem (KRP), a core localization challenge of relocalizing a robot in a known map without prior pose estimate when localization loss or at SLAM initialization. For this purpose, a passive 2-D global relocalization framework is proposed. It estimates the global pose efficiently and reliably from a single LiDAR scan and an occupancy grid map while the robot remains stationary, thereby enhancing the long-term autonomy of mobile robots. The proposed framework casts global relocalization as a non-convex problem and solves it via the multi-hypothesis scheme with batched multi-stage inference and early termination, balancing completeness and efficiency. The Rapidly-exploring Random Tree (RRT), under traversability constraints, asymptotically covers the reachable space to generate sparse, uniformly distributed feasible positional hypotheses, fundamentally reducing the sampling space. The hypotheses are preliminarily ordered by the proposed Scan Mean Absolute Difference (SMAD), a coarse beam-error level metric that facilitates the early termination by prioritizing high-likelihood candidates. The SMAD computation is optimized for non-panoramic scans. The Translation-Affinity Scan-to-Map Alignment Metric (TAM) is proposed for reliable orientation selection at hypothesized positions and accurate final pose evaluation to mitigate degradation in conventional likelihood-field metrics under translational uncertainty induced by sparse hypotheses, as well as non-panoramic LiDAR scan and environmental changes. Real-world experiments on a resource-constrained mobile robot with non-panoramic LiDAR scans show that the proposed framework achieves competitive performance in both global relocalization success rate and computational efficiency.

---

## Citation

If you use this code in your research, please cite the associated publication.

M. Zhang, L. Ma, Y. Wu, K. Shen, D. Huang, and H. Leung, “Tackling the Kidnapped Robot Problem via Sparse Feasible Hypothesis Sampling and Reliable Batched Multi-Stage Inference,” arXiv preprint arXiv:2511.01219, 2025.
