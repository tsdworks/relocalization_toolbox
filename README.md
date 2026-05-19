# relocalization_toolbox

**A passive 2-D global relocalization framework for LiDAR-only mobile robots**  
Developed on **Ubuntu 20.04** with **ROS Noetic**.

---

## Overview

This paper addresses the Kidnapped Robot Problem (KRP), a core localization challenge of relocalizing a robot in a known map without a prior pose estimate upon localization loss or at SLAM initialization. For this purpose, a passive 2-D global relocalization framework is proposed. It estimates the global pose efficiently and reliably from a single LiDAR scan and an occupancy grid map while the robot remains stationary, thereby enhancing the long-term autonomy of mobile robots. The proposed framework casts global relocalization as a non-convex problem and solves it using a multi-hypothesis scheme with batched multi-stage inference and early termination, balancing completeness and efficiency. The traversability-constrained Rapidly-exploring Random Tree (RRT) asymptotically covers the reachable space and restricts the search space to traversable regions in the occupancy grid, generating sparse and uniformly distributed feasible positional hypotheses. The hypotheses are preliminarily ordered by the proposed Scan Mean Absolute Difference (SMAD), a coarse beam-error-level metric that enables efficient early termination by prioritizing high-likelihood candidates and is optimized for limited scan measurements. The Translation-Affinity Scan-to-Map Alignment Metric (TAM) is introduced for reliable orientation selection and accurate final pose evaluation, mitigating the degradation of conventional likelihood-field-based metrics under translational uncertainty, non-panoramic LiDAR scans, and environmental changes. Real-world experiments on a resource-constrained mobile robot with non-panoramic LiDAR scans in two representative indoor scenarios show that the proposed framework improves the mean relocalization success rate by about **26.9 percentage points** over the strongest baseline, while reducing runtime from several seconds or even tens of seconds to the sub-second to **1.6 s** range in most comparable cases. These results demonstrate that the proposed method achieves higher success rates with substantially lower runtime under the same time constraint, validating its effectiveness as a practical relocalization module for laser SLAM.

---

## Citation

If you use this code in your research, please cite the associated publication.

M. Zhang, L. Ma, Y. Wu, K. Shen, D. Huang, and H. Leung, “Tackling the kidnapped robot problem via sparse feasible hypothesis sampling and reliable batched multi-stage inference,” IEEE Transactions on Instrumentation and Measurement, doi: 10.1109/TIM.2026.3694741.
