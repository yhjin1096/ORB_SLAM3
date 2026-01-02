#!/bin/bash
cd /root/evo/test
evo_traj tum /root/ORB_SLAM3/results/05_tum.txt /root/ORB_SLAM3/results/KFTrajectory_no_scale.txt /root/ORB_SLAM3/results/KFTrajectory_scale.txt --ref=/root/ORB_SLAM3/results/05_tum.txt -p --plot_mode=xz