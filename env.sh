#!/usr/bin/env bash
# One place for the environment of the new workspace.  CUDA_PATH points at the pip-installed toolkit
# wheels, which is what lets CuPy compile kernels in WSL without a system CUDA install.
export LIO_AVOID_ROOT=/home/fanxin/lio_avoid
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CUDA_PATH="$HOME/.local/lib/python3.10/site-packages/nvidia"
export PATH="$HOME/.local/bin:$PATH"
# GPU selection: the laptop has one GPU; keep CuPy on it and let RViz share.
export CUDA_VISIBLE_DEVICES=0
