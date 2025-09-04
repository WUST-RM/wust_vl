#!/bin/bash
export MVCAM_SDK_PATH=/opt/MVS
export MVCAM_COMMON_RUNENV=/opt/MVS/lib
export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
export ALLUSERSPROFILE=/opt/MVS/MVFG

export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/hy/TensorRT-10.6.0.26/lib:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-12.6
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/hy/onnxruntime-linux-x64-gpu-1.22.0/lib:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
blue="\033[1;34m"
yellow="\033[1;33m"
reset="\033[0m"
red="\033[1;31m"
echo -e "${blue}<--- 已加载环境 --->${reset}"
sudo ldconfig