#!/bin/bash
source /opt/ros/humble/setup.bash
# Source the hnurm_interfaces workspace
if [ -f "$HOME/yuelu/install/setup.bash" ]; then
    source $HOME/yuelu/install/setup.bash
fi
cargo run --release