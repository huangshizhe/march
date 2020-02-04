#!/usr/bin/env bash

# This script should be run from the root of the workspace.
# Also this script only collects coverage from python packages.
packages=(
    march_data_collector
    march_gain_scheduling
    march_gait_selection
    march_shared_classes
    march_state_machine
)

paths=()
for p in "${packages[@]}"
do
    path="build/$p/.coverage"
    [ -f "$path" ] && paths+=("$path")
done

if [ ${#paths[@]} -gt 0 ]
then
    coverage combine --rcfile src/march/.coveragerc "${paths[@]}"
    coverage xml --rcfile src/march/.coveragerc
else
    echo "No coverage files found"
fi
