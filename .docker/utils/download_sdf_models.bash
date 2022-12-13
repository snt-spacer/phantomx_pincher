#!/usr/bin/env bash

MODELS=(
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_surface4"
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_rock0"
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_rock1"
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_rock2"
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_rock3"
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_rock4"
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_rock5"
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_rock6"
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_rock7"
    "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/lunar_rock8"
    "https://fuel.gazebosim.org/1.0/Gambit/models/Strawberry"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/VisitorChair"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Mars Rover"
)

N_PARALLEL_DOWNLOADS=$(($(nproc) < 16 ? $(nproc) : 16))
for model in "${MODELS[@]}"; do
    echo "Info: Downloading model '${model}'."
    ign fuel download -t model -u "${model}" &
    while (($(jobs -p | wc -l | tr -d 0) >= "${N_PARALLEL_DOWNLOADS}")); do
        wait -n
    done
done
for job in $(jobs -p); do
    wait "${job}"
done

echo "Info: All models were downloaded."
