#!/bin/bash
source $HOME/venv/bin/activate

data_path="$HOME/Desktop/a3em/elephant-rumble-training-data/Clips"
sample_rate=4000
thread_count=12
file_path=$HOME/Desktop/a3em/a3em-firmware/tests/python/rumble_test/src/maximum_amplitude.py

python $file_path -d $data_path -tr $thread_count -s $sample_rate

deactivate
