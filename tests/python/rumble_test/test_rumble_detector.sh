#!/bin/bash
source $HOME/venv/bin/activate

data_path="$HOME/Desktop/a3em/elephant-rumble-training-data/Clips"
sample_rate=4000
thread_count=12

python test_rumble_detector.py -d $data_path -tr $thread_count -s $sample_rate

deactivate
