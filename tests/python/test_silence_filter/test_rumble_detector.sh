#!/bin/bash
source $HOME/venv/bin/activate

data_path="$HOME/Desktop/a3em/elephant-rumble-training-data/Clips"
thread_count=8

python test_rumble_detector.py -d $data_path -tr $thread_count

deactivate
