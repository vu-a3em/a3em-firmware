#!/usr/bin/env python3

import os, signal
import subprocess
import numpy as np
import struct, time
import soundfile

# Define script-local variables
num_channels, sample_rate, num_samples_per_chunk, num_chunks_per_clip = None, None, None, None
rtt_bin_name = 'JLinkRTTLogger.exe' if os.name == 'nt' else 'JLinkRTTLoggerExe'
current_timestring = time.strftime('%Y-%m-%d@%H-%M-%S')
wav_file_prefix = current_timestring + '-clip'
pcm_file_name = current_timestring + '.pcm'

# Generate runtime-specific constants from app_config.h
config_file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'src', 'app', 'static_config.h')
with open(config_file_path, 'r') as config_file:
  for line in config_file:
    if '#define AUDIO_NUM_CHANNELS' in line:
      num_channels = int(line.split()[2])
    elif '#define AUDIO_DEFAULT_SAMPLING_RATE_HZ' in line:
      sample_rate = int(line.split()[2])
    elif '#define AUDIO_DEFAULT_CLIP_LENGTH_SECONDS' in line:
      clip_length_seconds = int(line.split()[2])
bytes_per_sample = 2 * num_channels
num_samples_per_clip = sample_rate * clip_length_seconds
bytes_per_clip = bytes_per_sample * num_samples_per_clip

# Run the RTT Logger utility briefly to clear any buffered data
process = subprocess.Popen([rtt_bin_name, '-Device', 'AMAP42KK-KBR', '-If', 'SWD', '-speed', '4000', pcm_file_name])
time.sleep(2.0)
process.send_signal(signal.SIGINT)
time.sleep(0.5)
print('\n\nNOT ACTUALLY SHUTTING DOWN. Please wait a second...')
process.wait()

# Run the RTT Logger utility until interrupted by Ctrl+C
is_running = True
process = subprocess.Popen([rtt_bin_name, '-Device', 'AMAP42KK-KBR', '-If', 'SWD', '-speed', '4000', pcm_file_name])
while is_running:
  try:
    process.wait()
    is_running = False
  except KeyboardInterrupt:
    print('\nShutting down logger. Please wait...')
    process.send_signal(signal.SIGINT)

# Convert the downloaded PCM data to a WAV file
print('\nConverting PCM data to WAV file... ', end='')
with open(pcm_file_name, 'rb') as pcm_file:
  pcm_data = pcm_file.read()
  if len(pcm_data) % bytes_per_clip:
    pcm_data = pcm_data[:-(len(pcm_data) % bytes_per_clip)]
  audio_data = np.reshape(struct.unpack('<' + 'h'*(len(pcm_data)//2), pcm_data),
                          (-1, num_samples_per_clip, num_channels)).astype('int16', copy=False)
  for i, clip in enumerate(audio_data):
    soundfile.write(wav_file_prefix + str(i) + '.wav', clip, sample_rate)
  os.remove(pcm_file_name)
print('Done')
