import argparse
import numpy as np
import cmsisdsp as dsp
import csv
import os
import queue
from scipy.io import wavfile
from scipy import signal
import threading

def read_wav_file(file, sample_rate):
  fs, data = wavfile.read(file)
  if data.dtype != np.int16:
    raise ValueError('Only 16-bit WAV files are supported')
  data = dsp.arm_q15_to_float(data)
  fft_length = 1024 if sample_rate <= 8000 else (2048 if sample_rate <= 24000 else 4096)
  return fft_length, signal.resample(data, int(len(data) * sample_rate / fs))


def process_audio(file_path, parameters):
  # Read the WAV file and determine the ideal FFT length
  fft_length, audio = read_wav_file(file_path, parameters['sampleRate'])

  # Create and initialize an FFT structure
  fft = dsp.arm_rfft_fast_instance_f32()
  dsp.arm_rfft_fast_init_f32(fft, fft_length)

  # Create a Hanning window
  window = np.zeros(fft_length, dtype=np.float32)
  for i in range(fft_length):
    window[i] = 0.5 * (1 - dsp.arm_cos_f32(2 * np.pi * i / fft_length))

  # Locate the minimum and maximum FFT bins of interest
  min_bin, max_bin = 1, 1
  for bin in range(1, (fft_length // 2) - 1):
    center_frequency = bin * parameters['sampleRate'] / fft_length
    if center_frequency + (0.5 * parameters['sampleRate'] / fft_length) < parameters['minFreq']:
      min_bin = max_bin = bin + 1
    if center_frequency - (0.5 * parameters['sampleRate'] / fft_length) < parameters['maxFreq']:
      max_bin = bin

  # Iterate through all FFT windows
  silence = [False for _ in range(1 + (len(audio) - fft_length) // fft_length)]
  amplitudes = np.empty(1 + (len(audio) - fft_length) // fft_length, dtype=np.float32)
  spectrogram = np.empty((1 + (len(audio) - fft_length) // fft_length, 1 + max_bin - min_bin), dtype=np.float32)
  for i in range(0, len(audio) - fft_length, fft_length):

    # Apply the Hanning window and compute the FFT
    input = dsp.arm_mult_f32(audio[i:i + fft_length], window)
    output = dsp.arm_rfft_fast_f32(fft, input, 0)

    # Calculate the spectogram and total amplitude within the frequency bands of interest
    amplitudes[i // fft_length] = 0.0
    for bin in range(min_bin, max_bin + 1):
      real, imag = output[2 * bin], output[2 * bin + 1]
      spectrogram[i // fft_length, bin - min_bin] = (1.0 / fft_length) * np.sqrt(real * real + imag * imag)
      amplitudes[i // fft_length] += (1.0 / fft_length) * np.sqrt(real * real + imag * imag)

    if amplitudes[i // fft_length] < parameters['threshold']:
      silence[i // fft_length] = True

  # store max amplitudes of files that would have been discarded
  frames_with_sound_detection = 0
  for frame, amplitude in enumerate(amplitudes):
    if not silence[frame]:
      frames_with_sound_detection += 1
  if frames_with_sound_detection == 0:
    file_name = file_path.split('/')[-1:][0]
    max_amplitude = max(amplitudes)
    data_queue.put({ 'name': file_name, 'maxAmplitude': max_amplitude })


def create_thread_partitions(data, thread_count):
  partitions = []
  partition_size = len(data) // thread_count
  while thread_count > 0:
    part = data[:partition_size]
    partitions.append(part)
    data = data[partition_size:]
    thread_count -= 1
  if len(data) > 0:
    partitions[thread_count-1] = partitions[thread_count-1] + data
  return partitions


def thread_logic(data, parameters):
  for file_path in data:
    process_audio(file_path, parameters)


def start_threads(partitions, parameters):
  threads = []
  for partition in partitions:
    t = threading.Thread(target=thread_logic, args=(partition,parameters))
    t.start()
    threads.append(t)
  return threads


# Parse the expected sample rate and min and max frequencies of interest
parser = argparse.ArgumentParser(prog='test_silence_filter', description='Tests silence filtering on a WAV file', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-s', '--sample-rate', help='Target audio sample rate', default=8000, type=int)
parser.add_argument('-tr', '--threads', help='Number of threads used for processing', default=1, type=int)
parser.add_argument('-d', '--dir', help='Directory containing the clips of interest', default='$HOME/')
args = parser.parse_args()

# global data type for storing data across threads
data_queue = queue.Queue()
processed_data = []

file_names = os.listdir(args.dir)
file_paths = list(map(lambda x: os.path.join(args.dir, x), file_names))
partitions = create_thread_partitions(file_paths, args.threads)

# Run the algorithm across all configuration possibilities
parameters = {
  'sampleRate': args.sample_rate, 
  'maxFreq': args.sample_rate // 2 - 200, 
  'minFreq': 1,
  'threshold': 0.01
}

threads = start_threads(partitions, parameters)
for t in threads:
  t.join()

data = []
while not data_queue.empty():
  data.append(data_queue.get())

print(len(data))

# store data in csv file
fields = ['name', 'maxAmplitude']
with open('experiment-results/maximum_amplitude_discarded.csv', 'w', newline='') as csvfile:
  csv_writer = csv.DictWriter(csvfile, fieldnames=fields)
  csv_writer.writeheader()
  for row in data:
    csv_writer.writerow(row)
