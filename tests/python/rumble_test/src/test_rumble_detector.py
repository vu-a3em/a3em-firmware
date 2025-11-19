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

    # Check if the frame is silent
    if amplitudes[i // fft_length] < parameters['threshold']:
      silence[i // fft_length] = True

  # Output if a sound was detected
  frames_with_sound_detection = 0
  for frame, amplitude in enumerate(amplitudes):
    if not silence[frame]:
      frames_with_sound_detection += 1
  data_queue.put({ 'name': file_path, 'framesDetected': frames_with_sound_detection })


def create_configurations(min_frequencies, thresholds):
  configurations = []
  print('creating configurations...')
  for m in min_frequencies:
    for t in thresholds:
      configurations.append({'minFreq': m, 'threshold': t})
  return configurations


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

# Configurations used for testing various sample rates and thresholds
min_frequencies = [10, 20, 30, 40, 50, 100, 200]
thresholds = [0.000, 0.002, 0.004, 0.006, 0.008, 0.01, 0.012, 0.014, 0.016, 0.02]
configurations = create_configurations(min_frequencies, thresholds)

# global data type for storing data across threads
data_queue = queue.Queue()
processed_data = []

file_names = os.listdir(args.dir)
file_paths = list(map(lambda x: os.path.join(args.dir, x), file_names))
partitions = create_thread_partitions(file_paths, args.threads)

# Run the algorithm across all configuration possibilities
parameters = {'sampleRate': args.sample_rate, 'maxFreq': args.sample_rate // 2 - 200}
for conf in configurations:
  min_freq = conf['minFreq']
  threshold = conf['threshold']

  parameters['minFreq'] = min_freq
  parameters['threshold'] = threshold

  print(f'Processing --> minFreq: {min_freq} threshold: {threshold}')

  threads = start_threads(partitions, parameters)
  for t in threads:
    t.join()
  
  data = []
  while not data_queue.empty():
    data.append(data_queue.get())
  processed_data.append({ 'configuration': conf, 'data': data })

# format data for a csv files
csv_data = []
for data_log in processed_data:
  data = data_log['data']
  min_freq = data_log['configuration']['minFreq']
  threshold = data_log['configuration']['threshold']

  points_with_detections = 0
  for point in data:
    if point['framesDetected'] > 0:
      points_with_detections += 1
  percent_detection = round(points_with_detections / len(data) * 100, 2)

  csv_data.append({'Minimum Frequency': min_freq, 'Threshold': threshold, 'Percent Detection': percent_detection})

# store data in a csv file
frequency_bins = {}
for freq in min_frequencies:
  frequency_bins[freq] = []

for row in csv_data:
  print(row)
  frequency_bins[row['Minimum Frequency']].append(str(row['Percent Detection']) + '%')

formated_data = [thresholds]
for key, value in frequency_bins.items():
  row = [key]
  row += value
  formated_data.append(row)

with open('experiment-results/low-frequency-experiment.csv', 'w', newline='') as csvfile:
  csv_writer = csv.writer(csvfile)
  csv_writer.writerows(formated_data)
