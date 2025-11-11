import argparse
import numpy as np
import cmsisdsp as dsp
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


def process_audio(file_path):
  print(f'processing: {file_path}')

  # Read the WAV file and determine the ideal FFT length
  fft_length, audio = read_wav_file(file_path, args.sample_rate)

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
    center_frequency = bin * args.sample_rate / fft_length
    if center_frequency + (0.5 * args.sample_rate / fft_length) < args.min_freq:
      min_bin = max_bin = bin + 1
    if center_frequency - (0.5 * args.sample_rate / fft_length) < args.max_freq:
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
    if amplitudes[i // fft_length] < args.threshold:
      silence[i // fft_length] = True

  # Output if a sound was detected
  frames_with_sound_detection = 0
  for frame, amplitude in enumerate(amplitudes):
    if not silence[frame]:
      frames_with_sound_detection += 1
    #label = ', Sound Detected' if not silence[frame] else ''
    #print(f'Frame: {frame}, Amplitude: {amplitude:.10f}{label}')
  processed_data.put({ 'name': file_path, 'framesDetected': frames_with_sound_detection })


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


def thread_logic(data):
  for file_path in data:
    process_audio(file_path)


def start_threads(partitions):
  threads = []
  for partition in partitions:
    t = threading.Thread(target=thread_logic, args=(partition,))
    t.start()
    threads.append(t)
  return threads


# Parse the expected sample rate and min and max frequencies of interest
parser = argparse.ArgumentParser(prog='test_silence_filter', description='Tests silence filtering on a WAV file', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-s', '--sample-rate', help='Target audio sample rate', default=8000, type=int)
parser.add_argument('-f1', '--min-freq', help='Minimum frequency of interest', default=250, type=int)
parser.add_argument('-f2', '--max-freq', help='Maximum frequency of interest', default=3800, type=int)
parser.add_argument('-t', '--threshold', help='Silence threshold', default=0.02, type=float)
parser.add_argument('-tr', '--threads', help='Number of threads used for processing', default=1, type=int)
parser.add_argument('-d', '--dir', help='Directory containing the clips of interest', default='$HOME/')
args = parser.parse_args()

file_names = os.listdir(args.dir)
file_paths = list(map(lambda x: os.path.join(args.dir, x), file_names))
partitions = create_thread_partitions(file_paths, args.threads)

processed_data = queue.Queue()
threads = start_threads(partitions)
for t in threads:
  t.join()
data_points = processed_data.qsize()
print(f'data points: {data_points}')

#for file_name in audio_files:
#  file_path = os.path.join(args.dir, file_name)
#  process_audio(file_path)

