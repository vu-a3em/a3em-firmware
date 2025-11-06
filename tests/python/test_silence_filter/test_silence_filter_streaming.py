import argparse, os
import numpy as np
import cmsisdsp as dsp
from scipy import signal
from scipy.io import wavfile
from pydub import AudioSegment

def read_wav_file(file, sample_rate):
  fs, data = wavfile.read(file)
  if data.dtype != np.int16:
    raise ValueError('Only 16-bit WAV files are supported')
  data = dsp.arm_q15_to_float(data)
  fft_length = 1024 if sample_rate <= 8000 else (2048 if sample_rate <= 24000 else 4096)
  return fft_length, signal.resample(data, int(len(data) * sample_rate / fs))

# Parse the expected sample rate and min and max frequencies of interest
parser = argparse.ArgumentParser(prog='test_silence_filter', description='Tests silence filtering on a directory of WAV files', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('-s', '--sample-rate', help='Target audio sample rate', default=8000, type=int)
parser.add_argument('-f1', '--min-freq', help='Minimum frequency of interest', default=250, type=int)
parser.add_argument('-f2', '--max-freq', help='Maximum frequency of interest', default=3800, type=int)
parser.add_argument('-t', '--threshold', help='Silence threshold', default=0.02, type=float)
parser.add_argument('-l', '--clip-length', help='Output clip length in seconds', default=60.0, type=float)
parser.add_argument('-p', '--path', help='Input WAV directory path', required=True)
parser.add_argument('-o', '--output-path', help='Output WAV directory path', required=True)
args = parser.parse_args()

# Read a random WAV file and determine the ideal FFT length (assume all files have the same sample rate)
fft_length = None
for root, dirs, files in os.walk(args.path):
  if fft_length is not None:
    break
  for file in files:
    if file.lower().endswith('.wav'):
      file_path = os.path.join(root, file)
      fft_length, audio = read_wav_file(file_path, args.sample_rate)
      break

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

# Recursively iterate through all WAV files in the input directory
output_clip_ms_remaining, output_clip = 0, None
for root, dirs, files in os.walk(args.path):

  # Ensure that the directories and files are processed in alphabetical (time-relative) order
  dirs.sort()
  files.sort()

  # Iterate through all files in the current directory
  for file in files:
    if file.lower().endswith('.wav'):
      file_path = os.path.join(root, file)

      # Read the WAV file and determine the total number of FFT frames
      processing_frame_start = 0
      _, audio = read_wav_file(file_path, args.sample_rate)
      num_fft_frames = 1 + ((len(audio) - fft_length) // fft_length)

      # Check if we need to finish writing an already-started output clip
      if output_clip is not None:

        # Determine how to segment the input file to end after the specified clip length
        segment = AudioSegment.from_file(file_path)
        end_ms = output_clip_ms_remaining if output_clip_ms_remaining < len(segment) else len(segment)
        output_clip = output_clip.append(segment[:end_ms], crossfade=0)
        output_clip_ms_remaining -= end_ms

        # Check if we have finished writing the clip
        if output_clip_ms_remaining <= 0:
          output_dir = os.path.join(args.output_path, os.path.relpath(root, args.path))
          os.makedirs(output_dir, exist_ok=True)
          output_file = os.path.join(output_dir, file)
          output_clip.export(output_file, format='wav')
          processing_frame_start = int(num_fft_frames * (end_ms / len(segment)))
          output_clip_ms_remaining = 0
          output_clip = None
        else:
          continue  # Skip processing if we are still writing

      # Iterate through all FFT windows
      silence = [False for _ in range(num_fft_frames)]
      amplitudes = np.empty(num_fft_frames, dtype=np.float32)
      spectrogram = np.empty((num_fft_frames, 1 + max_bin - min_bin), dtype=np.float32)
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

      # Iterate through all FFT frame amplitudes
      for frame, amplitude in enumerate(amplitudes):

        # Skip frames that have already been written
        if frame < processing_frame_start:
          continue
        
        # Search for the beginning of a non-silent clip
        if not silence[frame]:

          # Determine how to segment the input file to start a new output clip
          segment = AudioSegment.from_file(file_path)
          output_clip_ms_remaining = int(args.clip_length * 1000)
          start_ms = int((frame * fft_length / args.sample_rate) * 1000)
          end_ms = start_ms + output_clip_ms_remaining if start_ms + output_clip_ms_remaining < len(segment) else len(segment)
          output_clip = segment[start_ms:end_ms]
          output_clip_ms_remaining -= (end_ms - start_ms)
          processing_frame_start = frame + int(((end_ms - start_ms) / 1000) * (args.sample_rate / fft_length))
          print(f'Sound Detected: File: {file}, Frame: {frame}, Amplitude: {amplitude:.10f}')

          # Check if the entire clip can be written from the current file
          if output_clip_ms_remaining <= 0:
            output_dir = os.path.join(args.output_path, os.path.relpath(root, args.path))
            os.makedirs(output_dir, exist_ok=True)
            output_file = os.path.join(output_dir, file)
            output_clip.export(output_file, format='wav')
            output_clip_ms_remaining = 0
            output_clip = None
