# Rumble Detector Tests
All of the experiments run in this folder are run using Jesse's elephant rumble training data.

The experiments as well as scripts used to run them can be found in `src/` and `scripts/` respectively. The results of the experiments can be found in `experiment-results/`.

Asside from `src/test_rumble_detector.py`, which uses a wider set of minimum frequencies and thresholds, all of the experiments use a minimum frequency of 1 hz and a threshold of 1%.

### Rumble Detector Test
This experiment tracks the percent of clips that were correctly marks as containing sound using various parameters.
* Minimum Frequency: 0.1 hz, 0.2 hz, 0.4 hz, 0.8 hz, 1 hz, 2 hz, 3 hz, 4 hz, 5 hz
* Thesholds: 1%, 2%, 3%, 4%, 5%, 10%

The results can be found at `experiment-results/low-frequency-experiment.csv`. Note that the "y-axis" is frequency and the "x-axis" is threshold

### Maximum Amplitude
Two experiments were run collecting the maximum amplitude of clips being tested. One collected the max amplitude for all files and the other collects the max amplitude for all files that would have been discarded.