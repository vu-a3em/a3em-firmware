import csv

possible_frequencies = ['1', '5', '10', '20', '40', '60', '80', '100', '120', '140', '160', '180', '200', '220', '240', '260']
possible_thresholds = ['', '0.0', '0.01', '0.02', '0.04', '0.06', '0.08', '0.1']

frequency_bins = {}
for freq in possible_frequencies:
  frequency_bins[freq] = []

with open('rumble_detector_test.csv', 'r', newline='') as initial_file:
  reader = csv.DictReader(initial_file)
  for row in reader:
    print(row)
    frequency_bins[row['Minimum Frequency']].append(row['Percent Detection'] + '%')

formated_data = [possible_thresholds]
for key, value in frequency_bins.items():
  row = [key]
  row += value
  formated_data.append(row)

with open('formated_data.csv', 'w', newline='') as csvfile:
  csv_writer = csv.writer(csvfile)
  csv_writer.writerows(formated_data)
