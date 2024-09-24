# A3EM
Apollo4 Firmware for the A3EM Project


## Configuration Options

* Deployment Options:
    + Device Label
    + Timezone
    + Start Date and Time
    + End Date and Time (optional)
    + Set RTC to Start Date/Time at Magnet Detect (yes/no)
    + LEDs enabled (yes/no)
    + If LEDs enabled:
        - For how many seconds after deployment starts
    + VHF Radio Start Date and Time
* Audio Recording Options:
    + Mode: Threshold-Triggered, Time-Scheduled, Interval-Scheduled, Continuous
    + If Threshold-Triggered:
        - Max num events per unit time
        - Threshold level (right now in percent of max, but prob should map dB to percent)
    + If Time-Scheduled:
        - List of start/end times (i.e., 6AM-11AM, 6PM-10PM)
    + If Interval-Scheduled:
        - Record every X time units
    + Clip length in seconds (even for continuous, should segment into clips)
    + Extend clip length if continuous audio detected (yes/no)
    + Audio sampling rate
    + Microphone amplification level (low, medium, high)
* IMU Recording Options:
    + Mode: Threshold-Triggered, Audio-Triggered (store IMU data whenever audio clip is recording)
    + If Threshold-Triggered:
        - Threshold level
    + Degrees of Freedom (only 3 available on current chip)
    + IMU sampling rate
