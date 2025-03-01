import bagpy
import matplotlib.pyplot as plt
from bagpy import bagreader
import pandas as pd
from scipy.signal import butter, filtfilt

# Define Butterworth low-pass filter function
def butter_lowpass(cutoff, fs, order=4):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

# Apply Butterworth filter to data
def butter_filter(data, cutoff, fs, order=4):
    b, a = butter_lowpass(cutoff, fs, order)
    return filtfilt(b, a, data)

# Read the bag file
b = bagreader('DATA_ANALYSIS/2025-02-28-20-17-07.bag')

# Check the available topics in the bag file
print(b.topic_table)

# Extract messages from topics (replace '/voltage_pressure_data' and '/voltage_pressure_data_2' with actual topic names from your bag file)
df_topic1 = pd.read_csv(b.message_by_topic('/voltage_pressure_data'))
df_topic2 = pd.read_csv(b.message_by_topic('/voltage_pressure_data_2'))

# Check the type and content of df_topic1 and df_topic2
print(type(df_topic1))
print(df_topic1)

# Assuming both topics have 'Time' and 'data' columns
time = df_topic1['Time']
data_topic1 = df_topic1['data']
data_topic2 = df_topic2['data']

# Sampling frequency (based on your time data)
fs = 1 / (time[1] - time[0])  # Calculate sampling frequency (inverse of time difference)
cutoff = 0.1  # Cutoff frequency for the low-pass filter (adjust this value as needed)

# Apply Butterworth filter to both datasets
filtered_data_topic1 = butter_filter(data_topic1, cutoff, fs)
filtered_data_topic2 = butter_filter(data_topic2, cutoff, fs)

# Plotting the original and filtered data
plt.figure(figsize=(10, 5))

# Plot original data for topic 1 and filtered data for topic 1
plt.plot(time - time[0], data_topic1, label='Original Topic 1', color='b', alpha=0.5)
plt.plot(time - time[0], filtered_data_topic1, label='Filtered Topic 1', color='b')

# Plot original data for topic 2 and filtered data for topic 2
plt.plot(time - time[0], data_topic2, label='Original Topic 2', color='r', alpha=0.5)
plt.plot(time - time[0], filtered_data_topic2, label='Filtered Topic 2', color='r')

# Customize the plot
plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.title('Comparison of Original and Filtered Data from .bag file')
plt.legend()

# Show the plot
plt.show()
