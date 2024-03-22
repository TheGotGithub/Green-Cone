import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
# df = pd.read_csv('D:\greenCone\Green-Cone\experimentO2Sensor\O2_DataLog_Caribate.csv')
df = pd.read_csv('D:\greenCone\Green-Cone\experimentO2Sensor\Data\O2_DataLog_Caribate.csv')
# Plotting
plt.plot(df['time'], df['val_1'], label='Value 1')
plt.plot(df['time'], df['val_2'], label='Value 2')

# Add labels and title
plt.xlabel('Time')
plt.ylabel('Values')
plt.title('Values Over Time')

# Add legend
plt.legend()

# Show plot
plt.show()
