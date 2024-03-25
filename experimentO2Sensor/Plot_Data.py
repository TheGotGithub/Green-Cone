import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
# df = pd.read_csv('D:\greenCone\Green-Cone\experimentO2Sensor\O2_DataLog_Caribate.csv')
df = pd.read_csv('D:\greenCone\Green-Cone\experimentO2Sensor\Data\Ex7.csv')
# Plotting
plt.plot(df['time'], df['val_1'], label='SEN0465')
plt.plot(df['time'], df['val_2'], label='22J23D1')
print(df['val_1'])
# Add labels and title
plt.xlabel('Time')
plt.ylabel('Values')
plt.title('Values Over Time')

# Add legend
plt.legend()

# Show plot
plt.show()
