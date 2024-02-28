
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

data = pd.read_csv('/home/tanakon/FRA532EXAM_31_62_WS/university_records.csv')

x1 = np.array(data['odom_x'])
y1 = np.array(data['odom_y'])
x2 = np.array(data['odom_filter_x'])
y2 = np.array(data['odom_filter_y'])

plt.plot(x1, y1, marker='o', linestyle='', label='Dataset 1')
plt.plot(x2, y2, marker='x', linestyle='', label='Dataset 2')

plt.xlabel('X Axis Label')
plt.ylabel('Y Axis Label')
plt.title('Multiple Datasets Plot')
plt.legend()  # Show legend

plt.grid(True)

plt.show()