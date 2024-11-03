import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev

# Function to interpolate points using spline
def interpolate_points_spline(x, y, num_points=2000, smoothing_factor=1):
    tck, u = splprep([x, y], s=smoothing_factor)
    u_new = np.linspace(0, 1, num_points)
    x_interp, y_interp = splev(u_new, tck)
    return x_interp, y_interp

# Load the provided CSV files
outer_track_path = 'csv_files/left_lane_bound.csv'
inner_track_path = 'csv_files/right_lane_bound.csv'
# center_track_path = 'csv_files/center_lane_line.csv'

# Defining column names
column_names = ['x', 'y']

# Re-load the CSV files with column names
outer_track_df = pd.read_csv(outer_track_path, names=column_names, header=0)
inner_track_df = pd.read_csv(inner_track_path, names=column_names, header=0)
# center_track_df = pd.read_csv(center_track_path, names=column_names, header=0)

# Interpolate points for outer and inner tracks
outer_x_interp, outer_y_interp = interpolate_points_spline(outer_track_df['x'], outer_track_df['y'], 400, 10)
inner_x_interp, inner_y_interp = interpolate_points_spline(inner_track_df['x'], inner_track_df['y'], 400, 10)

# plot the track
plt.plot(outer_x_interp, outer_y_interp, 'ro', label='Outer Track',markersize=2)
plt.plot(inner_x_interp, inner_y_interp, 'bo', label='Inner Track',markersize=2)

# Save interpolated outer track points to CSV
output_outer_path = '~/dwa_ws/src/csv_files/outer_track_interpolated.csv'
output_data = pd.DataFrame({'x': outer_x_interp, 'y': outer_y_interp})
# output_data = pd.DataFrame({'x': outer_x_interp, 'y': outer_y_interp, 'z': outer_track_df['z']})
output_data.to_csv(output_outer_path, index=False)
# Save interpolated outer track points to CSV
output_inner_path = '~/dwa_ws/src/csv_files/inner_track_interpolated.csv'
output_data = pd.DataFrame({'x': inner_x_interp, 'y': inner_y_interp})
# output_data = pd.DataFrame({'x': inner_x_interp, 'y': inner_y_interp, 'z': inner_track_df['z']})
output_data.to_csv(output_inner_path, index=False)

plt.show()