import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt

# Read the CSV file containing GPS coordinates
data = pd.read_csv('/media/tasi/InternalDrive1/POST_PROCESSED_DATA/16-11-23_14-44-40/gps/gps_2023-11-16-14-44-41_0/position_data.csv')

# Assuming columns in the CSV are named 'latitude' and 'longitude'
latitude = np.radians(data['latitude'].values)  # Convert to radians
longitude = np.radians(data['longitude'].values)  # Convert to radians

# Constants for Earth's radius and reference points (replace with your values)
R = 6371000  # Earth radius in meters
origin = np.radians([39.13797368787806, -85.92733363110298])  # Reference origin point

ref_y_point = np.radians([39.142940158611545, -85.92111363815388])  # Reference point on the y-axis

# Function to convert GPS coordinates to Local Tangent Plane (LTP) coordinates
def gps_to_ltp(lat, lon, origin_lat_lon, ref_y_lat_lon):
    delta_lon = lon - origin_lat_lon[1]
    x = R * math.cos(origin_lat_lon[0]) * delta_lon
    y = R * (lat - origin_lat_lon[0])
    z = 0  # Assuming 2D transformation on the plane, z is assumed zero
    return x, y, z

# Convert GPS coordinates to Local Tangent Plane (LTP) coordinates
ltp_coordinates = np.array([gps_to_ltp(lat, lon, origin, ref_y_point) for lat, lon in zip(latitude, longitude)])
#Plot the ltp_coordinates on a graph
plt.plot(ltp_coordinates[:,0],ltp_coordinates[:,1])
plt.show()

print(ltp_coordinates)

# Function to calculate angle between successive points and the y-axis
def calculate_heading(current_point, prev_point):
    delta_x = current_point[0] - prev_point[0]
    delta_y = current_point[1] - prev_point[1]
    heading_rad = math.atan2(delta_x, delta_y)  # Calculate angle w.r.t y-axis
    heading_deg = np.degrees(heading_rad)
    return heading_deg

# Calculate heading w.r.t y-axis for successive points
headings = [0.0]
prev_point = ltp_coordinates[0]  # Assuming the first point as the starting point
for point in ltp_coordinates[1:]:
    heading = calculate_heading(point, prev_point)
    headings.append(heading)
    prev_point = point
print(headings)
#plot the headings on a graph
# plt.plot(headings)
# plt.show()


#Make a csv file with headings for the corresponding lat and long
data['heading'] = headings
data.to_csv('position_data_with_headings.csv', index=False)
# headings list now contains the heading angles with respect to the y-axis for successive points

# Rest of the code remains the same as in the previous examples...
