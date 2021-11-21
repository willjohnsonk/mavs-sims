# Header placeholder

import math
import sys
import random
import time
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate.ndgriddata import griddata
import seaborn as sns

# Set the path to the mavs python api, mavs.py
# ***must be changed to your system-specific path***
sys.path.append(r'/usr/local/msu-autonomous-vehicle-simulator/src/mavs_python')

# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths

# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# Creating the scene and environment
# eco_file = 'american_southeast_meadow.json'
eco_file = 'american_southeast_forest.json'
scene_name = 'centered_scene'
scene_size = 100.0
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = scene_size 
random_scene.terrain_length = scene_size

random_scene.mesh_resolution= 0.15
random_scene.surface_roughness_type = "variable"

random_scene.lo_mag = 0.0
random_scene.hi_mag = 0.01
random_scene.plant_density = 0.0
random_scene.trail_width = 2.0
random_scene.track_width = 1.6
random_scene.wheelbase = 2.719

random_scene.basename = scene_name
random_scene.eco_file = eco_file
random_scene.path_type = 'Ridges'
random_scene.CreateScene()

env = mavs.MavsEnvironment()
env.SetScene(random_scene.scene)
random_scene.TurnOnLabeling()

env.SetTime(12)
env.SetFog(0)
env.SetTurbidity(1.0)
env.SetAlbedo(1.0)
env.SetCloudCover(0.0)
env.SetWind([0,0])


# Defining a heightmap
x_map = []
y_map = []
elevation_map = []

start = int(-scene_size/2)
end = int(scene_size/2)

for x_v in range(start, end):
    for y_v in range(start, end):
        for y_c in range((y_v*10), ((y_v*10) + 9)):
            for x_c in range((x_v*10), ((x_v*10) + 9)):
                x_map.append((x_c/10))
                y_map.append((y_c/10))
                elevation_map.append(random_scene.GetSurfaceHeight((x_c/10), (y_c/10)))




# Initializing sensors
overheadCam = mavs.MavsCamera()
overheadCam.Initialize(1080, 1080, 0.0054272, 0.0054272, 0.004)
overheadCam.SetOffset([0.0, 0.0, 36.0],[0.7071068, 0, 0.7071068, 0])
overheadCam.RenderShadows(True)
overheadCam.SetAntiAliasingFactor(2)
overheadCam.SetEnvironmentProperties(env.obj)

lidar_angle = (math.pi/180)*0

frontCam = mavs.MavsCamera()
frontCam.Initialize(1080, 1080, 0.0054272, 0.0054272, 0.004)
frontCam.SetOffset([0.0, 0.0, 3.0],[0.0, 0, 0, 0.0])
frontCam.SetAntiAliasingFactor(1)
frontCam.SetEnvironmentProperties(env.obj)

lidar = mavs.MavsLidar('os1')
lidar.SetOffset([0.0, 0.0, 3.0],[math.cos(0.5*lidar_angle),0.0,math.sin(0.5*lidar_angle),0.0])

overheadCam.SetPose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
overheadCam.Update(env, 0.03)
overheadCam.AnnotateFrame(env)
# overheadCam.Display()
overheadCam.SaveCameraImage("overhead.bmp")
overheadCam.SaveAnnotation(env, "overhead_annotated")

frontCam.SetPose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
frontCam.Update(env, 0.03)
# frontCam.Display()
frontCam.AnnotateFrame(env)
frontCam.SaveCameraImage("front.bmp")

lidar.SetPose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
lidar.Update(env, 0.03)
# lidar.Display()
lidar.AnnotateFrame(env)
lidar.SaveLabeledPcd("lidar_view.pcd")


rms_arr = []
heights = open("height_map.txt", "w+")
x = []
y = []
elevation = []

for y_vals in range(-5, 5):
    for x_vals in range(0, 30):
        z_array = []
        # need to break the elevation into n points for each (m x m) cell
        # so that a local average & rms can be found for each 1m x 1m voxel to compare
        for y_cell in range((y_vals*10), ((y_vals*10) + 9)):
            for x_cell in range((x_vals*10), ((x_vals*10) + 9)):
                z_array.append(random_scene.GetSurfaceHeight((x_cell/10), (y_cell/10)))
                heights.write(str(x_cell/10) + " " + str(y_cell/10) + " " + str(random_scene.GetSurfaceHeight((x_cell/10), (y_cell/10))) + "\n")
                x.append((x_cell/10))
                y.append((y_cell/10))
                elevation.append(random_scene.GetSurfaceHeight((x_cell/10), (y_cell/10)))

        if len(z_array) > 0:
            # take the average z value of the array
            avg = sum(z_array) / len(z_array)
            # subtract avg z from each z in array and square it
            for k in range(len(z_array)):
                z_array[k] = (z_array[k] - avg) ** 2
            # sum all these values and take the sqrt
            roughness = math.sqrt(sum(z_array) / len(z_array))
            # append the roughness value to the roughness array at the proper position
            rms_arr.append(roughness)
        else:
            rms_arr.append(0)

heights.close()
print("rms_arr len: ", len(rms_arr))
ra = np.empty([10,30], dtype=float)
ra.fill(0.0)

itr = 0
for m in range(-5,5):
    for n in range(0,30):
        ra[m][n] = np.format_float_positional(rms_arr[itr], precision=7)
        itr = itr + 1
        if itr > 300:
            break

#print(ra)

ra = np.flipud(ra)
hmap = sns.heatmap(ra)
hmap.set_title('Seaborn Lidar RMS Truth')
plt.show()





# # Elevation map for the full region
# x_map = np.array(x_map)
# y_map = np.array(y_map)
# elevation_map = np.array(elevation_map)

# # Plotting the heightmap
# xj = np.linspace(min(x_map), max(x_map))
# yj = np.linspace(min(y_map), max(y_map))
# x2, y2 = np.meshgrid(xj,yj)
# z_map = griddata((x_map,y_map), elevation_map, (x2,y2), method='linear')

# fig_map, ax=plt.subplots(1,1)

# #map = plt.contour(x1, y1, z)
# map_full = plt.contourf(x2, y2, z_map)

# fig_map.colorbar(map_full)
# ax.set_title('Full Elevation Map')

# plt.show()


# Elevation map for the filtered region
x = np.array(x)
y = np.array(y)
elevation = np.array(elevation)

# Plotting the heightmap
xi = np.linspace(min(x), max(x))
yi = np.linspace(min(y), max(y))
x1, y1 = np.meshgrid(xi,yi)
z = griddata((x,y), elevation, (x1,y1), method='linear')

fig, ax=plt.subplots(1,1)

#map = plt.contour(x1, y1, z)
map = plt.contourf(x1, y1, z)

fig.colorbar(map)
ax.set_title('Filtered Elevation Map')

plt.show()


# testHeight = random_scene.GetSurfaceHeight(0.125, 0.125)
# print("Surface height at (0.125, 0.125) " + str(testHeight))
# sys.stdout.flush()

# # Get the density grid
# # Arguments are : 
# # Lower left corner in ENU
# # Upper right corner in ENU
# # grid resolution in meters
# density_grid = env.GetVegDensityOnAGrid([-0.5*random_scene.terrain_width,-0.5*random_scene.terrain_length,0.0],
#                                         [0.5*random_scene.terrain_width,0.5*random_scene.terrain_width,20],0.5)

# # convert the python list to a numpy array
# veg_dens = np.array(density_grid)

# # get the shape of the numpy array
# nx,ny,nz = veg_dens.shape

# # plot the vegetation density slices, one by one
# for k in range(nz):
#     slice = veg_dens[:,:,k]
#     snx = len(slice)
#     sny = len(slice[0])
#     plt.xlim((nx,0))
#     plt.margins(0,0)
#     plt.axis('off')
#     plt.imshow(slice,interpolation='none')
#     plt.savefig('slice_'+str(k)+'.png',bbox_inches='tight')