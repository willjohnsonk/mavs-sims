# Creates a random off-road scene based on given parameters.
# The scene may be user-controlled or waypoint-controlled
# Should be run with "python3 data_exec.py" for Python3 or "python data_exec.py"
# MAVS:     Dr. Goodin      (cgoodin@cavs.msstate.edu)
# Scripts:  Peyton Johnson  (wj311@cavs.msstate.edu)

import math
import sys
import random
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

# Scene generation
# List of valid eco files to be picked from at random
# eco_files = ['american_southeast_forest_obstacles.json', 'forest_with_cubes.json', 'american_pine_forest.json', 
# 'american_southeast_meadow.json', "american_southeast_forest.json", 'american_southwest_desert.json', 
# 'american_southeast_forest_brushy.json', 'country_road.json']

# Randomly selects one of the above eco files, prints info on sim settings
#    Note: some scenes may not work /load with certain parameters
eco_file = 'american_southeast_forest.json'

scene_name = 'vis_rough'
scene_size = 100.0
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = scene_size 
random_scene.terrain_length = scene_size

random_scene.mesh_resolution= 0.3 # wheel is ~0.33 m radius for mrzr
random_scene.surface_roughness_type = "variable"

random_scene.lo_mag = 0.0
random_scene.hi_mag = 0.015
random_scene.plant_density = 0.05
random_scene.trail_width = 2.0
random_scene.track_width = 1.6
random_scene.wheelbase = 2.719

# for potholes in range(0, 15):
#     random_scene.AddPotholeAt(random.randrange(-50, 50), random.randrange(-50, 50), 0.5, 1.0)

# random_scene.num_potholes = (potholes)

random_scene.basename = scene_name
random_scene.eco_file = eco_file
random_scene.path_type = 'Ridges'
random_scene.CreateScene()

# x = []
# y = []
# elevation = []

# heights = open("height_map.txt", "w+")

# start = int(-scene_size/2)
# end = int(scene_size/2)

# for x_vals in range(start, end):
#     for y_vals in range(start, end):
#         heights.write(str(x_vals) + " " + str(y_vals) + " " + str(random_scene.GetSurfaceHeight(x_vals, y_vals)) + "\n")
#         x.append(x_vals)
#         y.append(y_vals)
#         elevation.append(random_scene.GetSurfaceHeight(x_vals, y_vals))

# heights.close()

# x = np.array(x)
# y = np.array(y)
# elevation = np.array(elevation)





rms_arr = []

for y_vals in range(-5, 5):
    for x_vals in range(0, 30):
        z_array = []
        # need to break the elevation into n points for each (m x m) cell
        # so that a local average & rms can be found for each 1m x 1m voxel to compare
        for y_cell in range((y_vals*100), ((y_vals*100) + 99)):
            for x_cell in range((x_vals*100), ((x_vals*100) + 99)):
                z_array.append(random_scene.GetSurfaceHeight((x_cell/100), (y_cell/100)))
                #print(("Z: ", random_scene.GetSurfaceHeight((x_cell/100), (y_cell/100))))

        if len(z_array) > 0:
            # take the average z value of the array
            avg = sum(z_array) / len(z_array)
            # subtract avg z from each z in array and square it
            for i in range(len(z_array)):
                z_array[i] = (z_array[i] - avg) ** 2
            # sum all these values and take the sqrt
            roughness = math.sqrt(sum(z_array))
            # append the roughness value to the roughness array at the proper position
            rms_arr.append(roughness)
            #print("RMS: ", roughness)
        else:
            rms_arr.append(0)

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

print(ra)

ra = np.flipud(ra)
hmap = sns.heatmap(ra)
hmap.set_title('Seaborn Lidar RMS Truth')
plt.show()





# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(random_scene.scene)
random_scene.TurnOnLabeling()

# Consider using log normal distributions using python random for more realistic env
# Sets the environmental settings to randomly-chosen intensitie
#    Note: Certain value combinations may cause the simulation to not function properly
env.SetTime(12)
env.SetFog(0)
env.SetTurbidity(1.0)
env.SetAlbedo(1.0)
env.SetCloudCover(0.0)
env.SetWind([0,0])

# Load a MAVS vehicle
# There are a few vehicles to choose from, check the data/vehicles/rp3d_vehicles for a list
veh = mavs.MavsRp3d()
veh_file = 'mrzr4.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(0.0, 0.0, 0.0)   # in global ENU
veh.SetInitialHeading(0.785)              # in radians, 0.785 ~= 45 deg (direction of trail)
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)


# Create a MAVS camera and set its properties
cam = mavs.MavsCamera()
cam.Model('XCD-V60')
cam.SetAntiAliasingFactor(5)  # AA significantly hurts performance
cam.SetGammaAndGain(0.85,1.0) 
cam_angle = (math.pi/180.0)*0 # 0 degrees -> straight ahead
cam.SetOffset([0.7, 0.0, 0.75],[math.cos(0.5*cam_angle),0.0,math.sin(0.5*cam_angle),0.0])

# Create a MAVS lidar and set its properties
lidar = mavs.MavsLidar('OS1')
lidar_angle = (math.pi/180)*0
lidar.SetOffset([0.0, 0.0, 1.9],[math.cos(0.5*lidar_angle),0.0,math.sin(0.5*lidar_angle),0.0])

# Load the waypoints that go with this scene
waypoints = mavs.MavsWaypoints()
waypoints.Load('./'+random_scene.basename+'_path.vprp')

# Get the scene geometry and put the waypoints on the ground
scene = mavs.MavsEmbreeScene()
scene.Load(scene_name+'_scene.json')
waypoints.PutWaypointsOnGround(scene)

# Load waypoints and create controller to follow them
# Should consider including a check to see if the goal was reached
waypoints.FillIn(0.5)
controller = mavs.MavsVehicleController()
controller.SetDesiredPath(waypoints.GetWaypoints2D())
controller.SetDesiredSpeed(5.0) # m/s 
controller.SetSteeringScale(1.5) # ???
controller.SetWheelbase(2.719) # meters
controller.SetMaxSteerAngle(0.611) # radians
# controller.TurnOnLooping() # If you want the simulation to loop

# Debug/stats generation for the randomized scene settings
scene_setup = open("scene_stats.txt", "a")

scene_setup.write("\n" + str(scene_name) + "\n" + str(scene_size) + "\n" + str(random_scene.lo_mag) + "\n" + str(random_scene.hi_mag) +
"\n" + str(random_scene.plant_density) + "\n" + str(random_scene.trail_width) + "\n" + str(random_scene.track_width) + "\n" + 
str(random_scene.wheelbase) + "\n" + str(eco_file) + "\n" + 'no rain ' + "undefined" + "\n" + str(env.hour) + "\n" + str(env.fog) + "\n" +
str(env.turbidity) + "\n" + str(env.albedo) + "\n" + str(env.cloud_cover) + "\n" + str(env.wind))
# scene name
# scene size
# low mag
# high mag
# plant density
# trail width
# track width
# wheelbase
# eco file name
# downfall state and rate if applicable
# hour of day
# fog
# turbidity
# albedo
# cloud cover
# wind vector
scene_setup.close()

# Shows a map of the terrain elevation in the scene
# xi = np.linspace(min(x), max(x))
# yi = np.linspace(min(y), max(y))
# x1, y1 = np.meshgrid(xi,yi)
# z = griddata((x,y), elevation, (x1,y1), method='linear')

# fig, ax=plt.subplots(1,1)

# #map = plt.contour(x1, y1, z)
# map = plt.contourf(x1, y1, z)

# fig.colorbar(map)
# ax.set_title('Elevation Map')

# plt.show()

# x = []
# y = []
# elevation = []