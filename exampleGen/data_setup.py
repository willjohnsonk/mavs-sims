# Creates a random off-road scene based on given parameters.
# The scene may be user-controlled or waypoint-controlled
# Should be run with "python3 data_exec.py" for Python3 or "python data_exec.py"
# MAVS:     Dr. Goodin      (cgoodin@cavs.msstate.edu)
# Scripts:  Peyton Johnson  (wj311@cavs.msstate.edu)

import math
import sys
import random

# Set the path to the mavs python api, mavs.py
# ***must be changed to your system-specific path***
sys.path.append(r'/usr/local/msu-autonomous-vehicle-simulator/src/mavs_python')

# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths

# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

# If 'ctrl' is entered as a command line arg vehicle takes user wasd input
userInput = False
if (len(sys.argv) > 1):
    arg = str(sys.argv[1])
    if arg == "ctrl":
        userInput = True

# Randomizes scene downfall type
rainCheck = False
snowCheck = False
downfalltxt = "none "
downfallrate = ""
downfall = random.randrange(0,2)

# Defines checks for downfall to be used later
if downfall == 1:
    rainCheck = True
    downfalltxt = "rain "
if downfall == 2:
    snowCheck = True
    downfalltxt = "snow "


# Scene generation
# List of valid eco files to be picked from at random
eco_files = ['american_southeast_forest_obstacles.json', 'forest_with_cubes.json', 'american_pine_forest.json', 
'american_southeast_meadow.json', "american_southeast_forest.json", 'american_southwest_desert.json', 
'american_southeast_forest_brushy.json', 'country_road.json']

# Randomly selects one of the above eco files, prints info on sim settings
#    Note: some scenes may not work /load with certain parameters
eco_file = random.choice(eco_files)
print("Eco file: " + eco_file)
print("Downfall: " + str(downfall))
sys.stdout.flush()

scene_name = 'gen_test'
scene_size = 150.0
random_scene = mavs.MavsRandomScene()
random_scene.terrain_width = scene_size 
random_scene.terrain_length = scene_size 
random_scene.lo_mag = 1.5 # 0.1*random.randrange(10,70)
random_scene.hi_mag = 0.01 #0.1*random.randrange(0,5)
random_scene.plant_density = 0.1*random.randrange(1,3)
random_scene.trail_width = 2.0 #0.1*random.randrange(30, 50) 
random_scene.track_width = 2.0 #0.1*random.randrange(5, 25)
random_scene.wheelbase = 2

for potholes in range(0, 15):
    random_scene.AddPotholeAt(random.randrange(10, 140), random.randrange(10, 140), 0.5, 1.0)

random_scene.num_potholes = (potholes)

random_scene.basename = scene_name
random_scene.eco_file = eco_file
random_scene.path_type = 'Ridges'
random_scene.CreateScene()

# Example for loading a scene
#   Note: Lines 41-72 should be commented out and adjustments will be made as necessary
# mavs_scenefile = "*directory of scene.json*"
# ped_scene = mavs.MavsEmbreeScene()
# ped_scene.Load(mavs_scenefile)

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

# Sets rain or snow to rand value based on downfall status
if snowCheck:
    env.SetSnow((0.1*random.randrange(0,250)))
    downfallrate = env.snow_rate
if rainCheck:
    env.SetRainRate((0.1*random.randrange(0,250)))
    downfallrate = env.rain_rate

# Load a MAVS vehicle
# There are a few vehicles to choose from, check the data/vehicles/rp3d_vehicles for a list
veh = mavs.MavsRp3d()
veh_file = 'mrzr4.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(0.0, 0.0, 0.0)   # in global ENU
veh.SetInitialHeading(0.785)              # in radians, 0.785 ~= 45 deg (direction of trail)
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

# Checks for flag, creates a camera for user control if needed. Must be selected to drive
if userInput:
    drive_cam = mavs.MavsCamera()
    drive_cam.Initialize(480, 480,0.0035,0.0035,0.0035) # h*w, h*v detector size in m, focal length in m
    cam_angle = (math.pi/180.0)*0
    drive_cam.SetOffset([-10, 0, 3],[math.cos(0.5*cam_angle),0.0,math.sin(0.5*cam_angle),0.0])
    drive_cam.RenderShadows(False)

# Create a MAVS camera and set its properties
cam = mavs.MavsCamera()
cam.Model('XCD-V60')
cam.SetAntiAliasingFactor(5)  # AA significantly hurts performance
cam.SetGammaAndGain(0.85,1.0) 
cam_angle = (math.pi/180.0)*0 # 0 degrees -> straight ahead
cam.SetOffset([0.7, 0.0, 0.75],[math.cos(0.5*cam_angle),0.0,math.sin(0.5*cam_angle),0.0])
if rainCheck:
    cam.SetDropsOnLens(True)

# Create a MAVS lidar and set its properties
lidar = mavs.MavsLidar('OS1')
lidar_angle = (math.pi/180)*0
lidar.SetOffset([0.0, 0.0, 1.9],[math.cos(0.5*lidar_angle),0.0,math.sin(0.5*lidar_angle),0.0])

# Create a MAVS radar and set its properties
# radar = mavs.MavsRadar()
# radar.SetMaxRange(80)
# radar.SetFieldOfView(60)
# radar_angle = (math.pi/180)*0
# radar.SetOffset([1.3, 0.0, 0.4],[math.cos(0.5*radar_angle),0.0,math.sin(0.5*radar_angle),0.0])

# Create a MAVS imu accelerometer
# Note: Unfinished. See documentation for details
# imu1 = mavs.MavsMems('accelerometer')
# imu2 = mavs.MavsMems('accelerometer')
# imu2.SetNoiseDensity(0.025)
# imu2.SetBiasInstability(0.01)
# imu2.SetTemperatureBias(0.0)
# imu2.SetConstantBias(0.0)

# Load the waypoints that go with this scene
waypoints = mavs.MavsWaypoints()
waypoints.Load('./'+random_scene.basename+'_path.vprp')


# Get the scene geometry and put the waypoints on the ground
if not userInput:
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
str(random_scene.wheelbase) + "\n" + str(eco_file) + "\n" + downfalltxt + str(downfallrate) + "\n" + str(env.hour) + "\n" + str(env.fog) + "\n" +
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