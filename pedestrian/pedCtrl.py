import math
import sys
import random
import time

# Set the path to the mavs python api, mavs.py
sys.path.append(r'/usr/local/msu-autonomous-vehicle-simulator/src/mavs_python')

# Load the mavs python modules
import mavs_interface as mavs
import mavs_python_paths

# Set the path to the mavs data folder
mavs_data_path = mavs_python_paths.mavs_data_path

def quaternion_to_euler(w, x, y, z):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

# Scene generation

# eco_file = 'american_southeast_meadow.json'

# scene_name = 'ped_test'
# scene_size = 100.0
# ped_scene = mavs.MavsRandomScene()
# ped_scene.terrain_width = scene_size 
# ped_scene.terrain_length = scene_size 
# ped_scene.lo_mag = 0.0
# ped_scene.hi_mag = 0.0 
# ped_scene.plant_density = 0.00
# ped_scene.trail_width = 2.0
# ped_scene.track_width = 2.0
# ped_scene.wheelbase = 2

# ped_scene.basename = scene_name
# ped_scene.eco_file = eco_file
# ped_scene.path_type = 'Ridges'
# ped_scene.CreateScene()

# Load scene
# mavs_scenefile = "/home/peyton/mavs/pedWalk/ped_test_scene.json"
# ped_scene = mavs.MavsEmbreeScene()
# ped_scene.Load(mavs_scenefile)

# Load crosswalk scene
mavs_scenefile = "/scenes/crosswalk_scene_nl.json" # "Position": [ 68, 62, 1.1 ],
ped_scene = mavs.MavsEmbreeScene()
ped_scene.Load(mavs_data_path + mavs_scenefile)


# Create and animation and add it to the scene
animation = mavs.MavsAnimation()
animation.Load((mavs_data_path+"/scenes/meshes/animations/GenericWalk"),
(mavs_data_path+"/scenes/meshes/animations/GenericWalk/walk_frames.txt"))
animation.SetScale(0.01)
animation.SetRotations(True,False)
animation.SetBehavior('straight')
animation.SetPosition(-11.0,11.0)
animation.SetHeading(-0.785)
animation_id = ped_scene.AddAnimation(animation)

# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(ped_scene.scene)
ped_scene.TurnOffLabeling()

# Sets the environmental settings to randomly-chosen intensities
env.SetTime(12)
env.SetFog(0)
env.SetTurbidity(1.0)
env.SetAlbedo(1.0)
env.SetCloudCover(0.0)
env.SetWind([0,0])

# Load a MAVS vehicle
veh = mavs.MavsRp3d()
veh_file = 'mrzr4.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(-3.5, -2.0, 0.0)   # in global ENU
veh.SetInitialHeading(0.0)              # in radians, 0.785 ~= 45 deg (direction of trail)
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

# Driver cam
drive_cam = mavs.MavsCamera()
drive_cam.Initialize(480, 480,0.0035,0.0035,0.0035) # h*w, h*v detector size in m, focal length in m
cam_angle = (math.pi/180.0)*0
drive_cam.SetOffset([-10, 0, 3],[math.cos(0.5*cam_angle),0.0,math.sin(0.5*cam_angle),0.0])
drive_cam.RenderShadows(False)

# Create a MAVS camera and set its properties
cam = mavs.MavsCamera()
cam.Initialize(720, 480,0.0035,0.0035,0.0035)
# cam.SetAntiAliasingFactor(4)
cam_angle = (math.pi/180.0)*0 # 0 degrees -> straight ahead
cam.SetOffset([0.75, 0.0, 1.0],[math.cos(0.5*cam_angle),0.0,math.sin(0.5*cam_angle),0.0])
cam.RenderShadows(False)



# Load the waypoints that go with this scene
waypoints = mavs.MavsWaypoints()
# waypoints.Load('./'+ped_scene.basename+'_path.vprp')
waypoints.Load('./ped_test_path.vprp')
waypoints.PutWaypointsOnGround(ped_scene)

# pedWaypoints = mavs.MavsWaypoints()
# pedWaypoints.Load('/home/peyton/mavs/pedWalk/ped1_wp.vprp')
# pedWaypoints.PutWaypointsOnGround(ped_scene)



### Simulation #################################

dt = 1.0/30.0 # time step, seconds
time_elapsed = 0.0
nloopu = 0

while (True):
    tw0 = time.time()
    
    # Update the environment for driven vehicle
    veh.free_driving = True
    dc = drive_cam.GetDrivingCommand()
    veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

    p = veh.GetPosition()
    orientation = veh.GetOrientation()
    env.SetActorPosition(0,p,orientation)

    ### Update the environment for the pedestrian
    # idx = int(time_elapsed*5)%len(pedWaypoints.waypoints)
    # Anim_Xpos = (pedWaypoints.waypoints[idx])[0]
    # Anim_Ypos = (pedWaypoints.waypoints[idx])[1]

    # ani_ori = [1,0,0,0]
    # yaw = quaternion_to_euler(ani_ori[0], ani_ori[1], ani_ori[2], ani_ori[3])
    # env.SetAnimationPosition(animation_id, Anim_Xpos,Anim_Ypos,yaw[0])



    # Proceed with the simulation
    env.AdvanceTime(dt)

    # Have the drive came update with the simulation
    drive_cam.SetPose(p,orientation)
    drive_cam.Update(env,dt)
    drive_cam.Display()

    # Update the camera sensors at 30 Hz
    if nloopu%3==0:
        tw0 = time.time()

        # Cam
        cam.SetPose(p,orientation)
        cam.Update(env,dt)
        cam.Display()



    nloopu = nloopu+1
    time_elapsed = time_elapsed + dt
    tw1 = time.time()
    wall_dt = tw1-tw0
    if (wall_dt<dt):
        time.sleep(dt-wall_dt)