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
animation.SetPosition(3.0, 1.5)
animation.SetHeading(-1.57)
animation_id = ped_scene.AddAnimation(animation)

# Create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(ped_scene.scene)
ped_scene.TurnOnLabeling()

# Sets the environmental settings to randomly-chosen intensities
env.SetTime(13)
env.SetFog(0)
env.SetTurbidity(1.0)
env.SetAlbedo(1.0)
env.SetCloudCover(0.0)
env.SetWind([0,0])

# Create a MAVS camera and set its properties
cam = mavs.MavsCamera()
cam.Initialize(720, 480,0.0035,0.0035,0.0035)
cam.SetAntiAliasingFactor(4)
cam_angle = (math.pi/180.0)*0 # 0 degrees -> straight ahead
cam.SetPose([-6, -1.7, 3.0],[math.cos(0.5*cam_angle),0.0,math.sin(0.5*cam_angle),0.0])
cam.RenderShadows(True)

# Create a MAVS Lidar and set its properties
lidar = mavs.MavsLidar('OS1')
lidar_angle = (math.pi/180)*0
lidar.SetPose([-6, -1.7, 3.0],[math.cos(0.5*lidar_angle),0.0,math.sin(0.5*lidar_angle),0.0])

# Load the waypoints that go with this scene
ped_waypoints = mavs.MavsWaypoints()
# waypoints.Load('./ped1_wp.vprp')
ped_waypoints.Load('./test_path.vprp')
ped_waypoints.PutWaypointsOnGround(ped_scene)


### Simulation #################################

dt = 1.0/30.0 # time step, seconds
time_elapsed = 0.0
nloop = 0

while (time_elapsed < 5):
    # env.SetActorPosition(0,p,orientation)

    ### Update the environment for the pedestrian
    idx = int(time_elapsed*5)%(len(ped_waypoints.waypoints))
    Anim_Xpos = (ped_waypoints.waypoints[idx])[0]
    Anim_Ypos = (ped_waypoints.waypoints[idx])[1]

    ani_ori = [1,0,0,0]
    yaw = quaternion_to_euler(ani_ori[0], ani_ori[1], ani_ori[2], ani_ori[3])
    env.SetAnimationPosition(animation_id, Anim_Xpos,Anim_Ypos,-1.57)

    # Proceed with the simulation
    env.AdvanceTime(dt)

    # Update the camera sensors at 30 Hz
    if nloop%3==0:

        # Static Cam
        cam.Update(env,dt)
        cam.Display()
        # # Save camera image
        #cam.AnnotateFrame(env)
        im_name_cam = (str(nloop).zfill(5)+'_cam_image')
        cam.SaveCameraImage(im_name_cam+'.bmp')
        # Save annotated / labeled data
        cam.AnnotateFrame(env)
        cam.SaveAnnotation(env,(str(nloop).zfill(5)+'_cam_annotated'))

        # Static Lidar
        # lidar.Update(env,dt)
        # # Save lidar data
        # im_name_lidar = (str(nloop).zfill(5)+'_lidar_image')
        # lidar.AnnotateFrame(env)
        # lidar.SaveLabeledPcd(im_name_lidar + '.pcd')


    nloop = nloop+1
    time_elapsed = time_elapsed + dt

print(time_elapsed)
sys.exit()