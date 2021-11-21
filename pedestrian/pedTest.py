'''
Non-Commercial License - Mississippi State University Autonomous Vehicle Software (MAVS)

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

*NOTICE*
Thank you for your interest in MAVS, we hope it serves you well!  We have a few small requests to ask of you that will help us continue to create innovative platforms:
-To share MAVS with a friend, please ask them to visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to register and download MAVS for free.
-Feel free to create derivative works for non-commercial purposes, i.e. academics, U.S. government, and industry research.  If commercial uses are intended, please contact us for a commercial license at otm@msstate.edu or jonathan.rudd@msstate.edu.
-Finally, please use the ACKNOWLEDGEMENT above in your derivative works.  This helps us both!

Please visit https://gitlab.com/cgoodin/msu-autonomous-vehicle-simulator to view the full license, or email us if you have any questions.

Copyright 2018 (C) Mississippi State University
'''

import sys
sys.path.append(r'/usr/local/msu-autonomous-vehicle-simulator/src/mavs_python')

# Load the mavs python modules
import mavs_interface
import math
import mavs_python_paths

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

mavs_data_path = mavs_python_paths.mavs_data_path

#----- Sensor Creation and parameters ------------------#
# Create a MAVS camera and initialize it
cam = mavs_interface.MavsCamera()
cam.Initialize(384,384,0.0035,0.0035,0.0035)
#offset of the camera from the vehicle cg, [x,y,z] and quaternion [10.0,0.0,5]
cam.SetOffset([1.5,0.0,1.7],[1.0,0.0,0.0,0.0])
cam.RenderShadows(False)
cam.SetGammaAndGain(0.5,2.0)
# Create a MAVS lidar
lidar = mavs_interface.MavsLidar('OS1')
lidar.SetOffset([1.5,0.0,1.7],[1.0,0.0,0.0,0.0])

# Load a scene
#mavs_scenefile = "/scenes/cube_scene.json"
#mavs_scenefile = "/scenes/crosswalk_scene.json"
mavs_scenefile = "/scenes/crosswalk_scene_nl.json"
scene = mavs_interface.MavsEmbreeScene()
scene.Load(mavs_data_path+mavs_scenefile)

# Create and animation and add it to the scene
animation = mavs_interface.MavsAnimation()
animation.Load((mavs_data_path+"/scenes/meshes/animations/GenericWalk"),
(mavs_data_path+"/scenes/meshes/animations/GenericWalk/walk_frames.txt"))
animation.SetScale(0.01)
animation.SetRotations(True,False)
animation.SetBehavior('straight')
animation.SetPosition(0.0,0.0)
animation.SetHeading(0.0)
#animation.SetSpeed(target_speed) 
animation_id = scene.AddAnimation(animation)

#print("Pedestrian ID: " + str(animation_id))
#print("Vehicle ID: " + str(animationVeh_id))


#----- Waypoint Loading------------------#
waypoints = mavs_interface.MavsWaypoints()
#print((mavs_data_path + "\waypoints" + " \\" + str(sys.argv[1])))
waypoints.Load((mavs_data_path + "/waypoints/" + str(sys.argv[1]))) 
#waypoints.Load(str(sys.argv[1])) 
#waypoints.Load("euro_ncap_path.vprp")
waypoints.PutWaypointsOnGround(scene)

#----- Waypoint Loading------------------#
waypointsVeh = mavs_interface.MavsWaypoints()
#print((mavs_data_path + "\waypoints" + " \\" + str(sys.argv[2])))
waypointsVeh.Load((mavs_data_path + "/waypoints/" + str(sys.argv[2]))) 
#waypointsVeh.Load(str(sys.argv[2])) 
#waypoints.Load("euro_ncap_path.vprp")
waypointsVeh.PutWaypointsOnGround(scene)

# Create a MAVS camera and initialize it
camPed = mavs_interface.MavsCamera()
camPed.Initialize(384,384,0.0035,0.0035,0.0035)
#offset of the camera from the vehicle cg, [x,y,z] and quaternion
camPed.SetOffset([-0.5,0.0,2.0],[1.0,0.0,0.0,0.0])
camPed.RenderShadows(False)
camPed.SetGammaAndGain(0.5,2.0)

# Create a MAVS camera and initialize it
camTop = mavs_interface.MavsCamera()
camTop.Initialize(384,384,0.0035,0.0035,0.0035)
#offset of the camera from the vehicle cg, [x,y,z] and quaternion
camTop.RenderShadows(False)
camTop.SetGammaAndGain(0.5,2.0)
camTop.SetOffset([0.0,0.0,10.0],[  0, -0.7071068, 0, 0.7071068])

scene.TurnOffLabeling()

# Create the environment and set properties
env = mavs_interface.MavsEnvironment()
env.SetScene(scene.scene)
env.SetCloudCover(0.4)
env.SetTurbidity(5.0)
env.SetFog(0.025)

veh_id = env.AddActor(mavs_data_path+'/actors/actors/hmmwv_actor.json')

# set up timing variables
dt = 0.1
camera_dt = 0.05
lidar_dt = 0.1
camera_elapsed = 0.0
lidar_elapsed = 0.0
elapsed_time = 0.0
env.AdvanceTime(0.01)
lidar.SetDisplayColorType("color")

start_Anim = [(waypoints.waypoints[0])[0], (waypoints.waypoints[0])[1]]
start_Veh = [(waypointsVeh.waypoints[0])[0], (waypointsVeh.waypoints[0])[1]]

#print ("Animation Start " + str(start_Anim))
#print ("Vehicle Start " + str(start_Veh)) 
counter = 0
#----- Simulation -------------------------------------#
while(True):
    #Grab correct waypoint based on time
    idx = int(elapsed_time*5)%len(waypoints.waypoints)
    #print(waypoints.waypoints[idx])
    Anim_Xpos = (waypoints.waypoints[idx])[0]
    Anim_Ypos = (waypoints.waypoints[idx])[1]
    #ani_ori = waypoints.GetOrientation(idx)
    ani_ori = [1,0,0,0]
    yaw = quaternion_to_euler(ani_ori[0], ani_ori[1], ani_ori[2], ani_ori[3])
    env.SetAnimationPosition(animation_id, Anim_Xpos,Anim_Ypos,yaw[0])

    #print ("Current Animation [" + str(Anim_Xpos) + ", "+ str(Anim_Ypos) + "]")
    #print(waypointsVeh.waypoints[idx])
    AnimVeh_Xpos = (waypointsVeh.waypoints[idx])[0]
    AnimVeh_Ypos = (waypointsVeh.waypoints[idx])[1]
    vehOri = waypointsVeh.GetOrientation(idx)
    #env.SetActorPosition(veh_id,waypointsVeh.waypoints[idx],waypointsVeh.GetOrientation(idx))
    env.SetActorPosition(veh_id,waypointsVeh.waypoints[idx],[0.707, 0 , 0, 0.707,])
    #yawVeh = quaternion_to_euler(vehOri[0], vehOri[1], vehOri[2], vehOri[3])
    #env.SetAnimationPosition(animationVeh_id, AnimVeh_Xpos,AnimVeh_Ypos,yawVeh[0])
	
    #print ("Current Vehicle [" + str(AnimVeh_Xpos) + ", "+ str(AnimVeh_Ypos) + "]")
    #if Anim_Xpos > 67.50:
        #print("Done")

    # advance the environment, which moves animations
    env.AdvanceTime(dt)
    
    camera_pos = (waypointsVeh.waypoints[idx])
    camera_ori = [0.707, 0 , 0, 0.707,]
    
    # Update the camera and lidar sensor
    if (lidar_elapsed >=lidar_dt):
        lidar.SetPose(camera_pos, camera_ori)
        lidar.Update(env,dt)
        lidar.DisplayPerspective()
        #lidar.Display()
        lidar_elapsed = 0.0
    if (camera_elapsed>=camera_dt):
        cam.SetPose(camera_pos, camera_ori)
        cam.Update(env,dt)
        cam.Display()
        
        camPed.SetPose( [Anim_Xpos, Anim_Ypos, 0.0] , ani_ori)
        camPed.Update(env,dt)
        camPed.Display()
        camTop.SetPose([Anim_Xpos,Anim_Ypos,15.0],[  1.0, 0.0, 0.0, 0.0])
        camTop.Update(env, dt)
        camTop.Display()
        camera_elapsed = 0.0

    # update the timers 
    camera_elapsed = camera_elapsed + dt
    lidar_elapsed = lidar_elapsed + dt
    elapsed_time = elapsed_time + dt