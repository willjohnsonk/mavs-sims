from data_setup import *
import time
import math

# You may use these in more specific circumstances
# altitude = 0
# speed = 5
# sens_dist - speed*dt
# temp = 25.0

dt = 1.0/30.0 # time step, seconds
time_elapsed = 0.0
nloopu = 0 # loop counter for user input
nloopw = 0 # loop counter for waypoints

# Code for running waypoint/automatic control
if not userInput: 
    p = veh.GetPosition()

    while(True):
        tw0 = time.time()

        # Definites vehicle movements and orientation
        controller.SetCurrentState(veh.GetPosition()[0],veh.GetPosition()[1],veh.GetSpeed(),veh.GetHeading())
        dc = controller.GetDrivingCommand(dt)
        veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)
        p = veh.GetPosition()
        orientation = veh.GetOrientation()

        # right now everything is running at 30hz, should be updated to fit sensors
        if nloopw%3==0:
            env.SetActorPosition(0,p,orientation)
            env.AdvanceTime(30.0*dt)

            # Camera
            cam.SetPose(p,orientation)
            cam.Update(env,dt)
            cam.Display()

            # Save camera image
            im_name_cam = (str(nloopu).zfill(5)+'_cam_image')
            cam.SaveCameraImage(im_name_cam+'.bmp')
            # Save annotated / labeled data
            cam.AnnotateFrame(env)
            cam.SaveAnnotation(env,(str(nloopu).zfill(5)+'_cam_annotated'))

            # Lidar
            lidar.SetPose(p, orientation)
            lidar.Update(env,dt)
            lidar.Display()

            # Save lidar data
            im_name_lidar = (str(nloopu).zfill(5)+'_lidar_image')
            lidar.AnnotateFrame(env)
            lidar.SaveLabeledPcd(im_name_lidar + '.pcd')

            # Radar
            # radar.SetPose(p, orientation)
            # radar.Update(env,dt)
            # radar.Display()

            # Save radar data
            #im_name_radar = (str(nloop).zfill(5)+'_radar_image')
            #radar.SaveRadarImage(im_name_radar + '.bmp')

            # Retrieves a list of objects spotted, needs to be saved to access
            # targets = radar.GetTargets()

            # Prints frame timings if needed
            # print('Frame time = '+str(time.time()-tw0))
            # sys.stdout.flush()

        nloopw = nloopw+1
        time_elapsed = time_elapsed + dt


# Code for running with keyboard control
if userInput:
    while (True):
        tw0 = time.time()
        # Get the driving command
        # Update the environment
        veh.free_driving = True
        dc = drive_cam.GetDrivingCommand()
        veh.Update(env, dc.throttle, dc.steering, dc.braking, dt)

        p = veh.GetPosition()
        orientation = veh.GetOrientation()
        env.SetActorPosition(0,p,orientation)
        env.AdvanceTime(dt)

        drive_cam.SetPose(p,orientation)
        drive_cam.Update(env,dt)
        drive_cam.Display()

        ## Update the camera sensors at 30 Hz
        if nloopu%3==0:
            tw0 = time.time()

            # Camera
            cam.SetPose(p,orientation)
            cam.Update(env,dt)
            cam.Display()

            # Save camera image
            im_name_cam = (str(nloopu).zfill(5)+'_cam_image')
            cam.SaveCameraImage(im_name_cam+'.bmp')
            # Save annotated / labeled data
            cam.AnnotateFrame(env)
            cam.SaveAnnotation(env,(str(nloopu).zfill(5)+'_cam_annotated'))

            # Lidar
            lidar.SetPose(p, orientation)
            lidar.Update(env,dt)
            lidar.Display()

            # Save lidar data
            im_name_lidar = (str(nloopu).zfill(5)+'_lidar_image')
            lidar.AnnotateFrame(env)
            lidar.SaveLabeledPcd(im_name_lidar + '.pcd')

            # Radar
            # radar.SetPose(p, orientation)
            # radar.Update(env,dt)
            # radar.Display()

            # Save radar data
            #im_name_radar = (str(nloop).zfill(5)+'_radar_image')
            #radar.SaveRadarImage(im_name_radar + '.bmp')

            # Retrieves a list of objects spotted
            # targets = radar.GetTargets()

            # Prints frame timings if needed
            # print('Frame time = '+str(time.time()-tw0))
            # sys.stdout.flush()

        nloopu = nloopu+1
        time_elapsed = time_elapsed + dt

        tw1 = time.time()
        wall_dt = tw1-tw0
        if (wall_dt<dt):
            time.sleep(dt-wall_dt)