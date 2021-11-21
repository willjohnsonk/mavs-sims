# Creates a random off-road scene based on given parameters.
# The scene may be user-controlled or waypoint-controlled
# Should be run with "python3 data_exec.py" for Python3 or "python data_exec.py"
# MAVS:     Dr. Goodin      (cgoodin@cavs.msstate.edu)
# Scripts:  Peyton Johnson  (wj311@cavs.msstate.edu)

# gnuplot> plot "rms_truth.txt" using 1:2:3:4 with points palette
# gnuplot> splot "rms_truth.txt" using 1:2:3:4 with points palette 

from data_setup import *
import time
import math

dt = 1.0/30.0 # time step, seconds
time_elapsed = 0.0
nloopw = 0 # loop counter for waypoints

# Code for running waypoint/automatic control
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
    if nloopw%10==0:
        env.SetActorPosition(0,p,orientation)
        env.AdvanceTime(30.0*dt)

        # Camera
        cam.SetPose(p,orientation)
        cam.Update(env,dt)
        cam.Display()

        # Save camera image
        im_name_cam = (str(nloopw).zfill(5)+'_cam_image')
        cam.SaveCameraImage(im_name_cam+'.bmp')
        # Save annotated / labeled data
        cam.AnnotateFrame(env)
        cam.SaveAnnotation(env,(str(nloopw).zfill(5)+'_cam_annotated'))

        # Lidar
        lidar.SetPose(p, orientation)
        lidar.Update(env,dt)
        lidar.Display()

        # Save lidar data
        im_name_lidar = (str(nloopw).zfill(5)+'_lidar_image')
        lidar.AnnotateFrame(env)
        lidar.SaveLabeledPcd(im_name_lidar + '.pcd')

    nloopw = nloopw+1
    time_elapsed = time_elapsed + dt

    if(nloopw == 250):
        # sys.exit()
        break


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