import matplotlib.pylab as plt
import numpy as np
import open3d as o3d
from math import sqrt
import seaborn as sns

pcd = o3d.io.read_point_cloud("/home/peyton/mavs/roughness_data/lidar_test/scene_gen/lidar_view.pcd")
pointCloud = np.asarray(pcd.points)


# Creates a voxel-based representation of the data
pcd2 = o3d.io.read_point_cloud("/home/peyton/mavs/roughness_data/lidar_test/scene_gen/lidar_view.pcd")
pointCloud2 = np.asarray(pcd2.points)
print("LiDAR data: ", pointCloud2)

N = 2000
pcd2.scale(1 / np.max(pcd2.get_max_bound() - pcd2.get_min_bound()), center=pcd2.get_center())
pcd2.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
# o3d.visualization.draw_geometries([pcd2])

# print('voxelization')
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd2,voxel_size=0.005)
# o3d.visualization.draw_geometries([voxel_grid])


# Filtering unnecessary points
filteredPC = []
val = 0
hits = 0

for val in range(len(pointCloud)):
    if (pointCloud[val, 0] >= 0.0 and pointCloud[val, 0] <= 30.0) and (pointCloud[val, 1] >= -5.0 and pointCloud[val, 1] <= 5.0):
        hits += 1
        filteredPC.append(pointCloud[val])

filteredPC = np.array(filteredPC)

print("Hits: ", hits)
# np.savetxt('test.txt', filteredPC)

# Calculating voxel roughness values
roughArray = []
count = 0

for y1 in range(-5, 5):
    for x1 in range(0, 30):
        # print("Pairs (x,y): " + str(x1) + ", " + str(y1))
        inst = 0
        z_array = []
        for inst in range(len(filteredPC)):
            # if(the value is in the voxel x1 -> x1+1, y1 -> y1+1)
            # if(filteredPC[inst, 0] >= x1 and filteredPC[inst, 0] < x1+2 and filteredPC[inst, 1] >= y1 and filteredPC[inst, 1] < y1+2):
            if((filteredPC[inst, 0] >= x1) and (filteredPC[inst, 0] < (x1+1)) and (filteredPC[inst, 1] >= y1) and (filteredPC[inst, 1] < (y1+1))):
                # extract the z value using inst as the index, append to z_array
                z_array.append(filteredPC[inst, 2])
                
        # if z_array isn't empty
        if len(z_array) > 0:
            # take the average z value of the array
            avg = sum(z_array) / len(z_array)
            # subtract avg z from each z in array and square it
            for i in range(len(z_array)):
                z_array[i] = (z_array[i] - avg) ** 2
            # sum all these values and take the sqrt
            roughness = sqrt(sum(z_array) / len(z_array))
            # append the roughness value to the roughness array at the proper position
            roughArray.append(roughness)
            # roughArray.append(max(z_array) - min(z_array))
        else:
            roughArray.append(0)


ra = np.empty([10,30], dtype=float)
ra.fill(0.0)

itr = 0
for m in range(0, 10):
    for n in range(0, 30):
        ra[m][n] = np.format_float_positional(roughArray[itr], precision=7)
        itr = itr + 1
        if itr > 300:
            break

roughArray = np.array(roughArray)
for item in range(len(roughArray)):
    np.format_float_positional(roughArray[item], precision=7)

# print(roughArray)
# print(len(roughArray))

# print(ra)
ra = np.flipud(ra)
hmap = sns.heatmap(ra)
hmap.set_title('Seaborn Lidar RMS')
plt.show()



# # Testing code for verifying roughness values
# xt = 4
# yt = -5
# voxel_size = 2
#
# v = 0
# test_arr = []
# for v in range(len(pointCloud)):
#     if (pointCloud[v, 0] >= xt and pointCloud[v, 0] < (xt + voxel_size)) and (pointCloud[v, 1] >= yt and pointCloud[v, 1] < (yt + voxel_size)):
#         test_arr.append(pointCloud[v, 2])
    
# avg1 = sum(test_arr) / len(test_arr)
# for j in range(len(test_arr)):
#     test_arr[j] = (test_arr[j] - avg1) ** 2

# result = sqrt(sum(test_arr))
# print(result)