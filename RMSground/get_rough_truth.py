import matplotlib.pylab as plt
import numpy as np
import open3d as o3d
from math import sqrt
import seaborn as sns

rms_truth = np.loadtxt("rms_truth.txt", dtype=float)
# print(rms_truth)

filtered_truth = []
val = 0
hits = 0

for val in range(len(rms_truth)):
    if (rms_truth[val, 0] >= 50.0 and rms_truth[val, 0] <= 80.0) and (rms_truth[val, 1] >= 45.0 and rms_truth[val, 1] <= 55.0):
        hits += 1
        filtered_truth.append(rms_truth[val])

filtered_truth = np.array(filtered_truth)

print(hits)

filtered_truth = np.delete(filtered_truth, 2, 1)
print(filtered_truth)


itr = 0
arr = []
avgs_arr = []

for y in range(45, 55):
    for x in range(50, 80):
        arr = []
        for i in range(len(filtered_truth)):
            if(filtered_truth[i, 0] >= x and filtered_truth[i, 0] < (x + 1) and filtered_truth[i, 1] >= y and filtered_truth[i, 1] < (y + 1)):
                arr.append(filtered_truth[i, 2])
        avg = sum(arr) / len(arr)
        avgs_arr.append(avg)

print(len(avgs_arr))


ra = np.empty([10,30], dtype=float)
ra.fill(0.0)

itr = 0
for m in range(0, 10):
    for n in range(0, 30):
        ra[m][n] = np.format_float_positional(avgs_arr[itr], precision=7)
        itr = itr + 1
        if itr > 300:
            break

avgs_arr = np.array(avgs_arr)
for item in range(len(avgs_arr)):
    np.format_float_positional(avgs_arr[item], precision=7)

print(avgs_arr)
print(len(avgs_arr))

print(ra)
ra = np.flipud(ra)
hmap = sns.heatmap(ra)
hmap.set_title('Averaged Values from truth.txt')
plt.show()