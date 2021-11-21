# time_end = 50
# path_start = -5.0 #x, y = -path_start
# path_end = 5.0 #x, y = -path_end
# midpoint = 0.0

# increment = (abs(path_start) - midpoint) / (time_end / 2)


# for step in range(1, (time_end + 1)):
#     if (step == 1):
#         x_val = path_start
#         y_val = -(x_val)

#     elif (step <= (time_end)):
#         x_val = path_start + (increment * step)
#         y_val = -(x_val)

#     print(str(step/100) + ": " + ("{:.2f}".format(x_val)) + " " + ("{:.2f}".format(y_val)) + " 2.0:1.0 0.0 0.0 0.0")

steps = 30
start = [3.0, 1.5]
end = [3.0, -4.8]

increment = (1.5 + 4.8) / steps

if ((start[0] - end[0] == 0) or (start[1] - end[1] == 0)):
    for i in range(1, (steps + 1)):
        if(i == 1):
            x_val = start[0]
            y_val = start[1]
        
        elif(i <= steps):
            x_val = x_val
            y_val = start[1] - (increment * i)

        print(str(i/100) + ": " + ("{:.2f}".format(x_val)) + " " + ("{:.2f}".format(y_val)) + " 0.0:1.0 0.0 0.0 0.0")
