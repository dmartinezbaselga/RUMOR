from matplotlib import pyplot as plt
import numpy as np
import sys

file = open("./validation_tests/logs_learning/log3.dat", 'r')
lines = file.readlines()
lines.remove(lines[0])

iterations = 0
goal_x = []
goal_y = []
loc_x = []
loc_y = []
loc_theta = []
v = []
w = []
atan = []
d = []

for line in lines:
    fields = line.split()
    goal_x.append(float(fields[2]))
    goal_y.append(float(fields[3]))
    loc_x.append(float(fields[4]))
    loc_y.append(float(fields[5]))
    loc_theta.append(float(fields[6]))
    v.append(float(fields[7]))
    w.append(float(fields[8]))
    atan.append(float(fields[9]))
    d.append(float(fields[10]))
    iterations += 1

print(np.mean(d))

# i = 0
# for arg in sys.argv:
#     plt.xlabel("Iteration")
#     if arg == "--goal_x":
#         plt.plot(goal_x)
#         plt.show()

#     elif arg == "--goal_y":
#         plt.plot(goal_y)
#         plt.show()

#     elif arg == "--loc_x":
#         plt.plot(loc_x)
#         plt.show()

#     elif arg == "--loc_y":
#         plt.plot(loc_y)
#         plt.show()

#     elif arg == "--loc_theta":
#         plt.plot(loc_theta)
#         plt.show()

#     elif arg == "--v":
#         plt.ylabel("Linear velocity")
#         plt.plot(v)
#         plt.show()

#     elif arg == "--w":
#         plt.ylabel("Angular velocity")
#         plt.plot(w)
#         plt.show()

#     elif arg == "--atan":
#         plt.plot(atan)
#         plt.show()
    
#     elif arg == "--d":
#         plt.ylabel("Minimum distance to obstacle")
#         plt.plot(d)
#         plt.show()

#     else:
#         if i > 0:
#             print("Argument ", arg, " is not recognized")

#         i += 1