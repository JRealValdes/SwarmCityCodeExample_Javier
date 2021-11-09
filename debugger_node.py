#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from swarmcity_msgs.msg import PredictedTraffic
from [IDK] import pose_1  # Y copy-paste hasta NUM_DRONES  # Interpreto que son: tiempo, posX, posY, posZ
from [IDK] import pose_2
from [IDK] import pose_3
from [IDK] import pose_4
from [IDK] import pose_5
from [IDK] import pose_6
from [IDK] import pose_7
from [IDK] import pose_8
from [IDK] import pose_9
from [IDK] import pose_10

NUM_DRONES = 10
NUM_MEASUREMENTS_EXPECTED = 30
XMIN = -270
XMAX = 370
YMIN = -280
YMAX = 310
X_grid_length = 130
Y_grid_length = 120
SQUARE_LENGTH_X = float(XMAX-XMIN)/X_grid_length
SQUARE_LENGTH_Y = float(YMAX-YMIN)/Y_grid_length

positions = [list() for i in range(NUM_DRONES)]
pose_changes = list()      # Each element is a list of three elements: time a drone changed pose, which drone, and new pos (list: X, Y, Z)
traffic_means = [list() for i in range(NUM_MEASUREMENTS_EXPECTED)]
traffic_vars = [list() for i in range(NUM_MEASUREMENTS_EXPECTED)]
traffic_times = [0 for i in range(NUM_MEASUREMENTS_EXPECTED)]
traffic_measurements_stored = 0

def plot(m1, m2, positions):
    m1 = np.reshape(m1, [len(m1), len(m1[0])])
    norm1 = plt.Normalize(m1.min(), m1.max())
    cmap1 = plt.cm.Blues
    rgba1 = cmap1(norm1(m1))

    m2 = np.reshape(m2, [len(m2), len(m2[0])])
    norm2 = plt.Normalize(m2.min(), m2.max())
    cmap2 = plt.cm.Reds
    rgba2 = cmap2(norm2(m2))

    for i in range(len(positions)):
        x = int((positions[i][0] - XMIN) // SQUARE_LENGTH_X)
        y = int((positions[i][1] - YMIN) // SQUARE_LENGTH_Y)
        rgba1[x][y][:3] = 1, 0, 0
        rgba2[x][y][:3] = 0, 1, 0

    fig1 = plt.figure()
    ax = fig1.add_subplot(121)
    ax.imshow(rgba1, interpolation='nearest')
    ax2 = fig1.add_subplot(122)
    ax2.imshow(rgba2, interpolation='nearest')
    plt.show()

def generateDebug (data):
    global traffic_measurements_stored
    global positions
    traffic_means[traffic_measurements_stored] = np.reshape(data.mean, [data.ldim1, data.ldim2])
    traffic_vars[traffic_measurements_stored] = np.reshape(data.var, [data.ldim1, data.ldim2])
    traffic_times[traffic_measurements_stored] = data.time
    traffic_measurements_stored += 1

    if traffic_measurements_stored == NUM_MEASUREMENTS_EXPECTED:
        traffic_recording_index = 0
        pose_index = -1
        traffic_t = traffic_times[1]
        pose_t = pose_changes[0][0]
        plot(traffic_means[0], traffic_vars[0], positions)
        while pose_index < len(pose_changes)-1 or traffic_recording_index < NUM_MEASUREMENTS_EXPECTED-1:
            if traffic_recording_index == NUM_MEASUREMENTS_EXPECTED-1:
                traffic_t += 999999
            else:
                traffic_t = traffic_times[traffic_recording_index + 1]
            if pose_index == len(pose_changes)-1:
                pose_t += 999999
            else:
                pose_t = pose_changes[pose_index+1][0]

            if traffic_t < pose_t:
            #    [UPDATE TRAFFIC]
                traffic_recording_index += 1
            else:
            #    [UPDATE POSITIONS]
                pose_index += 1
                positions[pose_changes[pose_index][1]] = pose_changes[pose_index][2]
            plot(traffic_means[traffic_recording_index], traffic_vars[traffic_recording_index], positions)

def poseHandler(data, drone):
    global pose_changes
    new_line = list()
    new_pos = list()
    new_pos.append(data.posX)
    new_pos.append(data.posY)
    new_pos.append(data.posZ)
    new_line.append(data.time)
    new_line.append(drone)
    new_line.append(new_pos)
    pose_changes.append(new_line)

def listener():
    rospy.init_node('Debugger_Node', anonymous=True)
    rospy.Subscriber('PredictedTraffic', PredictedTraffic, generateDebug)
    rospy.Subscriber('pose_1', pose_1, poseHandler(drone=1))  # Copy-paste hasta NUM_DRONES
    rospy.Subscriber('pose_2', pose_2, poseHandler(drone=2))
    rospy.Subscriber('pose_3', pose_3, poseHandler(drone=3))
    rospy.Subscriber('pose_4', pose_4, poseHandler(drone=4))
    rospy.Subscriber('pose_5', pose_5, poseHandler(drone=5))
    rospy.Subscriber('pose_6', pose_6, poseHandler(drone=6))
    rospy.Subscriber('pose_7', pose_7, poseHandler(drone=7))
    rospy.Subscriber('pose_8', pose_8, poseHandler(drone=8))
    rospy.Subscriber('pose_9', pose_9, poseHandler(drone=9))
    rospy.Subscriber('pose_10', pose_10, poseHandler(drone=10))
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
