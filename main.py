#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import GPy
import numpy as np
from swarmcity_msgs3.msg import SensorArray
from swarmcity_msgs1.msg import StateArray
from swarmcity_msgs.msg import PredictedTraffic
from swc_central_monitoring_system.srv import CarsDetectionNotification

MAX_MEASUREMENTS = 10
NUM_DRONES = 20
MAX_DETECTIONS = 50000
DET_VARIABLES = 15    # Drone, Time (secs), Time (nsecs), Type, Features, dposX, dposY, dposZ, posX, posY, posZ, orX, orY, orZ, W
MAX_CARS_SIMPLIFIED = 1400
MAX_PEOPLE_SIMPLIFIED = 1300
MAX_CLIMATE_SIMPLIFIED = 1300
XMIN = -270
XMAX = 370
YMIN = -280
YMAX = 310
X_grid_length = 130
Y_grid_length = 120
# SQUARE_LENGTH_X = 5         # ideal: 5
# SQUARE_LENGTH_Y = 5
time_window_cars = 15
time_window_people = 3
time_window_climate = 3
minutes_on_time0 = 0
cars_lengthscale = [2, 2, 100]
crowd_lengthscale = [2, 2, 100]
climate_lengthscale = [2, 2, 100]

cars_recording_t0 = -1
cars_recording_tf = -1
SQUARE_LENGTH_X = float(XMAX-XMIN)/X_grid_length
SQUARE_LENGTH_Y = float(YMAX-YMIN)/Y_grid_length
time_detections = -1
positions = [0 for i in range(NUM_DRONES)]
# position_times = list()
detections = list()    # Each item: list of four elements: drone, time, position, list of detections
drones_squared_cars = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
traffic_model = 0
cars_t0 = -1
squared_cars = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
cars_taking_measurement = False
cars_measurements_taken = 0
cars_simplified = list()
drones_squared_people = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
people_t0 = -1
squared_people = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
people_simplified = list()
people_model = 0
people_taking_measurement = False
people_measurements_taken = 0
temperatures = [[0 for y in range(NUM_DRONES)] for x in range(MAX_MEASUREMENTS)]
particles = [[0 for y in range(NUM_DRONES)] for x in range(MAX_MEASUREMENTS)]
rain = [[0 for y in range(NUM_DRONES)] for x in range(MAX_MEASUREMENTS)]
sulfur_oxides = [[0 for y in range(NUM_DRONES)] for x in range(MAX_MEASUREMENTS)]
carbon_oxides = [[0 for y in range(NUM_DRONES)] for x in range(MAX_MEASUREMENTS)]
nitrogen_oxides = [[0 for y in range(NUM_DRONES)] for x in range(MAX_MEASUREMENTS)]
times = [0 for x in range(MAX_MEASUREMENTS)]
pub = 0
# climate = [list() for i in range(8)]  # temperature, rain, sulfur_oxides, carbon_oxides, nitrogen_oxides, particles, time, position
squared_climate = [[[0 for z in range(Y_grid_length)] for y in range(X_grid_length)] for x in range(6)]
drones_squared_climate = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
climate_t0 = -1
climate_simplified = [list() for i in range(7)]     # 6 for each climate field, and an extra one for positions and times
climate_model = list()
climate_taking_measurement = False
climate_measurements_taken = 0

def getCars(show=False, delete_0_values=False):
    # Updates: cars_simplified, squared_cars, cars_t0,
    # cars_taking_measurement, cars_measurements_taken, drones_squared_cars
    global squared_cars   # Will tell how many cars in mean are detected in each area
    global cars_t0
    global cars_taking_measurement
    global cars_measurements_taken
    global cars_simplified
    global drones_squared_cars     # Tells how many measurements are made in each area

    if len(detections) == 0:    # No detections to work with yet
        return

    if cars_t0 == -1 or cars_t0 > detections[len(detections)-1][1]:      # Handles possible time incoherences
        cars_t0 = detections[len(detections)-1][1]
    if cars_t0 + time_window_cars > detections[len(detections)-1][1]:   # Not meant to take a measurement yet
        cars_taking_measurement = False
        return

    # Else:
    cars_taking_measurement = True
    cars_measurements_taken += 1
    cars_t0 = detections[len(detections)-1][1]  # Setting time t0 for next measurement

    index_cars = len(detections)-1             # Setting how far to search among the detections stored
    while index_cars > 0 and detections[len(detections)-1][1]-time_window_cars <= detections[index_cars-1][1]:
        x = int((detections[index_cars][2][0] - XMIN) // SQUARE_LENGTH_X)
        y = int((detections[index_cars][2][1] - YMIN) // SQUARE_LENGTH_Y)
        drones_squared_cars[x][y] += 1
        for j in range(len(detections[index_cars][3])):
            if detections[index_cars][3][j][3] == 'car':
                squared_cars[x][y] += 1
        index_cars -= 1

    for x in range(len(squared_cars)):   # Normalizing accumulative values stored by number of measurements in each area
        for y in range(len(squared_cars[0])):
            if drones_squared_cars[x][y] != 0:
                print(drones_squared_cars[x][y], squared_cars[x][y])
                squared_cars[x][y] = float(squared_cars[x][y]) / max(drones_squared_cars[x][y], 1)
                if len(cars_simplified) >= MAX_CARS_SIMPLIFIED:   # Deleting oldest value if max size is reached
                    del cars_simplified[0]
                # print(not delete_0_values, squared_cars[x][y])
                if not delete_0_values or squared_cars[x][y] != 0:   # Adding new simplified value
                    new_line = list()
                    new_line.append(squared_cars[x][y])
                    new_line.append(x)
                    new_line.append(y)
                    new_line.append(detections[len(detections)-1][1])
                    cars_simplified.append(new_line)

    squared_cars = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]  # Resetting variable values
    drones_squared_cars = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]

    if show and len(cars_simplified) != 0:       # Displaying stored simplified values if meant to
        for j in range(len(cars_simplified)):
            print(cars_simplified[j][0], cars_simplified[j][1], cars_simplified[j][2], cars_simplified[j][3])
        print('     -----------')
        print('Length = ', len(cars_simplified))
        print('     -----------')

    return


def getPeople(show=False, delete_0_values=False):
    # Updates: people_simplified, squared_people, people_t0,
    # people_taking_measurement, people_measurements_taken, drones_squared_people
    global squared_people      # Will tell how many people in mean are detected in each area
    global people_t0
    global people_taking_measurement
    global people_measurements_taken
    global people_simplified
    global drones_squared_people    # Tells how many measurements are made in each area

    if len(detections) == 0:   # No detections to work with yet
        return

    if people_t0 == -1 or people_t0 > detections[len(detections)-1][1]:   # Handles possible time incoherences
        people_t0 = detections[len(detections)-1][1]
    if people_t0 + time_window_people > detections[len(detections)-1][1]:   # Not meant to take a measurement yet
        people_taking_measurement = False
        return

    # Else:
    people_taking_measurement = True
    people_measurements_taken += 1
    people_t0 = detections[len(detections)-1][1]  # Setting time t0 for next measurement

    index_people = len(detections) - 1  # Setting how far to search among the detections stored
    while index_people > 0 and detections[len(detections) - 1][1] - time_window_people <= detections[index_people - 1][1]:
        x = int((detections[index_people][2][0] - XMIN) // SQUARE_LENGTH_X)
        y = int((detections[index_people][2][1] - YMIN) // SQUARE_LENGTH_Y)
        drones_squared_people[x][y] += 1
        for j in range(len(detections[index_people][3])):
            if detections[index_people][3][j][3] == 'person':
                squared_people[x][y] += 1
        index_people -= 1

    for x in range(len(squared_people)):   # Normalizing accumulative values stored by number of measurements in each area
        for y in range(len(squared_people[0])):
            if drones_squared_people[x][y] != 0:
                squared_people[x][y] = float(squared_people[x][y]) / max(drones_squared_people[x][y], 1)
                if len(people_simplified) >= MAX_PEOPLE_SIMPLIFIED:   # Deleting oldest value if max size is reached
                    del people_simplified[0]
                if not delete_0_values or squared_cars[x][y] != 0:  # Adding new simplified value
                    new_line = list()
                    new_line.append(squared_people[x][y])
                    new_line.append(x)
                    new_line.append(y)
                    new_line.append(detections[len(detections)-1][1])
                    people_simplified.append(new_line)

    squared_people = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]  # Resetting variable values
    drones_squared_people = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]

    if show and len(people_simplified) != 0:       # Displaying stored simplified values if meant to
        for j in range(len(people_simplified)):
            print(people_simplified[j][0], people_simplified[j][1], people_simplified[j][2], people_simplified[j][3])
        print('     -----------')
        print('Length = ', len(people_simplified))
        print('     -----------')

    return


def getMinutesFromTime(time, timeOn0 = minutes_on_time0):
    res = time + timeOn0
    res = res % (60 * 24)
    return res


def getDetections(data):
    global detections
    time_window = max(time_window_cars, time_window_people)
    if len(detections) != 0:
        while len(detections) > 0 and detections[len(detections) - 1][1] - time_window > detections[0][1]:
            del detections[0]
    for j in range(NUM_DRONES):
        if positions[j] != 0:
            new_line = list()
            new_line.append(j)
            new_line.append(data.time)
            new_line.append(list())
            new_line[2].append(positions[j][0])
            new_line[2].append(positions[j][1])
            new_line[2].append(positions[j][2])
            new_line.append(list())
            for k in range(len(data.sensors[j].detections)):
                new_detection = list()
                new_detection.append(j)
                new_detection.append(data.time)
                new_detection.append(data.sensors[j].laserscan.header.stamp.nsecs)
                new_detection.append(data.sensors[j].detections[k].type)
                new_detection.append(data.sensors[j].detections[k].features)
                new_detection.append(positions[j][0])
                new_detection.append(positions[j][1])
                new_detection.append(positions[j][2])
                new_detection.append(data.sensors[j].detections[k].position.position.x)
                new_detection.append(data.sensors[j].detections[k].position.position.y)
                new_detection.append(data.sensors[j].detections[k].position.position.z)
                new_detection.append(data.sensors[j].detections[k].position.orientation.x)
                new_detection.append(data.sensors[j].detections[k].position.orientation.y)
                new_detection.append(data.sensors[j].detections[k].position.orientation.z)
                new_detection.append(data.sensors[j].detections[k].position.orientation.w)
                new_line[3].append(new_detection)
            detections.append(new_line)
    return


def dispDetections():  # Programmed based on the assumption that DET_VARIABLES = 15
    for j in range(len(detections)):
        print(detections[j][0], detections[j][1], detections[j][2], detections[j][3], detections[j][4],
              detections[j][5], detections[j][6], detections[j][7], detections[j][8], detections[j][9],
              detections[j][10], detections[j][11], detections[j][12], detections[j][13], detections[j][14])
    print('     -----------------')
    return


def getPositions(data):  # Updates positions,
    global positions
    for j in range(NUM_DRONES):
        new_line = list()
        new_line.append(data.poses[j].position.x)
        new_line.append(data.poses[j].position.y)
        new_line.append(data.poses[j].position.z)
        positions[j] = new_line
    return


def dispPositions():
    for j in range(NUM_DRONES):
        print('Drone number: ', j)
        print('x: ', positions[j][0])
        print('y: ', positions[j][1])
        print('z: ', positions[j][2])
    print('     ----------------------')
    return


def getClimate(data):
    # Updates: squared_climate, climate_t0, climate_measurements_taken, climate_taking_measurement,
    # drones_squared_climate, and, most importantly, climate_simplified
    global squared_climate   # List of mean detected values of climate fields in each area
    global climate_t0
    global climate_simplified
    global drones_squared_climate     # Tells how many measurements are made in each area
    global climate_taking_measurement
    global climate_measurements_taken

    if climate_t0 == -1 or climate_t0 > data.sensors[0].laserscan.header.stamp.secs:  # Handles possible incoherences in time
        climate_t0 = data.sensors[0].laserscan.header.stamp.secs

    if positions[0] == 0:       # No info about drones' positions yet
        print('GetClimate: Wait a second, there are no positions stored yet')
        return

    for j in range(NUM_DRONES):    # Stores accumulative values of each climate field, and number of measurements in each area
        x = int((positions[j][0] - XMIN) // SQUARE_LENGTH_X)
        y = int((positions[j][1] - XMIN) // SQUARE_LENGTH_Y)
        squared_climate[0][x][y] += data.sensors[j].climate.temperature
        squared_climate[1][x][y] += data.sensors[j].climate.rain
        squared_climate[2][x][y] += data.sensors[j].climate.sulfur_oxides
        squared_climate[3][x][y] += data.sensors[j].climate.carbon_oxides
        squared_climate[4][x][y] += data.sensors[j].climate.nitrogen_oxides
        squared_climate[5][x][y] += data.sensors[j].climate.particles
        drones_squared_climate[x][y] += 1

    if climate_t0 + time_window_climate > data.sensors[0].laserscan.header.stamp.secs:
        climate_taking_measurement = False
    else:
        climate_taking_measurement = True
        climate_measurements_taken += 1
        climate_t0 = data.sensors[0].laserscan.header.stamp.secs  # Else: ready to store simplified data. Setting t0 for next time

        for x in range(X_grid_length):        # Normalizes accumulative values by number of measurements in each area
            for y in range(Y_grid_length):
                if drones_squared_climate[x][y] != 0:
                    for i in range(len(squared_climate)):
                        squared_climate[i][x][y] = float(squared_climate[i][x][y]) / max(drones_squared_climate[x][y], 1)
                        if len(climate_simplified[i]) >= MAX_CLIMATE_SIMPLIFIED:  # Deleting oldest value if max size is reached
                            del climate_simplified[i][0]
                        climate_simplified[i].append(squared_climate[i][x][y])
                        if i == 0:
                            new_line = list()
                            new_line.append(x)
                            new_line.append(y)
                            new_line.append(data.sensors[0].laserscan.header.stamp.secs)
                            climate_simplified[6].append(new_line)

        # Resetting variables for next storement:
        squared_climate = [[[0 for z in range(Y_grid_length)] for y in range(X_grid_length)] for x in range(6)]
        drones_squared_climate = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
    return climate_taking_measurement


def evaluateTraffic(showOptimization=False):       # Updates: traffic_model
    global traffic_model
    if len(cars_simplified) == 0:
        return
    XYT = np.zeros([len(cars_simplified), 3])
    Z = np.zeros([len(cars_simplified), 1])
    for i in range(len(cars_simplified)):
        Z[i] = cars_simplified[i][0]
        XYT[i][0] = cars_simplified[i][1]
        XYT[i][1] = cars_simplified[i][2]
        XYT[i][2] = getMinutesFromTime(cars_simplified[i][3])

    kernel = GPy.kern.RBF(input_dim=3, variance=1., ARD=True)
    traffic_model = GPy.models.GPRegression(XYT, Z, kernel)
    # traffic_model.optimize()
    length_scales = cars_lengthscale
    variance = 3
    if showOptimization:
        traffic_model.optimize()
        print(traffic_model.rbf.variance)
        print(traffic_model.rbf.lengthscale)
    traffic_model.rbf.lengthscale = length_scales
    traffic_model.rbf.variance = variance
    return


def evaluateCrowd(showOptimization=False):       # Updates: people_model
    global people_model
    if len(people_simplified) == 0:
        return
    XYT = np.zeros([len(people_simplified), 3])
    Z = np.zeros([len(people_simplified), 1])
    for i in range(len(people_simplified)):
        Z[i] = people_simplified[i][0]
        XYT[i][0] = people_simplified[i][1]
        XYT[i][1] = people_simplified[i][2]
        XYT[i][2] = getMinutesFromTime(people_simplified[i][3])

    kernel = GPy.kern.RBF(input_dim=3, variance=1., ARD=True)
    people_model = GPy.models.GPRegression(XYT, Z, kernel)
    # people_model.optimize()
    length_scales = crowd_lengthscale
    variance = 3
    if showOptimization:
        people_model.optimize()
        print(people_model.rbf.variance)
        print(people_model.rbf.lengthscale)
    people_model.rbf.lengthscale = length_scales
    people_model.rbf.variance = variance
    return


def evaluateClimate(showOptimization=False):       # Updates: climate_model
    global climate_model
    if len(climate_simplified[0]) == 0:
        return
    XYT = np.zeros([len(climate_simplified[6]), 3])
    Z = list()
    for i in range(6):
        Z.append(np.zeros([len(climate_simplified[i]), 1]))
    for j in range(len(climate_simplified[0])):
        XYT[j] = climate_simplified[6][j]
        for i in range(6):
            Z[i][j] = (climate_simplified[i][j])

    print(XYT)
    print('     ------')
    print(Z)

    length_scales = climate_lengthscale
    variance = 3
    kernel = GPy.kern.RBF(input_dim=3, variance=1., ARD=True)
    for i in range(6):
        new_model = GPy.models.GPRegression(XYT, Z[i], kernel)
        climate_model.append(new_model)
        if showOptimization:
            climate_model[i].optimize()
            print(climate_model[i].rbf.variance)
            print(climate_model[i].rbf.lengthscale)
            print('       ------')
        climate_model[i].rbf.lengthscale = length_scales
        climate_model[i].rbf.variance = variance
    return


def predictTraffic(time=-1, show=False, public=False, negative_equals_0=True):
    global pub
    if traffic_model == 0:
        return 1
    if time == -1:
        if len(detections) == 0:
            print('No detections yet')
            return
        else:
            time = detections[len(detections)-1][1]
    testedXYT = np.zeros([Y_grid_length * X_grid_length, 3])
    traffic_mean = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
    traffic_var = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
    k = 0
    for i in range(X_grid_length):
        for j in range(Y_grid_length):
            testedXYT[k][0] = i + 0.5
            testedXYT[k][1] = j + 0.5
            testedXYT[k][2] = getMinutesFromTime(time)
            k += 1
    mean, var = traffic_model.predict(testedXYT)
    k = 0
    for i in range(X_grid_length):
        for j in range(Y_grid_length):
            if negative_equals_0:
                traffic_mean[i][j] = max(mean[k][0], 0)   # We don't want to distinguish zero traffic from negative traffic
            else:
                traffic_mean[i][j] = mean[k][0]
            traffic_var[i][j] = var[k][0]
            k += 1
    if show:
        plot(traffic_mean, traffic_var, positions)
        # plt.matshow(np.transpose(traffic_mean), cmap=plt.cm.Blues)
        # plt.matshow(np.transpose(traffic_var), cmap=plt.cm.Reds)
        # plt.show()
    if public and pub != 0:
        mean_vector = list()
        var_vector = list()
        for i in range(len(traffic_mean)):
            for j in range(len(traffic_mean[0])):
                mean_vector.append(traffic_mean[i][j])
                var_vector.append(traffic_var[i][j])
        pub.publish(mean_vector, var_vector, time, len(traffic_mean), len(traffic_mean[0]))
        # print('I just published a traffic prediction!')
    return traffic_mean, traffic_var


def predictCrowd(time=-1, show=False, public=False, negative_equals_0=True):
    global pub
    if people_model == 0:
        return 1
    if time == -1:
        if len(detections) == 0:
            print('No detections yet')
            return
        else:
            time = detections[len(detections)-1][1]
    testedXYT = np.zeros([Y_grid_length * X_grid_length, 3])
    people_mean = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
    people_var = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
    k = 0
    for i in range(X_grid_length):
        for j in range(Y_grid_length):
            testedXYT[k][0] = i + 0.5
            testedXYT[k][1] = j + 0.5
            testedXYT[k][2] = getMinutesFromTime(time)
            k += 1
    mean, var = people_model.predict(testedXYT)
    k = 0
    for i in range(X_grid_length):
        for j in range(Y_grid_length):
            if negative_equals_0:
                people_mean[i][j] = max(mean[k][0], 0)   # We don't want to distinguish zero crowd from negative crowd
            else:
                people_mean[i][j] = mean[k][0]
            people_var[i][j] = var[k][0]
            k += 1
    if show:
        plt.matshow(np.transpose(people_mean), cmap=plt.cm.Blues)
        plt.matshow(np.transpose(people_var), cmap=plt.cm.Reds)
        plt.show()
    if public and pub != 0:
        mean_vector = list()
        var_vector = list()
        for i in range(len(people_mean)):
            for j in range(len(people_mean[0])):
                mean_vector.append(people_mean[i][j])
                var_vector.append(people_var[i][j])
        pub.publish(mean_vector, var_vector, len(people_mean), len(people_mean[0]))
        # print('I just published a traffic prediction!')
    return people_mean, people_var


def predictClimate(climateField, time=-1, show=False, public=False, negative_equals_0=True):
    global pub
    if climate_model == 0:
        return 1
    if time == -1:
        if len(detections) == 0:
            print('No detections yet')
            return
        else:
            time = detections[len(detections)-1][1]
    testedXYT = np.zeros([Y_grid_length * X_grid_length, 3])
    climate_mean = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
    climate_var = [[0 for y in range(Y_grid_length)] for x in range(X_grid_length)]
    k = 0
    for i in range(X_grid_length):
        for j in range(Y_grid_length):
            testedXYT[k][0] = i + 0.5
            testedXYT[k][1] = j + 0.5
            testedXYT[k][2] = getMinutesFromTime(time)
            k += 1
    mean, var = climate_model[climateField].predict(testedXYT)
    k = 0
    for i in range(X_grid_length):
        for j in range(Y_grid_length):
            if negative_equals_0:
                climate_mean[i][j] = max(mean[k][0], 0)   # We don't want to distinguish zero crowd from negative crowd
            else:
                climate_mean[i][j] = mean[k][0]
            climate_var[i][j] = var[k][0]
            k += 1
    if show:
        plt.matshow(np.transpose(climate_mean), cmap=plt.cm.Blues)
        plt.matshow(np.transpose(climate_var), cmap=plt.cm.Reds)
        plt.show()
    if public and pub != 0:
        mean_vector = list()
        var_vector = list()
        for i in range(len(climate_mean)):
            for j in range(len(climate_mean[0])):
                mean_vector.append(climate_mean[i][j])
                var_vector.append(climate_var[i][j])
        pub.publish(mean_vector, var_vector, len(climate_mean), len(climate_mean[0]))
        # print('I just published a traffic prediction!')
    return climate_mean, climate_var

def plot(m1, m2, positions, transpose=True):
    m1 = np.reshape(m1, [len(m1), len(m1[0])])
    if transpose:
        m1 = np.transpose(m1)
    norm1 = plt.Normalize(m1.min(), m1.max())
    cmap1 = plt.cm.Blues
    rgba1 = cmap1(norm1(m1))

    if transpose:
        m2 = np.transpose(m2)
    m2 = np.reshape(m2, [len(m2), len(m2[0])])
    norm2 = plt.Normalize(m2.min(), m2.max())
    cmap2 = plt.cm.Reds
    rgba2 = cmap2(norm2(m2))

    for i in range(len(positions)):
        if transpose:
            y = int((positions[i][0] - XMIN) // SQUARE_LENGTH_X)
            x = int((positions[i][1] - YMIN) // SQUARE_LENGTH_Y)
        else:
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

def handle_new_car_detection(req):
    global squared_cars
    global drones_squared_cars
    x = int((req.x_m - XMIN) // SQUARE_LENGTH_X)
    y = int((req.y_m - YMIN) // SQUARE_LENGTH_Y)
    drones_squared_cars[x][y] += 1
    squared_cars[x][y] += req.num_cars
    print(req.time)
    return []


def sensorCallback(data):
    global cars_recording_t0
    global cars_recording_tf
    getClimate(data)
    getDetections(data)
    getCars(show=True, delete_0_values=False)
    # getPeople(show=False)
    # dispDetections()
    # print(climate_measurements_taken, climate_taking_measurement, people_taking_measurement, cars_taking_measurement)
    # if climate_taking_measurement:
    #     print('Measurements taken: ', climate_measurements_taken)
    #     if climate_measurements_taken % 3 == 0:
    #         evaluateClimate()
    #         predictClimate(0, show=True)
    if cars_taking_measurement:
        print('Measurements taken: ', cars_measurements_taken)
        if cars_measurements_taken % 1 == 0:
            evaluateTraffic()
            predictTraffic(show=True)
        #     number_of_predictions = 30
        #     for i in range(number_of_predictions):
        #         predictTraffic(time=(cars_recording_tf-cars_recording_t0) * i / (number_of_predictions-1), public=True)


def stateCallback(data):
    getPositions(data)



def listener():
    global pub
    rospy.init_node('Main', anonymous=True)
    rospy.Subscriber('/Swarm/StateArray', StateArray, stateCallback)
    rospy.Subscriber('/Swarm/SensorArray', SensorArray, sensorCallback)
    pub = rospy.Publisher('PredictedTraffic', PredictedTraffic, queue_size=10)
    s = rospy.Service('CarsDetection', CarsDetectionNotification, handle_new_car_detection)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
