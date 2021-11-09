#!/usr/bin/env python

import sys
import rospy
import numpy as np
import time
from swc_central_monitoring_system.srv import CarsDetectionNotification

XMIN = -270
XMAX = 370
YMIN = -280
YMAX = 310
SQUARE_LENGTH_X = 5         # ideal: 5
SQUARE_LENGTH_Y = 5

def client():
    rospy.wait_for_service('CarsDetection')
    while True:
        x = np.random.randint(XMAX-XMIN)+XMIN   # X_pos
        y = np.random.randint(YMAX-YMIN)+YMIN   # Y_pos
        t = np.random.randint(1600000000)
        print('Sending random info: ', 'x = ', (x - XMIN) // SQUARE_LENGTH_X, 'y = ', (y - YMIN) // SQUARE_LENGTH_Y)
        id_code = np.random.randint(1000)
        n = np.random.randint(5)
        try:
            new_car_det = rospy.ServiceProxy('CarsDetection', CarsDetectionNotification)   # Se define como "una funcion": le llamas con los valores input y te devuelve un struct con los output
            new_car_det(x, y, t, n, id_code)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        time.sleep(1)
    return

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    client()
    # if len(sys.argv) == 3:
    #     x = int(sys.argv[1])
    #     y = int(sys.argv[2])
    # else:
    #     print usage()
    #     sys.exit(1)
    # print "Requesting %s+%s"%(x, y)
    # print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))