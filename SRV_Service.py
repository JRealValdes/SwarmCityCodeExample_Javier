#!/usr/bin/env python

from swc_central_monitoring_system.srv import CarsDetectionNotification
import rospy

nc = list()
x = list()
y = list()
id_code = list()

def handle_new_car_detection(req):
    nc.append(req.num_cars)
    y.append(req.y_m)
    x.append(req.x_m)
    id_code.append(req.drone_id)
    print('nc', nc)
    print('x', x)
    print('y', y)
    print('id_code', id_code)
    return []

def add_two_ints_server():
    rospy.init_node('CarsDetection')
    s = rospy.Service('CarsDetection', CarsDetectionNotification, handle_new_car_detection)
    print "Ready to store detections"
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
