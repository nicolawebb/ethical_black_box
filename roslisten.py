#!/usr/bin/env python3
import rospy
import miro2 as miro
import os
import message_filters
import std_msgs
from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo, Range, JointState, BatteryState
from std_msgs.msg import Int32, UInt16, Float32MultiArray
from nav_msgs.msg import Odometry
import csv
import datetime

headers = ["dateTime, touch_head, touch_body, wheel_speed, sonar, odom, kinematic_joints, battery, light, cliff"]
# file = pd.read_csv("metadata.csv") 
# file.to_csv("metadata.csv", header=headers, index=False) 
with open('metadata.csv', 'w') as file:
    writer = csv.writer(file) 
    writer.writerow(headers)
    file.close()


def callback(touch_head, touch_body, wheel_speed, sonar, odom, kinematic_joints, battery, light, cliff):
    # rospy.loginfo("Head touch: [%i]", touch_head.data)
    # rospy.loginfo("Body touch: [%i]", touch_body.data)
    # rospy.loginfo("Wheel speed: [%s]", wheel_speed.data)
    # rospy.loginfo("Sonar: [%s]", sonar)
    # rospy.loginfo("Odometry: [%s]", odom)
    # rospy.loginfo("Kinematic_joints: [%s]", kinematic_joints)
    # rospy.loginfo("Battery: [%s]", battery)
    # rospy.loginfo("Light: [%s]", light.data)
    # rospy.loginfo("Cliff: [%s]", cliff)
    dataList = []
    dataList.append([datetime.datetime.now(), touch_head.data, touch_body.data])

    # print(touch_head.data)
    # print(len(dataDict))

    data = dataList
    with open('metadata.csv', 'w') as file:
        writer = csv.writer(file) 
        writer.writerow(data[0])
        file.close()

    print(dataList[0])


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    touch_head = message_filters.Subscriber("/miro/sensors/touch_head", std_msgs.msg.UInt16, queue_size=10)
    touch_body = message_filters.Subscriber("/miro/sensors/touch_body", std_msgs.msg.UInt16, queue_size=10)
    wheel_speed = message_filters.Subscriber("/miro/sensors/wheel_speed_cmd", std_msgs.msg.Float32MultiArray, queue_size=10)
    sonar = message_filters.Subscriber("/miro/sensors/sonar", Range, queue_size=10)
    odom = message_filters.Subscriber("/miro/sensors/odom", Odometry, queue_size=10)
    kinematic_joints = message_filters.Subscriber("/miro/sensors/kinematic_joints", JointState, queue_size=10)
    battery = message_filters.Subscriber("/miro/sensors/battery", BatteryState, queue_size=10)
    light = message_filters.Subscriber("/miro/sensors/light", Float32MultiArray, queue_size=10)
    cliff = message_filters.Subscriber("/miro/sensors/cliff", Float32MultiArray, queue_size=10)


    ts = message_filters.ApproximateTimeSynchronizer([touch_head, touch_body, wheel_speed, sonar, odom, kinematic_joints, battery, light, cliff], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)

    try:
        rospy.loginfo("Started subscriber node...")
        rospy.spin()
    except rospy.ROSInterruptException:
    # Shutdown
        rospy.loginfo("Shutting down subscriber!")
        sp.shutdown()


