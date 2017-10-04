#!/usr/bin/env python
import tf
import math
import time
import rospy
import traceback
from tf import TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point

tf_ = None
main_tf = 'None'
target_tf = 'None'
tf_broadcaster = None
angle = 0.0

def init():
    global tf_, tf_broadcaster, target_tf, main_tf
    rospy.init_node('tf_look_at_tf')
    target_tf = rospy.get_param("~target_tf", 'target')
    main_tf = rospy.get_param("~main_tf", 'main')
    tf_ = TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("tf", TFMessage, tf_callback)
    time.sleep(2)
    tf_broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            rospy.Time.now(),
            target_tf,
            'world')
    while not rospy.is_shutdown():
        rospy.spin()

def tf_callback(tf2):
    global tf_, target_tf, main_tf, tf_broadcaster, angle
    try:
        angle += 0.1
        if angle > 2 * 3.14:
            angle = 0
        t = rospy.Time.now()
        tx = math.cos(-angle)
        ty = math.sin(-angle)
        tf_broadcaster.sendTransform(
                (tx, ty, 0),
                (0, 0, 0, 1),
                rospy.Time.now(),
                target_tf,
                'world')
        main_pos = Point()
        main_pos.x = 2 * math.cos(angle)
        main_pos.y = 2 * math.sin(angle)
        dx = tx - main_pos.x
        dy = ty - main_pos.y
        yaw = math.atan2(dy,dx)
        main_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        tf_broadcaster.sendTransform(
                (main_pos.x, main_pos.y, main_pos.z),
                main_quat,
                rospy.Time.now(),
                main_tf,
                'world')
        time.sleep(1)
    except Exception as e:
        print traceback.format_exc()

if __name__ == '__main__':
    init()