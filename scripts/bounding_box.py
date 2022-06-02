#!/usr/bin/env python3

import roslib

roslib.load_manifest('learning_tf')
 
import rospy

import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    boundin_box = 0.3
    while not rospy.is_shutdown():
        br.sendTransform((-boundin_box/2, boundin_box/2, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "bb1",
                         "base_link")

        br.sendTransform((-boundin_box/2, -boundin_box/2, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "bb2",
                         "base_link")  

        br.sendTransform((-boundin_box/2, boundin_box/2, boundin_box),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "bb3",
                         "base_link")   

        br.sendTransform((-boundin_box/2, -boundin_box/2, boundin_box),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "bb4",
                         "base_link")   

        br.sendTransform((boundin_box-boundin_box/2, boundin_box/2, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "bb5",
                         "base_link") 
        br.sendTransform((boundin_box-boundin_box/2, -boundin_box/2, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "bb6",
                         "base_link") 
        br.sendTransform((boundin_box-boundin_box/2, boundin_box/2, boundin_box),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "bb7",
                         "base_link") 
        br.sendTransform((boundin_box-boundin_box/2, -boundin_box/2, boundin_box),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "bb8",
                         "base_link") 

        rate.sleep()