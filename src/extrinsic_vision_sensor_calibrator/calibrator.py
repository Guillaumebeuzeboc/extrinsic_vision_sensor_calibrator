#!/usr/bin/env python

from threading import Lock

# ROS
import rospy
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from angles import normalize_angle


class Calibrator(object):
    def __init__(self):
        target_position_source_frame = \
            rospy.get_param('~target_position_source_frame')
        self.tf_source_frame = rospy.get_param('~tf_source_frame')
        self.tf_target_frame = rospy.get_param('~tf_target_frame')

        x_target_position = rospy.get_param('~target_position/x')
        y_target_position = rospy.get_param('~target_position/y')
        z_target_position = rospy.get_param('~target_position/z')
        roll_target_rotation = rospy.get_param('~target_rotation/roll')
        pitch_target_rotation = rospy.get_param('~target_rotation/pitch')
        yaw_target_rotation = rospy.get_param('~target_rotation/yaw')

        self.estimated_sensor_position_x = rospy.get_param('~estimated_sensor_position/x')
        self.estimated_sensor_position_y = rospy.get_param('~estimated_sensor_position/y')
        self.estimated_sensor_position_z = rospy.get_param('~estimated_sensor_position/z')
        self.estimated_sensor_rotation_roll = rospy.get_param('~estimated_sensor_rotation/roll')
        self.estimated_sensor_rotation_pitch = rospy.get_param('~estimated_sensor_rotation/pitch')
        self.estimated_sensor_rotation_yaw = rospy.get_param('~estimated_sensor_rotation/yaw')

        cube_scale_x = rospy.get_param('~cube_scale/x')
        cube_scale_y = rospy.get_param('~cube_scale/y')
        cube_scale_z = rospy.get_param('~cube_scale/z')
        cube_alpha = rospy.get_param('~cube_alpha')

        self.pub_marker = rospy.Publisher('~calibration_target_marker',
                                          Marker, latch=True, queue_size=1)

        self.mutex = Lock()

        q = quaternion_from_euler(
            roll_target_rotation,
            pitch_target_rotation,
            yaw_target_rotation)

        # init the marker

        cube = Marker()
        cube.header.frame_id = target_position_source_frame
        cube.id = 0
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = x_target_position
        cube.pose.position.y = y_target_position
        cube.pose.position.z = z_target_position
        cube.pose.orientation.x = q[0]
        cube.pose.orientation.y = q[1]
        cube.pose.orientation.z = q[2]
        cube.pose.orientation.w = q[3]
        cube.scale.x = cube_scale_x
        cube.scale.y = cube_scale_y
        cube.scale.z = cube_scale_z
        cube.color.a = cube_alpha
        cube.color.r = 1.0
        cube.color.g = 0.0
        cube.color.b = 0.0

        self.pub_marker.publish(cube)

        self.br = tf2_ros.TransformBroadcaster()

    def add_x(self, val):
        self.mutex.acquire()
        self.estimated_sensor_position_x += val
        self.mutex.release()

    def add_y(self, val):
        self.mutex.acquire()
        self.estimated_sensor_position_y += val
        self.mutex.release()

    def add_z(self, val):
        self.mutex.acquire()
        self.estimated_sensor_position_z += val
        self.mutex.release()

    def add_roll(self, val):
        self.mutex.acquire()
        self.estimated_sensor_rotation_roll = normalize_angle(
            self.estimated_sensor_rotation_roll + val)
        self.mutex.release()

    def add_pitch(self, val):
        self.mutex.acquire()
        self.estimated_sensor_rotation_pitch = normalize_angle(
            self.estimated_sensor_rotation_pitch + val)
        self.mutex.release()

    def add_yaw(self, val):
        self.mutex.acquire()
        self.estimated_sensor_rotation_yaw = normalize_angle(
            self.estimated_sensor_rotation_yaw + val)
        self.mutex.release()

    def publish_tf(self):
        self.mutex.acquire()

        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.tf_source_frame
        t.child_frame_id = self.tf_target_frame
        t.transform.translation.x = self.estimated_sensor_position_x
        t.transform.translation.y = self.estimated_sensor_position_y
        t.transform.translation.z = self.estimated_sensor_position_z
        q = quaternion_from_euler(
            self.estimated_sensor_rotation_roll,
            self.estimated_sensor_rotation_pitch,
            self.estimated_sensor_rotation_yaw)
        self.mutex.release()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

    def print_calib(self):
        self.mutex.acquire()
        rospy.loginfo("source frame: {}".format(self.tf_source_frame))
        rospy.loginfo("target frame: {}".format(self.tf_target_frame))
        rospy.loginfo("x: {}".format(self.estimated_sensor_position_x))
        rospy.loginfo("y: {}".format(self.estimated_sensor_position_y))
        rospy.loginfo("z: {}".format(self.estimated_sensor_position_z))
        rospy.loginfo("Roll: {}".format(self.estimated_sensor_rotation_roll))
        rospy.loginfo("Pitch: {}".format(self.estimated_sensor_rotation_pitch))
        rospy.loginfo("Yaw: {}".format(self.estimated_sensor_rotation_yaw))
        self.mutex.release()
