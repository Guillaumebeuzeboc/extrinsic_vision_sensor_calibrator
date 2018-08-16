#!/usr/bin/env python
from cmd import Cmd
import signal

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

        self.pub_marker = rospy.Publisher('~calibration_target_marker',
                                          Marker, latch=True, queue_size=1)

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
        cube.color.a = 1.0
        cube.color.r = 1.0
        cube.color.g = 0.0
        cube.color.b = 0.0

        self.pub_marker.publish(cube)

        self.br = tf2_ros.TransformBroadcaster()

    def publish_tf(self, add_x=0, add_y=0, add_z=0, add_roll=0,
                   add_pitch=0, add_yaw=0):
        self.estimated_sensor_position_x += add_x
        self.estimated_sensor_position_y += add_y
        self.estimated_sensor_position_z += add_z

        self.estimated_sensor_rotation_roll = normalize_angle(
            self.estimated_sensor_rotation_roll + add_roll)
        self.estimated_sensor_rotation_pitch = normalize_angle(
            self.estimated_sensor_rotation_pitch + add_pitch)
        self.estimated_sensor_rotation_yaw = normalize_angle(
            self.estimated_sensor_rotation_yaw + add_yaw)

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
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)


class CalibrationPrompt(Cmd):
    def __init__(self):
        Cmd.__init__(self)
        self.calibrator = Calibrator()

    def get_arg(self, args):
        return 0.0 if len(args) == 0 else float(args)

    def do_add_x(self, args):
        self.calibrator.publish_tf(add_x=self.get_arg(args))

    def do_add_y(self, args):
        self.calibrator.publish_tf(add_y=self.get_arg(args))

    def do_add_z(self, args):
        self.calibrator.publish_tf(add_z=self.get_arg(args))

    def do_add_roll(self, args):
        self.calibrator.publish_tf(add_roll=self.get_arg(args))

    def do_add_pitch(self, args):
        self.calibrator.publish_tf(add_pitch=self.get_arg(args))

    def do_add_yaw(self, args):
        self.calibrator.publish_tf(add_yaw=self.get_arg(args))

    def do_quit(self, args):
        rospy.loginfo("source frame: {}".format(self.calibrator.tf_source_frame))
        rospy.loginfo("target frame: {}".format(self.calibrator.tf_target_frame))
        rospy.loginfo("x: {}".format(self.calibrator.estimated_sensor_position_x))
        rospy.loginfo("y: {}".format(self.calibrator.estimated_sensor_position_y))
        rospy.loginfo("z: {}".format(self.calibrator.estimated_sensor_position_z))
        q = quaternion_from_euler(
            self.calibrator.estimated_sensor_rotation_roll,
            self.calibrator.estimated_sensor_rotation_pitch,
            self.calibrator.estimated_sensor_rotation_yaw)
        rospy.loginfo("qx: {}".format(q[0]))
        rospy.loginfo("qy: {}".format(q[1]))
        rospy.loginfo("qz: {}".format(q[2]))
        rospy.loginfo("qw: {}".format(q[3]))
        rospy.loginfo("Good bye")
        raise SystemExit

    def cmdloop(self, intro=None):
        while True:
            try:
                Cmd.cmdloop(self, intro="")
                break
            except KeyboardInterrupt:
                rospy.logerr("ctrl+c")


if __name__ == '__main__':
    rospy.init_node('Calibrator')

    #rate = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    rate.sleep()
    prompt = CalibrationPrompt()
    prompt.prompt = '<--->'
    prompt.cmdloop('starting calibation')
