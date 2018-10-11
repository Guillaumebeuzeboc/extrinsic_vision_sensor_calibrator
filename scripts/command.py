#!/usr/bin/env python

from cmd import Cmd
from threading import Thread

# ROS
import rospy
from extrinsic_vision_sensor_calibrator.calibrator import Calibrator


class CalibrationPrompt(Cmd):
    def __init__(self):
        rospy.loginfo('Do not ctrl+c the program, enter -> quit <- so you can get the finale ouput')
        Cmd.__init__(self)
        self.calibrator = Calibrator()
        self.prompt = '<--->'

    def get_arg(self, args):
        return 0.0 if len(args) == 0 else float(args)

    def do_add_x(self, args):
        self.calibrator.add_x(self.get_arg(args))

    def do_add_y(self, args):
        self.calibrator.add_y(self.get_arg(args))

    def do_add_z(self, args):
        self.calibrator.add_z(self.get_arg(args))

    def do_add_roll(self, args):
        self.calibrator.add_roll(self.get_arg(args))

    def do_add_pitch(self, args):
        self.calibrator.add_pitch(self.get_arg(args))

    def do_add_yaw(self, args):
        self.calibrator.add_yaw(self.get_arg(args))

    def do_print_calib(self, args):
        self.calibrator.print_calib()

    def do_quit(self, args):
        self.calibrator.print_calib()
        rospy.loginfo("Good bye")

        rospy.signal_shutdown('Quit')
        raise SystemExit

    def cmdloop(self, intro=None):
        try:
            Cmd.cmdloop(self, intro="")
        except KeyboardInterrupt:
            rospy.logerr("ctrl+c")

    def update_tf(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.calibrator.publish_tf()
            rate.sleep()

    def run(self):
        thread = Thread(target=self.update_tf)
        thread.start()
        prompt.cmdloop('starting calibation')
        thread.join()


if __name__ == '__main__':
    rospy.init_node('Calibrator')

    prompt = CalibrationPrompt()
    prompt.run()
