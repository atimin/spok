import os
import rospy
import math
import time

from spok.srv import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import QTimer

class VSlider(Plugin):

    def __init__(self, context):
        super(VSlider, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('VSlider')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'VSlider.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('VSliderUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        # Make client
        rospy.wait_for_service('turn_joint')
        rospy.wait_for_service('status_joint')
        try:
          self._turn_joint = rospy.ServiceProxy('turn_joint', TurnJoint)
          self._status_joint = rospy.ServiceProxy('status_joint', StatusJoint)
        except Exception, e:
          raise e

        # Set parametrs of arm mover
        rospy.set_param('/spok/arm_mover/port', '/dev/ttyACM0')
        rospy.set_param('/spok/arm_mover/baud', 19200)
        rospy.set_param('/spok/arm_mover/debug', 1)

        # Params of widget
        self._period = 100
        self._joint = 0

        # Init timer
        self._timer = QTimer()
        self._timer.timeout.connect(self.timer_tick)
        self._timer.start(self._period)


        # Init angle slider
        js = self._widget.jointSlider
        angle =  self._status_joint(self._joint).angle
        js.setValue(angle / math.pi * 100)
        js.valueChanged.connect(self.change_angle)

        self._last_angle = angle
        self._last_vel = 0.0
        self._time_last_change = time.time()
        self._alfa = 0.1

        # Init combox of joints
        jn = self._widget.jointNumber
        jn.currentIndexChanged.connect(self.change_joint)
        jn.currentIndex = 0



    def change_joint(self, index):
        self._joint = index


    def timer_tick(self):
        angle = self._status_joint(self._joint).angle
        self._widget.currentAngle.setText(str(angle))


    def change_angle(self, value):
        self._angle_changed = True
        angle = math.pi * (float(value) / 100)
        vel = abs((angle - self._last_angle)
                / (time.time() - self._time_last_change))

        vel = self._alfa*vel + ((1 - self._alfa)
            *self._last_vel)
        vel = max(vel, 0.2)
        vel = min(vel, 1)
        self._turn_joint(self._joint, angle, vel)
        self._time_last_change = time.time()
        self._last_angle = angle
        self._last_vel = vel

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
