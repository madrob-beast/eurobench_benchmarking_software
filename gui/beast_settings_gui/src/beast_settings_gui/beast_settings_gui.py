import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QSlider

from std_msgs.msg import UInt16


class beast_settings_gui(Plugin):
    def __init__(self, context):
        super(beast_settings_gui, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('BEAST Settings')

        # Create QWidget
        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'beast_settings_gui.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('BEAST Settings')
        self._widget.setWindowTitle('BEAST Settings')

        self.wheel_stiffness_value_label = self._widget.findChild(QLabel, 'wheel_stiffness_value')

        self.wheel_stiffness_slider = self._widget.findChild(QSlider, 'wheel_stiffness_slider')
        self.wheel_stiffness_slider.valueChanged.connect(self.update_wheel_stiffness)

        context.add_widget(self._widget)

        self.run_rospy_node()

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

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget
        # title bar
        # Usually used to open a modal configuration dialog

    def update_wheel_stiffness(self, value):
        self.wheel_stiffness_value_label.setText(str(value) + ' N')

    def run_rospy_node(self):
        self.wheel_stiffness_pub = rospy.Publisher('beast/wheel_stiffness', UInt16, queue_size=1)
        
        rospy.Timer(rospy.Duration(1), self.publish_settings)

    def publish_settings(self, _):
        if not rospy.is_shutdown():
            self.wheel_stiffness_pub.publish(self.wheel_stiffness_slider.value())