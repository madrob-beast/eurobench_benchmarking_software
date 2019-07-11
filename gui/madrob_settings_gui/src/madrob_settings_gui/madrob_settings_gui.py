import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QSlider

from std_msgs.msg import UInt16


class madrob_settings_gui(Plugin):
    def __init__(self, context):
        super(madrob_settings_gui, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MADROB Settings')

        self._widget = QWidget()

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'madrob_settings_gui.ui')

        loadUi(ui_file, self._widget)
        
        self._widget.setObjectName('MADROB Settings')
        self._widget.setWindowTitle('MADROB Settings')

        self.stiffness_value_label = self._widget.findChild(QLabel, 'stiffness_value_label')
        self.offset_value_label = self._widget.findChild(QLabel, 'offset_value_label')

        self.stiffness_slider = self._widget.findChild(QSlider, 'stiffness_slider')
        self.stiffness_slider.valueChanged.connect(self.update_stiffness)

        self.offset_slider = self._widget.findChild(QSlider, 'offset_slider')
        self.offset_slider.valueChanged.connect(self.update_offset)

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

    def update_stiffness(self, value):
        self.stiffness_value_label.setText(str(value) + ' N')

    def update_offset(self, value):
        self.offset_value_label.setText(str(value) + ' deg')

    def run_rospy_node(self):
        self.door_stiffness_pub = rospy.Publisher('madrob/door_stiffness', UInt16, queue_size=1)
        self.obstacle_offset_pub = rospy.Publisher('madrob/obstacle_offset', UInt16, queue_size=1)
        
        rospy.Timer(rospy.Duration(1), self.publish_settings)

    def publish_settings(self, _):
        if not rospy.is_shutdown():
            self.door_stiffness_pub.publish(self.stiffness_slider.value())
            self.obstacle_offset_pub.publish(self.offset_slider.value())