import rospy
from os import path, makedirs

# Gets the output directory path. Creates the dir if it doesn't exist
def get_output_dir():
    output_dir = path.expanduser(rospy.get_param('benchmark_output_directory'))
    if (not path.exists(output_dir)) or path.isfile(output_dir):
        rospy.loginfo('Creating output dir: %s' % (output_dir))
        makedirs(output_dir)

    return output_dir