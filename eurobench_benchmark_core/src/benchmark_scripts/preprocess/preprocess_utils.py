import bms_utils
from os import path

def open_preprocessed_csv(benchmark_group, robot_name, run_number, start_time, data_type):
    output_dir = bms_utils.get_output_dir()

    filename = 'subject_%s_%s_%s_%03d_%s.csv' % (robot_name, benchmark_group, data_type, run_number, start_time.strftime('%Y%m%d_%H%M%S'))
    file = open(path.join(output_dir, filename), 'w+')

    header = 'timestamp,' + data_type
    file.write(header + '\n')

    return file