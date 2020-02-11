import bms_utils
from os import path

def open_preprocessed_csv(preprocess_dir, benchmark_group, robot_name, run_number, start_time, data_type):
    filename = 'subject_%s_%s_%s_%03d_%s.csv' % (robot_name, benchmark_group, data_type, run_number, start_time.strftime('%Y%m%d_%H%M%S'))
    file = open(path.join(preprocess_dir, filename), 'w+')

    header = 'timestamp,' + data_type
    file.write(header + '\n')

    return file