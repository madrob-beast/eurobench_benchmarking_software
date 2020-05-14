import bms_utils
from os import path


def preprocessed_csv_file_path(preprocess_dir, benchmark_group, robot_name, run_number, start_time, data_type):
    file_name = 'subject_{robot_name}_{benchmark_group}_{data_type}_{run_number:03d}_{start_time}.csv'.format(
        robot_name=robot_name,
        benchmark_group=benchmark_group,
        data_type=data_type,
        run_number=run_number,
        start_time=start_time.strftime('%Y%m%d_%H%M%S'))
    file_path = path.join(preprocess_dir, file_name)

    return file_path
