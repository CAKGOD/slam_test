#!/usr/bin/env python3

import os
import pprint
import shlex
import subprocess
import signal
from multiprocessing import Process, Manager
from time import sleep
import time

start = time.time()

def source_envfile(envfile_path):
    command = shlex.split("env -i bash -c 'source {} && env'".format(envfile_path))
    proc = subprocess.Popen(command, stdout = subprocess.PIPE)
    for line in proc.stdout:
        (key, _, value) = line.decode('utf-8').partition("=")
        os.environ[key] = value.rstrip()

def init_env():
    source_envfile('/opt/ros/noetic/setup.bash')
    source_envfile('/opt/ros/foxy/setup.bash')
    source_envfile('/home/cakgod/lidarslam_ros2/install/setup.bash')

init_env()


def get_pid(cmd_filter):
    ps_proc = subprocess.Popen(shlex.split('ps -ef'), stdout=subprocess.PIPE)
    grep_proc = subprocess.Popen(shlex.split('grep {}'.format(cmd_filter)), stdin=ps_proc.stdout, stdout=subprocess.PIPE)
    grep_v_proc = subprocess.Popen(shlex.split('grep -v grep'), stdin=grep_proc.stdout, stdout=subprocess.PIPE)
    awk_proc = subprocess.Popen(shlex.split('awk "{print $2}"'), stdin=grep_v_proc.stdout, stdout=subprocess.PIPE)

    pid = awk_proc.stdout.readline().decode('utf-8').strip()
    awk_proc.communicate()
    return pid


def run_cmd(cmd_str, env, output=None):
    proc = subprocess.Popen(shlex.split(cmd_str), env=env, stdout=output)
    proc.communicate()


# pipe to subprocessing.PIPE will hide the 'SIGINT' handling error on console screen
lidar_proc = subprocess.Popen(shlex.split('ros2 launch lidarslam lidarslam.launch.py'), env=os.environ.copy(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
def run_lidar_cmd():
    lidar_proc.communicate()


mp_manager = Manager()
perf_results = mp_manager.list()
def parse_top_result(pid):
    print('parse top result for ', pid)
    proc = subprocess.Popen(shlex.split('top -b -d 2 -p ' + pid), stdout=subprocess.PIPE)

    while True:
        output = proc.stdout.readline().decode('utf-8').strip()
        if output.find(pid) == 0:
            parts = output.split()
            res_mem, cpu_percentage = parts[5], parts[8]
            # print('res(KiB): {}, res(MiB): {}, cpu%: {}'.format(res_mem, round(float(res_mem)/1024, 2), cpu_percentage))
            print('res(KiB): {}, cpu%: {}'.format(res_mem, cpu_percentage))
            perf_results.append([res_mem, cpu_percentage])

# Here we run the commands!
slam_proc = Process(target=run_lidar_cmd)
slam_proc.start()
sleep(2)

top_proc = Process(target=parse_top_result, args=(get_pid('scanmatcher'),))
top_proc.start()

'''
playbag_proc_0 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-49-12_0.bag', os.environ.copy()))
playbag_proc_1 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-50-12_1.bag', os.environ.copy()))
playbag_proc_2 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-51-12_2.bag', os.environ.copy()))
playbag_proc_3 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-52-12_3.bag', os.environ.copy()))
playbag_proc_4 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-53-12_4.bag', os.environ.copy()))
playbag_proc_5 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-54-12_5.bag', os.environ.copy()))
playbag_proc_6 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-55-12_6.bag', os.environ.copy()))
playbag_proc_7 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-56-12_7.bag', os.environ.copy()))
playbag_proc_8 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-57-12_8.bag', os.environ.copy()))
playbag_proc_9 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-58-12_9.bag', os.environ.copy()))
playbag_proc_10 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-14-59-12_10.bag', os.environ.copy()))
playbag_proc_11 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-00-12_11.bag', os.environ.copy()))
playbag_proc_12 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-01-12_12.bag', os.environ.copy()))
playbag_proc_13 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-02-12_13.bag', os.environ.copy()))
playbag_proc_14 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-03-12_14.bag', os.environ.copy()))
playbag_proc_15 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-04-12_15.bag', os.environ.copy()))
playbag_proc_16 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-05-12_16.bag', os.environ.copy()))
playbag_proc_17 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-06-12_17.bag', os.environ.copy()))
playbag_proc_18 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-07-12_18.bag', os.environ.copy()))
playbag_proc_19 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-08-12_19.bag', os.environ.copy()))
playbag_proc_20 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-09-12_20.bag', os.environ.copy()))
playbag_proc_21 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-10-12_21.bag', os.environ.copy()))
playbag_proc_22 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-11-12_22.bag', os.environ.copy()))
playbag_proc_23 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-12-12_23.bag.bag', os.environ.copy()))
playbag_proc_24 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-13-12_24.bag', os.environ.copy()))
playbag_proc_25 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-14-12_25.bag', os.environ.copy()))
playbag_proc_26 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-15-12_26.bag', os.environ.copy()))
playbag_proc_27 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-16-12_27.bag', os.environ.copy()))
playbag_proc_28 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-17-12_28.bag', os.environ.copy()))
playbag_proc_29 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-18-12_29.bag', os.environ.copy()))
playbag_proc_30 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-19-12_30.bag', os.environ.copy()))
playbag_proc_31 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-20-12_31.bag', os.environ.copy()))
playbag_proc_32 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-21-12_32.bag', os.environ.copy()))
playbag_proc_33 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-22-12_33.bag', os.environ.copy()))
playbag_proc_34 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-23-12_34.bag', os.environ.copy()))
playbag_proc_35 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-24-12_35.bag', os.environ.copy()))
playbag_proc_36 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-25-12_36.bag', os.environ.copy()))
playbag_proc_37 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-26-12_37.bag', os.environ.copy()))
playbag_proc_38 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-27-12_38.bag', os.environ.copy()))
playbag_proc_39 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /home/cak/dataset/kitti/2018-05-18-15-28-12_39.bag', os.environ.copy()))
playbag_proc_0.start()
playbag_proc_0.join()
playbag_proc_1.start()
playbag_proc_1.join()
playbag_proc_2.start()
playbag_proc_2.join()
playbag_proc_3.start()
playbag_proc_3.join()
playbag_proc_4.start()
playbag_proc_4.join()
playbag_proc_5.start()
playbag_proc_5.join()
playbag_proc_6.start()
playbag_proc_6.join()
playbag_proc_7.start()
playbag_proc_7.join()
playbag_proc_8.start()
playbag_proc_8.join()
playbag_proc_9.start()
playbag_proc_9.join()
playbag_proc_10.start()
playbag_proc_10.join()
playbag_proc_11.start()
playbag_proc_11.join()
playbag_proc_12.start()
playbag_proc_12.join()
playbag_proc_13.start()
playbag_proc_13.join()
playbag_proc_14.start()
playbag_proc_14.join()
playbag_proc_15.start()
playbag_proc_15.join()
playbag_proc_16.start()
playbag_proc_16.join()
playbag_proc_17.start()
playbag_proc_17.join()
playbag_proc_18.start()
playbag_proc_18.join()
playbag_proc_19.start()
playbag_proc_19.join()
playbag_proc_20.start()
playbag_proc_20.join()
playbag_proc_21.start()
playbag_proc_21.join()
playbag_proc_22.start()
playbag_proc_22.join()
playbag_proc_23.start()
playbag_proc_23.join()
playbag_proc_24.start()
playbag_proc_24.join()
playbag_proc_25.start()
playbag_proc_25.join()
playbag_proc_26.start()
playbag_proc_26.join()
playbag_proc_27.start()
playbag_proc_27.join()
playbag_proc_28.start()
playbag_proc_28.join()
playbag_proc_29.start()
playbag_proc_29.join()
playbag_proc_30.start()
playbag_proc_30.join()
playbag_proc_31.start()
playbag_proc_31.join()
playbag_proc_32.start()
playbag_proc_32.join()
playbag_proc_33.start()
playbag_proc_33.join()
playbag_proc_34.start()
playbag_proc_34.join()
playbag_proc_35.start()
playbag_proc_35.join()
playbag_proc_36.start()
playbag_proc_36.join()
playbag_proc_37.start()
playbag_proc_37.join()
playbag_proc_38.start()
playbag_proc_38.join()
playbag_proc_39.start()
playbag_proc_39.join()
'''
playbag_proc_0 = Process(target=run_cmd, args=('ros2 bag play -s rosbag_v2 /mnt/share_host_virtual/dataset/kitti/10.bag', os.environ.copy()))
playbag_proc_0.start()
playbag_proc_0.join()

print('wait 10 secs for top command')
sleep(10)
top_proc.terminate()

# Notice: the ros2 launch will handle SIGINT(simulate a ctrl-c) and release all resources it occupied.
slam_proc.terminate()
lidar_proc.send_signal(signal.SIGINT)


with open('/mnt/share_host_virtual/lidarslam_ros2/kitti_cpu_meomory_pc_scan.txt', 'w') as f:
    content_str = '\n'.join(['{0}\t{1}'.format(*t) for t in perf_results])
    f.write(content_str)
print('finished')

end = time.time()
print('runtime is:', end-start)