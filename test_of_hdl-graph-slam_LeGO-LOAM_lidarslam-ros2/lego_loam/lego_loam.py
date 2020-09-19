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
    source_envfile('/opt/ros/melodic/setup.bash')
    source_envfile('/home/cakgod/LeGO_LOAM/devel/setup.bash')

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

'''
lidar_proc1 = subprocess.Popen(shlex.split('rosparam set use_sim_time true'), env=os.environ.copy(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
def run_lidar_cmd1():
        lidar_proc1.communicate()
'''

# pipe to subprocessing.PIPE will hide the 'SIGINT' handling error on console screen
lidar_proc = subprocess.Popen(shlex.split('roslaunch lego_loam run.launch'), env=os.environ.copy(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
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
            print('res(KiB): {}, res(MiB): {}, cpu%: {}'.format(res_mem, round(float(res_mem)/1024, 2), cpu_percentage))
            perf_results.append([res_mem, cpu_percentage])

# Here we run the commands!
slam_proc = Process(target=run_lidar_cmd)
slam_proc.start()
sleep(2)

top_proc = Process(target=parse_top_result, args=(get_pid('mapOptmization'),))
top_proc.start()

'''
playbag_proc = Process(target=run_cmd, args=('rosbag play /home/cak/dataset/kitti/2018-05-18-14-49-12_0.bag /home/cak/dataset/kitti/2018-05-18-14-50-12_1.bag /home/cak/dataset/kitti/2018-05-18-14-51-12_2.bag /home/cak/dataset/kitti/2018-05-18-14-52-12_3.bag /home/cak/dataset/kitti/2018-05-18-14-53-12_4.bag /home/cak/dataset/kitti/2018-05-18-14-54-12_5.bag /home/cak/dataset/kitti/2018-05-18-14-55-12_6.bag /home/cak/dataset/kitti/2018-05-18-14-56-12_7.bag /home/cak/dataset/kitti/2018-05-18-14-57-12_8.bag /home/cak/dataset/kitti/2018-05-18-14-58-12_9.bag /home/cak/dataset/kitti/2018-05-18-14-59-12_10.bag /home/cak/dataset/kitti/2018-05-18-15-00-12_11.bag /home/cak/dataset/kitti/2018-05-18-15-01-12_12.bag  /home/cak/dataset/kitti/2018-05-18-15-02-12_13.bag /home/cak/dataset/kitti/2018-05-18-15-03-12_14.bag /home/cak/dataset/kitti/2018-05-18-15-04-12_15.bag /home/cak/dataset/kitti/2018-05-18-15-05-12_16.bag /home/cak/dataset/kitti/2018-05-18-15-06-12_17.bag /home/cak/dataset/kitti/2018-05-18-15-07-12_18.bag /home/cak/dataset/kitti/2018-05-18-15-08-12_19.bag /home/cak/dataset/kitti/2018-05-18-15-09-12_20.bag /home/cak/dataset/kitti/2018-05-18-15-10-12_21.bag  /home/cak/dataset/kitti/2018-05-18-15-11-12_22.bag /home/cak/dataset/kitti/2018-05-18-15-12-12_23.bag /home/cak/dataset/kitti/2018-05-18-15-13-12_24.bag /home/cak/dataset/kitti/2018-05-18-15-14-12_25.bag /home/cak/dataset/kitti/2018-05-18-15-15-12_26.bag /home/cak/dataset/kitti/2018-05-18-15-16-12_27.bag /home/cak/dataset/kitti/2018-05-18-15-17-12_28.bag /home/cak/dataset/kitti/2018-05-18-15-18-12_29.bag /home/cak/dataset/kitti/2018-05-18-15-19-12_30.bag /home/cak/dataset/kitti/2018-05-18-15-20-12_31.bag /home/cak/dataset/kitti/2018-05-18-15-21-12_32.bag /home/cak/dataset/kitti/2018-05-18-15-22-12_33.bag /home/cak/dataset/kitti/2018-05-18-15-23-12_34.bag /home/cak/dataset/kitti/2018-05-18-15-24-12_35.bag /home/cak/dataset/kitti/2018-05-18-15-25-12_36.bag /home/cak/dataset/kitti/2018-05-18-15-26-12_37.bag /home/cak/dataset/kitti/2018-05-18-15-27-12_38.bag /home/cak/dataset/kitti/2018-05-18-15-28-12_39.bag', os.environ.copy()))
'''
playbag_proc = Process(target=run_cmd, args=('rosbag play --clock /mnt/shared/dataset/HDL/hdl_400.bag', os.environ.copy()))
playbag_proc.start()
playbag_proc.join()

print('wait 10 secs for top command')
sleep(10)
top_proc.terminate()

# Notice: the ros2 launch will handle SIGINT(simulate a ctrl-c) and release all resources it occupied.
slam_proc.terminate()
lidar_proc.send_signal(signal.SIGINT)


with open('/mnt/shared/lego_loam/400_cpu_memory_mapOptmization_pc.txt', 'w') as f:
    content_str = '\n'.join(['{0}\t{1}'.format(*t) for t in perf_results])
    f.write(content_str)
print('finished')

end = time.time()
print('run time is:', end-start)
