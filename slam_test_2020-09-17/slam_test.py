#!/usr/bin/env python3

import os
import pprint
import shlex
import subprocess
import signal
from multiprocessing import Process, Manager
from time import sleep

# source environment
def source_envfile(envfile_path):
    command = shlex.split("env -i bash -c 'source {} && env'".format(envfile_path))
    source_proc = subprocess.Popen(command, stdout = subprocess.PIPE)
    for line in source_proc.stdout:
        (key, _, value) = line.decode('utf-8').partition("=")
        os.environ[key] = value.rstrip()


# launch the slam framework
def launch_slam_framework():
    slam_launch_proc.communicate()


# play the rosbag
def play_bag(dataset_path):
    command = "rosbag play --clock " + dataset_path
    play_proc = subprocess.Popen(shlex.split(command), env = os.environ.copy())
    play_proc.communicate()


# get the pids
def get_pids(units):
    ps_proc = subprocess.Popen(shlex.split('ps -ef'), stdout=subprocess.PIPE)
    grep_proc = subprocess.Popen(shlex.split("grep -E \'" + units + "\'"), stdin=ps_proc.stdout, stdout=subprocess.PIPE)
    grep_v_proc = subprocess.Popen(shlex.split('grep -v grep'), stdin=grep_proc.stdout, stdout=subprocess.PIPE)
    awk_proc = subprocess.Popen(shlex.split('awk "{print $2}"'), stdin=grep_v_proc.stdout, stdout=subprocess.PIPE)
    pid_list = [i.decode('utf-8').strip() for i in awk_proc.stdout.readlines()]
    awk_proc.communicate()
    pids = ','.join(pid_list)
    print("The pids of slam unit are: ",pids)
    return pids


# get the top results
def get_result(pids):
    command = shlex.split("top -b -d 1 -p " + pids)
    top_proc = subprocess.Popen(command, stdout=subprocess.PIPE)
    while True:
        top_list = top_proc.stdout.readline().decode('utf-8').split()
        if len(top_list) != 0 and top_list[0] in pids:
            memory.append(top_list[5])
            cpu.append(top_list[8])


# write the results to text
def write_result(cpu_result_list, memory_result_list, save_result_path, num_unit):
    i = 0
    temp_cpu, temp_memory = [], []
    result = []
    while i < (len(cpu_result_list) - num_unit):
        temp_cpu.append(float(cpu_result_list[i]))
        temp_memory.append(float(memory_result_list[i]))
        if len(temp_cpu) == num_unit:
            a = str(sum(temp_cpu)) + ',' + str(sum(temp_memory)) + '\n'
            result.append(a)
            temp_cpu, temp_memory = [], []
        i += 1    
    f = open(save_result_path, 'w')
    for j in result:
        f.write(j)
    f.close()


if __name__ == "__main__":
    # get some names
    framework_name = input("Input the slam framework name you select (1 - hdl_graph_slam, 2 - LeGO_LOAM, 3 - LIO-SAM): ")
    ros_source_file_path = input("Input the path of ros source file path (for example: /opt/ros/melodic/setup.bash): ")
    framework_source_file_path = input("Input the path of slam framework source file (for example: /home/user/hdl_ws/devel/setup.bash): ")
    dataset_path = input("Input the path of dataset you use (for example: /home/user/a.bag **make sure that the slam framework you choose could launch using this dataset**): ")
    save_result_path = input("Input the path of saving result (for example: /home/user/result.txt): ")

    # choose slam launching commands
    if framework_name == '1':
        slam_command = 'roslaunch hdl_graph_slam hdl_graph_slam.launch'
        slam_units = "nodelet"
        num_unit = 5
    elif framework_name == '2':
        slam_command = 'roslaunch lego_loam run.launch'
        slam_units = "featureAss|imagePro|mapOptmiza|transform"
        num_unit = 4
    else:
        slam_command = 'roslaunch lio_sam run.launch'
        slam_units = "lio_sam_fe|lio_sam_image|lio_sam_imu|lio_sam_ma"
        num_unit = 4

    # set temp lists
    mp_manager = Manager()
    cpu = mp_manager.list()
    memory = mp_manager.list()

    # source the environment
    source_envfile(ros_source_file_path)
    source_envfile(framework_source_file_path)

    # start the slam proc
    slam_launch_proc = subprocess.Popen(shlex.split(slam_command), env=os.environ.copy(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    slam_proc = Process(target=launch_slam_framework)
    slam_proc.start()
    sleep(2)

    # start the top proc
    pids = get_pids(slam_units)
    top_proc = Process(target=get_result, args = (pids,))
    top_proc.start()

    # start the play bag proc
    playbag_proc = Process(target = play_bag, args = (dataset_path,))
    playbag_proc.start()
    
    # stop the processes
    playbag_proc.join()
    sleep(2)
    top_proc.terminate()
    slam_proc.terminate()
    slam_launch_proc.send_signal(signal.SIGINT)

    # write to file
    write_result(cpu, memory, save_result_path, num_unit)
