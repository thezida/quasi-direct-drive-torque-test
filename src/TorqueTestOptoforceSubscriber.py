#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import std_msgs.msg
from odrive_controller.msg import ODriveMsg
import matplotlib.pyplot as plt
import math
import threading
import os
import time


fxs = []
fys = []
fzs = []
fs = []
trq_sensor = []
time_axis_sensor = []
trq_measured = []
time_axis_measured = []
trq_reference = []
time_axis_reference = []
motor_curr = []
time_axis_mc = []
motor_volt_q = []
time_axis_mvq = []
motor_volt_d = []
time_axis_mvd = []
odrive_curr = []
time_axis_oc = []
odrive_volt = []
time_axis_ov = []
position_measured = []
time_axis_pos = []
velocity_measured = []
time_axis_vel = []

record_flag = False
crank_length = 0.17  # 0.115 # meters
transmission_ratio = 6
number_of_samples = 12000
dt = 0.001 # seconds
step_counter = 0
lock = threading.Lock()

def transmission_friction(motor_torque, dtrq):
    if (dtrq>0):
        return -0.4169 * motor_torque - 0.002
    else:
        return 0.4169 * motor_torque + 0.002 #0.634 * motor_torque + 0.1074

def optoforce_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    #rospy.loginfo(">>> fx : %f fy : %f fz : %f", data.wrench.force.x, data.wrench.force.y, data.wrench.force.z)
    fxs.append(data.wrench.force.x)
    fys.append(data.wrench.force.y)
    fzs.append(data.wrench.force.z)
    fs.append(math.sqrt(fxs[-1]**2 + fys[-1]**2 + fzs[-1]**2))  # fxs[-1]**2 + ommited
    trq_sensor.append(fs[-1]*crank_length)
    time_axis_sensor.append(step_counter*dt)

def measured_torque_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    trq_measured.append(data.data)
    time_axis_measured.append(step_counter*dt)

def reference_torque_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    trq_reference.append(data.data)
    time_axis_reference.append(step_counter*dt)

def motor_current_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    motor_curr.append(data.data)
    time_axis_mc.append(step_counter * dt)

def motor_voltage_q_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    motor_volt_q.append(data.data)
    time_axis_mvq.append(step_counter * dt)

def motor_voltage_d_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    motor_volt_d.append(data.data)
    time_axis_mvd.append(step_counter * dt)

def odrive_current_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    odrive_curr.append(data.data)
    time_axis_oc.append(step_counter * dt)

def odrive_voltage_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    odrive_volt.append(data.data)
    time_axis_ov.append(step_counter * dt)

def position_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    position_measured.append(data.data)
    time_axis_pos.append(step_counter * dt)

def velocity_callback(data):
    lock.acquire()
    flag = record_flag
    if (flag == False):
        lock.release()
        return
    lock.release()
    velocity_measured.append(data.data)
    time_axis_vel.append(step_counter * dt)


def torque_test():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print("Initializing node...")
    rospy.init_node('Optoforce_listener', anonymous=True)

    state_pub = rospy.Publisher("odrive0/axis0/requested_state/write", ODriveMsg, queue_size=1)
    torque_pub = rospy.Publisher("odrive0/axis0/torque/write", ODriveMsg, queue_size=10)

    rospy.Subscriber("optoforce_node/OptoForceWrench", geometry_msgs.msg.WrenchStamped, optoforce_callback)
    rospy.Subscriber("odrive0/axis0/torque/read", std_msgs.msg.Float64, measured_torque_callback)
    rospy.Subscriber("odrive0/axis0/torque/reference_from_current/read", std_msgs.msg.Float64, reference_torque_callback)
    rospy.Subscriber("odrive0/axis0/motor_current/measured/read", std_msgs.msg.Float64, motor_current_callback)
    rospy.Subscriber("odrive0/axis0/motor_voltage_q/read", std_msgs.msg.Float64, motor_voltage_q_callback)
    rospy.Subscriber("odrive0/axis0/motor_voltage_d/read", std_msgs.msg.Float64, motor_voltage_d_callback)
    rospy.Subscriber("odrive0/odrive_current/read", std_msgs.msg.Float64, odrive_current_callback)
    rospy.Subscriber("odrive0/odrive_voltage/read", std_msgs.msg.Float64, odrive_voltage_callback)
    rospy.Subscriber("odrive0/axis0/position/read", std_msgs.msg.Float64, position_callback)
    rospy.Subscriber("odrive0/axis0/velocity/read", std_msgs.msg.Float64, velocity_callback)

    print("Initializing experiment...")
    global number_of_samples
    global dt
    # array initialization
    trq_ref = [0 for i in range(number_of_samples)]
    time_axis = [0 for i in range(number_of_samples)]

    input_type = 2 # 0 - step, 1 - ramp, 2 - sine, 3 - saw

    # Four step input
    if (input_type==0):
        first_trq_ref = -0.4
        second_trq_ref = -0.7
        third_trq_ref = -0.95
        fourth_trq_ref = -1.0
        first = int(2 / dt)
        second = int(4 / dt)
        third = int(6 / dt)
        fourth = int(8 / dt)
        for i in range(number_of_samples):
            if (i > fourth):
                trq_ref[i] = fourth_trq_ref
            elif (i > third):
                trq_ref[i] = third_trq_ref
            elif (i > second):
                trq_ref[i] = second_trq_ref
            elif (i > first):
                trq_ref[i] = first_trq_ref
    # ramp input
    if (input_type==1):
        trq_start = 0
        trq_end = -1.1
        number_of_flat_samples = 2000
        dtrq = (trq_end-trq_start) / (number_of_samples - number_of_flat_samples)
        for i in range(number_of_samples):
            if (i<(number_of_samples-number_of_flat_samples)):
                trq_ref[i] = i * dtrq + trq_start
            else:
                trq_ref[i] = trq_end
    # sine input
    if (input_type==2):
        trq_amp = -0.5
        trq_off = trq_amp
        trq_w = 2 * math.pi * 10 / number_of_samples
        for i in range(number_of_samples):
            trq_ref[i] = trq_amp * math.sin(trq_w * i) + trq_off

    if (input_type==3):
        trq_start = 0
        trq_end = -1.3
        dtrq = (trq_end - trq_start)*2 / (number_of_samples - 0)
        for i in range(number_of_samples):
            if (i < number_of_samples/2):
                trq_ref[i] = i * dtrq + trq_start
            else:
                trq_ref[i] = trq_end - (i - number_of_samples/2) * dtrq

    ready_state = ODriveMsg()
    ready_state.odrv = 0
    ready_state.axis = 0
    ready_state.input_value = 8
    stop_state = ODriveMsg()
    stop_state.odrv = 0
    stop_state.axis = 0
    stop_state.input_value = 1

    torque_msg = ODriveMsg()

    rate = rospy.Rate(1/dt)  # 100hz
    global step_counter
    print("Experiment starts in")
    for i in range(5):
        print(str(5 - i) + "secs")
        rospy.sleep(1)
    print("Starting...")
    while not rospy.is_shutdown():
        dtrq = 0
        if (step_counter==0):
            lock.acquire()
            global record_flag
            record_flag = True
            lock.release()
            state_pub.publish(ready_state)
            dtrq = trq_ref[0]
        else:
            dtrq = trq_ref[step_counter]-trq_ref[step_counter-1]
        torque_msg.odrv = 0
        torque_msg.axis = 0
        input_torque = trq_ref[step_counter] + transmission_friction(trq_ref[step_counter], dtrq)/6
        input_torque = min(1.5, input_torque)
        torque_msg.input_value = input_torque
        torque_pub.publish(torque_msg)
        time_axis[step_counter] = dt*step_counter
        step_counter += 1
        print str(step_counter*100.0/number_of_samples)+"%\r",
        if (step_counter>=number_of_samples):
            print "Exeperiment done."
            lock.acquire()
            record_flag = False
            lock.release()
            state_pub.publish(stop_state)
            break
        rate.sleep()

    path = "/home/martindzida/Diplomski/TorqueTest/" + str(time.time())
    try:
        os.mkdir(path)
    except OSError:
        print("Creation of the directory %s failed" % path)
    else:
        print("Successfully created the directory %s " % path)

    print "Plotting..."
    plt.figure()
    plt.plot(time_axis_sensor, fxs, label="fx")
    plt.xlabel("time [s]")
    plt.ylabel("force [N]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/fx.png"))
    plt.show()

    plt.figure()
    plt.plot(time_axis_sensor, fys, label="fy")
    plt.xlabel("time [s]")
    plt.ylabel("force [N]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/fy.png"))
    plt.show()

    plt.figure()
    plt.plot(time_axis_sensor, fzs, label="fz")
    plt.xlabel("time [s]")
    plt.ylabel("force [N]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/fz.png"))
    plt.show()

    plt.figure()
    plt.plot(time_axis_sensor, fs, label="f")
    plt.xlabel("time [s]")
    plt.ylabel("force [N]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/f.png"))
    plt.show()

    trq_ref_transmission = [-trq_ref[i]*transmission_ratio for i in range(len(trq_ref))]
    plt.figure()
    plt.plot(time_axis_measured, trq_measured, label="odrive_torque")
    plt.plot(time_axis_reference, trq_reference, label="reference_torque")
    plt.plot(time_axis_sensor, trq_sensor, "r", label="optoforce_torque")
    plt.plot(time_axis, trq_ref_transmission, label="transmission_reference_torque")
    plt.ylabel("torque [Nm]")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/torque.png"))
    plt.show()

    plt.figure()
    plt.plot(time_axis_mc, motor_curr, label="motor current")
    plt.ylabel("A")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/motor_current.png"))
    plt.show()

    plt.figure()
    plt.plot(time_axis_oc, odrive_curr, label="odrive current")
    plt.ylabel("A")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/odrive_current.png"))
    plt.show()

    plt.figure()
    plt.plot(time_axis_pos, position_measured, label="position")
    plt.ylabel("turns")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/position.png"))
    plt.show()

    plt.figure()
    plt.plot(time_axis_vel, velocity_measured, label="velocity")
    plt.ylabel("turns/s")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/velocity.png"))
    plt.show()

    plt.figure()
    plt.plot(time_axis_mvd, motor_volt_d, label="voltage_d")
    plt.plot(time_axis_mvq, motor_volt_q, label="voltage_q")
    plt.ylabel("V")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/motor_voltage.png"))
    plt.show()

    plt.figure()
    plt.plot(time_axis_ov, odrive_volt, label="odrive voltage")
    plt.ylabel("V")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/odrive_voltage.png"))
    plt.show()

    odrive_power = [odrive_volt[i]*odrive_curr[i] for i in range(min(len(odrive_curr), len(odrive_volt)))]
    motor_power = [motor_curr[i]*motor_volt_q[i] for i in range(min(len(motor_curr), len(motor_volt_q)))]
    plt.figure()
    plt.plot(time_axis_oc, odrive_power, label="odrive power")
    plt.plot(time_axis_mc, motor_power, label="motor power")
    plt.ylabel("W")
    plt.xlabel("time [s]")
    plt.legend()
    plt.grid(True)
    plt.savefig(path + str("/power.png"))
    plt.show()

    f = open(path+str("/sensor_output.csv"), "w")
    for i in range(len(trq_sensor)):
        f.write(str(time_axis_sensor[i])+","+str(trq_sensor[i])+"\n")
    f.close()
    f = open(path + str("/motor_output.csv"), "w")
    for i in range(len(trq_ref_transmission)):
        f.write(str(time_axis[i]) + "," + str(trq_ref_transmission[i]) + "\n")
    f.close()



if __name__ == '__main__':
    torque_test()
