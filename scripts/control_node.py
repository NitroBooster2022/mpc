#!/usr/bin/env python3
import time
import numpy as np
import rospy
import threading
import argparse
from utility import Utility
from std_srvs.srv import Trigger
from MPC import MPC
from mpc_acados import MobileRobotOptimizer

def call_trigger_service():
    try:
        trigger_service = rospy.ServiceProxy('trigger_service', Trigger)
        response = trigger_service()
        rospy.loginfo("Service response: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--useEkf', action='store_true', help='Boolean for whether to use ekf node or not')
    parser.add_argument('--odom', action='store_true', help='Boolean for whether to use odometry or gps data')
    parser.add_argument('--sign', action='store_true', help='Boolean for whether to use sign or not')
    args = parser.parse_args()
    # mpc = MPC(gazebo=args.gazebo)
    mpc = MobileRobotOptimizer()
    gaz_bool = "_gazebo_"
    rospy.init_node("mpc_node")
    mpc.real_x = None
    mpc.real_y = None
    mpc.yaw = None
    mpc.lock = threading.Lock()
    timer2 = rospy.Time.now()
    rospy.init_node("mpc_node")
    utils = Utility(subLane=False, subImu=True, subModel=True, pubOdom=True, useEkf=args.useEkf, subSign=args.sign)
    print("gazebo!")
    def spin_thread():
        rospy.spin()
    thread = threading.Thread(target=spin_thread)
    thread.start()
    rate = rospy.Rate(1/mpc.T)
    # wait until the first state is received
    print("waiting for first state...")
    while not utils.initializationFlag:
        time.sleep(0.1)
    mpc.real_x = utils.gps_x
    mpc.real_y = utils.gps_y
    mpc.yaw = utils.yaw
    while (mpc.real_x is None or mpc.real_y is None or mpc.yaw is None):
        time.sleep(0.1)
    print("starting... x: ", mpc.real_x, ", y: ", mpc.real_y, ", yaw: ", mpc.yaw)
    mpc.detected_index = 0
    # stop when last waypoint is reached
    length = len(mpc.waypoints_x)
    mpc.target_waypoint_index = 0
    print("length: ", length)
    while True:
        if mpc.target_waypoint_index >= mpc.num_waypoints-1:
        # if mpc.target_waypoint_index >= 375:
            break
        t = time.time()
        if args.sign:
            distance = utils.object_detected(12)
            if distance>0 and mpc.target_waypoint_index>=mpc.detected_index:
                distance = max(distance, 0.75)-0.75 #0.75 is minimum distance
                print("car detected at a distance of: ", distance+0.75)
                offset = int(distance*mpc.density)
                mpc.path.detected_index = mpc.target_waypoint_index+20
                mpc.path.change_lane(mpc.target_waypoint_index+offset, mpc.detected_index+offset, mpc.wp_normals)
        with mpc.lock:
            if args.odom:
                mpc.update_current_state(x=utils.odomX, y=utils.odomY, yaw=utils.yaw)
                mpc.update_real_state(x=utils.gps_x, y=utils.gps_y, yaw=utils.yaw)
            else:
                mpc.update_current_state(x=utils.gps_x, y=utils.gps_y, yaw=utils.yaw)
        mpc.x_errors.append(utils.gps_x - mpc.next_trajectories[0, 0])
        mpc.y_errors.append(utils.gps_y - mpc.next_trajectories[0, 1])
        mpc.x_refs.append(mpc.next_trajectories[0, :])
        mpc.yaw_errors.append(mpc.current_state[2] - mpc.next_trajectories[0, 2])
        t_ = time.time()
        u_res = mpc.update_and_solve()
        t2 = time.time()- t_
        mpc.index_t.append(t2)
        mpc.integrate_next_states(u_res)
        # u_res = u_res[0]
        # print("time: ", t2, "u_res: ", u_res)
        mpc.xx.append(mpc.real_state) if args.odom else mpc.xx.append(mpc.current_state)
        mpc.mpciter = mpc.mpciter + 1
        utils.steer_command = -float(u_res[1]*180/np.pi)
        utils.velocity_command = float(u_res[0])
        utils.publish_cmd_vel(steering_angle=utils.steer_command, velocity=utils.velocity_command)
        rate.sleep()
        now = rospy.Time.now()
        print("iteration time: ", (now-timer2).to_sec())
        timer2 = now
        print("i) ", mpc.mpciter,"ref_traj:", np.around(mpc.next_trajectories, decimals=2)[0], 
            "cur:", np.around(mpc.current_state, decimals=2), "ctrl:", 
            np.around(u_res, decimals=2), "idx:", mpc.target_waypoint_index, "time:", round(t2, 4),
            "kappa:", round(mpc.kappa[mpc.target_waypoint_index],2))
    print("done")
    try:
        call_trigger_service()
    except:
        print("service not available")
    ## draw function
    mpc.draw_result(mpc.compute_stats())