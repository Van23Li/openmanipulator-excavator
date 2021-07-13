#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MoveItFkDemo(object):
    """MoveItFkDemo"""

    def __init__(self):
        super(MoveItFkDemo, self).__init__()

        ###############################Basic Settings##################################################
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
        
        # Instantiate a 'RobotCommander'_object. Provides information such as
        # the robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        # Instantiate a 'PlanningSceneInterface'_object. Provides a remote interface
        # for getting, setting, and updating the robot's internal understanding of the
        # surrounding world
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a' MoveGroupCommander '_bject. This object is an interface to
        # a planning group (group of joints).
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        
        # Create a DisplayTrajectory ROS publisher which is used to display trajectories
        # in Rviz:
        display_trajectory_publisher = rospy.Publisher("/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)



        ################################Getting Basic Information##################################################
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", group_names)

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print("============ Printing robot state: %s")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.table_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # 设置机械臂运动的允许误差值
        tolerance = 0.001
        self.move_group.set_goal_joint_tolerance(tolerance)

        # 设置允许的最大速度和加速度
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        
        # 控制机械臂先回到初始化位置
        self.move_group.set_named_target('home')
        # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
        self.move_group.go()
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        # rospy.sleep(1)
        
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 1
        joint_goal[1] = -1.5
        joint_goal[2] = 0.8
        joint_goal[3] = -0.7

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        
        # for testing:
        current_joints = self.move_group.get_current_joint_values()
        if all_close(joint_goal, current_joints, tolerance):
            print("============ Success")
        else:
            print("============ Fail")
        print("============ Joint goal: %s" % joint_goal)
        print("============ Current joints: %s" % current_joints)

    def go_to_pose_goal(self):
        # 获取终端link的名称
        end_effector_link = self.move_group.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        self.move_group.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.move_group.allow_replanning(True)

        # 设置机械臂运动的允许误差值
        tolerance = 0.001
        self.move_group.set_goal_joint_tolerance(tolerance)

        # 设置允许的最大速度和加速度
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        
        # 控制机械臂先回到初始化位置
        self.move_group.set_named_target('home')
        # The go command can be called with joint values, poses, or without
        # any parameters if you have already set the pose or joint target for the group
        self.move_group.go()
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        # rospy.sleep(1)
        
        #测试避障
        current_pose = self.move_group.get_current_pose().pose
        target_pose = geometry_msgs.msg.PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = current_pose.position.x
        target_pose.pose.position.y = current_pose.position.y
        target_pose.pose.position.z = current_pose.position.z - 0.1
        target_pose.pose.orientation.x = current_pose.orientation.x
        target_pose.pose.orientation.y = current_pose.orientation.y
        target_pose.pose.orientation.z = current_pose.orientation.z
        target_pose.pose.orientation.w = current_pose.orientation.w

        # # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # # 姿态使用四元数描述，基于base_link坐标系
        # target_pose = geometry_msgs.msg.PoseStamped()
        # target_pose.header.frame_id = reference_frame
        # target_pose.header.stamp = rospy.Time.now()     
        # target_pose.pose.position.x = 0.00679945197362
        # target_pose.pose.position.y = -0.00808431981913
        # target_pose.pose.position.z = 0.313538539318
        # target_pose.pose.orientation.x = 0.308489290951
        # target_pose.pose.orientation.y = -0.565254120802
        # target_pose.pose.orientation.z = 0.366506887599
        # target_pose.pose.orientation.w = 0.671561492067

        # 设置机器臂当前的状态作为运动初始状态
        self.move_group.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        self.move_group.set_pose_target(target_pose, end_effector_link)
        
        # 规划运动路径
        traj = self.move_group.plan()
        
        # 按照规划的运动路径控制机械臂运动
        self.move_group.execute(traj)
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        # for testing:
        current_pose = self.move_group.get_current_pose().pose
        if all_close(target_pose, current_pose, tolerance):
            print("============ Success")
        else:
            print("============ Fail")
        print("============ Target pose: %s" % target_pose.pose)
        print("============ Current pose: %s" % current_pose)

    def plan_cartesian_path(self):
        # 获取终端link的名称
        end_effector_link = self.move_group.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        self.move_group.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.move_group.allow_replanning(True)

        # 设置机械臂运动的允许误差值
        tolerance = 0.001
        self.move_group.set_goal_joint_tolerance(tolerance)

        # 设置允许的最大速度和加速度
        self.move_group.set_max_acceleration_scaling_factor(0.5)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        
        # 控制机械臂先回到初始化位置
        self.move_group.set_named_target('home')
        # The go command can be called with joint values, poses, or without
        # any parameters if you have already set the pose or joint target for the group
        self.move_group.go()
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        # rospy.sleep(1)

        # 设置机器臂当前的状态作为运动初始状态
        self.move_group.set_start_state_to_current_state()

        # 初始化路点列表
        waypoints = []

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = self.move_group.get_current_pose(end_effector_link).pose

        # 设置路点数据，并加入路点列表
        wpose = copy.deepcopy(start_pose)
        wpose.position.z += 0.05
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= 0.15
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.z += 0.1
        waypoints.append(copy.deepcopy(wpose))

        # 设置、初始化规划参数
        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
 
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.move_group.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.001,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            # 尝试次数累加
            attempts += 1
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction   
        
    def display_trajectory(self, plan):
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        ## Use execute if you would like the robot to follow the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.table_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.table_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_table(self, timeout=4):
        # 设置桌面的高度
        table_ground = 0.2
        
        # 设置table和tool的三维尺寸
        table_size = [0.1, 0.7, 0.01]
        tool_size = [0.2, 0.02, 0.02]

        # 将table加入场景当中
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "world"
        table_pose.pose.position.x = 0.18
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        self.table_name = "table"
        self.scene.add_box(self.table_name, table_pose, table_size)

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_object(self, timeout=4):
        ## Manipulating objects requires the robot be able to touch them without the 
        ## planning scene reporting the contact as a collision. By adding link names to
        ## the ``touch_links`` array, we are telling theplanning scene to ignore
        ## collisions between those links and the box. 

        # 设置tool的三维尺寸
        tool_size = [0.2, 0.02, 0.02]

        # 设置tool的位姿
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.eef_link
        
        p.pose.position.x = tool_size[0] / 2.0 - 0.025
        p.pose.position.y = 0
        p.pose.position.z = 0.0
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1

        # 将tool附着到机器人的终端
        self.scene.attach_box(self.eef_link, 'tool', p, tool_size)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_object(self, object, timeout=4):
        ## We can also detach and remove the object from the planning scene:
        self.scene.remove_attached_object(self.eef_link, name=object)

        # We wait for the planning scene to update.print
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_object(self, object, timeout=4):
        ## **Note:** The object must be detached before we can remove it from the world
        
        ## We can remove the box from the world.
        self.scene.remove_world_object(object)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

def main():
    try:
        demo = MoveItFkDemo()

        # raw_input("============ Press `Enter` to execute a movement using a joint state goal ...")
        # demo.go_to_joint_state()

        # raw_input("============ Press `Enter` to execute a movement using a pose goal ...")
        # demo.go_to_pose_goal()

        # raw_input("============ Press `Enter` to plan and display a Cartesian path")
        # cartesian_plan, fraction = demo.plan_cartesian_path()
        # # print("============ Display a saved trajectory (this will replay the Cartesian path)")
        # # demo.display_trajectory(cartesian_plan)
        # raw_input("============ Press `Enter` to execute a saved path")
        # if fraction == 1.0:
        #     rospy.loginfo("Path computed successfully. Moving the arm.")
        #     demo.execute_plan(cartesian_plan)
        #     rospy.loginfo("Path execution complete.")
        # # 如果路径规划失败，则打印失败信息
        # else:
        #     rospy.loginfo("Path planning failed with only " + str(fraction) + " success after maxtries attempts.") 

        # raw_input("============ Press `Enter` to add a table to the planning scene")
        # demo.add_table()
        # raw_input("============ Press `Enter` to execute a movement using a pose goal ...")
        # demo.go_to_pose_goal()

        raw_input("============ Press `Enter` to add a table to the planning scene")
        demo.add_table()
        raw_input("============ Press `Enter` to attach a tool to the planning scene")
        demo.attach_object()
        raw_input("============ Press `Enter` to add a table to the planning scene")
        demo.go_to_pose_goal()
        raw_input("============ Press `Enter` to detach the tool from the robot ...")
        demo.detach_object('tool')
        ## **Note:** The object must be detached before we can remove it from the world
        raw_input("============ Press `Enter` to remove the tool from the planning scene")
        demo.remove_object('tool')
        raw_input("============ Press `Enter` to remove the table from the planning scene")
        demo.remove_object('table')

        print("============ Python demo complete!")

    except rospy.ROSInterruptException:
        return

if __name__ == "__main__":
        main()
