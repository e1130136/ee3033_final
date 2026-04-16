#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import actionlib
import copy

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8
from tf.transformations import quaternion_from_euler
from math import pi

from darknet_ros_msgs.msg import BoundingBoxes

STATUS_EXPLORING = 0
STATUS_CLOSE_TARGET = 1   # 这版先不真正使用，保留
STATUS_GO_HOME = 2

class MultiWaypointNav():
    def __init__(self):
        rospy.init_node('multi_waypoint_nav', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        self.rest_time = rospy.get_param("~rest_time", 0.5)
        self.target_class = rospy.get_param("~target_class", "bottle")
        self.min_prob = rospy.get_param("~min_prob", 0.3)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # rospy.Subscriber("/exploring_cmd", Int8, self.cmdCallback)
        # rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
        # rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yoloCallback)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base server")

        # 状态变量
        self.exploring_cmd = STATUS_EXPLORING
        self.current_pose = None
        self.home_pose = None
        self.target_found = False
        self.target_recorded_pose = None
        self.location_idx = 0

        # 记录运行时间
        self.start_time = rospy.Time.now()

        rospy.loginfo("Starting exploring maze")

        # 这里是你的 waypoint 列表
        self.locations = []
        self.locations.append(Pose(Point(0.5, 0.0, 0.0), self.yaw_to_quaternion(pi/2)))
        self.locations.append(Pose(Point(0.5, 0.5, 0.0), self.yaw_to_quaternion(0.0)))
        self.locations.append(Pose(Point(0.5, 1.5, 0.0), self.yaw_to_quaternion(0.0)))
        self.locations.append(Pose(Point(1.0, 0.5, 0.0), self.yaw_to_quaternion(-pi/2)))
        self.locations.append(Pose(Point(0.5, 0.0, 0.0), self.yaw_to_quaternion(pi)))
        rospy.Subscriber("/exploring_cmd", Int8,self.cmdCallback)
        rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,self.poseCallback)
        rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.yoloCallback)
        
        self.main_loop()


    def poseCallback(self, msg):
        """
        订阅机器人在 map 下的当前位姿。
        第一次收到位姿时，把它记成 home pose。
        """
        self.current_pose = copy.deepcopy(msg.pose.pose)

        if self.home_pose is None:
            self.home_pose = copy.deepcopy(msg.pose.pose)
            rospy.loginfo("Home pose recorded: x=%.3f, y=%.3f",
                          self.home_pose.position.x,
                          self.home_pose.position.y)
    
    def yoloCallback(self, msg):
        """
        订阅 YOLO 的 bounding boxes。
        一旦发现目标类别，就记录当前机器人 pose，然后切换为 GO_HOME。
        """
        if self.target_found:
            return

        best_box = None
        best_prob = -1.0

        for box in msg.bounding_boxes:
            if box.Class == self.target_class and box.probability >= self.min_prob:
                if box.probability > best_prob:
                    best_prob = box.probability
                    best_box = box

        if best_box is None:
            return

        if self.current_pose is None:
            rospy.logwarn("Target detected, but current pose not available yet.")
            return

        self.target_found = True
        self.target_recorded_pose = copy.deepcopy(self.current_pose)
        self.exploring_cmd = STATUS_GO_HOME

        rospy.loginfo("======================================")
        rospy.loginfo("TARGET DETECTED!")
        rospy.loginfo("class=%s, prob=%.3f", best_box.Class, best_box.probability)
        rospy.loginfo("Recorded robot pose at detection:")
        rospy.loginfo("x=%.3f, y=%.3f",
                      self.target_recorded_pose.position.x,
                      self.target_recorded_pose.position.y)
        rospy.loginfo("Cancelling current exploration goal and returning home...")
        rospy.loginfo("======================================")

        self.move_base.cancel_goal()
    
    def cmdCallback(self, msg):
        rospy.loginfo("Receive exploring cmd : %d", msg.data)
        self.exploring_cmd = msg.data

        if self.exploring_cmd == STATUS_CLOSE_TARGET:
            rospy.loginfo("Stopping the robot...")
            self.move_base.cancel_goal()

        elif self.exploring_cmd == STATUS_GO_HOME:
            rospy.loginfo("Manual command: go home")
            self.move_base.cancel_goal()

    def build_goal_from_pose(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = copy.deepcopy(pose)
        return goal

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        try:
            self.move_base.cancel_goal()
        except:
            pass

        rospy.sleep(1.0)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1.0)

    def yaw_to_quaternion(self, yaw):
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(q[0], q[1], q[2], q[3])
    def main_loop(self):
        while not rospy.is_shutdown():
            if self.home_pose is None:
                rospy.loginfo_throttle(2.0, "Waiting for /amcl_pose to initialize home pose...")
                rospy.sleep(0.1)
                continue
            if self.locations is None:
                self.locations[self.location_idx]
                rospy.sleep(0.1)
                continue

            # 根据状态选择目标点
            if self.exploring_cmd == STATUS_EXPLORING:
                target_pose = self.locations[self.location_idx]
                rospy.loginfo("Exploring waypoint %d / %d",
                              self.location_idx + 1, len(self.locations))
                rospy.loginfo("Going to x=%.3f, y=%.3f",
                              target_pose.position.x, target_pose.position.y)

                self.location_idx = (self.location_idx + 1) % len(self.locations)

            elif self.exploring_cmd == STATUS_GO_HOME:
                target_pose = self.home_pose
                rospy.loginfo("Returning home: x=%.3f, y=%.3f",
                              target_pose.position.x, target_pose.position.y)

            else:
                rospy.sleep(0.1)
                continue

            goal = self.build_goal_from_pose(target_pose)
            self.move_base.send_goal(goal)

            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logwarn("Time ran out achieving goal")
            else:
                state = self.move_base.get_state()

                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal finished successfully!")
                elif state == GoalStatus.PREEMPTED:
                    rospy.loginfo("Goal was preempted/cancelled.")
                else:
                    rospy.logwarn("Goal failed! state=%d", state)

            running_time = rospy.Time.now() - self.start_time
            running_time = running_time.secs / 60.0
            rospy.loginfo("Current time: %.1f min", running_time)

            # 只有真正回到 home 并成功后才退出
            if self.exploring_cmd == STATUS_GO_HOME:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Robot returned home successfully.")

                    if self.target_recorded_pose is not None:
                        rospy.loginfo("Final recorded target-detection robot pose:")
                        rospy.loginfo("x=%.3f, y=%.3f",
                                      self.target_recorded_pose.position.x,
                                      self.target_recorded_pose.position.y)
                    break

            rospy.sleep(self.rest_time)

        self.shutdown()




if __name__ == '__main__':
    try:
        MultiWaypointNav()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exploring maze finished.")