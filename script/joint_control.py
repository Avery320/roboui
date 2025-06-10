#!/usr/bin/env python3
import rospy
import sys
import math
from moveit_commander import MoveGroupCommander, roscpp_initialize, PlanningSceneInterface
from std_msgs.msg import Float64MultiArray, Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import tf.transformations as tf_trans
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class JointController:
    def __init__(self):
        roscpp_initialize([])
        rospy.init_node('joint_controller', anonymous=True)
        
        # 初始化 MoveGroup
        try:
            self.group = MoveGroupCommander("manipulator")
            self.scene = PlanningSceneInterface()
            rospy.loginfo("成功初始化 MoveGroup: manipulator")
        except Exception as e:
            rospy.logerr(f"初始化 MoveGroup 失敗: {str(e)}")
            sys.exit(1)
        
        # 設置規劃參數
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)
        
        # 設置規劃器
        self.group.set_planner_id("RRTConnect")
        
        # 發布可視化標記
        self.marker_publisher = rospy.Publisher(
            '/visualization_marker_array',
            MarkerArray,
            queue_size=10
        )
        
        # 發布規劃路徑
        self.trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            DisplayTrajectory,
            queue_size=10
        )
        
        # 獲取關節名稱
        self.joint_names = self.group.get_active_joints()
        rospy.loginfo(f"關節名稱: {self.joint_names}")
        
        # 獲取當前關節值
        self.current_joint_values = self.group.get_current_joint_values()
        rospy.loginfo(f"當前關節值: {self.current_joint_values}")
        
        # 設置目標關節值
        self.target_joint_values = self.current_joint_values.copy()
        
        # 預覽狀態
        self.preview_mode = False
        self.preview_plan = None
        
        rospy.loginfo("關節控制器已啟動")

        # 訂閱目標關節狀態話題
        self.subscriber = rospy.Subscriber(
            '/target_joint_states',
            Float64MultiArray,
            self.joint_state_callback
        )
        
        # 訂閱控制命令話題
        self.control_subscriber = rospy.Subscriber(
            '/joint_control_command',
            String,
            self.control_callback
        )
        
        rospy.loginfo("已訂閱話題 /target_joint_states 和 /joint_control_command")

    def control_callback(self, msg):
        """處理控制命令"""
        command = msg.data
        if command == "plan":
            self.move_to_target(preview_only=True)
        elif command == "execute":
            if self.preview_mode and self.preview_plan:
                rospy.loginfo("收到執行命令，開始執行運動...")
                success = self.group.execute(self.preview_plan, wait=True)
                if success:
                    rospy.loginfo("執行成功")
                    self.visualize_joint_state()
                else:
                    rospy.logwarn("執行失敗")
                self.preview_mode = False
                self.preview_plan = None
            else:
                rospy.logwarn("沒有可執行的預覽計劃")
        elif command == "cancel":
            self.preview_mode = False
            self.preview_plan = None
            rospy.loginfo("已取消預覽")

    def plan_cartesian_path(self, waypoints):
        """規劃笛卡爾路徑"""
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        return plan, fraction

    def add_collision_object(self, name, pose, size):
        """添加碰撞物體"""
        self.scene.add_box(name, pose, size)
        rospy.loginfo(f"添加碰撞物體: {name}")

    def remove_collision_object(self, name):
        """移除碰撞物體"""
        self.scene.remove_world_object(name)
        rospy.loginfo(f"移除碰撞物體: {name}")

    def visualize_trajectory(self, plan):
        """可視化規劃路徑"""
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.group.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.trajectory_publisher.publish(display_trajectory)

    def move_to_pose(self, pose_goal, preview_only=False):
        """移動到指定姿態"""
        self.group.set_pose_target(pose_goal)
        plan = self.group.plan()
        if plan[0]:
            self.visualize_trajectory(plan[1])
            if preview_only:
                self.preview_mode = True
                self.preview_plan = plan[1]
                rospy.loginfo("已生成預覽，等待執行確認...")
            else:
                success = self.group.execute(plan[1], wait=True)
                if success:
                    rospy.loginfo("執行成功")
                    self.visualize_joint_state()
                else:
                    rospy.logwarn("執行失敗")
        else:
            rospy.logwarn("規劃失敗")

    def joint_state_callback(self, msg):
        """處理接收到的目標關節狀態消息 (角度)"""
        rospy.loginfo(f"接收到目標關節狀態 (角度): {msg.data}")
        if len(msg.data) == len(self.target_joint_values):
            # 將接收到的角度轉換為弧度供 MoveIt 使用
            self.target_joint_values = [math.radians(deg) for deg in msg.data]
            # 不再自動執行，等待規劃命令
        else:
            rospy.logwarn(f"接收到的關節數值數量不匹配: 需要 {len(self.target_joint_values)}, 收到 {len(msg.data)}")

    def visualize_joint_state(self):
        """在 RViz 中可視化當前關節狀態"""
        marker_array = MarkerArray()
        
        # 獲取當前末端執行器位置
        current_pose = self.group.get_current_pose().pose
        
        # 創建末端執行器標記
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "end_effector"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = current_pose
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)

    def set_joint_value(self, joint_index, value):
        """設置單個關節的值"""
        if 0 <= joint_index < len(self.target_joint_values):
            self.target_joint_values[joint_index] = value
            rospy.loginfo(f"設置關節 {joint_index} 的值為: {value}")
        else:
            rospy.logwarn(f"無效的關節索引: {joint_index}")

    def set_all_joints(self, values):
        """設置所有關節的值"""
        if len(values) == len(self.target_joint_values):
            self.target_joint_values = values.copy()
            rospy.loginfo(f"設置所有關節值: {values}")
        else:
            rospy.logwarn(f"關節值數量不匹配: 需要 {len(self.target_joint_values)}, 收到 {len(values)}")

    def move_to_target(self, preview_only=False):
        """移動到目標位置"""
        try:
            # 設置目標位置
            self.group.set_joint_value_target(self.target_joint_values)
            
            # 進行規劃
            rospy.loginfo("開始規劃...")
            plan = self.group.plan()
            if plan[0]:
                rospy.loginfo("規劃成功")
                self.visualize_trajectory(plan[1])
                
                if preview_only:
                    self.preview_mode = True
                    self.preview_plan = plan[1]
                    rospy.loginfo("已生成預覽，等待執行確認...")
                else:
                    rospy.loginfo("開始執行...")
                    success = self.group.execute(plan[1], wait=True)
                    if success:
                        rospy.loginfo("執行成功")
                        self.visualize_joint_state()
                    else:
                        rospy.logwarn("執行失敗")
            else:
                rospy.logwarn("規劃失敗")
            
        except Exception as e:
            rospy.logerr(f"移動過程中發生錯誤: {str(e)}")

    def run(self):
        """主循環，等待接收消息"""
        rospy.spin()

def main():
    try:
        controller = JointController()
        controller.run()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序發生錯誤: {str(e)}")
        sys.exit(1)

if __name__ == '__main__':
    main() 