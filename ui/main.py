#!/usr/bin/env python3
import sys
import threading
import time
from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                           QLabel, QSlider, QPushButton, QGroupBox, QStatusBar,
                           QSizePolicy)
from PyQt6.QtCore import Qt, QTimer, QSize
from PyQt6.QtGui import QColor
import roslibpy

# ROS Bridge 配置
ROS_BRIDGE_IP = '127.0.0.1'  # 修改為您的 ROS Bridge 服務器 IP
ROS_BRIDGE_PORT = 9090

# ROS 話題
JOINT_TOPIC = '/target_joint_states'
CONTROL_TOPIC = '/joint_control_command'
JOINT_STATE_TOPIC = '/joint_states'  # 添加關節狀態話題
COLLISION_TOPIC = '/collision_detection'  # 添加碰撞檢測話題

# 關節名稱和限制
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
JOINT_LIMITS = {
    'joint1': (-170, 170),
    'joint2': (-150, 95),
    'joint3': (-85, 185),
    'joint4': (-190, 190),
    'joint5': (-135, 135),
    'joint6': (-360, 360)
}

class JointSlider(QWidget):
    def __init__(self, name, limits, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout()  # 改為水平佈局
        
        # 關節名稱標籤
        self.name_label = QLabel(name)
        self.name_label.setFixedWidth(60)  # 固定寬度
        layout.addWidget(self.name_label)
        
        # 滑塊
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(int(limits[0] * 10))
        self.slider.setMaximum(int(limits[1] * 10))
        self.slider.setValue(0)
        self.slider.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.slider.valueChanged.connect(self.update_value)
        layout.addWidget(self.slider)
        
        # 數值標籤
        self.value_label = QLabel('0.0')
        self.value_label.setFixedWidth(60)  # 固定寬度
        layout.addWidget(self.value_label)
        
        self.setLayout(layout)
        
    def update_value(self):
        try:
            value = self.slider.value() / 10.0
            self.value_label.setText(f'{value:.1f}')
            if self.parent():
                self.parent().update_joint_values()
        except Exception as e:
            print(f"更新滑桿值時發生錯誤: {str(e)}")
            
    def set_value(self, value):
        try:
            self.slider.setValue(int(value * 10))
            self.value_label.setText(f'{value:.1f}')
        except Exception as e:
            print(f"設置滑桿值時發生錯誤: {str(e)}")
            
    def set_collision_state(self, is_collision):
        try:
            if is_collision:
                self.name_label.setStyleSheet("color: red;")
                self.value_label.setStyleSheet("color: red;")
            else:
                self.name_label.setStyleSheet("")
                self.value_label.setStyleSheet("")
        except Exception as e:
            print(f"設置碰撞狀態時發生錯誤: {str(e)}")

class JointControlUI(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_ros()
        
    def init_ui(self):
        self.setWindowTitle('關節控制器')
        self.setMinimumSize(800, 400)  # 設置最小尺寸
        self.setMaximumSize(1200, 600)  # 設置最大尺寸
        
        # 主佈局
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)  # 設置間距
        main_layout.setContentsMargins(10, 10, 10, 10)  # 設置邊距
        
        # 關節控制組
        joint_group = QGroupBox('關節控制')
        joint_layout = QVBoxLayout()
        joint_layout.setSpacing(5)  # 設置間距
        
        # 創建關節滑塊
        self.joint_sliders = {}
        for name in JOINT_NAMES:
            slider = JointSlider(name, JOINT_LIMITS[name])
            self.joint_sliders[name] = slider
            joint_layout.addWidget(slider)
        
        joint_group.setLayout(joint_layout)
        main_layout.addWidget(joint_group)
        
        # 控制按鈕組
        control_group = QGroupBox('控制')
        control_layout = QHBoxLayout()
        control_layout.setSpacing(10)  # 設置按鈕間距
        
        # 規劃按鈕
        self.plan_button = QPushButton('規劃運動')
        self.plan_button.setMinimumWidth(100)  # 設置最小寬度
        self.plan_button.clicked.connect(self.plan_motion)
        control_layout.addWidget(self.plan_button)
        
        # 執行按鈕
        self.execute_button = QPushButton('執行運動')
        self.execute_button.setMinimumWidth(100)
        self.execute_button.clicked.connect(self.execute_motion)
        control_layout.addWidget(self.execute_button)
        
        # 取消按鈕
        self.cancel_button = QPushButton('取消預覽')
        self.cancel_button.setMinimumWidth(100)
        self.cancel_button.clicked.connect(self.cancel_preview)
        control_layout.addWidget(self.cancel_button)
        
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)
        
        # 碰撞狀態組
        collision_group = QGroupBox('碰撞狀態')
        collision_layout = QVBoxLayout()
        
        # 碰撞狀態標籤
        self.collision_label = QLabel('無碰撞')
        self.collision_label.setStyleSheet("color: green; font-weight: bold;")
        collision_layout.addWidget(self.collision_label)
        
        collision_group.setLayout(collision_layout)
        main_layout.addWidget(collision_group)
        
        # 狀態欄
        self.status_bar = QStatusBar()
        self.status_bar.setFixedHeight(25)  # 設置固定高度
        main_layout.addWidget(self.status_bar)
        
        self.setLayout(main_layout)
        
        # 更新定時器
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.send_joint_values)
        self.update_timer.start(100)  # 每100ms更新一次
        
    def init_ros(self):
        try:
            # 連接到 ROS Bridge
            self.client = roslibpy.Ros(host=ROS_BRIDGE_IP, port=ROS_BRIDGE_PORT)
            
            # 創建發布者
            self.joint_publisher = roslibpy.Topic(
                self.client,
                JOINT_TOPIC,
                'std_msgs/Float64MultiArray'
            )
            
            self.control_publisher = roslibpy.Topic(
                self.client,
                CONTROL_TOPIC,
                'std_msgs/String'
            )
            
            # 創建關節狀態訂閱者
            self.joint_state_subscriber = roslibpy.Topic(
                self.client,
                JOINT_STATE_TOPIC,
                'sensor_msgs/JointState'
            )
            self.joint_state_subscriber.subscribe(self.joint_state_callback)
            
            # 創建碰撞檢測訂閱者
            self.collision_subscriber = roslibpy.Topic(
                self.client,
                COLLISION_TOPIC,
                'std_msgs/Bool'
            )
            self.collision_subscriber.subscribe(self.collision_callback)
            
            # 在背景執行緒中啟動 ROS 連接
            self.ros_thread = threading.Thread(target=self.run_ros)
            self.ros_thread.daemon = True
            self.ros_thread.start()
            
            # 檢查連接狀態
            self.connection_timer = QTimer()
            self.connection_timer.timeout.connect(self.check_connection)
            self.connection_timer.start(1000)
            
        except Exception as e:
            self.status_bar.showMessage(f'ROS 初始化錯誤: {str(e)}')
    
    def collision_callback(self, message):
        try:
            is_collision = message.data
            if is_collision:
                self.collision_label.setText('檢測到碰撞！')
                self.collision_label.setStyleSheet("color: red; font-weight: bold;")
                # 更新所有滑桿的碰撞狀態
                for slider in self.joint_sliders.values():
                    slider.set_collision_state(True)
            else:
                self.collision_label.setText('無碰撞')
                self.collision_label.setStyleSheet("color: green; font-weight: bold;")
                # 重置所有滑桿的碰撞狀態
                for slider in self.joint_sliders.values():
                    slider.set_collision_state(False)
        except Exception as e:
            print(f"處理碰撞檢測消息時發生錯誤: {str(e)}")
    
    def joint_state_callback(self, message):
        try:
            # 確保消息包含關節名稱和位置
            if not hasattr(message, 'name') or not hasattr(message, 'position'):
                return
                
            # 更新每個關節的滑桿值
            for i, name in enumerate(message.name):
                if name in self.joint_sliders:
                    # 將弧度轉換為角度
                    angle = message.position[i] * 180.0 / 3.14159
                    self.joint_sliders[name].set_value(angle)
                    
        except Exception as e:
            print(f"處理關節狀態消息時發生錯誤: {str(e)}")
    
    def run_ros(self):
        try:
            self.client.run()
        except Exception as e:
            print(f"ROS 連接錯誤: {str(e)}")
            self.status_bar.showMessage(f"ROS 連接錯誤: {str(e)}")
        
    def check_connection(self):
        try:
            if self.client.is_connected:
                self.status_bar.showMessage('已連接到 ROS Bridge')
            else:
                self.status_bar.showMessage('未連接到 ROS Bridge')
        except Exception as e:
            self.status_bar.showMessage(f'檢查連接時發生錯誤: {str(e)}')
            
    def update_joint_values(self):
        # 只更新顯示，不發送消息
        pass
            
    def send_joint_values(self):
        try:
            if not hasattr(self, 'client') or not self.client.is_connected:
                return
                
            values = [slider.slider.value() / 10.0 for slider in self.joint_sliders.values()]
            self.joint_publisher.publish(roslibpy.Message({'data': values}))
        except Exception as e:
            print(f"發送關節值時發生錯誤: {str(e)}")
        
    def plan_motion(self):
        try:
            if not hasattr(self, 'client') or not self.client.is_connected:
                self.status_bar.showMessage('未連接到 ROS Bridge')
                return
                
            self.control_publisher.publish(roslibpy.Message({'data': 'plan'}))
            self.status_bar.showMessage('已發送規劃命令')
        except Exception as e:
            self.status_bar.showMessage(f'發送規劃命令時發生錯誤: {str(e)}')
        
    def execute_motion(self):
        try:
            if not hasattr(self, 'client') or not self.client.is_connected:
                self.status_bar.showMessage('未連接到 ROS Bridge')
                return
                
            self.control_publisher.publish(roslibpy.Message({'data': 'execute'}))
            self.status_bar.showMessage('已發送執行命令')
        except Exception as e:
            self.status_bar.showMessage(f'發送執行命令時發生錯誤: {str(e)}')
        
    def cancel_preview(self):
        try:
            if not hasattr(self, 'client') or not self.client.is_connected:
                self.status_bar.showMessage('未連接到 ROS Bridge')
                return
                
            self.control_publisher.publish(roslibpy.Message({'data': 'cancel'}))
            self.status_bar.showMessage('已發送取消命令')
        except Exception as e:
            self.status_bar.showMessage(f'發送取消命令時發生錯誤: {str(e)}')
        
    def closeEvent(self, event):
        try:
            if hasattr(self, 'client'):
                self.client.terminate()
            if hasattr(self, 'update_timer'):
                self.update_timer.stop()
            if hasattr(self, 'connection_timer'):
                self.connection_timer.stop()
        except Exception as e:
            print(f"關閉程序時發生錯誤: {str(e)}")
        event.accept()

def main():
    try:
        app = QApplication(sys.argv)
        ui = JointControlUI()
        ui.show()
        sys.exit(app.exec())
    except Exception as e:
        print(f"程序運行時發生錯誤: {str(e)}")

if __name__ == '__main__':
    main() 