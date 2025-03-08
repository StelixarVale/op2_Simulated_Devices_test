"""op2_test_controller controller.
测试OP2机器人的各种传感器和舵机参数，使用官方步态管理器
"""
import math
import time
import os
import sys

# 添加Webots Python API路径
current_path = os.path.dirname(os.path.abspath(__file__))
webots_home = os.environ.get('WEBOTS_HOME', 'D:/webots-R2021a/Webots')
darwin_op_lib = os.path.join(webots_home, 'projects/robots/robotis/darwin-op/libraries/python37')
sys.path.append(darwin_op_lib)

# 尝试导入必要的模块
try:
    from controller import Robot, Motor, PositionSensor, Gyro, Accelerometer, TouchSensor, LED
    print("成功导入controller模块")
except ImportError as e:
    print(f"无法导入controller模块: {e}")
    print(f"当前Python路径: {sys.path}")
    sys.exit(1)

# 尝试导入RobotisOp2管理器
try:
    from managers import RobotisOp2GaitManager, RobotisOp2MotionManager
    print("成功导入RobotisOp2管理器")
except ImportError as e:
    print(f"无法导入RobotisOp2管理器: {e}")
    print("请确保Webots Darwin-OP库已正确安装")
    print(f"尝试从以下路径导入: {darwin_op_lib}")
    print(f"当前Python路径: {sys.path}")
    
    # 如果无法导入管理器，创建模拟的类以便代码可以继续运行
    class DummyManager:
        def __init__(self, *args, **kwargs):
            print("警告: 使用模拟的管理器")
        
        def __getattr__(self, name):
            def dummy_method(*args, **kwargs):
                print(f"调用模拟方法: {name}{args}")
                return False
            return dummy_method
    
    RobotisOp2GaitManager = DummyManager
    RobotisOp2MotionManager = DummyManager

class Op2TestController(Robot):
    """OP2机器人测试控制器类"""
    
    # 电机名称常量
    MOTOR_NAMES = [
        "ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR",
        "ArmLowerL", "PelvYR", "PelvYL", "PelvR", "PelvL",
        "LegUpperR", "LegUpperL", "LegLowerR", "LegLowerL", "AnkleR",
        "AnkleL", "FootR", "FootL", "Neck", "Head"
    ]
    
    def __init__(self):
        """初始化控制器"""
        super(Op2TestController, self).__init__()
        
        # 获取仿真的时间步长
        self.time_step = int(self.getBasicTimeStep())
        print(f"仿真时间步长: {self.time_step}毫秒")
        
        # 初始化传感器和执行器字典
        self.motors = {}
        self.position_sensors = {}
        
        # 初始化IMU传感器
        self.accelerometer = self.getDevice("Accelerometer")
        if not self.accelerometer:
            self.accelerometer = self.getDevice("accelerometer")
        if self.accelerometer:
            self.accelerometer.enable(self.time_step)
            print("加速度计已启用")
        
        self.gyro = self.getDevice("Gyro")
        if not self.gyro:
            self.gyro = self.getDevice("gyro")
        if self.gyro:
            self.gyro.enable(self.time_step)
            print("陀螺仪已启用")
        
        # 初始化脚底触觉传感器
        self.left_foot_sensor = self.getDevice("leftFootTouchSensor")
        if self.left_foot_sensor:
            self.left_foot_sensor.enable(self.time_step)
            print("左脚触觉传感器已启用")
        
        self.right_foot_sensor = self.getDevice("rightFootTouchSensor")
        if self.right_foot_sensor:
            self.right_foot_sensor.enable(self.time_step)
            print("右脚触觉传感器已启用")
        
        # 获取所有电机并启用位置传感器
        for name in self.MOTOR_NAMES:
            motor = self.getDevice(name)
            if motor:
                self.motors[name] = motor
                print(f"电机 {name} 已启用")
                
                # 获取位置传感器 (名称加上"S"后缀)
                sensor_name = name + "S"
                position_sensor = self.getDevice(sensor_name)
                if position_sensor:
                    position_sensor.enable(self.time_step)
                    self.position_sensors[name] = position_sensor
                    print(f"位置传感器 {sensor_name} 已启用")
        
        # 获取LED
        self.eye_led = self.getDevice("EyeLed")
        if self.eye_led:
            print("眼部LED已获取")
            self.eye_led.set(0x00FF00)  # 设置为绿色
        
        # 初始化步态管理器和动作管理器
        try:
            self.motion_manager = RobotisOp2MotionManager(self)
            print("动作管理器初始化成功")
        except Exception as e:
            print(f"动作管理器初始化失败: {e}")
            self.motion_manager = None
        
        try:
            self.gait_manager = RobotisOp2GaitManager(self, "")
            print("步态管理器初始化成功")
        except Exception as e:
            print(f"步态管理器初始化失败: {e}")
            self.gait_manager = None
    
    def my_step(self):
        """执行一步仿真，并检查是否应该退出"""
        ret = self.step(self.time_step)
        return ret != -1
    
    def wait(self, ms):
        """等待指定的毫秒数"""
        start_time = self.getTime()
        s = ms / 1000.0
        while s + start_time >= self.getTime():
            if not self.my_step():
                break
    
    def test_motor(self, motor_name, target_position, max_torque=None, velocity=0.3):
        """测试特定电机的参数"""
        if motor_name not in self.motors:
            print(f"错误: 找不到电机 {motor_name}")
            return False
        
        motor = self.motors[motor_name]
        
        # 设置最大扭矩（如果提供）
        if max_torque is not None:
            motor.setAvailableTorque(max_torque)
            print(f"设置 {motor_name} 最大扭矩为 {max_torque} N·m")
        
        # 设置速度（如果提供）
        if velocity is not None:
            motor.setVelocity(velocity)
            print(f"设置 {motor_name} 速度为 {velocity} rad/s")
        
        # 设置目标位置
        motor.setPosition(target_position)
        print(f"设置 {motor_name} 目标位置为 {target_position} rad")
        
        return True
    
    def walk_with_gait_manager(self, duration_seconds=5):
        """使用步态管理器让机器人行走指定的时间（秒）"""
        if not self.gait_manager:
            print("警告: 步态管理器不可用")
            return False
        
        print("使用步态管理器行走...")
        
        # 设置步态参数
        self.gait_manager.setXAmplitude(0.4)  # X方向幅度
        self.gait_manager.setYAmplitude(0.0)  # Y方向幅度（左右）
        self.gait_manager.setAAmplitude(0.0)  # 角度幅度（转向）
        self.gait_manager.setBalanceEnable(True)  # 启用平衡
        
        # 开始行走
        print(f"开始行走，计划行走 {duration_seconds} 秒...")
        self.gait_manager.start()  # 启动步态算法
        
        # 记录开始时间
        start_time = self.getTime()
        elapsed = 0
        
        # 按时间行走
        while elapsed < duration_seconds:
            if not self.my_step():
                break
            
            # 调用步态管理器的step函数，传入时间步长
            self.gait_manager.step(self.time_step)
            
            # 计算已经行走的时间
            current_time = self.getTime()
            elapsed = current_time - start_time
            
            # 每秒打印一次状态
            if int(elapsed) > int(elapsed - self.time_step/1000):
                print(f"行走中... 已行走 {elapsed:.2f} 秒 / {duration_seconds} 秒")
        
        # 停止行走
        print("停止行走...")
        self.gait_manager.stop()  # 停止步态算法
        
        # 重置步态参数
        self.gait_manager.setXAmplitude(0.0)
        self.gait_manager.setYAmplitude(0.0)
        self.gait_manager.setAAmplitude(0.0)
        
        print("行走完成")
        return True
    
    def play_motion(self, motion_id):
        """使用动作管理器播放预定义的动作"""
        if not self.motion_manager:
            print(f"警告: 动作管理器不可用，无法播放动作 {motion_id}")
            return False
        
        print(f"播放动作: {motion_id}")
        
        # 播放动作
        self.motion_manager.playPage(motion_id)
        
        # 等待动作完成（isMotionPlaying返回true表示动作正在播放，所以等待它变为false）
        while self.motion_manager.isMotionPlaying():
            if not self.my_step():
                break
        
        print(f"动作 {motion_id} 已完成")
        return True
    
    def print_sensor_data(self):
        """打印所有传感器的数据"""
        if self.accelerometer:
            acc_values = self.accelerometer.getValues()
            print(f"加速度计: X={acc_values[0]:.4f}, Y={acc_values[1]:.4f}, Z={acc_values[2]:.4f} m/s²")
        
        if self.gyro:
            gyro_values = self.gyro.getValues()
            print(f"陀螺仪: X={gyro_values[0]:.4f}, Y={gyro_values[1]:.4f}, Z={gyro_values[2]:.4f} rad/s")
        
        if self.left_foot_sensor:
            left_contact = self.left_foot_sensor.getValue()
            print(f"左脚触觉传感器: {left_contact}")
        
        if self.right_foot_sensor:
            right_contact = self.right_foot_sensor.getValue()
            print(f"右脚触觉传感器: {right_contact}")
        
        for name, sensor in self.position_sensors.items():
            position = sensor.getValue()
            print(f"电机 {name} 位置: {position:.4f} rad")
    
    def run(self):
        """主运行函数"""
        print("开始测试OP2机器人传感器和舵机...")
        
        # 第一步更新传感器值
        self.my_step()
        
        # 设置眼部LED为绿色
        if self.eye_led:
            self.eye_led.set(0x00FF00)
        
        # 初始化机器人姿势
        print("\n让机器人先站起来...")
        self.play_motion(1)  # ini动作，移动到站立位置
        self.wait(500)  # 等待500毫秒
        
        # 测试头部电机
        print("\n测试头部电机...")
        if "Head" in self.motors and "Neck" in self.motors:
            self.test_motor("Head", 0.3, velocity=0.5)
            self.test_motor("Neck", 0.2, velocity=0.5)
            self.wait(2000)  # 等待2秒
        
        # 测试手臂电机
        print("\n测试手臂电机...")
        if "ShoulderL" in self.motors and "ArmUpperL" in self.motors:
            self.test_motor("ShoulderL", 0.5, velocity=0.5)
            self.test_motor("ArmUpperL", 0.2, velocity=0.5)
            self.wait(2000)  # 等待2秒
        
        # 准备行走
        print("\n准备行走...")
        self.play_motion(9)  # walkready动作，准备行走
        self.wait(1000)  # 等待1秒
        
        # 使用步态管理器行走
        print("\n开始行走测试...")
        self.walk_with_gait_manager(duration_seconds=10)  # 行走10秒
        
        # 测试脚底触觉传感器
        print("\n测试脚底触觉传感器...")
        self.play_motion(1)  # 回到初始姿势
        self.wait(500)
        
        # 尝试播放抬腿动作
        print("\n尝试抬腿动作...")
        if self.play_motion(15):  # 尝试播放坐下动作
            print("成功播放坐下动作")
            self.wait(1000)
            self.play_motion(16)  # 站起来
            self.wait(1000)
        
        # 恢复初始姿势
        print("\n恢复初始姿势...")
        self.play_motion(1)  # 回到初始姿势
        
        print("\n测试完成！")

# 主程序
if __name__ == "__main__":
    try:
        print("正在初始化OP2测试控制器...")
        controller = Op2TestController()
        controller.run()
    except Exception as e:
        import traceback
        print(f"运行控制器时出错: {e}")
        traceback.print_exc()
