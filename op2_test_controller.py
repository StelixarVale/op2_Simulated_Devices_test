"""op2_test_controller controller.
测试OP2机器人的各种传感器和舵机参数，使用官方步态管理器
"""
import math
import time
import os
import sys
sys.path.append('D:\webots-R2021a\Webots\projects\robots\robotis\darwin-op\libraries\python37')
from managers import RobotisOp2GaitManager
from managers import RobotisOp2MotionManager
from controller import Robot, Motor, PositionSensor, Gyro, Accelerometer, TouchSensor, LED

# 创建Robot实例
robot = Robot()

# 获取仿真的时间步长
timestep = int(robot.getBasicTimeStep())
print(f"仿真时间步长: {timestep}毫秒")

# 初始化传感器和执行器字典
motors = {}
position_sensors = {}
force_sensors = {}

# 初始化IMU传感器
try:
    # 尝试使用正确的加速度计名称
    accelerometer = robot.getDevice("Accelerometer")
    if not accelerometer:
        accelerometer = robot.getDevice("accelerometer")
    accelerometer.enable(timestep)
    print("加速度计已启用")
except:
    print("警告: 无法获取加速度计")
    accelerometer = None

try:
    # 尝试使用正确的陀螺仪名称
    gyro = robot.getDevice("Gyro")
    if not gyro:
        gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    print("陀螺仪已启用")
except:
    print("警告: 无法获取陀螺仪")
    gyro = None

# 初始化脚底触觉传感器
try:
    left_foot_sensor = robot.getDevice("leftFootTouchSensor")
    left_foot_sensor.enable(timestep)
    print("左脚触觉传感器已启用")
except:
    print("警告: 无法获取左脚触觉传感器")
    left_foot_sensor = None

try:
    right_foot_sensor = robot.getDevice("rightFootTouchSensor")
    right_foot_sensor.enable(timestep)
    print("右脚触觉传感器已启用")
except:
    print("警告: 无法获取右脚触觉传感器")
    right_foot_sensor = None

# 根据提供的映射表定义电机名称
motor_mapping = {
    # 头部和颈部电机
    "Head": 20,
    "Neck": 19,
    
    # 手臂电机
    "ShoulderR": 1,
    "ShoulderL": 2,
    "ArmUpperR": 3,
    "ArmUpperL": 4,
    "ArmLowerR": 5,
    "ArmLowerL": 6,
    
    # 腿部电机
    "PelvYR": 7,
    "PelvYL": 8,
    "PelvR": 9,
    "PelvL": 10,
    "LegUpperR": 11,
    "LegUpperL": 12,
    "LegLowerR": 13,
    "LegLowerL": 14,
    "AnkleR": 15,
    "AnkleL": 16,
    "FootR": 17,
    "FootL": 18
}

# 获取所有电机并启用位置传感器
for name, motor_id in motor_mapping.items():
    try:
        # 获取电机
        motor = robot.getDevice(name)
        if motor:
            motors[name] = motor
            print(f"电机 {name} 已启用")
            
            # 获取位置传感器 (名称加上"S"后缀)
            sensor_name = name + "S"
            position_sensor = robot.getDevice(sensor_name)
            if position_sensor:
                position_sensor.enable(timestep)
                position_sensors[name] = position_sensor
                print(f"位置传感器 {sensor_name} 已启用")
    except Exception as e:
        print(f"警告: 无法获取电机或传感器 {name}: {e}")

# 获取LED
try:
    eye_led = robot.getDevice("EyeLed")
    print("眼部LED已获取")
except:
    print("警告: 无法获取眼部LED")
    eye_led = None

# 初始化步态管理器
gait_manager = RobotisOp2GaitManager(robot, "")

# 初始化动作管理器
motion_manager = RobotisOp2MotionManager(robot)

# 测试电机函数
def test_motor(motor_name, target_position, max_torque=None, velocity=0.3):
    """测试特定电机的参数"""
    if motor_name not in motors:
        print(f"错误: 找不到电机 {motor_name}")
        return False
    
    motor = motors[motor_name]
    
    # 设置最大扭矩（如果提供）
    if max_torque is not None:
        try:
            motor.setAvailableTorque(max_torque)
            print(f"设置 {motor_name} 最大扭矩为 {max_torque} N·m")
        except:
            print(f"警告: 无法设置 {motor_name} 的扭矩")
    
    # 设置速度（如果提供）
    if velocity is not None:
        try:
            motor.setVelocity(velocity)
            print(f"设置 {motor_name} 速度为 {velocity} rad/s")
        except:
            print(f"警告: 无法设置 {motor_name} 的速度")
    
    # 设置目标位置
    try:
        motor.setPosition(target_position)
        print(f"设置 {motor_name} 目标位置为 {target_position} rad")
    except:
        print(f"警告: 无法设置 {motor_name} 的位置")
    
    return True

# 使用步态管理器走路
def walk_with_gait_manager(steps=10):
    """使用步态管理器让机器人走几步"""
    if not gait_manager:
        print("警告: 步态管理器不可用")
        return False
    
    try:
        print("使用步态管理器行走...")
        
        # 设置步态参数
        gait_manager.setXAmplitude(0.1)  # X方向幅度
        gait_manager.setYAmplitude(0.0)  # Y方向幅度（左右）
        gait_manager.setAAmplitude(0.0)  # 角度幅度（转向）
        gait_manager.setBalanceEnabled(True)  # 启用平衡
        
        # 开始行走
        print("开始行走...")
        
        # 执行指定步数
        for i in range(steps):
            if robot.step(timestep) == -1:
                break
            
            # 每一步都调用步态管理器的步进函数
            gait_manager.step(timestep)
            
            # 每5步打印一次状态
            if i % 5 == 0:
                print(f"行走中... 步数: {i+1}/{steps}")
        
        # 停止行走
        gait_manager.setXAmplitude(0.0)
        gait_manager.setYAmplitude(0.0)
        gait_manager.setAAmplitude(0.0)
        
        # 再执行几步让机器人停下来
        for i in range(5):
            if robot.step(timestep) == -1:
                break
            gait_manager.step(timestep)
        
        print("行走完成")
        return True
    except Exception as e:
        print(f"警告: 使用步态管理器时出错: {e}")
        return False

# 使用动作管理器播放动作
def play_motion(motion_name):
    """使用动作管理器播放预定义的动作"""
    if not motion_manager:
        print(f"警告: 动作管理器不可用，无法播放动作 {motion_name}")
        return False
    
    try:
        print(f"播放动作: {motion_name}")
        
        # 播放动作
        motion_manager.playPage(motion_name)
        
        # 等待动作完成
        while not motion_manager.isMotionDone():
            if robot.step(timestep) == -1:
                break
        
        print(f"动作 {motion_name} 已完成")
        return True
    except Exception as e:
        print(f"警告: 播放动作时出错: {e}")
        return False

# 打印传感器数据
def print_sensor_data():
    """打印所有传感器的数据"""
    # 打印IMU数据
    if accelerometer:
        try:
            acc_values = accelerometer.getValues()
            print(f"加速度计: X={acc_values[0]:.4f}, Y={acc_values[1]:.4f}, Z={acc_values[2]:.4f} m/s²")
        except:
            print("警告: 无法读取加速度计数据")
    
    if gyro:
        try:
            gyro_values = gyro.getValues()
            print(f"陀螺仪: X={gyro_values[0]:.4f}, Y={gyro_values[1]:.4f}, Z={gyro_values[2]:.4f} rad/s")
        except:
            print("警告: 无法读取陀螺仪数据")
    
    # 打印触觉传感器数据
    if left_foot_sensor:
        try:
            left_contact = left_foot_sensor.getValue()
            print(f"左脚触觉传感器: {left_contact}")
        except:
            print("警告: 无法读取左脚触觉传感器数据")
    
    if right_foot_sensor:
        try:
            right_contact = right_foot_sensor.getValue()
            print(f"右脚触觉传感器: {right_contact}")
        except:
            print("警告: 无法读取右脚触觉传感器数据")
    
    # 打印电机位置传感器数据
    for name, sensor in position_sensors.items():
        try:
            position = sensor.getValue()
            print(f"电机 {name} 位置: {position:.4f} rad")
        except:
            pass

# 主循环
print("开始测试OP2机器人传感器和舵机...")

test_phase = 0
test_start_time = robot.getTime()

while robot.step(timestep) != -1:
    current_time = robot.getTime()
    elapsed_time = current_time - test_start_time
    
    # 每5秒打印一次传感器数据
    if elapsed_time % 5 < 0.1:
        print(f"\n===== 时间: {current_time:.2f}s =====")
        print_sensor_data()
    
    # 根据不同阶段测试不同的功能
    if test_phase == 0 and elapsed_time > 2:
        # 测试头部电机
        if "Head" in motors and "Neck" in motors:
            test_motor("Head", 0.3, velocity=0.5)
            test_motor("Neck", 0.2, velocity=0.5)
            test_phase = 1
            print("\n开始测试头部电机...")
        else:
            test_phase = 1
            print("\n跳过头部电机测试（未找到电机）...")
    
    elif test_phase == 1 and elapsed_time > 7:
        # 测试手臂电机
        if "ShoulderL" in motors and "ArmUpperL" in motors:
            test_motor("ShoulderL", 0.5, velocity=0.5)
            test_motor("ArmUpperL", 0.2, velocity=0.5)
            test_phase = 2
            print("\n开始测试手臂电机...")
        else:
            test_phase = 2
            print("\n跳过手臂电机测试（未找到电机）...")
    
    elif test_phase == 2 and elapsed_time > 12:
        # 尝试使用步态管理器走几步
        print("\n尝试使用步态管理器走几步...")
        if gait_manager and walk_with_gait_manager(steps=20):
            print("成功使用步态管理器行走")
        else:
            print("无法使用步态管理器，尝试手动控制腿部电机...")
            # 简单的腿部动作
            if "LegUpperR" in motors and "LegUpperL" in motors:
                test_motor("LegUpperR", 0.1, velocity=0.3)
                test_motor("LegUpperL", 0.1, velocity=0.3)
        
        test_phase = 3
    
    elif test_phase == 3 and elapsed_time > 20:
        # 测试脚底触觉传感器 - 通过让机器人抬起一只脚
        print("\n测试脚底触觉传感器...")
        
        # 尝试播放抬腿动作
        if motion_manager and play_motion(1):  # 尝试播放ID为1的动作
            print("成功播放预定义动作")
        else:
            # 如果没有预定义动作，手动抬腿
            if "LegUpperL" in motors and "LegLowerL" in motors and "AnkleL" in motors:
                print("手动抬起左腿...")
                test_motor("LegUpperL", 0.5, velocity=0.3)
                test_motor("LegLowerL", -0.3, velocity=0.3)
                test_motor("AnkleL", -0.2, velocity=0.3)
        
        test_phase = 4
    
    elif test_phase == 4 and elapsed_time > 25:
        # 恢复初始姿势
        print("\n恢复初始姿势...")
        
        # 尝试播放初始姿势动作
        if motion_manager and play_motion(0):  # 尝试播放ID为0的动作（通常是初始姿势）
            print("成功恢复初始姿势")
        else:
            # 如果没有预定义动作，手动恢复
            for name in motors:
                test_motor(name, 0.0, velocity=0.3)
        
        test_phase = 5
    
    elif test_phase == 5 and elapsed_time > 30:
        # 测试完成
        print("\n测试完成！")
        break

print("控制器已退出")
