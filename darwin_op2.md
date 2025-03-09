# Webots Motor

[官方op2机器人文档](https://cyberbotics.com/doc/guide/robotis-op2?version=R2022a)

[官方Motor文档](https://cyberbotics.com/doc/reference/motor?version=R2022a&tab-language=python#motor-functions)

## 一、位置控制

* **设置目标位置:**

```
motor.setPosition(position)
```

   `position`: 目标位置，单位取决于电机类型（旋转电机为弧度，线性电机为米）。对于舵机，通常使用弧度。
   对于无限旋转的电机（例如连续旋转的舵机），可以设置为 `float('inf')` (正无穷) 或 `float('-inf')` (负无穷) 来让电机持续朝一个方向旋转。

* **获取目标位置:**

```
target_position = motor.getTargetPosition()
```

* **获取最小位置:**

```
min_position = motor.getMinPosition()
```

* **获取最大位置:**

```
max_position = motor.getMaxPosition()
```

## 二、速度控制

* **设置速度:**

```
motor.setVelocity(velocity)
```

    *`velocity`: 目标速度，单位取决于电机类型（旋转电机为弧度/秒，线性电机为米/秒）。

* **获取当前速度:**

```
current_velocity = motor.getVelocity()
```

* **获取最大速度:**

```
max_velocity = motor.getMaxVelocity()
```

## 三、加速度控制

* **设置加速度:**

```
motor.setAcceleration(acceleration)
```

    `acceleration`: 加速度，单位取决于电机类型（旋转电机为弧度/秒²，线性电机为米/秒²）。
    -1 表示使用默认加速度（无限制）。

* **获取当前加速度:**

```
current_acceleration = motor.getAcceleration()
```

## 四、力/力矩控制

* **设置可用最大力 (线性电机):**

```
motor.setAvailableForce(force)
```

    `force`: 最大力值（牛顿）。

* **获取可用最大力 (线性电机):**

```
available_force = motor.getAvailableForce()
```

* **获取最大力 (线性电机)**

```
max_force = motor.getMaxForce()
```

* **设置可用最大扭矩 (旋转电机):**

```
motor.setAvailableTorque(torque)
```

    `torque`: 最大扭矩值（牛顿·米）。

* **获取可用最大扭矩 (旋转电机):**

```
available_torque = motor.getAvailableTorque()
```

* **获取最大扭矩(旋转电机):**

```
max_torque = motor.getMaxTorque()
```

* **直接设置力 (线性电机, 绕过PID):**

```
motor.setForce(force)
```

* **直接设置扭矩 (旋转电机, 绕过PID):**

```
motor.setTorque(torque)
```

### 力/力矩反馈

* **启用力的反馈(线性电机)**:

```
motor.enableForceFeedback(sampling_period)
```

+ **禁用力的反馈**

```
motor.disableForceFeedback()
```

* **获取力的反馈值:**

```
force_feedback = motor.getForceFeedback()
```

* **获取力反馈的采样周期:**

```
force_feedback_sampling_period = motor.getForceFeedbackSamplingPeriod()
```

* **启用扭矩的反馈(旋转电机):**

```
motor.enableTorqueFeedback(sampling_period)
```

* **禁用扭矩反馈:**

```
motor.disableTorqueFeedback()
```

* **获取扭矩的反馈值:**

```
torque_feedback = motor.getTorqueFeedback()
```

* **获取扭矩反馈的采样周期:**

```
torque_feedback_sampling_period = motor.getTorqueFeedbackSamplingPeriod()
```

## 其他

* **获取电机类型:**

```
motor_type = motor.getType() # 0: 旋转电机, 1: 线性电机
```

* **获取关联的制动设备**

```
brake = motor.getBrake()
```

* **获取关联的位置传感器**

```
position_sensor = motor.getPositionSensor()
```

* **multiplier**

```
multiplier = motor.getMultiplier()
```

## 注意事项

* 直接使用 `motor.setForce()`或 `motor.setTorque()`会绕过内置的PID控制器。
* 在使用力/扭矩反馈之前，需要先使用 `motor.enableForceFeedback(sampling_period)`或 `motor.enableTorqueFeedback(sampling_period)`启用它（类似启动传感器），并设置采样周期,`sampling_period`是整数，且需要大于0.

[部分测试代码](https://github.com/StelixarVale/op2_Simulated_Devices_test)
