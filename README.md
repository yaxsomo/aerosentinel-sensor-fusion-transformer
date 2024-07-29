# Arduino_AHRS_System
This is a compact realtime embedded Inertial Measurement System (IMU) based Attitude and Heading Reference System (AHRS) using Recursive Least Squares (RLS) for magnetometer calibration, and EKF/UKF for sensor fusion for Arduino platform.

- It's not using Eigen (small source code - more simple to understand).
- It's not using C++ Standard Library/std (for embedded consideration).
- If you set `SYSTEM_IMPLEMENTATION` to `SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT` in `konfig.h`, the code is platform agnostic (not using any library beside these C header files: `stdlib.h`, `stdint.h`, and `math.h`).
- There's no malloc/new/free dynamic memory allocation (for real time application). But it use heavy stack local variables, so you need to run it through memory analyzer if you are really concerned about implement this in mission critical hard real time application.

This code is the application of Extended Kalman Filter and Unscented Kalman Filter library I've made.

- The EKF library and documentation can be found in [this repository](https://github.com/pronenewbits/Embedded_EKF_Library).
- The UKF library and documentation can be found in [this repository](https://github.com/pronenewbits/Embedded_UKF_Library).

Please read them to gain insight on how I implement the filters.


# The Background

This repository is made to explain the basic system of AHRS system based on an IMU sensor. For ease of use and simple implementation, I use low cost MPU-9250 sensor as the IMU sensor and Teensy 4.0 (an Arduino compatible platform) as the computing module. You can get the Teensy from [here](https://www.pjrc.com/store/teensy40.html), while you can get the MPU-9250 from, well, everywhere (just make sure the MPU-9250 module you get is 3.3V version).

Before we delve deeper, some results for motivational purposes:

1. EKF result using single precision floating math:
<p align="center"><img src="ekf_imu_float32_mute_rot2.gif"></p>
<p align="center">(See mp4 files for every result using EKF/UKF filter with single and double precision floating math).</p>

2. UKF result using double precision floating math:
<p align="center"><img src="ukf_imu_double64_mute_rot2.gif"></p>
<p align="center">(See mp4 files for every result using EKF/UKF filter with single and double precision floating math).</p>

3. Online hard-iron bias identification using RLS for magnetometer data compensation:
<p align="center"><img src="Hard_Iron_Compensation_Using_RLS.png"></p>



# The Theoretical Description

The AHRS system implemented here consists of 2 major subsystems:

### Subsystem 1: The Sensor Fusion Algorithm

Sensor fusion algorithm works by combining Gyroscope sensor (good for short measurement because of low noise, but not good for long measurement because of drifting), Accelerometer sensor (good for long measurement, but noisy and can only sense one direction, namely [earth's gravitational vector](http://weelookang.blogspot.com/2015/01/ejss-gravity-field-visualisation-model.html0)) and Magnetometer sensor (good for long measurement, but noisy and can only sense one direction, namely [earth's magnetic vector](https://en.wikipedia.org/wiki/Earth%27s_magnetic_field#Characteristics)).

To avoid [gimbal lock](https://en.wikipedia.org/wiki/Gimbal_lock), we use [quaternion](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) to represent where the system is heading (i.e. [the roll, pitch, and yaw of the sensor body](https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft))). The quaternion kinematics equations can be described as:
<p align="center"><img src="Quaternion_Kinematics_Equations.png" alt="Quaternion Kinematics Equations"></p>

We then can re-describe the kinematics equations into a (continuous) nonlinear state space equation:
<p align="center"><img src="Continuous_State_Space_Equation.png" alt="Continuous State Space Equation"></p>

For sensor fusion algorithm, I use (discrete) Extended Kalman Filter and (discrete) Unscented Kalman Filter library I've made in [this repository](https://github.com/pronenewbits/Embedded_EKF_Library) (for EKF) and [this repository](https://github.com/pronenewbits/Embedded_UKF_Library) (for UKF). Because of that, we need to transform the equations above into the discrete form that can be used by EKF & UKF. Assuming ZOH-based sensor sampling, the discrete system (and the Jacobian for EKF) can be derived using Euler method as:
<p align="center"><img src="Discrete_State_Space_Equation.png" alt="Discrete State Space Equation"></p>

**Remark**: This is the simplest state space system for quaternion based AHRS using MEMS IMU sensor. Many publication use additional state to compensate gyroscope bias and accelerometer bias (some also set the magnetometer bias as another estimated state, not as parameters in the calibration phase), while others expand further by adding state for 3D speed or position (with additional GPS and pressure sensor, or even camera based machine vision) to make a complete guidance system. I hope by using this framework as a base, you can easily explore many different model.

&nbsp;

### Subsystem 2: The Magnetometer Calibration Algorithm

Each of the three sensors (accelerometer, gyroscope, magnetometer) have 12 parameters that needs to be calibrated to make sure sensor data have no offset or deformity. They are:

- 3 parameters of sensor bias <img src="eq_render/bias_sensor.gif" align="top"/>.
- 9 parameters of sensor deformity matrix that represented by 3x3 matrix <img src="eq_render/deformity_sensor.gif" align="middle"/>.

In general, if we have measurement from one of the three sensors <img src="eq_render/measured_sensor.gif" align="top"/>, then the true sensor data <img src="eq_render/true_sensor.gif" align="top"/> can be calculated by this equation:

<p align="center"><img src="eq_render/sensor_calib_eq.gif" align="middle"/></p>
**Note** that this equation doesn't consider stochastic noise (i.e. this is still a deterministic calibration equation), the stochastic noise will be dealt with sensor fusion algorithm described above.

&nbsp;

In total, we have 12 parameters x 3 sensors = 36 parameters total for IMU sensor (actually we haven't consider cross sensor calibration, e.g. gyroscopic sensitivity from linear acceleration parameters, or the temperature-dependence parameters. [Analog Devices made excellent article about them](https://www.analog.com/en/technical-articles/gyro-mechanical-performance.html)). So the total calibration parameters is more than that.

Fortunately, for many cases the magnetometer bias (the so called hard-iron bias) is the only dominating one. So by compensating the hard-iron bias we can get a good enough sensor data (at least good enough for our purpose). The other sensor's bias and sensor's structural error (hopefully) is not significant enough and can be considered general noise and (hopefully) will be compensated by sensor fusion algorithm. The hard-iron bias identification equation can be derived as:
<p align="center"><img src="Hard_Iron_Bias_Identification_by_Least_Squares.png" align="middle"/></p>

The equation above is an offline or batch identification (you take big enough sample, preferably more than 1000 samples, then using least-squares to identify the bias). The problem with batch identification is the needs for big memory, but we can use [Recursive Least Squares](https://en.wikipedia.org/wiki/Recursive_least_squares_filter) as the online version to calculate the bias (this way we don't need to store all samples data):
<p align="center"><img src="Hard_Iron_Bias_Identification_by_Recursive_Least_Squares.png" align="middle"/></p>

**Note**: The RLS algorithm above is general enough that you could also use it to identify not only hard-iron bias, but also soft-iron bias (or the deformity matrix described above). For example, [this paper](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7014484/pdf/sensors-20-00535.pdf) explore further by using RLS/ML combination to calculate 12 magnetometer calibration parameters.
