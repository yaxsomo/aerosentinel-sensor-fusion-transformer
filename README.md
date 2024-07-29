<p align="center"><img src="https://github.com/user-attachments/assets/68554aaa-ac31-47b2-a1a9-1eeb457715db" align="middle"/></p>

#

The Aerosentinel Sensor Fusion Transformer is a compact realtime embedded Inertial Measurement System (IMU) based Attitude and Heading Reference System (AHRS) using Recursive Least Squares (RLS) for magnetometer calibration, individual Unscented Kalman Filter for sensor fusion on [Aerosentinel Argus Navigation Module](https://github.com/yaxsomo/aerosentinel-argus), and combined state-estimation.

## Features:
- Eigen library not used in this repo (small source code - more simple to understand).
- No third-party library used beside these C header files: `stdlib.h`, `stdint.h`, and `math.h`.
- Setting `SYSTEM_IMPLEMENTATION` to `SYSTEM_IMPLEMENTATION_EMBEDDED_NO_PRINT` in `konfig.h`, makes the code platform-indipendent.
- There's no malloc/new/free dynamic memory allocation (for real time application). However, heavy stack local variables are used, so consider execute it through a memory analyzer if the implementation in  **mission-critical hard real-time applications** is a concern.

- The UKF library and documentation can be found in [this repository](https://github.com/pronenewbits/Embedded_UKF_Library).

Please read them to gain insight on how I implement the filters.

# The Background

This repository is made to have ultra-precise attidude and orientation estimations for rocketry applications, and will be based on multiple IMUs (with different sensitivities) running at the same time.

Before we dive deeper, some results for motivational purposes:

[RESULTS TO BE ADDED]
<!-- 2. UKF result using double precision floating math:
<p align="center"><img src="https://github.com/user-attachments/assets/867dc891-3ecc-4242-a262-1d89f7d7026e"></p>
<p align="center">(See mp4 files for every result using EKF/UKF filter with single and double precision floating math).</p> -->

<!-- 3. Online hard-iron bias identification using RLS for magnetometer data compensation:
<p align="center"><img src="https://github.com/user-attachments/assets/b6dd16d8-7063-44a1-baa8-1a8bb3d2ce78"></p> -->

# The Theoretical Description

The system implemented here consists of 2 major subsystems:

### Subsystem 1: The Sensor Fusion Algorithm

Sensor fusion algorithm works by combining Gyroscope sensor (good for short measurement because of low noise, but not good for long measurement because of drifting), Accelerometer sensor (good for long measurement, but noisy and can only sense one direction, namely [earth's gravitational vector](http://weelookang.blogspot.com/2015/01/ejss-gravity-field-visualisation-model.html0)) and Magnetometer sensor (good for long measurement, but noisy and can only sense one direction, namely [earth's magnetic vector](https://en.wikipedia.org/wiki/Earth%27s_magnetic_field#Characteristics)).

To avoid [gimbal lock](https://en.wikipedia.org/wiki/Gimbal_lock), we use [quaternion](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) to represent where the system is heading (i.e. [the roll, pitch, and yaw of the sensor body](https://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft))). The quaternion kinematics equations can be described as:
<p align="center"><img src="https://github.com/user-attachments/assets/82be5937-5375-4567-923f-c972033694af" alt="Quaternion Kinematics Equations"></p>


We then can re-describe the kinematics equations into a (continuous) nonlinear state space equation:
<p align="center"><img src="https://github.com/user-attachments/assets/b36c1736-42e0-44d5-83bb-23a12f372f9d" alt="Continuous State Space Equation"></p>

For sensor fusion algorithm, I use (discrete) Extended Kalman Filter and (discrete) Unscented Kalman Filter library I've made in [this repository](https://github.com/pronenewbits/Embedded_EKF_Library) (for EKF) and [this repository](https://github.com/pronenewbits/Embedded_UKF_Library) (for UKF). Because of that, we need to transform the equations above into the discrete form that can be used by EKF & UKF. Assuming ZOH-based sensor sampling, the discrete system (and the Jacobian for EKF) can be derived using Euler method as:
<p align="center"><img src="https://github.com/user-attachments/assets/1ebf06bd-72bd-400b-bc9e-4bc1bc55d794" alt="Discrete State Space Equation"></p>


**Remark**: This is the simplest state space system for quaternion based AHRS using MEMS IMU sensor. Many publication use additional state to compensate gyroscope bias and accelerometer bias (some also set the magnetometer bias as another estimated state, not as parameters in the calibration phase), while others expand further by adding state for 3D speed or position (with additional GPS and pressure sensor, or even camera based machine vision) to make a complete guidance system. I hope by using this framework as a base, you can easily explore many different model.

&nbsp;

### Subsystem 2: The Magnetometer Calibration Algorithm

Each of the three sensors (accelerometer, gyroscope, magnetometer) have 12 parameters that needs to be calibrated to make sure sensor data have no offset or deformity. They are:

- 3 parameters of sensor bias <img src="https://github.com/user-attachments/assets/143151cc-1039-44c1-840f-7e250e2510ab" align="top"/>.
- 9 parameters of sensor deformity matrix that represented by 3x3 matrix <img src="https://github.com/user-attachments/assets/2c5d9a23-445d-4314-b571-1cdd93917e0a" align="middle"/>.

In general, if we have measurement from one of the three sensors <img src="https://github.com/user-attachments/assets/a2497c41-a25b-4f53-81f1-a98745634de1" align="top"/>, then the true sensor data <img src="https://github.com/user-attachments/assets/ce5fc65f-bf3a-403a-9886-d093ed37145a" align="top"/> can be calculated by this equation:

<p align="center"><img src="https://github.com/user-attachments/assets/0428bfd9-ea7e-4b2b-b77a-90ba5eb36111" align="middle"/></p>
**Note** that this equation doesn't consider stochastic noise (i.e. this is still a deterministic calibration equation), the stochastic noise will be dealt with sensor fusion algorithm described above.

&nbsp;

In total, we have 12 parameters x 3 sensors = 36 parameters total for IMU sensor (actually we haven't consider cross sensor calibration, e.g. gyroscopic sensitivity from linear acceleration parameters, or the temperature-dependence parameters. [Analog Devices made excellent article about them](https://www.analog.com/en/technical-articles/gyro-mechanical-performance.html)). So the total calibration parameters is more than that.

Fortunately, for many cases the magnetometer bias (the so called hard-iron bias) is the only dominating one. So by compensating the hard-iron bias we can get a good enough sensor data (at least good enough for our purpose). The other sensor's bias and sensor's structural error (hopefully) is not significant enough and can be considered general noise and (hopefully) will be compensated by sensor fusion algorithm. The hard-iron bias identification equation can be derived as:
<p align="center"><img src="https://github.com/user-attachments/assets/fc34322f-0b6a-4847-b0f5-1681910c9b34" align="middle"/></p>


The equation above is an offline or batch identification (you take big enough sample, preferably more than 1000 samples, then using least-squares to identify the bias). The problem with batch identification is the needs for big memory, but we can use [Recursive Least Squares](https://en.wikipedia.org/wiki/Recursive_least_squares_filter) as the online version to calculate the bias (this way we don't need to store all samples data):
<p align="center"><img src="https://github.com/user-attachments/assets/49d2bd98-7169-4e63-ae89-19d421bbd5b4" align="middle"/></p>


**Note**: The RLS algorithm above is general enough that you could also use it to identify not only hard-iron bias, but also soft-iron bias (or the deformity matrix described above). For example, [this paper](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7014484/pdf/sensors-20-00535.pdf) explore further by using RLS/ML combination to calculate 12 magnetometer calibration parameters.
