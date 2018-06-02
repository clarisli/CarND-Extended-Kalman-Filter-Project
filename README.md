# Extended Kalman Filter Project

The goal is to utilize a kalman filter to estimate the state of a moving object of interst with noisy lidar and radar measurements. 

[//]: # (Image References)
[image1]: ./images/x.gif
[image2]: ./images/x_prime.gif
[image3]: ./images/F.gif
[image4]: ./images/p_prime.gif
[image5]: ./images/state_transition_matrix.png
[image6]: ./images/P_prime.png
[image7]: ./images/noise.png
[image8]: ./images/Q.png
[image9]: ./images/y.png
[image10]: ./images/H_laser1.png
[image11]: ./images/H_laser2.png
[image12]: ./images/z.gif
[image13]: ./images/R.png
[image14]: ./images/z.png
[image15]: ./images/omega.png
[image16]: ./images/h.png
[image17]: ./images/y_radar.png
[image18]: ./images/Hj.png
[image19]: ./images/KF.png
[image20]: ./images/ekf.gif

![alt text][image20]

### Setup
* Download Simulator [here](https://github.com/udacity/self-driving-car-sim/releases)
* Install uWebSocketIO [here](https://github.com/uWebSockets/uWebSockets)

### Basic Build Instructions
1. Make a build director  `mkdir build && cd biuld`
2. Compile: `cmake .. && make`
3. Run it: `./ExtendedKF`

### Implementation

![alt text][image19]

The goal is to track an object in 2D with state x that is described by a position and velocity in x and y directions.

![alt text][image1]

#### Step 1. First Measurment

I initialize the state differently for lidar and radar. 

If it's radar, I converted the coordinates from polar to cartesian first, and update all four elements (**px, py, vx, py**) in the state vector **x** .

If it's lidar, I used the measurement data directly as **px** and **py**.

I did this in lines 61 to 102 in `FusionEKF.cpp`.

#### Step 2. Prediction Step

Prediction has two parts: state mean and state covariance.

#### 2.1 State Mean

The state transition function is

![alt text][image2]

where

![alt text][image3]

and **x'** is the state we predict the object to be at after Δt.

**F** is a matrix that, when multiply with **x**, predicts the state the object will be after Δt.

By using the linear motion model with a constant velocity, the new location **p'** is calculated as

![alt text][image4]

where **p** is the old location, and the velocity **v** will be the same and the new velocity **v = v'** because velocity is constant.

We can express this in a matrix form:

![alt text][image5]

Since we are representing the object's location and velocity as Gaussian distributions with mean x. The noise is also represented by a Gaussian distribution but with mean zero; hence, noise = 0 is saying that the mean noise is zero. The equation then becomes **x' = F * x**.

#### 2.2 State Covariance

In reality the velocity can change due to acceleration. The model includes this uncertainty as process noise **P**. When we predict the position one second later, the uncertainty increase:
 
![alt text][image6]

where **Q** is the covariance of noice 

![alt text][image7]

and it's defined as:

![alt text][image8]

The values for variance of ax and ay are chosen to be **9**.

I did this in lines 29 to 31 of the function `Predict()` in `kalman_filter.cpp`. And I defined **Q** and **F** in lines 120 to 138 of the function `ProcessMeasurement()` in `FusionEKF.cpp`.

### Step 3. Update Step

Lidar and Radar measurements are handled differently in lines 153 to 163 in `FusionEKF.cpp`.

#### 3.1 Lidar Measurements

Lidar only measures position, the measurement vector **z** is as follows:

![alt text][image12]

We first need to compare where we think we are **x'** with the sensor data **z**:

![alt text][image9]

I did this in line 39 in `kalman_filter.cpp`. 

**H** is the matrix that projects the belief about the object's current state into the measurement space of the sensor.

For lidar, H has to project from 4D state to 2D measurement space:

![alt text][image10]

where

![alt text][image11]

The measurements **z** has noise and represents as ω:

![alt text][image14]

where

![alt text][image15]

The model includes this uncertainty as measurement noise covariance matrix **R**. 

![alt text][image13]


The Kalman filter gain, **K**, combines the uncertainty of where we think we are **P'** and the undertainty of measurement **R**.  If our sensor measurements are very uncertain (**R** is high relative to **P'**), then the Kalman filter will give more weight to where we think we are: **x'**. If where we think we are is uncertain (**P'** is high relative to **R**), the Kalman filter will put more weight on the sensor measurement: **z**. 

I did this in lines 75 to 85 of the function `KF()` in `kalman_filter.cpp`.


#### 3.2 Radar Measurements

There's no **H** matrix will map the state vector **x** into polar coordinate. Instead, a **h(x)** funciton maps the predicted position and velocity into polar coordinates or range, bearing, and range rate.

![alt text][image16]

and the radar **y** becomes

![alt text][image17]

I did this in lines 49 to 69 of the function `UpdateEKF()` in `kalman_filter.cpp`.

Since **h(x)** is non-linear, we need to take the Jacobian to linearize **h(x)**:

![alt text][image18]

I did this in line 155 in `FusionEKF.cpp`.

### Result

I successfully obtained an RMSE [0.097, 0.855, 0.4513, 0.4399], which satisfies the requirement RMSE <= [.11, .11, 0.52, 0.52].


