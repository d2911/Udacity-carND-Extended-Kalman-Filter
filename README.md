# Sensor Fusion using Extended Kalman Filter

Self Driving Car Nano Degree to use extended Kalman filter 2-D for Sensor Fusion of Radar and Lidar Data using c++. 

## Radar and Lidar Data

simulated input data format for lidar and Radar are  
L(for laser) meas_px meas_py timestamp

![](/IMG/1.jpg)

R(for radar) meas_rho meas_phi meas_rho_dot timestamp

![](/IMG/2.jpg)

Additionally Ground Truth data is given to measure the error in the implementation with  gt_px gt_py gt_vx gt_vy

**Example Data**  
L	1.173848e+00	4.810729e-01	**1477010443100000**	1.119984e+00	6.002246e-01	5.199429e+00	5.389957e-03  
R	1.047505e+00	3.892401e-01	4.511325e+00	**1477010443150000**	1.379955e+00	6.006288e-01	5.198979e+00	1.077814e-02

## Extendedn Kalman Filter for Sensor Fusion - Implementation

Basic idea is with the past state, current state is predicted. With the measurement values, predicted state is updated to new State. Kalman Filter happens in these two steps Predict and Update. We will see below how to use kalman filter for Lidar and Radar Data, then we will Fuse them together.

State is represented by x_position, y_position, Velocity in x-direction and velocity in y-direction which is a 1x4 Matrix and state covaiance is represented in 4x4 Matrix **"P"**. “Covariance” indicates the direction of the linear relationship between variables. 

### Processing Lidar Data 

**Prediction Step**

From the past State, current state is predicted by applying a Transition Matrix (4x4) so that predictedState = [x_position * dt * vel-x-direction, y_position * dt * vel-y-direction, vel-x-direction, vel-y-direction]

`X_pred = F * X_past`  
(4x1) = (4x4) * (4x1)

Predicted Covariance is

`P_pred = F * P_past * Ft + Q` where Ft is Transpose of F and Q is Process Covariance.  
(4x4) = (4x4)*(4x4)*(4x4) + (4x4)

**Update Step**

First Step in Updation is to measure the error in prediction against measurement. Here From the state only relevent parametes inline with measurment parameters are used. For Lidar measurment Z_meas gives only position X and y, so it is sufficient to get position information from state.

`Z_pred = H * X_pred`  
(2x1) = (2x4) * (4x1)  
`y_err = Z_meas - Z_pred`
(2x1) = (2x1) - (2x1)

Other required matrix are calculated.
`S = H*P*Ht + R`where R is Measurement covariance  
(2x2) = (2x4)*(4x4)*(4x2)+(2x2)  
`K = P*Ht*Si` where Si is Inverse of S  
(4x2) = (4x4)*(4x2)*(2x2)

With these Matrix calculated, new state and its covariance are estimated.

`x = (K * y_err) + X_Pred`  
(4x1) = (4x2)*(2x1)  + (4x1)  
`p = (I - K*H) + P_pred` where I is identity matrix  
(4x4) = ((4x4) - (4x2) * (2x4)) + (4x4)

### processing Radar Data

**Prediction Step**

**Update Step**

### Fusion

![](/IMG/3.jpg)

## Visualizing in simulator

Lidar - Blue Points
Radar - Red points
Predicted states - Green
Ground Truth - Black

![](/IMG/4.gif)
