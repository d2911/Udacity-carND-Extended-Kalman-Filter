# Sensor Fusion using Extended Kalman Filter

Self Driving Car Nano Degree to use extended Kalman filter for Sensor Fusion of Radar and Lidar Data using c++. 

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

### Processing Lidar Data 

**Prediction Step**

**Update Step**

### processing Radar Data

**Prediction Step**

**Update Step**

### Fusion
![](/IMG/3.jpg)
