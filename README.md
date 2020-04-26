# Sensor Fusion using Extended Kalman Filter

Self Driving Car Nano Degree to use extended Kalman filter for Sensor Fusion of Radar and Lidar Data.

### Radar and Lidar Data

imput data format for Radar and Lidar are  
#L(for laser) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy

![](/IMG/1.jpg)

#R(for radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy

![](/IMG/2.jpg)

**Example Data**  
L	1.173848e+00	4.810729e-01	**1477010443100000**	1.119984e+00	6.002246e-01	5.199429e+00	5.389957e-03	1.036644e-03	2.072960e-02  
R	1.047505e+00	3.892401e-01	4.511325e+00	**1477010443150000**	1.379955e+00	6.006288e-01	5.198979e+00	1.077814e-02	2.073124e-03 2.763437e-02

