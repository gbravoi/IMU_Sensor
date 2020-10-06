# IMU sensor fusion algorithm for monitoring knee kinematics in ACL reconstructed patients
These codes are an example of the use of our proposed sensor fusion algorithm (ProposedFilter.m) in the computation of the knee flexion and extension. In this example, our proposed algorithm is compared against a sensor fusion algorithm Kalman-filter based (kalmanfilter.m), and a reference obtained from a MOCAP (motion capture) system. The output are the Root mean squared  error (RMSE) and correlation coefficients (CCC and ICC), and a graph that compare our proposed and Kalman-filter algorithm against the MOCAP system.

The Arduino code for the data extraction and circuit connection are included in the data section.

The original paper can be found in https://ieeexplore.ieee.org/abstract/document/8857431
