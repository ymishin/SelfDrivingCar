# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

PID Controller Implementation


The basic PID controller is implemented. The steering value is calculated as:

...
Kp, Kd and Ki are hyperparameters controlling .. ... ... respectively.

error_p, error_d and error_i are error terms calculated as:

error_p
error_d
error_i

---

Hyperparameters Tuning

Optimal hypeparameters can be found using some optimization algorithm like Twiddle. However, in this project manual tuning was sufficient. After some experiments the values for Kp, Kd and Ki were choose as:
1 2 3

---