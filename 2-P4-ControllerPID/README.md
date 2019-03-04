# **PID Controller**

---

**The goal of this project is to implement a PID controller to control a vehicle**

---

#### PID Controller Implementation

The classic PID controller is implemented. The steering value **&alpha;** is calculated as:

**&alpha; = -&tau;<sub>p</sub>&sdot;CTE -&tau;<sub>d</sub>&sdot;dCTE/dt -&tau;<sub>i</sub>&sdot;&sum;CTE**

where **CTE** is cross track error and **&tau;<sub>p</sub>**, **&tau;<sub>d</sub>** and **&tau;<sub>i</sub>** are hyperparameters controlling contributions of proportional, derivative and integral error terms respectively.

#### Hyperparameters Tuning

Optimal values for hypeparameters **&tau;<sub>p</sub>**, **&tau;<sub>d</sub>** and **&tau;<sub>i</sub>** can be found using some optimization algorithm like Twiddle. However, in this project manual tuning was sufficient. After some experiments, the values for hyperparameters were choosen as:

| Parameter             |     Value	    | 
|:----------------------|:--------------| 
| &tau;<sub>p</sub>  		| 0.2   				| 
| &tau;<sub>d</sub>  		| 3.5   				| 
| &tau;<sub>i</sub>  		| 5e-4  				|
