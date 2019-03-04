# **MPC Controller**

---

**The goal of this project is to implement a MPC controller to control a vehicle**

---

#### The Model

Vehicle state is defined as:

**[x, y, &psi;, v, cte, e&psi;]**

Actuators to be predicted are:

steering angle **&delta** and throttle/brake **a**

Model update equations are formulated as:

**x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub>&sdot;cos(&psi;<sub>t</sub>)&sdot;dt**

**y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub>&sdot;sin(&psi;<sub>t</sub>)&sdot;dt**

**&psi;<sub>t+1</sub> = &psi;<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>)&sdot;&delta;<sub>t</sub>&sdot;dt**

**v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub>&sdot;&sdot;dt**

**cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub>&sdot;sin(e&psi;<sub>t</sub>)&sdot;dt**

**e&psi;<sub>t+1</sub> = &psi;<sub>t</sub> - &psi;des<sub>t</sub> + (v<sub>t</sub>/L<sub>f</sub>)&sdot;&delta;<sub>t</sub>&sdot;dt**

Parameters descriptions follow:

| Parameter             |     Description	    | 
|:----------------------|:--------------| 
| **x** and **y**   		 |  Vehicle coordinates 				| 
| **v** | Vehicle velocity    				| 
| **&psi;** |     				| 
| **dt** | Timestep     				| 
| **cte** | Cross track error    				| 
| **e&psi;** |     				| 
| **&psi;des** |     				| 
| **L<sub>f</sub>** | Distance between vehicle front and its center of gravity |

#### Preprocessing

All operations are perfomed in vehicle's coordinate system. Thus, prior to polynomial fitting, all waypoints are transformed to vehicle's coordinate system and initial vehicle position is set to **(0,0)**.

#### Timestep and Elapsed Duration

Different values for timestep **dt** and elapsed duration **N** have be tested. Timestep **dt** has been evaluated in the range **0.05-0.2** and elapsed duration **N** has been evaluated in the range **5-20**. The following values have been choosen:

| Parameter             |     Value	    | 
|:----------------------|:--------------| 
| timestep **dt**   		 | 0.1   				| 
| elapsed duration **N** | 10    				| 

These choosen values provide good balance between accuracy and performance.

#### Latency

The MPC contoller predicts actuators values for elapsed duration **N**. To deal with latency, the actuators values to be used by a vehicle were calculated as arithmetic averages of the first **n** predicted timesteps. After some experiments, **n** has been choosen to be **3**.
