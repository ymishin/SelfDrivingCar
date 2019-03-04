# **MPC Controller**

---

**The goal of this project is to implement a MPC controller to control a vehicle**

---

#### The Model

Student describes their model in detail. This includes the state, actuators and update equations.

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
