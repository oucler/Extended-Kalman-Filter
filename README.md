# Object Tracking with Sensor Fusion (Extended-Kalman-Filter)

## Table Content: ##
- [Objective](#objective)
- [Results](#results)
- [How to run](#howto)
- [Code Flow](#codeflow)
- [Measurement Process](#measurement)
- [Lidar measurement](#lidar)
- [Radar measurement](#radar)
- [References](#references)

## Objective: <a name="objective"></a>

	Utilize sensor data from both LIDAR and RADAR measurements for object (e.g. pedestrian, vehicles, 
	or other moving objects) tracking with the Extended Kalman Filter.

## Results: <a name="results"></a>

- Dataset 1
![](/home/oucler/Classes/Udacity/Term2/commit/Extended-Kalman-Filter/images/Result_Dataset1.PNG) 

- Dataset 2
![](/home/oucler/Classes/Udacity/Term2/Extended-Kalman-Filter/images/Result_Dataset2.png) 
## How to run: <a name="howto"></a>

	Perform steps following:

	- Clone this repo: **git clone https://github.com/oucler/Extended-Kalman-Filter.git**
	- Enter into directory: **cd ~/<repository>**
	- Make build directory: **mkdir build && cd build**
	- Compile the code: **cmake .. && make**
	- Run the code: **./ExtendedKF **
	- Start and run Project 1/1 EKF and UKF of Self Driving Car Simulator
	
## Code Flow: <a name="codeflow"></a>

	The code cordination happens in main.cpp then wait for Simulator to start once the connections is established
	then it starts the process flow described below. main.cpp calls FusionEFK.cpp to make initilization, update, and predictions and update and prediction logic are implemented in kalman_filter.cpp. tools.cpp has an implementation of RMSE 
	and Jacobian matrix.  

![](images/CodeFlow.png) 

## Measurement Process: <a name="measurement"></a>

   ** first measurement**: the filter will receive initial measurements of an object's position relative to the car. These measurements will come from a radar or lidar sensor.
   ** initialize state and covariance matrices**: the filter will initialize the object's position based on the first measurement then the car will receive another sensor measurement after a time period Δt.
   **predict**: the algorithm will predict where the bicycle will be after time Δt. One basic way to predict the bicycle location after Δt is to assume the object's velocity is constant; thus the object will have moved velocity * Δt. 
   **update**: the filter compares the "predicted" location with what the sensor measurement says. The predicted location and the measured location are combined to give an updated location. The Kalman filter will put more weight on either the predicted location or the measured location depending on the uncertainty of each value then the car will receive another sensor measurement after a time period Δt. The algorithm then does another predict and update step.


![](images/ekf_flow.jpg) 

## Lidar measurement: <a name="lidar"></a>

	- z = transpose (px py) is the measurement vector. For a lidar sensor, the z vector contains the position−x and position−y measurements.

	- H is the matrix that projects your belief about the object current state into the measurement space of the sensor. For lidar, this is a fancy way of saying that we discard velocity information from the state variable since the lidar sensor only measures position: The state vector x contains information about [p​x​​,p​y​​,v​x​​,v​y​​] whereas the z vector will only contain [px,py]. Multiplying Hx allows us to compare x, our belief, with z, the sensor measurement.

![](images/lidar.jpg) 

## Radar measurement: <a name="radar"></a>


	- The range, (ρ), is the distance to the pedestrian. The range is basically the magnitude of the position vector ρ which can be defined as ρ=sqrt(p​x​2​​+p​y​2​​).
	- φ=atan(p​y​​/p​x​​). Note that φ is referenced counter-clockwise from the x-axis, so φ from the video clip above in that situation would actually be negative.
	- The range rate, ​ρ​˙​​, is the projection of the velocity, v, onto the line, L.

![](images/radar.jpg) 

## References: <a name="references"></a>
	1. https://github.com/JunshengFu/tracking-with-Extended-Kalman-Filter
