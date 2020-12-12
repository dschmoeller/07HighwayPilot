# Project 7: Path Planning



## **Project Goals: **

* Safely navigate around a virtual highway with other traffic
* The car should try to go as close as possible to the 50 MPH speed limit
* The car should pass slower traffic when possible
* The car should avoid hitting other cars at all cost
* The car should be able to make one complete loop around the 6946m highway
* The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3



## Compilation

#### 1. The code compiles correctly?

[]: https://github.com/dschmoeller/07HighwayPilot/blob/master/images/CodeCompiles.PNG



## Valid Trajectories

| Specification                                                | Achieved? |
| ------------------------------------------------------------ | --------- |
| The car is able to drive at least 4.32 miles without incident | Yes       |
| The car drives according to the speed limit                  | Yes       |
| Max Acceleration and Jerk are not Exceeded                   | Yes       |
| Car does not have collisions                                 | Yes       |
| The car stays in its lane, except for the time between changing lanes | Yes       |
| The car is able to change lanes                              | Yes       |

[]: https://github.com/dschmoeller/07HighwayPilot/blob/master/images/TrajectoryGenerationSnip.PNG



## Code Reflection 

The highway planner implementation builds up on the Udacity introduction lesson. So, the general strategy is to define anchor points and use splines to create a curve which passes those anchor points. In comparison to trajectory generation techniques, e.g. jerk minimizing trajectories, using the spline approach is very beneficial for replanning i.e. updating the trajectory. Rather than replanning the entire trajectory for every single iteration, the highway planner implementation only extends the previous calculated trajectory by the number of points which has been traversed by the simulated AV (Autonomous Vehicle) in the previous iteration. 



The following code snippet shows the global variables which are mostly used to sort of configure the highway planner.  

```c++
int number_pts = 100; // Defines length of trajectory

int av_lane = 1; // Middle lane 

double av_ref_vel = 0; // [m/s]

double av_max = 22; // [m/s]

double a_max = 0.15; // [m/ss]

int state = 0;

bool lock_LC = false; 

unsigned long cnt = 0;

double av_pass_vel_min = 15; // [m/s] 
```

The trajectory comprises 100 points. The starting position of the AV is defined to be the middle lane and it starts with a reference velocity of 0 m/s. The maximal acceleration is defined to be 0.15 m/ss and the speed limit is 22 m/s. There are four states in which the AV can be: 

- Keep Lane (1)
- Track Behind (2)
- Pass Left (3)
- Pass Right (4)

The ***state*** variable reflects the current state and is initialized with a value of 0. Once the AV has identified that it´s possible/safe to do a lane change, it´s committed to this lane change maneuver for 1 s. Intuitively this means that the AV is confident to execute the lane change no matter what happens in between. This is pretty much the same how humans behave. Once they identified it´s safe to go, they just execute the lane change until its end, rather than constantly rechecking during the maneuver. Being able to confidently perform actions is vital for AVs. Acting too cautiously can lead to an undesired loitering or even stucking behavior. The ***lock_LC*** variable is used to flag an ongoing overtaking maneuver. The counter variable ***cnt*** determines how long the AV is locked for other lane changing maneuvers. The lane change trajectory is defined to be rather smooth. This means, the AV is slightly increasing its d value (Frenet coordinates) in order to slowly approach the adjacent lane. However, this kind of slowly swerve can lead to dangerous situations if the AVs velocity is low. It turns out that cars from the back often cause collisions in these low velocity lane changing cases. In order to avoid these situations, the AV is only allowed to change lanes if its current velocity is greater than a specific threshold. This threshold is defined by the ***av_pass_vel_min*** variable  to be 15 m/s.  



The actual highway pilot logic is implemented in the ***onMessage( )*** function, which is being called evey 20 ms. The core of the highway planner is described below. The following code snippet shows the part of the implementation which takes care of reusing the previously calculated trajectory points. 

```c++
// Write already calculated points back from previous iteration

// and increment speed if v_desired hasn't reached 

for (int i = 0; i < path_size; ++i) {

    next_x_vals.push_back(previous_path_x[i]);

    next_y_vals.push_back(previous_path_y[i]);

}
```



The next step is to apply collision checking. This is done by using the sensor fusion information which comprises tracked data from all other cars. The collision checker logic iterates through every single car (actor) and identifies whether the current car is even in the AVs lane. If this applies, it´s further being checked whether the AV trajectory endpoint "collides" with the current position (s value) of another car. 

```c++
         // Collision Checker

          bool collision = false; 

          for (auto actor : sensor_fusion){

            double actor_s = actor[5]; 

            double actor_d = actor[6]; 

            // Is the actor in the AVs lane?

            double left_lane_bound = 4*av_lane; 

            double right_lane_bound = 4 + 4*av_lane; 

            if (actor_d > left_lane_bound && actor_d < right_lane_bound){

              // Check for future collision, i.e. at the end of AVs trajectory

              // Do not consider cars in the back 

              if (actor_s > car_s && actor_s < end_path_s ){ 

                collision = true; 

              } 

            }

          }
```



In case there has been a future collision identified, the next step is to check whether it´s safe to change lanes. The logic is the same for both left and right lane changes. The following code snippet only shows the left case. First thing to check is whether a lane change is locked. For instance if the AV is currently about to do a left lane change, it should not happen that suddenly a right lane change kicks in. Rather the AV should execute the entire lane change maneuver and wait some time before it´s allowed to change lanes again. In this particular setting, the counter variable checks against a value of 100. This means that overtaking maneuvers are locked for roughly 2 seconds. If the AV would be allowed to do a lane change, it´s further being checked whether it´s safe to do so. The corresponding logic is similar to the collision checker. First, all cars which are in the desired AV lane (that´s the lane where the AV plans to change) are filtered out. Then, the s-values (Frenet) of these cars are checked against a safety distance threshold, which is 25 meters in this particular implementation. This means, at time of potential lane change, the AV checks whether there´s a 50 meter free corridor as the safety distance threshold applies to both cars in front and also cars behind.   

```c++
         // Is it possible to change lanes?  

          if (collision == true){

            // Check if lane change is still locked

            if (cnt > 100){

              lock_LC = false; 

            }  

            // Is it possible to do a left lane change?

            bool pass_left = true;

            int desired_av_lane_left = av_lane - 1; 

            if (desired_av_lane_left < 0){ 

              // AV is already in the most left lane

              // Change lane left is not possible

              pass_left = false;  

            }

            // Check whether it's safe to change lane left

            else {

              for (auto actor : sensor_fusion){

                double actor_s = actor[5]; 

                double actor_d = actor[6]; 

                // Is this actor in the desired AV lane? 

                double left_lane_bound = 4*desired_av_lane_left; 

                double right_lane_bound = 4 + 4*desired_av_lane_left;

                if (actor_d > left_lane_bound && actor_d < right_lane_bound){

                  // Check whether this car occupies space nearby the AV

                  double safety_distance = 25; 

                  if (actor_s > car_s - safety_distance && actor_s < car_s + safety_distance ){

                    pass_left = false; 

                  }

                }

              }

            } 
```



The next step is to actually update the state variables according to what the AV is supposed to do. The first state corresponds to the situation of no future collisions. This means the AV can just keep its current lane and even increase the velocity if the speed limit hasn´t  been reached yet. In case of a potential collision, it´s preferable to change lanes rather than just brake, since the AV should drive as fast as possible. Also, due to the German laws, it´s always preferable to change to the left lane than to the right one. In case no lane change is possible, i.e. safe enough, the least preferable state is to track behind the leading car and just brake. The following code snippet shows the corresonding implemented logic. 

```c++
            // Adapt AV states

            //    --> State 1: Keep Lane

            //    --> State 2: Track Behind (i.e. Brake)

            //    --> State 3: Pass Left

            //    --> State 4: Pass Right

            // Only change lanes if AV speed is high enough!   

            if (pass_left == true && lock_LC == false && car_speed > av_pass_vel_min){ state = 3; }

            else if (pass_right == true && lock_LC == false && car_speed > av_pass_vel_min){ state = 4; }

            else { state = 2; } // If it is not possible to pass, brake

          }

          // If there's no collision predicted, just keep lane

          else { state = 1; } 
```



Once the AV state has been updated, te next step is to actually execute the corresponding actions. Due to Frenet coordinates, this is very straightforward. The AV should always increase its velocity if the speed limit hasn´t reached yet, except the AV is supposed to brake (state 2) due to a potential collision. In the both lane changing cases, only the desired ***av_lane*** variable is increased/decreased.   

```c++
          // Execute corresponding state actions

          // Increment speed if v_max hasn't reached yet and the AV isn't supposed to brake

          if (state != 2 && av_ref_vel < av_max){ av_ref_vel += a_max; }

          // Either change lanes left/right or brake

          // It's always preferable to do a left lane change

          if (state == 3){

            av_lane -= 1;

            if (av_lane < 0){

              av_lane = 0; 

            }

            // Lock lane changes for next iterations

            lock_LC = true;

            cnt = 0;

          }

          else if (state == 4 ){

            av_lane += 1;

            if (av_lane > 2){

              av_lane = 2; 

            }

            // Lock lane changes for next iterations

            lock_LC = true; 

            cnt = 0;

          }

          // Brake (i.e. decrease reference velocity 

          else if (state == 2) { 

            av_ref_vel -= a_max; 

          }
```



The remaining code pretty much follows the example of the project introduction lesson. It´s about defining the anchor points for re calculating the splines in each  single iteration. There are five anchor points being used, three future waypoints and the last two trajectory points, i.e. the "most future trajectory points". In order to make the math easier, there´s a coordinate system transformation applied from the Map coordinate system to the AV coordinate system. Finally, the splines are used to sample new trajectory points. The reference velocity is used to determine the appropriate distance between trajectory points according to velocity and acceleration constraints. The last step is to transform the determined trajectory points back into the global coordinate system and append them to the list of next x and y values. 

```c++
          // Set two starting anchor points for spline calculation

          // Distinguish inital starting mode and running mode

          if (path_size < 2) { 

            // Identify (assumed) previous point as anchor point for the spline

            // Use any previous point along heading line --> 1 * sin/cos

            double prev_x = car_x - cos(car_yaw); 

            double prev_y = car_y - sin(car_yaw);

            spline_points_x.push_back(prev_x); 

            spline_points_x.push_back(car_x); 

            spline_points_y.push_back(prev_y); 

            spline_points_y.push_back(car_y); 

          } 

          else {

            // --> Use the two most future waypoint as new starting/anchor points

            start_ref_x = previous_path_x[path_size-1];

            start_ref_y = previous_path_y[path_size-1];

            double prev_x = previous_path_x[path_size-2];

            double prev_y = previous_path_y[path_size-2];

            start_ref_yaw = atan2(start_ref_y - prev_y, start_ref_x - prev_x);

            // Add previous and current car points as anchor to the spline list

            spline_points_x.push_back(prev_x); 

            spline_points_x.push_back(start_ref_x); 

            spline_points_y.push_back(prev_y); 

            spline_points_y.push_back(start_ref_y); 

          }


          // Add three more far away waypoints in order to define the splines

          double waypoint1 = 70.0; 

          double waypoint2 = 100.0; 

          double waypoint3 = 130.0; 

          vector<double> wp1 = getXY(car_s + waypoint1, (2+4*av_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 

          vector<double> wp2 = getXY(car_s + waypoint2, (2+4*av_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 

          vector<double> wp3 = getXY(car_s + waypoint3, (2+4*av_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y); 

          spline_points_x.push_back(wp1[0]); 

          spline_points_x.push_back(wp2[0]);

          spline_points_x.push_back(wp3[0]);

          spline_points_y.push_back(wp1[1]); 

          spline_points_y.push_back(wp2[1]);

          spline_points_y.push_back(wp3[1]);

          
          // Transform all five waypoints from global into AV coordinate system

          // Applying rotation and translation

          for (int i = 0; i < spline_points_x.size(); i++){

            // Translation

            double shift_x = spline_points_x[i] - start_ref_x; 

            double shift_y = spline_points_y[i] - start_ref_y; 

            // Applying translatin and rotation

            spline_points_x[i] = (shift_x*cos(0-start_ref_yaw) - shift_y*sin(0-start_ref_yaw));

            spline_points_y[i] = (shift_x*sin(0-start_ref_yaw) + shift_y*cos(0-start_ref_yaw));  



          }

          // Define the spline

          tk::spline s; 

          s.set_points(spline_points_x, spline_points_y); 



          // Use spline to sample points onto the trajectory   

          // Linearize spline in order to calculate incremental x values dependend on desired velocity

          double target_x = waypoint1; 

          double target_y = s(target_x); 

          double target_dist = sqrt(target_x*target_x + target_y*target_y); 

          // Remember previous point when going along the spline curve

          double x_add_on = 0; 

          // Iterate over to be filled up points  

          for (int i = 0; i < (number_pts - path_size); ++i) {

            // Sample points from spline using linearized x values

            // Identify equally distributed (vel dependend) x points on triangle

            double N = (target_dist/(0.02*av_ref_vel)); 

            double increment = target_x/N;  

            double x_point = x_add_on + increment; 

            double y_point = s(x_point); 

            x_add_on = x_point; 

            // Rotate x,y point back to Map coordinate system (They're still in AV COS)

            double x_tmp = x_point; 

            double y_tmp = y_point; 

            x_point = x_tmp*cos(start_ref_yaw) - y_tmp*sin(start_ref_yaw); 

            y_point = x_tmp*sin(start_ref_yaw) + y_tmp*cos(start_ref_yaw); 

            x_point += start_ref_x; 

            y_point += start_ref_y; 

            // Add point to Simulator buffer 

            next_x_vals.push_back(x_point);

            next_y_vals.push_back(y_point);

          }
```



