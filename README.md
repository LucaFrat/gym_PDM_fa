# Autonomous Driving 
> This is developed by Frattini Luca, Iskandar Alain, Nandkumar Chandran and Sotirchos Georgios as part of the Planning & Decision Making (RO47005) course of TU Delft on 2023-01-16.
>
In this project, an autonomous prius car is driving in a dynamic parking lot. It has to go from a location A to a location B while avoiding walls and walking pedestrians.


## Structure

The folder contains several files, but hereunder you can find the most important ones :

- gym_PDM_fa/simulation.py
- gym_PDM_fa/utils/mpc_utils.py
- gym_PDM_fa/utils/trajectory_utils.py
- gym_PDM_fa/rrt_star_dubins.py
- gym_PDM_fa/model_predictive_speed_and_steer_control.py


The center file is the simulation.py that will launch a lot of other files to run the simulation. Among these files, it uses the mpc_utils.py, which runs model_predictive_speed_and_steer_control.py; and trajectory_utils.py in which the path is retrieved by running rrt_star_dubins.py and finally a trajectory is generated.

## Functioning

Assuming the static obstacles (the wall) fixed, we use RRT* as global planner to find the path to the goal. The vehicle being non-holonomic, we used Dubins path as steering function to follow the path. Both of these implementations were made in the rrt_star_dubins.py file. After generating the path that we have to follow, we used an MPC as controller. Moreover, we used the same MPC as local planner to avoid the dynamic obstacles not taken into account by RRT* (the pedestrians). These two actions were implementated in the model_predictive_speed_and_steer_control.py file. Finally, a graphical simulation using Matplotlib is done by running the simulation.py file.




## Simulation

To run the simulation, you can go to the gym_PDM_fa directory and launch the simulation manually from the terminal, as shown below.

 ```
 user@computer:~$ cd gym_PDM_fa
 user@computer:~/gym_PDM_fa$ python3 simulation.py
 ```

