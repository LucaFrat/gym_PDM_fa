# Autonomous Driving 
> This is developed by Frattini Luca, Iskandar Alain, Nandkumar Chandran and Sotirchos Georgios as part of the Planning & Decision Making (RO47005) course of TU Delft on 2023-01-16.
>
In this project, an autonomous prius car is driving in a dynamic parking lot. It has to go from a location A to a location B while avoiding walls and walking pedestrians.


## File Structure

The folder contains several files, but hereunder you can find the most important ones :

- simulation.py
- rrt_star_dubins.py
- utils/mpc_utils.py
- utils/trajectory_utils.py
- utils/specs_utils.py
- utils/plot_utils.py
- 

The center file is the simulation.py that will launch the rest of the files to run the simulation. Among these files, it uses the utils/mpc_utils.py, which contains the Model Predictive Control implementation; and utils/trajectory_utils.py which contains the functions used to obtain a path from rrt_star_dubins.py and generate a trajectory. The file utils/specs_utils.py contains all the global variable definitions while the file utils/plot_utils.py contains utility functions for plotting the results.


## Code Functionality

Assuming the static obstacles (the wall and parked cars) fixed, we use RRT* as global planner to find the path to the goal. The vehicle being non-holonomic, we used Dubins path as steering function for the path generation. Both of these implementations are in the rrt_star_dubins.py file. After generating the path that we have to follow, we used an MPC as controller. Moreover, we used the same MPC as local planner to avoid the dynamic obstacles not taken into account by RRT* (namely, the pedestrians). This funcionality ere implemented in the utils/mpc_utils.py file. Finally, a graphical simulation using Matplotlib is shown after executing simulation.py.


## Simulation

To run the simulation, you can go to the gym_PDM_fa directory and launch the simulation manually from the terminal, as shown below.

``` bash
cd gym_PDM_fa
python3 simulation.py
```

