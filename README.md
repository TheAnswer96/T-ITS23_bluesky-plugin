# T-ITS23_bluesky-plugin

The repository contais the code written for the scientific paper titled "How the Wind Can Be Leveraged for Saving Energy in a Truck-Drone Delivery System", F. B. Sorbelli, F. Corò, L. Palazzetti, C. M. Pinotti and G. Rigoni, in IEEE Transactions on Intelligent Transportation Systems, doi: 10.1109/TITS.2023.3234627.

Abstract: In this work, we investigate the impact of the wind in a drone-based delivery system. For the first time, to the best of our knowledge, we adapt the trajectory of the drone to the wind. We consider a truck-drone tandem delivery system. The drone actively reacts to the wind adopting the “most tailwind” trajectory available between the truck’s path and the delivery. The truck moves on a predefined route and carries the drone close to the delivery point. We propose the Minimum-energy Drone-trajectory Problem (MDP) which aims, when the wind affects the delivery area, at planning minimum-energy trajectories for the drone to serve the customers starting from and returning to the truck. We then propose two algorithms that optimally solve under two different routes of the truck. We also analytically study the feasibility of sending drones with limited battery to deliver packages. Finally, we first numerically compare our algorithms on randomly generated synthetic and real data, and then we evaluate our model simulating the drone’s flight in the simulator.

URL: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10014537&isnumber=4358928

CITE US TO SUPPORT OUR WORK!

@ARTICLE{10014537,
  author={Sorbelli, Francesco Betti and Corò, Federico and Palazzetti, Lorenzo and Pinotti, Cristina M. and Rigoni, Giulio},
  journal={IEEE Transactions on Intelligent Transportation Systems}, 
  title={How the Wind Can Be Leveraged for Saving Energy in a Truck-Drone Delivery System}, 
  year={2023},
  volume={},
  number={},
  pages={1-12},
  keywords={},
  doi={10.1109/TITS.2023.3234627},
  ISSN={1558-0016},
  month={},}
  
BRIEF OVERVIEW OF THE REPO:
 
For evaluating the algorithms provided in our paper on a simulated environment, we rely on BlueSky (https://github.com/TUDelft-CNS-ATM/bluesky).
BlueSky is an open Air Traffic Simulator (ATS), and is meant as a tool to perform research on Air Traffic Management and Air Traffic Flows.
At the best of our knowledge, BlueSky is the only simulator in which it is possible to simulate the flight of different aircraft by injecting the presence of the wind.
Although BlueSky can model aircraft under the effect of the wind, there is not any available option that computes the actual energy consumption of them.
So, we created an ad hoc plug-in for BlueSky (still at a very preliminary stage though) that can estimate the energy consumed when a drone flies.
The idea behind the plug-in is the following: when a flight simulation is done, BlueSky creates a log file (in CSV format) in which each line comprises different fields, like GPS coordinates, drone's speed, global wind, and so on.
In order to estimate a semi-realistic energy consumption, we parse the log file and for each two consecutive lines, we extract the two GPS coordinates, so that we can compute the actual flown distance and the drone's heading.
Accordingly, knowing the wind experienced by the drone, we can then \textit{precisely} estimate the energy consumption of it by using our energy consumption model derived from the paper "Energy use and life cycle greenhouse gas emissions of drones for commercial package delivery" (https://www.nature.com/articles/s41467-017-02411-5).
In particular, we propose the idea of wind classes to simplify the estimation of energy consumption (by default the code uses wind classes).
It should be noted that the code may also be executed without the idea of wind classes; this makes the computation as exact as feasible at the price of severe performance deterioration.
As detailed in our work, the use of the developed plug-in allows us to validate the theoretical consumption of the proposed model, with only minor differences due to more realistic UAV trajectories, namely the path of the drone in the simulator is not exactly the euclidian trajectory between the take-off and the rendevouz point.

OVERVIEW OF THE SCRIPTS:

- model.py: module which contains functions for creating the compass rose, wind classes, and energy consumptions according to the paper's system model section.

- line.py: module which contains functions for creating and manipulating mathematical objects such as lines, points, and segments. It includes a function for generating a line with a given angle and direction, a function for determining which side of a segment a point is on, and a function for generating a random segment on a line. It also includes a helper function for calculating intermediate points between two given points.

- energy.py: the code described in this module defines several functions to compute the energy consumed by a drone during a flight, given its payload weight, distance, speed, and the wind conditions. The get_energy function takes the relevant parameters and computes the energy consumed, while the compute_prefixes and compute_prefixes_extended functions compute the energy consumed for different combinations of wind direction and payload weight. The code makes use of several physical and mathematical formulas to compute the drag force, thrust, power, and energy consumption of the drone, as well as to solve for the induced speed required for hovering. The code makes use of the numpy, sympy, and mpmath libraries to perform numerical calculations and solve equations.

- simulator.py: module which contains the functions used for creating and managing the flight instances, converting them into .bat files, and running them into the BlueSky simulator. What's more there are functions that allow to record the energy into csv.


FOR ANY TYPE OF ISSUE OR QUESTIONS DO NOT HESITATE TO CONTACT THE AUTHORS OF THE PAPER!!

A little pun for drone fans: Why did the drone refuse to fly in high winds? It was too air-DRONE-matic for its liking!
