# Traffic Flow

A collection of microscopic traffic flow simulations built using Python. We can visualise live vehicle behaviour and traffic throughput across various road networks.

## Project overview

This project models the kinematic movements of individual movements to understand and observe how macroscopic phenomena, such as shock wave propagation, materialise. We simulate reaction times, distance headways, lane changing and human error to demonstrate how traffic jams form.

Each simulation inclues a live visual representation of the road network alongside real-time data displays or graphs tracking several variables.

## Included simulations

### 1. Motorways
* **'three lane motorway.py' & 'two lane motorway.py'**
  Simulates multi-lane traffic. Vehicles have desired speeds set using a normal distribution. They actively look for gaps to overtake slower drivers, or to merge left when the road is clear. A sinusoidal inflow is used to test the network under varying traffic densities.
* **'motorway lane closure.py'**
  Models a three-lane motorway where the 'fast' lane is closed ahead. This is a severe bottleneck scenario where vehicles in the inner lanes are forced to yield to merging vehicles in the blocked lane.

### 2. Urban intersections
* **'stockton road junction.py' & 'modified stockton road junction.py**
  A four-way box junction simulation modelling the A177 Stockton Road junction in Durham. It features a cycle through a set of phases for the traffic lights, and multi-lane routing. Vehicles yield to oncoming traffic with right of way when turning across lanes. The simulation tracks overall throughput and average journey times, allowing us to compare the two road layouts.

### 3. Ring road dynamics
* **'closed loop track.py'**
  Simulates a single-lane ring road (visualised as a straight track with looping) where the number of vehicles gradually increases so it transitions from a state of free flow to congestion. (Influenced by an experiment conducted: https://iopscience.iop.org/article/10.1088/1367-2630/10/3/033001.)

## System Requirements
This has been tested with Python 3.12 and requires the following Python packages:

* matplotlib, numpy
