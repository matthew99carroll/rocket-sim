# Rocket Simulator

## Overview

This project takes an XML configuration file containing the specifications of a launch vehicle and will attempt to simulate the vehicles trajectory.
The intent of the project is to accuratetly simulate the full state of the vehicle by correctly simulating the forces and moments acting on the vehicle in 6 degrees of freedom.
If accurate simulations of a passive launch vehicle can be achieved, then it is likely that I will add active control in the form of thrust-vectoring, canards, moveable fins and other potential systems.

## Features

- Design and simulate rocket via an XML configuration file
- Component system to build up an accurate set of point masses
- Simulation of vehicles with solid motors based on thrust curves
- Accurate atmospheric modelling of density, pressure, temperature, gravity and local speed of sound up to 1000 kilometers
- Dynamic center of mass based on fuel consumption
- C++ wrapper of Pythons MatPlotLib plotting library
- Export data csv file format

## To do
- Implement basic liquid fuel systems by modelling fuel tank and liquid fuel engines
- Move to 6DOF modelling of forces and moments
- Engine thrust vectoring
- Active control law
- Stability analysis
