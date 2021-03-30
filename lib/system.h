/*
 * RocketSim, a 6DOF simulation platform for launch vehicles.
 * 
 * @Author: Matthew Carroll
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * 
 * Reach out to the author at the following email address:
 * matthew99carroll@gmail.com
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <iostream>
#include "types.h"
#include "environment.h"
#include "rocket.h"
#include "../include/matplotlibcpp.h"
#include "../include/Eigen/Dense"

namespace plt = matplotlibcpp;

class System
{
private:
    Params p;
    EnvironmentVars vars;

    //Rocket rocket;

    float isp;
    float avg_thrust;
    float thrust;
    float burn_time;
    std::vector<float> thrust_curve_x;
    std::vector<float> thrust_curve_y;

    float fuel_reserve;

    float avg_mass_flow_rate;
    float mass_flow_rate;

    float propellant_mass;

    float dry_mass;

    float cd;
    float cs_area;

    float elevation;
    float dt;
    float num_steps;

    float t;
    float altitude;
    float asl;
    float mass;

    float drag;
    float twr;
    float vel;
    float mach;
    float acc;

    void CalculateMass();
    void CalculatePropellant();
    void CalculateAltitude();
    void CalculateVelocity();
    void CalculateAcceleration();
    void CalculateThrust();
    void CalculateDrag();
    void CalculateTWR();

public:
    SimOutput output;

    System();

    System(Params _params);

    void RunSimulation();

    void UpdateEnvironment();

    ~System();
};

#endif