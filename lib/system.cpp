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

#include "system.h"

System::System()
{
    
}

System::System(Params _params, float _burn_time)
{
    p = _params;

    // Engine specs
    isp = p.engine.isp;
    avg_thrust = p.engine.thrust;
    burn_time = _burn_time;

    // Fuel specs
    fuel_reserve = p.fuel.fuelReserve;

    // Flow rate
    avg_mass_flow_rate = (avg_thrust / g_0) / isp;

    // Fuel & LOX
    propellant_mass = (avg_mass_flow_rate * burn_time) / (1 - fuel_reserve / 100);

    // Mass
    dry_mass = p.mass.dryMass;

    // Aerodynamics
    cd = p.aero.cd;
    cs_area = p.aero.cs_area;

    // Environment
    elevation = p.env.elevation;
    dt = p.env.dt;
    num_steps = floorf(burn_time / dt);

    // Initalise
    t = 0.0f;
    altitude = 0.0f;
    asl = altitude + elevation;
    CalculateMass();
    UpdateEnvironment();
    CalculateThrust();
    CalculateTWR();
    drag = 0.0f;
    thrust = 0.0f;
    vel = 0.0f;
    mach = 0.0f;
    acc = 0.0f;
}

void System::RunSimulation()
{
    for (int i = 0; i < num_steps; i++)
    {
        UpdateEnvironment();
        CalculateThrust();
        CalculateAcceleration();
        CalculateAltitude();
        CalculateVelocity();
        CalculateDrag();
        CalculateTWR();
        CalculatePropellant();
        CalculateMass();

        std::cout << "ASL: " << asl << std::endl;
        std::cout << "Velocity: " << vel << std::endl;
        std::cout << "Velocity (Mach): " << mach << std::endl;
        std::cout << "Acceleration: " << acc << std::endl;
        std::cout << "Mass: " << mass << std::endl;
        std::cout << "Mass Flow Rate: " << mass_flow_rate << std::endl;
        std::cout << "Propellant Mass: " << propellant_mass << std::endl;
        std::cout << "Thrust: " << thrust << std::endl;
        std::cout << "TWR: " << twr << std::endl;
        std::cout << "Drag: " << drag << std::endl;
        std::cout << "Atmospheric Density: " << vars.density << std::endl;
        std::cout << "Atmospheric Pressure: " << vars.density << std::endl;
        std::cout << "Atmospheric Temperature: " << vars.tempFunc.temp << std::endl; 
        std::cout << "Local Gravity: " << vars.g << std::endl;

        t += dt;
    }

    avg_thrust = 0;

    while(vel >= 0)
    {
        UpdateEnvironment();
        CalculateAcceleration();
        CalculateAltitude();
        CalculateVelocity();
        CalculateDrag();
        CalculateTWR();
        CalculateMass();
        
        t += dt;
    }
}

void System::CalculateMass()
{
    mass = propellant_mass + dry_mass;
}

void System::CalculatePropellant()
{
    mass_flow_rate = (thrust / g_0) / isp;
    propellant_mass -= mass_flow_rate * dt;
}

void System::CalculateAltitude()
{
    altitude += vel * dt + (acc * pow(dt, 2)) / 2;
    asl = altitude + elevation;
}

void System::CalculateVelocity()
{
    vel += acc * dt;
    mach = vel / vars.c;
}

void System::CalculateAcceleration()
{
    acc = (thrust - (mass * vars.g + drag)) / mass;
}

void System::CalculateThrust()
{
    /*
    * UPDATE THIS FOR THRUST CURVES
    */
    thrust = p.engine.thrust;
}

void System::CalculateDrag()
{
    drag = 0.5 * (vars.density * pow(vel, 2) * cd * cs_area);
}

void System::CalculateTWR()
{
    twr = thrust / (mass * vars.g);
}

void System::UpdateEnvironment()
{
    vars = CalculateEnvironmentVariables(asl);
}

System::~System() {}