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
    avg_thrust = p.engine.avg_thrust;
    burn_time = _burn_time;

    // Fuel specs
    fuel_reserve = p.fuel.fuelReserve;

    // Flow rate
    avg_mass_flow_rate = (avg_thrust / g_0) / isp;

    // Fuel & LOX
    propellant_mass = (avg_mass_flow_rate * burn_time) / (1 - fuel_reserve / 100);

    // Mass
    dry_mass = p.dryMass;

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

        output.vec_asl.push_back(asl);
        output.vec_vel.push_back(vel);
        output.vec_vel_mach.push_back(mach);
        output.vec_acc.push_back(acc);
        output.vec_mass.push_back(mass);
        output.vec_mass_flow_rate.push_back(mass_flow_rate);
        output.vec_prop_mass.push_back(propellant_mass);
        output.vec_thrust.push_back(thrust);
        output.vec_twr.push_back(twr);
        output.vec_drag.push_back(drag);
        output.vec_rho.push_back(vars.density);
        output.vec_pressure.push_back(vars.pressure);
        output.vec_temp.push_back(vars.tempFunc.temp);
        output.vec_g.push_back(vars.g);
        output.vec_t.push_back(t);

        t += dt;
    }

    avg_thrust = 0;
    thrust = 0;

    while(asl >= elevation)
    {
        UpdateEnvironment();
        CalculateAcceleration();
        CalculateAltitude();
        CalculateVelocity();
        CalculateDrag();
        CalculateTWR();
        CalculateMass();

        output.vec_asl.push_back(asl);
        output.vec_vel.push_back(vel);
        output.vec_vel_mach.push_back(mach);
        output.vec_acc.push_back(acc);
        output.vec_mass.push_back(mass);
        output.vec_mass_flow_rate.push_back(mass_flow_rate);
        output.vec_prop_mass.push_back(propellant_mass);
        output.vec_thrust.push_back(thrust);
        output.vec_twr.push_back(twr);
        output.vec_drag.push_back(drag);
        output.vec_rho.push_back(vars.density);
        output.vec_pressure.push_back(vars.pressure);
        output.vec_temp.push_back(vars.tempFunc.temp);
        output.vec_g.push_back(vars.g);
        output.vec_t.push_back(t);
        
        t += dt;
    }

    std::cout << "Apogee: " << CalcMaximum(output.vec_asl) << std::endl;
    std::cout << "Peak Velocity: " << CalcMaximum(output.vec_vel) << std::endl;
    std::cout << "Peak Velocity (Mach): " << CalcMaximum(output.vec_vel_mach) << std::endl;
    std::cout << "Peak Acceleration: " << CalcMaximum(output.vec_acc) << std::endl;
    std::cout << "Elapsed Time: " << CalcMaximum(output.vec_t) << std::endl;
    
    plt::figure(1);
    plt::xlabel("t (s)");
    plt::ylabel("Alt (m)");
    plt::plot(output.vec_t, output.vec_asl);
    plt::figure(2);
    plt::xlabel("t (s)");
    plt::ylabel("Vel (m/s)");
    plt::plot(output.vec_t, output.vec_vel,"r");
    plt::figure(3);
    plt::xlabel("t (s)");
    plt::ylabel("Vel (Mach)");
    plt::plot(output.vec_t, output.vec_acc, "b");
    plt::figure(4);
    plt::xlabel("t (s)");
    plt::ylabel("Mass (kg)");
    plt::plot(output.vec_t, output.vec_mass,"g");
    plt::figure(5);
    plt::xlabel("t (s)");
    plt::ylabel("Drag (N)");
    plt::plot(output.vec_t, output.vec_drag,"o");
    plt::figure(6);
    plt::xlabel("t (s)");
    plt::ylabel("Atmospheric Density (kg/m^3)");
    plt::plot(output.vec_t, output.vec_rho, "p");

    plt::show();

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
    mach = vel / 343.0f;
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
    thrust = p.engine.avg_thrust;
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

System::~System() 
{

}