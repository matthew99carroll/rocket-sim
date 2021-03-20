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

#include "rocket.h"
#include <math.h>

Rocket::Rocket(vector<Engine> _engines,
               Vector3f _com,
               Vector3f _cop,
               Vector3f _moi,
               float _dry_mass,
               float _cs_area,
               float _cd,
               Vector3f _pos,
               Vector3f _rot)
{
    engines = _engines;
    dry_com = _com;
    cop = _cop;
    com = CalculateCOM();

    dry_mass = _dry_mass;
    mass = CalculateMass();
    pos = _pos;
    vel = { };
    acc = { };

    dry_moi = _moi;
    moi = CalculateMOI();
    rot = _rot;
    ang_vel = { };
    ang_acc = { };

    cs_area = _cs_area;
    cd = _cd;

    weight = CalculateWeight(g_0);
    total_thrust = CalculateTotalThrust();
    drag = CalculateTotalDrag(air_rho_0);
    total_force = CalculateTotalForce();

    total_torque = CalculateTotalTorque();
}

Vector3f Rocket::CalculateCOM()
{
    float engine_masses = 0;
    Vector3f engine_weighted_position = {};

    for(int i=0; i < engines.size(); i++)
    {
        engine_masses += engines[i].mass;
        engine_weighted_position = engines[i].mass * engines[i].rel_pos + engines[i].com;
    }

    com = dry_mass * dry_com + engine_weighted_position / (dry_mass + engine_masses);

    return com;
}

float Rocket::CalculateMass()
{
    float total_mass = dry_mass;

    for(int i = 0; i < engines.size(); i++)
    {
        total_mass += engines[i].mass;
    }

    return total_mass;
}

Vector3f Rocket::CalculatePosition(float dt)
{
    Vector3f new_pos = pos + dt * vel + pow(dt,2) * acc / 2;

    return new_pos;
}

Vector3f Rocket::CalculateVelocity(float dt)
{
    Vector3f new_vel = vel + dt * acc;

    return new_vel;
}

Vector3f Rocket::CalculateAcceleration()
{
    Vector3f new_acc = total_force / mass;
}

Vector3f Rocket::CalculateMOI()
{
    moi = dry_moi;
    for(int i = 0; i < engines.size(); i++)
    {
        moi += engines[i].abs_moi;
    }

    return moi;
}

Vector3f Rocket::CalculateAngularPosition(float dt)
{
    Vector3f new_rot = rot + dt * ang_vel + pow(dt,2) * ang_acc / 2;

    return new_rot;
}

Vector3f Rocket::CalculateAngularVelocity(float dt)
{
    Vector3f new_ang_vel = ang_vel + dt * ang_acc;

    return new_ang_vel;
}

Vector3f Rocket::CalculateAngularAcceleration()
{
    Vector3f new_ang_acc;

    new_ang_acc[0] = total_force[0] / moi[0];
    new_ang_acc[1] = total_force[1] / moi[1];
    new_ang_acc[2] = total_force[2] / moi[2];

    return new_ang_acc;
}

Vector3f Rocket::CalculateWeight(float g)
{
    Vector3f weight(0,0, mass * -g);

    return weight;
}

Vector3f Rocket::CalculateTotalThrust()
{
    Vector3f total_rel_vec = {};

    for(int i = 0; i < engines.size(); i++)
    {
        total_rel_vec = total_rel_vec + engines[i].rel_thrust_vec;
    }

    // Set initial values to initial rotation i.e. align with rocket
    Vector3f gamma = rot;
    Vector3f beta = rot;
    Vector3f alpha = rot;

    MatrixXf matrix(3,3);

    matrix(0,0) = alpha.cos() * beta.cos();
    matrix(0,1) = alpha.cos() * beta.sin() * gamma.sin() - alpha.sin() * gamma.cos();
    matrix(0,2) = alpha.cos() * beta.sin() * gamma.cos() + alpha.sin() * gamma.sin();

    matrix(1,0) = alpha.sin() * beta.cos();
    matrix(1,1) = alpha.sin() * beta.sin() * gamma.sin() + alpha.cos() * gamma.cos();
    matrix(1,2) = alpha.sin() * beta.sin() * gamma.cos() - alpha.cos() * gamma.sin();

    matrix(2,0) = -beta.sin();
    matrix(2,1) = beta.cos() * gamma.sin();
    matrix(2,2) = beta.cos() * gamma.cos();
    
    // Apply rocket rotation to thrust vector
    Vector3f total_thrust = matrix * total_rel_vec;

    return total_thrust;
}

Vector3f Rocket::CalculateTotalDrag(float rho)
{
    Vector3f drag = 0.5 * rho * vel * vel * cd * cs_area;

    return drag;
}

Vector3f Rocket::CalculateTotalForce()
{
    Vector3f total_ext = weight + total_thrust + drag;

    return total_ext;
}

Vector3f Rocket::CalculateTotalTorque()
{
    total_torque.Zero();

    Vector3f position_vec;

    for(int i = 0; i < engines.size(); i++)
    {
        position_vec = com - (engines[i].rel_pos + engines[i].cot);
        total_torque += position_vec.cross(engines[i].rel_thrust_vec);
    }

    position_vec = com - cop;
    total_torque += position_vec.cross(drag);

    return total_torque;
}