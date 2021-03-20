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
#include <Eigen/Dense>

Rocket::Rocket(vector<Engine> _engines,
               Vec3 _com,
               Vec3 _cop,
               Vec3 _moi,
               float _dry_mass,
               float _cs_area,
               float _cd,
               Vec3 _pos,
               Vec3 _rot)
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

Vec3 Rocket::CalculateCOM()
{
    float engine_masses = 0;
    Vec3 engine_weighted_position = {};

    for(int i=0; i < engines.size(); i++)
    {
        engine_masses += engines[i].mass;
        engine_weighted_position = Vec3Multiply(engines[i].mass, (Vec3Addition(engines[i].rel_pos, engines[i].com)));
    }

    com = Vec3Divide(Vec3Addition(Vec3Multiply(dry_mass, dry_com), engine_weighted_position), dry_mass + engine_masses);

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

Vec3 Rocket::CalculatePosition(float dt)
{
    Vec3 new_pos = Vec3Addition(Vec3Addition(pos, Vec3Multiply(dt, vel)), Vec3Divide(Vec3Multiply(pow(dt,2), acc), 2));

    return new_pos;
}

Vec3 Rocket::CalculateVelocity(float dt)
{
    Vec3 new_vel = Vec3Addition(vel, Vec3Multiply(dt, acc));

    return new_vel;
}

Vec3 Rocket::CalculateAcceleration()
{
    Vec3 new_acc = Vec3Divide(total_force, mass);
}

Vec3 Rocket::CalculateMOI()
{
    moi = dry_moi;
    for(int i = 0; i < engines.size(); i++)
    {
        moi = Vec3Addition(moi, engines[i].abs_moi);
    }

    return moi;
}

Vec3 Rocket::CalculateAngularPosition(float dt)
{
    Vec3 new_rot = Vec3Addition(Vec3Addition(rot, Vec3Multiply(dt, ang_vel)), Vec3Divide(Vec3Multiply(pow(dt,2), ang_acc), 2));

    return new_rot;
}

Vec3 Rocket::CalculateAngularVelocity(float dt)
{
    Vec3 new_ang_vel = Vec3Addition(ang_vel, Vec3Multiply(dt, ang_acc));

    return new_ang_vel;
}

Vec3 Rocket::CalculateAngularAcceleration()
{
    Vec3 new_ang_acc = Vec3Divide(total_torque, moi);

    return new_ang_acc;
}

Vec3 Rocket::CalculateWeight(float g)
{
    Vec3 weight;
    weight.x = 0;
    weight.y = 0;
    weight.z = mass * -g;

    return weight;
}

Vec3 Rocket::CalculateTotalThrust()
{
    Vec3 total_rel_vec = {};

    for(int i = 0; i < engines.size(); i++)
    {
        total_rel_vec = Vec3Addition(total_rel_vec, engines[i].rel_thrust_vec);
    }

    // Set initial values to initial rotation i.e. align with rocket
    Vec3 gamma = rot;
    Vec3 beta = rot;
    Vec3 alpha = rot;


}

Vec3 Rocket::CalculateTotalDrag(float rho)
{

}

Vec3 Rocket::CalculateTotalForce()
{

}

Vec3 Rocket::CalculateTotalTorque()
{

}

Rocket::~Rocket()
{
}