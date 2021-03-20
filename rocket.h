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

#include<vector>
#include<string>
#include "types.h"
#include "engine.h"

using namespace std;

class Rocket
{
private:
    // Number of engines
    vector<Engine> engines;

    // Center of masses
    Vec3 dry_com;
    Vec3 com;


    // Center of pressure
    Vec3 cop;

    // Masses
    float mass;
    float dry_mass;

    // Moment of inertias
    Vec3 dry_moi;
    Vec3 moi;

    // Linear Motion
    Vec3 pos;
    Vec3 vel;
    Vec3 acc;

    // Rotational Motion
    Vec3 rot;
    Vec3 ang_vel;
    Vec3 ang_acc;

    // Aerodynamics
    float cs_area;
    float cd;

    // Forces
    Vec3 weight;
    Vec3 total_thrust;
    Vec3 drag;
    Vec3 total_force;

    // Torques
    Vec3 total_torque;

    float CalculateMass();

    Vec3 CalculateCOM();
    Vec3 CalculatePosition(float dt);
    Vec3 CalculateVelocity(float dt);
    Vec3 CalculateAcceleration();
    Vec3 CalculateMOI();
    Vec3 CalculateAngularPosition(float dt);
    Vec3 CalculateAngularVelocity(float dt);
    Vec3 CalculateAngularAcceleration();
    Vec3 CalculateWeight(float g);
    Vec3 CalculateTotalThrust();
    Vec3 CalculateTotalDrag(float rho);
    Vec3 CalculateTotalForce();
    Vec3 CalculateTotalTorque();

public:
    Rocket(vector<Engine> _engines,
           Vec3 _com,
           Vec3 _cop,
           Vec3 _moi,
           float _dry_mass,
           float _cs_area,
           float _cd,
           Vec3 _pos,
           Vec3 _rot);
    ~Rocket();
};
