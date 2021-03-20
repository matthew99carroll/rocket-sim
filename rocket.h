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
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class Rocket
{
private:
    // Number of engines
    vector<Engine> engines;

    // Center of masses
    Vector3f dry_com;
    Vector3f com;


    // Center of pressure
    Vector3f cop;

    // Masses
    float mass;
    float dry_mass;

    // Moment of inertias
    Vector3f dry_moi;
    Vector3f moi;

    // Linear Motion
    Vector3f pos;
    Vector3f vel;
    Vector3f acc;

    // Rotational Motion
    Vector3f rot;
    Vector3f ang_vel;
    Vector3f ang_acc;

    // Aerodynamics
    float cs_area;
    float cd;

    // Forces
    Vector3f weight;
    Vector3f total_thrust;
    Vector3f drag;
    Vector3f total_force;

    // Torques
    Vector3f total_torque;

    float CalculateMass();

    Vector3f CalculateCOM();
    Vector3f CalculatePosition(float dt);
    Vector3f CalculateVelocity(float dt);
    Vector3f CalculateAcceleration();
    Vector3f CalculateMOI();
    Vector3f CalculateAngularPosition(float dt);
    Vector3f CalculateAngularVelocity(float dt);
    Vector3f CalculateAngularAcceleration();
    Vector3f CalculateWeight(float g);
    Vector3f CalculateTotalThrust();
    Vector3f CalculateTotalDrag(float rho);
    Vector3f CalculateTotalForce();
    Vector3f CalculateTotalTorque();

public:
    Rocket(vector<Engine> _engines,
           Vector3f _com,
           Vector3f _cop,
           Vector3f _moi,
           float _dry_mass,
           float _cs_area,
           float _cd,
           Vector3f _pos,
           Vector3f _rot);
    ~Rocket();
};
