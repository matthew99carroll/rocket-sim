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

#include "types.h"
#include "engine.h"
#include "../include/Eigen/Dense"

class Rocket
{
private:
    // Number of engines
    std::vector<Engine> engines;

    // Center of masses
    Eigen::Vector3f dry_com;
    Eigen::Vector3f com;


    // Center of pressure
    Eigen::Vector3f cop;

    // Masses
    float mass;
    float dry_mass;

    // Moment of inertias
    Eigen::Vector3f dry_moi;
    Eigen::Vector3f moi;

    // Linear Motion
    Eigen::Vector3f pos;
    Eigen::Vector3f vel;
    Eigen::Vector3f acc;

    // Rotational Motion
    Eigen::Vector3f rot;
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f ang_acc;

    // Aerodynamics
    float cs_area;
    float cd;

    // Forces
    Eigen::Vector3f weight;
    Eigen::Vector3f total_thrust;
    Eigen::Vector3f drag;
    Eigen::Vector3f total_force;

    // Torques
    Eigen::Vector3f total_torque;

    float CalculateMass();

    Eigen::Vector3f CalculateCOM();
    Eigen::Vector3f CalculatePosition(float dt);
    Eigen::Vector3f CalculateVelocity(float dt);
    Eigen::Vector3f CalculateAcceleration();
    Eigen::Vector3f CalculateMOI();
    Eigen::Vector3f CalculateAngularPosition(float dt);
    Eigen::Vector3f CalculateAngularVelocity(float dt);
    Eigen::Vector3f CalculateAngularAcceleration();
    Eigen::Vector3f CalculateWeight(float g);
    Eigen::Vector3f CalculateTotalThrust();
    Eigen::Vector3f CalculateTotalDrag(float rho);
    Eigen::Vector3f CalculateTotalForce();
    Eigen::Vector3f CalculateTotalTorque();

public:
    Rocket(std::vector<Engine> _engines,
           Eigen::Vector3f _com,
           Eigen::Vector3f _cop,
           Eigen::Vector3f _moi,
           float _dry_mass,
           float _cs_area,
           float _cd,
           Eigen::Vector3f _pos,
           Eigen::Vector3f _rot);

    Rocket();
    
    ~Rocket();
};
