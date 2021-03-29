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

#include <iostream>
#include "../lib/parameters.h"
#include "../lib/system.h"
#include "../lib/fileio.h"

float burn_time;

FileIO parser;
Params parameters;
System s;

int main()
{
    parser.file_path = "include/rocket.xml";

    parameters = parser.ParseFile();

    // ThrustCurve e1_thrust_curve;
    // e1_thrust_curve.thrust_curve_x = {0, 0.1, 0.2};
    // e1_thrust_curve.thrust_curve_y = {8000.0f, 8000.0f, 8000.0f};
    // Eigen::Vector3f e1_com(0, 0.2, 0);
    // Eigen::Vector3f e1_rel_pos(0, 0, 0);
    // Eigen::Vector3f e1_moi(1000.0f, 1000.0f, 1000.0f);
    // Eigen::Vector3f e1_rel_rot(0, 0, 0);
    // Eigen::Vector3f e1_cot(0, 0.1, 0);
    // Eigen::Vector3f e1_gimbal(0, 0, 0);
    // std::vector<float> e1_gimbal_limits = {-5.0f, 5.0f};

    // SolidMotor e1((std::string) "Engine1",
    //               20.0f,
    //               e1_com,
    //               e1_rel_pos,
    //               e1_moi,
    //               e1_rel_rot,
    //               218.0f,
    //               300.0f,
    //               5.12f,
    //               e1_thrust_curve, // Not considered atm
    //               e1_cot,
    //               e1_gimbal,
    //               e1_gimbal_limits,
    //               0.0f,
    //               0.1f,
    //               2.0f,
    //               30.0f);

    // std::vector<Engine> e = {e1};
    // Eigen::Vector3f rocket_com(0.0, 2.0f, 0.0);
    // Eigen::Vector3f rocket_cop(0.0, 1.0f, 0.0);
    // Eigen::Vector3f rocket_moi(20.0f, 200.0f, 20.0f);
    // Eigen::Vector3f pos(0, 0, 0);
    // Eigen::Vector3f rot(0, 0, 0);

    // Rocket rocket(e,
    //        rocket_com,
    //        rocket_cop,
    //        rocket_moi,
    //        100.0f,
    //        0.025f,
    //        0.01f,
    //        pos,
    //        rot);

    burn_time = 5.12f; // Aerodynamics

    s = System(parameters, burn_time);

    s.RunSimulation();

    parser.WriteOutput(s, parameters.sim.csvFilename);

    //std::cin.get();
}