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
    parameters = parser.ParseRocketConfig("include/rocket.xml");

    ThrustCurve motor_curve = parser.ParseThrustCurve("include/Cesaroni_O8000.xml");

    SolidMotor motor("O8000",
                     32.672f,
                     Eigen::Vector3f(0.0805f, 0.4785f, 0.0805f),
                     Eigen::Vector3f(0, 0, 0),
                     Eigen::Vector3f(100.0f, 100.0f, 100.0f),
                     Eigen::Vector3f(0, 0, 0),
                     218.0f,
                     8034.50f,
                     5.12f,
                     motor_curve,
                     Eigen::Vector3f(0, 0, 0),
                     Eigen::Vector3f(0, 0, 0),
                     std::vector<float>{-5.0f, 5.0f},
                     0,
                     80.5f,
                     0.957f,
                     18.61f);

    //s = System(parameters);

    //s.RunSimulation();

    //parser.WriteOutput(s, parameters.sim.csvFilename);

    //std::cin.get();
}