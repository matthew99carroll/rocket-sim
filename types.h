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

#pragma once

#include<string>
#include<vector>
#include "Eigen/Dense"

using namespace Eigen;

// Constant parameters
const float pi = 3.14159265359f;
const float g_0 = 9.80665f;
const float air_molar_mass = 0.02896968f;
const float gas_constant = 8.314462618f;
const float air_gamma = 1.4f;
const float air_rho_0 = 1.2252f;
const float earth_radius = 6356766.0f;

// Layer base altitudes
inline std::vector<float> hb = {0.0f, 11000.0f, 20000.0f, 32000.0f, 47000.0f, 51000.0f, 71000.0f};

// Layer base pressures
inline std::vector<float> pb = {101325.0f, 22632.1f, 5474.89f, 868.019f, 110.906f, 66.9389f, 3.95642f};

// Layer base temperatures
inline std::vector<float> tb = {288.15f, 216.85f, 216.85f, 228.65f, 270.65f, 270.65f, 214.65f};

// Layer lapse rates
inline std::vector<float> lm = {-0.0065f, 0.0f, 0.001f, 0.0028f, 0.0f, -0.0028f, -0.002f};

struct TempFunction
{
    float temp;
    float b;
};

struct EngineParameters
{
    std::string type;
    float isp;
    float thrust;
};

struct FuelParameters
{
    float ofMixtureRatio; // Oxidizer/Fuel Mixture Ratio
    float fuelReserve; 
};

struct MassParameters
{
    float dryMass;
};

struct AerodynamicsParameters
{
    float cd;
    float cs_area;
};

struct EnvironmentParameters
{
    float elevation;
    float dt;
    float g_0;
    float air_molar_mass;
    float gas_constant;
    float air_gamma;
    float atmo_pressure;
};

struct SimulationParameters
{
    std::string logFilename;
    std::string csvFilename;
};

struct Params
{
    EngineParameters engine;
    FuelParameters fuel;
    MassParameters mass;
    AerodynamicsParameters aero;
    EnvironmentParameters env;
    SimulationParameters sim;
};


// Stores environment variables
struct EnvironmentVars
{
    float g;
    TempFunction tempFunc;
    float pressure;
    float density;
    float c;
};

/*
* Linear interpolation function
*/
inline float Interpolate(float x0, float x1, float y0, float y1, float xp)
{
    return y0 + ((y1-y0)/(x1-x0)) * (xp - x0);
}


/*
* Clamping function
*/
inline float Clamp(float x, float upper, float lower)
{
    return fmin(upper, fmax(x, lower));
}