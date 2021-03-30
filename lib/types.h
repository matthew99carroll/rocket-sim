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

#include <string>
#include <vector>
#include "../include/Eigen/Dense"

// Constant parameters
const float pi = 3.14159265359f;
const float g_0 = 9.80665f;
const float air_molar_mass = 0.02896968f;
const float gas_constant = 8.314462618f;
const float air_gamma = 1.4f;
const float air_rho_0 = 1.2252f;
const float earth_radius = 6356766.0f;
const float atmo_pressure_0 = 101325.0f;

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
    float mass;
    Eigen::Vector3f com;
    Eigen::Vector3f rel_pos;
    Eigen::Vector3f moi;
    Eigen::Vector3f rel_rot;
    float isp;
    float avg_thrust;
    float burn_time;
    std::string thrust_curve;
    Eigen::Vector3f cot;
    Eigen::Vector3f gimbal;
    std::vector<float> gimbal_limits;
    float delay;
    float diameter;
    float length;
    float prop_mass;
};

struct FuelParameters
{
    float ofMixtureRatio; // Oxidizer/Fuel Mixture Ratio
    float fuelReserve;
};

struct AerodynamicsParameters
{
    float cd;
    float cs_area;
};

struct EnvironmentParameters
{
    float elevation;
    float latitude;
    float longitude;
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

// Stores environment variables
struct EnvironmentVars
{
    float g;
    TempFunction tempFunc;
    float pressure;
    float density;
    float c;
};

struct Params
{
    EngineParameters engine;
    FuelParameters fuel;
    float dryMass;
    Eigen::Vector3f com;
    Eigen::Vector3f cop;
    Eigen::Vector3f moi;
    AerodynamicsParameters aero;
    EnvironmentParameters env;
    SimulationParameters sim;
};

struct ThrustCurve
{
    std::vector<float> thrust_curve_x;
    std::vector<float> thrust_curve_y;
};

struct SimOutput
{
    std::vector<float> vec_asl;
    std::vector<float> vec_vel;
    std::vector<float> vec_vel_mach;
    std::vector<float> vec_acc;
    std::vector<float> vec_mass;
    std::vector<float> vec_mass_flow_rate;
    std::vector<float> vec_prop_mass;
    std::vector<float> vec_thrust;
    std::vector<float> vec_twr;
    std::vector<float> vec_drag;
    std::vector<float> vec_rho;
    std::vector<float> vec_pressure;
    std::vector<float> vec_temp;
    std::vector<float> vec_g;
    std::vector<float> vec_t;
};

/*
* Linear interpolation function
*/
inline float Interpolate(float x0, float x1, float y0, float y1, float xp)
{
    return y0 + ((y1 - y0) / (x1 - x0)) * (xp - x0);
}

/*
* Clamping function
*/
inline float Clamp(float x, float upper, float lower)
{
    return fmin(upper, fmax(x, lower));
}

inline float CalcMaximum(std::vector<float> vec)
{
    float max = vec[0];
    for (int i = 1; i < vec.size(); i++)
    {
        if (vec[i] > max)
        {
            max = vec[i];
        }
    }

    return max;
}

inline float CalcMinimum(std::vector<float> vec)
{
    float min = vec[0];
    for (int i = 1; i < vec.size(); i++)
    {
        if (vec[i] < min)
        {
            min = vec[i];
        }
    }

    return min;
}