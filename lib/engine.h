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
#include "component.h"
#include "../include/Eigen/Dense"

using namespace Eigen;
using namespace std;

class Engine : public Component
{
private:
    // Specific impulse
    float isp;

    // Thrust
    ThrustCurve thrust_curve;
    float thrust_scalar;

    // Gimbal
    Vector3f gimbal;
    vector<float> gimbal_limits;

    float avg_mass_flow_rate;
    
    float CalculateThrustScalar(float t);
    Vector3f CalculateThrustVector();
    float CalculateMassFlowRate();
    void GimbalEngine(Vector3f spherical_coords, float t);

public:
    // Burn time
    float burn_time;

    // Thrust
    float avg_thrust;

    Vector3f cot;
    Vector3f rel_thrust_vec;

    float mass_flow_rate;

    Engine(string _name,
           float _mass,
           Vector3f _com,
           Vector3f _rel_pos,
           Vector3f _moi,
           Vector3f _rel_rot,
           float _isp,
           float _avg_thrust,
           float _burn_time,
           ThrustCurve _thrust_curve,
           Vector3f _cot,
           Vector3f _gimbal,
           vector<float> _gimbal_limits);
    Engine();
    ~Engine();

    void UpdateEngine(float t);
};

class SolidMotor : Engine
{
private:
    // Delay
    float delay;

    // Dimensions
    float radius;
    float length;

    // Masses
    float prop_mass;
    float initial_prop_mass;
    float prop_volume;
    float prop_density;


    // Specific impulse
    float isp;

    void UpdateSolidMotor(float t, float step);
    Vector3f CalculateMOI();

public:

    Vector3f moi;

    SolidMotor(float _delay,
               float _diameter,
               float _length,
               float _prop_mass);
    ~SolidMotor();
};

class LiquidEngine : Engine
{
private:

public:
    LiquidEngine();
    ~LiquidEngine();
};