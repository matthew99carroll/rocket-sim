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
#include "component.h"

using namespace std;

class Engine : public Component
{
private:
    // Specific impulse
    float isp;

    // Thrust
    float avg_thrust;
    Vec3 cot;
    vector<float> thrust_curve;

    // Burn time
    float burn_time;

    // Gimbal
    Vec3 gimbal;
    vector<float> gimbal_limits;

    void UpdateEngine(float t);
    float CalculateThrustScalar(float t);
    vector<float> CalculateThrustVector();
    float CalculateMassFlowRate();
    void GimbalEngine(vector<float> spherical_coords, float t);

public:
    Engine(/* args */);
    ~Engine();
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
    float moi;

    // Specific impulse
    float isp;

    void UpdateSolidMotor(float t, float step);
    Vec3 CalculateMOI();
public:
    SolidMotor();
    ~SolidMotor();
};

class LiquidEngine : Engine
{
private:

public:
    LiquidEngine();
    ~LiquidEngine();
};