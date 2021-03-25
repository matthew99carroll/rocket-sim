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

#include "engine.h"

Engine::Engine(string _name,
           float _mass,
           Vector3f _com,
           Vector3f _rel_pos,
           Vector3f _moi,
           Vector3f _rel_rot,
           float _isp,
           float _avg_thrust,
           float _burn_time,
           vector<vector<float>, vector<float>> _thrust_curve,
           Vector3f _cot,
           Vector3f _gimbal,
           vector<float> _gimbal_limits)
{
    isp = _isp;
    avg_thrust = _avg_thrust;
    burn_time = _burn_time;
    thrust_curve = _thrust_curve;
    cot = _cot;
    thrust_scalar = CalculateThrustScalar(0);
    rel_thrust_vec = CalculateThrustVector();

    avg_mass_flow_rate = (avg_thrust / g_0) / isp;
    mass_flow_rate = CalculateMassFlowRate();

    gimbal = _gimbal;
    gimbal_limits = _gimbal_limits;
}

void Engine::UpdateEngine(float t)
{
    UpdateComponent();
    thrust_scalar = CalculateThrustScalar(t);
    rel_thrust_vec = CalculateThrustVector();
    mass_flow_rate = CalculateMassFlowRate();
}

float Engine::CalculateThrustScalar(float t)
{
    float thrust;

    if (t >= 0 && t <= burn_time)
    {
        vector<float> t_vec = thrust_curve[0];
        vector<float> y_vec = thrust_curve[1];

        for(int i = 1; i <= t_vec.size(); i++)
        {
            if(t_vec[i] >= t)
            {
                thrust = Interpolate(t_vec[i-1], t_vec[i], y_vec[i-1], y_vec[i], t);
            }
        }
    }
    else
    {
        thrust = 0;
    }

    return thrust;
}

Vector3f Engine::CalculateThrustVector()
{
    Vector3f theta = gimbal + rel_rot;
    Vector3f phi = gimbal + rel_rot;

    Vector3f vec(thrust_scalar * theta.sin() * phi.cos(),
                 thrust_scalar * theta.sin() * phi.sin(),
                 thrust_scalar * theta.cos());

    return vec;
}

float Engine::CalculateMassFlowRate()
{
    return (thrust_scalar / g_0) / isp;
}

void Engine::GimbalEngine(Vector3f spherical_coords, float t)
{
    gimbal[0] = Clamp(spherical_coords[0], gimbal_limits[0], gimbal_limits[1]);
    gimbal[1] = Clamp(spherical_coords[1], gimbal_limits[0], gimbal_limits[1]);
    gimbal[2] = Clamp(spherical_coords[2], gimbal_limits[0], gimbal_limits[1]);

    UpdateEngine(t);
}