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

#include <Eigen/Dense>
using namespace Eigen;

const float pi = 3.14159265359;
const float g_0 = 9.80665;
const float air_molar_mass = 0.02896968;
const float gas_constant = 8.314462618;
const float air_gamma = 1.4;
const float air_rho_0 = 1.2252;
const float earth_radius = 6356766;

float Interpolate(float x0, float x1, float y0, float y1, float xp)
{
    return y0 + ((y1-y0)/(x1-x0)) * (xp - x0);
}

float Clamp(float x, float upper, float lower)
{
    return min(upper, max(x, lower));
}