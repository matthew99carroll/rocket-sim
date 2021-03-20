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

#include<math.h>
#include "component.h"

Component::Component(string _name,
             float _mass,
             Vec3 _com,
             Vec3 _rel_pos,
             Vec3 _moi,
             Vec3 _rel_rot)
{
    name = _name;
    mass = _mass;
    com = _com;
    rel_pos = _rel_pos;
    moi = _moi;
    rel_rot = _rel_rot;
}

void Component::UpdateComponent()
{
    abs_moi = CalculateAbsMoi();
}

Vec3 Component::CalculateAbsMoi()
{
    // Parallel Axis Theorem
    abs_moi.x = moi.x + (pow(rel_pos.y, 2.0) + pow(rel_pos.z, 2.0)) * mass;
    abs_moi.y = moi.y + (pow(rel_pos.x, 2.0) + pow(rel_pos.z, 2.0)) * mass;
    abs_moi.z = moi.z + (pow(rel_pos.x, 2.0) + pow(rel_pos.y, 2.0)) * mass;

    return abs_moi;
}