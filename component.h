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

using namespace std;

class Component
{
private:

    // Rotation
    Vec3 moi;
    Vec3 rel_rot;

    // Name of component
    string name;

    Vec3 CalculateAbsMoi();
    
public:
    // Mass data
    float mass;
    Vec3 com;

    // Position
    Vec3 rel_pos;

    // MOI
    Vec3 abs_moi;

    Component(string _name,
             float _mass,
             Vec3 _com,
             Vec3 _rel_pos,
             Vec3 _moi,
             Vec3 _rel_rot);

    void UpdateComponent();     
        
    ~Component();
};
