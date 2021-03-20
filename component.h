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
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class Component
{
private:

    // Rotation
    Vector3f moi;

    // Name of component
    string name;

    Vector3f CalculateAbsMoi();
    
public:
    // Mass data
    float mass;
    Vector3f com;

    // Position
    Vector3f rel_pos;

    // Rotation
    Vector3f rel_rot;

    // MOI
    Vector3f abs_moi;

    Component(string _name,
             float _mass,
             Vector3f _com,
             Vector3f _rel_pos,
             Vector3f _moi,
             Vector3f _rel_rot);
    Component();
    
    void UpdateComponent();     
        
    ~Component();
};
