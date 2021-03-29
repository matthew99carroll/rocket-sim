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
#include "../include/Eigen/Dense"

class Component
{
private:

    // Rotation
    Eigen::Vector3f moi;

    // Name of component
    std::string name;

    Eigen::Vector3f CalculateAbsMoi();
    
public:
    // Mass data
    float mass;
    Eigen::Vector3f com;

    // Position
    Eigen::Vector3f rel_pos;

    // Rotation
    Eigen::Vector3f rel_rot;

    // MOI
    Eigen::Vector3f abs_moi;

    Component();

    Component(std::string _name,
            float _mass,
            Eigen::Vector3f _com,
            Eigen::Vector3f _rel_pos,
            Eigen::Vector3f _moi,
            Eigen::Vector3f _rel_rot);
    
    void UpdateComponent();     
        
    ~Component();
};
