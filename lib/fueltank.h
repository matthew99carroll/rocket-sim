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
#include<math.h>
#include "types.h"
#include "component.h"
#include "../include/Eigen/Dense"

class FuelTank : Component
{
private:
    // Fuel tank name
    std::string name;

    // Masses
    float mass;
    Eigen::Vector3f com;
    Eigen::Vector3f moi;
    float dry_mass;
    float prop_mass;
    float prop_density;
    Eigen::Vector3f prop_com;
    Eigen::Vector3f prop_moi;
    Eigen::Vector3f dry_com;

    // Dimensions
    float diameter;
    float radius;
    float length;

    // Position
    Eigen::Vector3f rel_pos;

    // Rotation
    Eigen::Vector3f rel_rot;

    Eigen::Vector3f CalculateCOM();
    Eigen::Vector3f CalculateMOI();

    Eigen::Vector3f CalculateFluidMOI();
    Eigen::Vector3f CalculateFluidCOM();

    void UpdateTank(float mass_flow_rate, float dt);

public:
    FuelTank(std::string _name,
             float _dry_mass,
             float _prop_mass,
             float _prop_density,
             float _diameter,
             float _length,
             Eigen::Vector3f _dry_com,
             Eigen::Vector3f _rel_pos,
             Eigen::Vector3f _rel_rot);
    ~FuelTank();
};
