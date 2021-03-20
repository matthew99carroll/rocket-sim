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
#include"fueltank.h"
#include"types.h"

FuelTank::FuelTank(string _name,
             float _dry_mass,
             float _prop_mass,
             float _prop_density,
             float _diameter,
             float _length,
             Vec3 _dry_com,
             Vec3 _rel_pos,
             Vec3 _rel_rot)
{
    mass = {};
    com = {};
    moi = {};

    name = _name;
    dry_mass = _dry_mass;
    prop_mass = _prop_mass;
    prop_density = _prop_density;
    prop_com = CalculateFluidCOM();
    prop_moi = CalculateFluidMOI();

    diameter = _diameter;
    length = _length;
    dry_com = _dry_com;
    rel_pos = _rel_pos;
    rel_rot = _rel_rot;

    radius = diameter / 2;

    com = CalculateCOM();
    moi = CalculateMOI();
}

void FuelTank::UpdateTank(float mass_flow_rate, float dt)
{
    prop_mass -= mass_flow_rate * dt;
    prop_com = CalculateFluidCOM();
    prop_moi = CalculateFluidMOI();
    com = CalculateCOM();
    moi = CalculateMOI();
    UpdateComponent();
}

Vec3 FuelTank::CalculateFluidMOI()
{
    Vec3 fluid_moi;

    float volume = prop_mass / prop_density;
    float height = volume / (pi * pow(radius, 2));
    
    fluid_moi.x = (1/12) * prop_mass * (3 * pow(radius, 2) + pow(height, 2));
    fluid_moi.y = (1/12) * prop_mass * (3 * pow(radius, 2) + pow(height, 2));
    fluid_moi.z = (1/2) * prop_mass * pow(radius, 2);

    return fluid_moi;
}

Vec3 FuelTank::CalculateFluidCOM()
{
    Vec3 fluid_com;

    float volume = prop_mass / prop_density;
    float height = volume / (pi * pow(radius, 2));

    fluid_com.x = 0;
    fluid_com.y = 0;
    fluid_com.z = height / 2;

    return fluid_com;
}

Vec3 FuelTank::CalculateMOI()
{
    com = Vec3Divide(Vec3Addition(Vec3Multiply(dry_mass, dry_com), Vec3Multiply(prop_mass, prop_com)), mass);
}

Vec3 FuelTank::CalculateCOM()
{
    Vec3 dry_moi;
    dry_moi.x = (1/12) * mass * (6 * pow(radius, 2) + pow(length, 2));
    dry_moi.y = (1/12) * mass * (6 * pow(radius, 2) + pow(length, 2));
    dry_moi.z = mass * pow(radius, 2);

    Vec3 prop_abs_moi;
    prop_abs_moi.x = prop_moi.x + mass * pow((com.z - prop_com.z),2);
    prop_abs_moi.y = prop_moi.y + mass * pow((com.z - prop_com.z),2);
    prop_abs_moi.z = prop_moi.z;

    return Vec3Addition(dry_moi, prop_abs_moi);
}

FuelTank::~FuelTank()
{
}