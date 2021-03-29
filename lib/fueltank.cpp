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

#include"fueltank.h"

FuelTank::FuelTank(string _name,
             float _dry_mass,
             float _prop_mass,
             float _prop_density,
             float _diameter,
             float _length,
             Vector3f _dry_com,
             Vector3f _rel_pos,
             Vector3f _rel_rot)
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

Vector3f FuelTank::CalculateFluidMOI()
{
    Vector3f fluid_moi;

    float volume = prop_mass / prop_density;
    float height = volume / (pi * pow(radius, 2));
    
    fluid_moi[0] = (1/12) * prop_mass * (3 * pow(radius, 2) + pow(height, 2));
    fluid_moi[1] = (1/12) * prop_mass * (3 * pow(radius, 2) + pow(height, 2));
    fluid_moi[2] = (1/2) * prop_mass * pow(radius, 2);

    return fluid_moi;
}

Vector3f FuelTank::CalculateFluidCOM()
{
    Vector3f fluid_com;

    float volume = prop_mass / prop_density;
    float height = volume / (pi * pow(radius, 2));

    fluid_com[0] = 0;
    fluid_com[1] = 0;
    fluid_com[2] = height / 2;

    return fluid_com;
}

Vector3f FuelTank::CalculateMOI()
{
    com = dry_mass * dry_com + prop_mass * prop_com / mass;
    
    return com;
}

Vector3f FuelTank::CalculateCOM()
{
    Vector3f dry_moi;
    dry_moi[0] = (1/12) * mass * (6 * pow(radius, 2) + pow(length, 2));
    dry_moi[1] = (1/12) * mass * (6 * pow(radius, 2) + pow(length, 2));
    dry_moi[2] = mass * pow(radius, 2);

    Vector3f prop_abs_moi;
    prop_abs_moi[0] = prop_moi[0] + mass * pow((com[2] - prop_com[2]),2);
    prop_abs_moi[1] = prop_moi[1] + mass * pow((com[2] - prop_com[2]),2);
    prop_abs_moi[2] = prop_moi[2];

    return dry_moi + prop_abs_moi;
}

FuelTank::~FuelTank()
{
}