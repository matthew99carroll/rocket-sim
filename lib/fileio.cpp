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

#include "fileio.h"
#include <iostream>

FileIO::FileIO(const char* _file_path)
{
    file_path = _file_path;
}

FileIO::FileIO()
{
}

Params FileIO::ParseFile()
{
    Params p;

    pugi::xml_document doc;
   
    // load the XML file
    if (!doc.load_file(file_path))
        std::cout << "Error: Cannot load XML file!" << std::endl;

    pugi::xml_node rocket = doc.child("Rocket");

    for (pugi::xml_node_iterator it = rocket.begin(); it != rocket.end(); ++it)
    {
        pugi::xml_node child_node = doc.child(it->name());

        std::string node_name = (std::string)it->name();
    
        if(node_name == (std::string)"Engine")
        {
            for (pugi::xml_node_iterator cit = it->begin(); cit != it->end(); ++cit)
            {
                pugi::xml_attribute cit_val = cit->attribute("value");
                pugi::xml_attribute cit_name = cit->attribute("name");

                std::string child_node_name = (std::string)cit_name.value();
                
                if(child_node_name == (std::string)"type")
                    p.engine.type = cit_val.value();
                else if(child_node_name == (std::string)"isp")
                    p.engine.isp = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"thrust")
                    p.engine.thrust = std::stof((std::string)cit_val.value());
            }
        }
        else if(node_name == (std::string)"Fuel")
        {
            for (pugi::xml_node_iterator cit = it->begin(); cit != it->end(); ++cit)
            {
                pugi::xml_attribute cit_val = cit->attribute("value");
                pugi::xml_attribute cit_name = cit->attribute("name");

                std::string child_node_name = (std::string)cit_name.value();

                if(child_node_name == (std::string)"ofratio")
                    p.fuel.ofMixtureRatio = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"fuel_reserve")
                    p.fuel.fuelReserve = std::stof((std::string)cit_val.value());
            }
        }
        else if(node_name == (std::string)"Mass")
        {
            for (pugi::xml_node_iterator cit = it->begin(); cit != it->end(); ++cit)
            {
                pugi::xml_attribute cit_val = cit->attribute("value");
                pugi::xml_attribute cit_name = cit->attribute("name");

                std::string child_node_name = (std::string)cit_name.value();

                if(child_node_name == (std::string)"dry_mass")
                    p.mass.dryMass = std::stof((std::string)cit_val.value());
            }
        }
        else if(node_name == (std::string)"Aerodynamics")
        {
            for (pugi::xml_node_iterator cit = it->begin(); cit != it->end(); ++cit)
            {
                pugi::xml_attribute cit_val = cit->attribute("value");
                pugi::xml_attribute cit_name = cit->attribute("name");

                std::string child_node_name = (std::string)cit_name.value();

                if(child_node_name == (std::string)"cd")
                    p.aero.cd = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"cs_area")
                    p.aero.cs_area = std::stof((std::string)cit_val.value());
            }
        }
        else if(node_name == (std::string)"Environment")
        {
            for (pugi::xml_node_iterator cit = it->begin(); cit != it->end(); ++cit)
            {
                pugi::xml_attribute cit_val = cit->attribute("value");
                pugi::xml_attribute cit_name = cit->attribute("name");

                std::string child_node_name = (std::string)cit_name.value();
                
                if(child_node_name == (std::string)"elevation")
                    p.env.elevation = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"sim_step")
                    p.env.dt = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"standard_gravity")
                    p.env.g_0 = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"air_molar_mass")
                    p.env.air_molar_mass = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"gas_constant")
                    p.env.gas_constant = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"air_gamma")
                    p.env.air_gamma = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"atmo_pressure")
                    p.env.atmo_pressure = std::stof((std::string)cit_val.value());
            }
        }
        else if(node_name == (std::string)"Simulation")
        {
            for (pugi::xml_node_iterator cit = it->begin(); cit != it->end(); ++cit)
            {
                pugi::xml_attribute cit_val = cit->attribute("value");
                pugi::xml_attribute cit_name = cit->attribute("name");

                std::string child_node_name = (std::string)cit_name.value();

                if(child_node_name == (std::string)"log_file")
                    p.sim.logFilename = (std::string)cit_val.value();
                else if(child_node_name == (std::string)"csv_file")
                    p.sim.csvFilename = (std::string)cit_val.value();
            }
        }
    }
    return p;
}

void FileIO::WriteOutput(System& s, std::string filename)
{
    file.open(filename);

    file << "Time" << ","
             << "ASL" << ","
             << "Vel" << ","
             << "Vel_Mach" << ","
             << "Acc" << ","
             << "Mass" << ","
             << "Mass_flow_rate" << ","
             << "Prop_mass" << ","
             << "Thrust" << ","
             << "Twr" << ","
             << "Drag" << ","
             << "Rho" << ","
             << "Pressure" << ","
             << "Temp" << ","
             << "Gravity" << "," << std::endl;

    for(int i = 0; i < s.output.vec_t.size(); i++)
    {
        file << s.output.vec_t[i] << ","
             << s.output.vec_asl[i] << ","
             << s.output.vec_vel[i] << ","
             << s.output.vec_vel_mach[i] << ","
             << s.output.vec_acc[i] << ","
             << s.output.vec_mass[i] << ","
             << s.output.vec_mass_flow_rate[i] << ","
             << s.output.vec_prop_mass[i] << ","
             << s.output.vec_thrust[i] << ","
             << s.output.vec_twr[i] << ","
             << s.output.vec_drag[i] << ","
             << s.output.vec_rho[i] << ","
             << s.output.vec_pressure[i] << ","
             << s.output.vec_temp[i] << ","
             << s.output.vec_g[i] << "," << std::endl;
    }

    file.close();
}

FileIO::~FileIO()
{
}