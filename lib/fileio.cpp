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

FileIO::FileIO() { }

Params FileIO::ParseRocketConfig(const char *file_path)
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
                else if(child_node_name == (std::string)"mass")
                    p.engine.mass = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"com_x")
                    p.engine.com.x() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"com_y")
                    p.engine.com.y() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"com_z")
                    p.engine.com.z() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"rel_pos_x")
                    p.engine.rel_pos.x() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"rel_pos_y")
                    p.engine.rel_pos.y() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"rel_pos_z")
                    p.engine.rel_pos.z() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"moi_x")
                    p.engine.moi.x() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"moi_y")
                    p.engine.moi.y() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"moi_z")
                    p.engine.moi.z() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"rel_rot_x")
                    p.engine.rel_rot.x() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"rel_rot_y")
                    p.engine.rel_rot.y() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"rel_rot_z")
                    p.engine.rel_rot.z() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"isp")
                    p.engine.isp = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"avg_thrust")
                    p.engine.avg_thrust = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"burn_time")
                    p.engine.burn_time = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"thrust_curve")
                    p.engine.type = cit_val.value();
                else if(child_node_name == (std::string)"cot_x")
                    p.engine.cot.x() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"cot_y")
                    p.engine.cot.y() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"cot_z")
                    p.engine.cot.z() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"gimbal_x")
                    p.engine.gimbal.x() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"gimbal_y")
                    p.engine.gimbal.y() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"gimbal_z")
                    p.engine.gimbal.z() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"gimbal_lim_max")
                    p.engine.gimbal_limits.push_back(std::stof((std::string)cit_val.value()));
                else if(child_node_name == (std::string)"gimbal_z")
                    p.engine.gimbal_limits.push_back(std::stof((std::string)cit_val.value()));
                else if(child_node_name == (std::string)"delay")
                    p.engine.delay = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"diameter")
                    p.engine.diameter = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"length")
                    p.engine.length = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"prop_mass")
                    p.engine.prop_mass = std::stof((std::string)cit_val.value());
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
                    p.dryMass = std::stof((std::string)cit_val.value());
            }
        }
        else if(node_name == (std::string)"COM")
        {
            for (pugi::xml_node_iterator cit = it->begin(); cit != it->end(); ++cit)
            {
                pugi::xml_attribute cit_val = cit->attribute("value");
                pugi::xml_attribute cit_name = cit->attribute("name");

                std::string child_node_name = (std::string)cit_name.value();

                if(child_node_name == (std::string)"com_x")
                    p.com.x() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"com_y")
                    p.com.y() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"com_z")
                    p.com.z() = std::stof((std::string)cit_val.value());
            }
        }
        else if(node_name == (std::string)"COP")
        {
            for (pugi::xml_node_iterator cit = it->begin(); cit != it->end(); ++cit)
            {
                pugi::xml_attribute cit_val = cit->attribute("value");
                pugi::xml_attribute cit_name = cit->attribute("name");

                std::string child_node_name = (std::string)cit_name.value();

                if(child_node_name == (std::string)"cop_x")
                    p.cop.x() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"cop_y")
                    p.cop.y() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"cop_z")
                    p.cop.z() = std::stof((std::string)cit_val.value());
            }
        }
        else if(node_name == (std::string)"MOI")
        {
            for (pugi::xml_node_iterator cit = it->begin(); cit != it->end(); ++cit)
            {
                pugi::xml_attribute cit_val = cit->attribute("value");
                pugi::xml_attribute cit_name = cit->attribute("name");

                std::string child_node_name = (std::string)cit_name.value();

                if(child_node_name == (std::string)"moi_x")
                    p.moi.x() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"moi_y")
                    p.moi.y() = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"moi_z")
                    p.moi.z() = std::stof((std::string)cit_val.value());
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
                else if(child_node_name == (std::string)"latitude")
                    p.env.latitude = std::stof((std::string)cit_val.value());
                else if(child_node_name == (std::string)"longitude")
                    p.env.longitude = std::stof((std::string)cit_val.value());
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

ThrustCurve FileIO::ParseThrustCurve(const char *file_path)
{
    ThrustCurve curve;

    pugi::xml_document doc;
   
    // load the XML file
    if (!doc.load_file(file_path))
        std::cout << "Error: Cannot load XML file!" << std::endl;

    pugi::xml_node data = doc.child("data");
    for (pugi::xml_node_iterator it = data.begin(); it != data.end(); ++it)
    {
       
        std::string node_name = (std::string)it->name();
        if(node_name == (std::string)"eng-data")
        {                          
            pugi::xml_attribute it_t = it->attribute("t");
            pugi::xml_attribute it_f = it->attribute("f");
            pugi::xml_attribute it_m = it->attribute("m");
            pugi::xml_attribute it_cg = it->attribute("cg");

            curve.thrust_curve_x.push_back(std::stof((std::string)it_t.value()));
            curve.thrust_curve_y.push_back(std::stof((std::string)it_f.value()));
        }
    }
    return curve;
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