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

FileIO::~FileIO()
{
}