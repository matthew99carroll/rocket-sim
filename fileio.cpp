#include "fileio.h"
#include <iostream>

FileIO::FileIO(const char* _file_path)
{
    file_path = _file_path;
}

Params FileIO::ParseFile()
{
    pugi::xml_document doc;
   
    // load the XML file
    if (!doc.load_file(file_path))
        std::cout << "Error: Cannot load XML file!" << std::endl;

    pugi::xml_node rocket = doc.child("Rocket");

    for (pugi::xml_node_iterator it = rocket.begin(); it != rocket.end(); ++it)
    {
        std::cout << "Rocket:";

        for (pugi::xml_attribute_iterator ait = it->attributes_begin(); ait != it->attributes_end(); ++ait)
        {
            // if(ait->name() == "type")
            //     config_params.engine.type = ait->value();
            // else if(ait->name() == "isp")
            //     config_params.engine.isp = std::stof(ait->value());
            // else if(ait->name() == "thrust")
            //     config_params.engine.thrust = std::stof(ait->value());

            std::cout << " " << ait->name() << "=" << ait->value();
        }

        std::cout << std::endl;
    }

    return config_params;
}