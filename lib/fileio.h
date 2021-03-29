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

#include <iostream>
#include <fstream>
#include <string>
#include "types.h"
#include "system.h"
#include "../include/PugiXML/pugixml.hpp"

class FileIO
{
private:
    std::ofstream file;
    std::string filename;

public:
    const char *file_path;

    FileIO();
    FileIO(const char *_file_path);

    Params ParseFile();
    void WriteOutput(System &s, std::string filename);

    ~FileIO();
};