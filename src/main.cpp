#include<iostream>
// #include "parameters.h"
// #include "environment.h"
// #include "system.h"
#include "../lib/fileio.h"

FileIO parser;

Params parameters;

int main()
{
    parser.file_path = "../include/rocket.xml"; 

    parameters = parser.ParseFile();

    std::cout << parameters.aero.cd << std::endl;
    std::cout << parameters.aero.cs_area << std::endl;

    std::cin.get();

}