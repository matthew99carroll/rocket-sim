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

#include "parameters.h"

Parameters::Parameters(EngineParameters _engine,
               FuelParameters _fuel,
               MassParameters _mass,
               AerodynamicsParameters _aero,
               EnvironmentParameters _env,
               SimulationParameters _sim)
{
    params.engine = _engine;
    params.fuel = _fuel;
    params.mass = _mass;
    params.aero = _aero;
    params.env = _env;
    params.sim = _sim;
}