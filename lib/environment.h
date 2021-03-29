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

#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include<math.h>
#include "types.h"

inline float HeterosphereEquation(float alt, float a, float b, float c, float d, float e)
{
    float alt_km = alt / 1000.0f;

    return exp(a * pow(alt_km, 4) 
               + b * pow(alt_km, 3) 
               + c * pow(alt_km, 2) 
               + d * alt_km 
               + e);
}

inline float CalculateGeopotentialAltitude(float alt)
{
    return earth_radius * alt / (earth_radius + alt);
}

inline float CalculateGravity(float alt)
{
    return g_0 * pow((earth_radius / (earth_radius + alt)), 2);
}

inline TempFunction CalculateTemperature(float alt, float geo_alt)
{
    TempFunction tempFunc;
    
    if(0 <= geo_alt <= 11000)
    {
        tempFunc.temp = 288.15f + (lm[0] * (geo_alt - 0.0f));
        tempFunc.b = 0;
        return tempFunc;
    }
    else if(11000.0f < geo_alt <= 20000.0f)
    {
        tempFunc.temp = 216.65f + (lm[1] * (geo_alt - 11000.0f));
        tempFunc.b = 1;
        return tempFunc;
    }
    else if(20000.0f < geo_alt <= 32000.0f)
    {
        tempFunc.temp = 216.65f + (lm[2] * (geo_alt - 20000.0f));
        tempFunc.b = 2;
        return tempFunc;
    }
    else if(32000.0f < geo_alt <= 47000.0f)
    {
        tempFunc.temp = 228.65f + (lm[3] * (geo_alt - 32000.0f));
        tempFunc.b = 3;
        return tempFunc;
    }
    else if(47000.0f < geo_alt <= 51000.0f)
    {
        tempFunc.temp = 270.65f + (lm[4] * (geo_alt - 47000.0f));
        tempFunc.b = 4;
        return tempFunc;
    }
    else if(51000.0f < geo_alt <= 71000.0f)
    {
        tempFunc.temp = 270.65f + (lm[5] * (geo_alt - 51000.0f));
        tempFunc.b = 5;
        return tempFunc;
    }
    else if(71000.0f < geo_alt <= 84852.0f)
    {
        tempFunc.temp = 214.65f + (lm[6] * (geo_alt - 71000.0f));
        tempFunc.b = 6;
        return tempFunc;
    }
    else if(86000.0f < geo_alt <= 91000.0f)
    {
        tempFunc.temp = 186.87f;
        tempFunc.b = 7;
        return tempFunc;
    }
    else if (91000.0f < geo_alt <= 110000.0f)
    {
        float layer;

        if (91000.0f < geo_alt <= 100000.0f)
        {
            layer = 8;
        }
        else
        {
            layer = 9;
        }

        tempFunc.temp = 263.1905f - 76.3232f * sqrt(1.0f - pow(((geo_alt - 91000.0f) / -19942.9f), 2));
        tempFunc.b = layer;
        return tempFunc;
    }
    else if(110000.0f < geo_alt <= 120000.0f)
    {
        tempFunc.temp = 240.0f + 0.012f * (geo_alt - 110000);
        tempFunc.b = 10;
        return tempFunc;
    }
    else if(120000.0f < geo_alt <= 1000000.0f)
    {
        float layer;
        if(120000.0f < geo_alt <= 150000.0f)
        {
            layer = 11;
        }
        else if(150000.0f < geo_alt <= 200000.0f)
        {
            layer = 12;
        }
        else if(200000.0f < geo_alt <= 300000.0f)
        {
            layer = 13;
        }
        else if(300000.0f < geo_alt <= 500000.0f)
        {
            layer = 14;
        }
        else if(500000.0f < geo_alt <= 750000.0f)
        {
            layer = 15;
        }
        else
        {
            layer = 16;
        }

        float xi = (geo_alt - 120000.0f) * (6356766.0f + 120000.0f) / (6356766.0f + geo_alt);

        tempFunc.temp = 1000.0f - 640.0f * exp(-0.00001875f * xi);
        tempFunc.b = layer;

        return tempFunc;
    }
}

inline float CalculatePressure(float alt, float geo_alt, float temp, int b)
{
    if(b <= 6)
    {
        if(lm[b] != 0)
        {
            return pb[b] * pow((tb[b]/temp), (g_0 * air_molar_mass / (gas_constant * lm[b])));
        }
        else
        {
            return pb[b] * exp(-g_0 * air_molar_mass * (geo_alt - hb[b]) / (gas_constant * tb[b]));
        }
    }
    else if(b == 7)
    {
        return HeterosphereEquation(alt, 0.0f, 2.159582e-6f, -4.836957e-4f, -0.1425192f, 13.47530f);
    }
    else if(b == 8)
    {
        return HeterosphereEquation(alt, 0.0f, 3.304895e-5f, -0.009062730f, 0.6516698f, -11.03037f);
    }
    else if(b == 9)
    {
        return HeterosphereEquation(alt, 0.0f, 6.693926e-5f, -0.01945388f, 1.719080f, -47.75030f);
    }
    else if(b == 10)
    {
        return HeterosphereEquation(alt, 0.0f, -6.539316e-5f, 0.02485568f, -3.223620f, 135.9355f);
    }
    else if(b == 11)
    {
        return HeterosphereEquation(alt, 2.283506e-7f, -1.343221e-4f, 0.02999016f, -3.055446f, 113.5764f);
    }
    else if(b == 12)
    {
        return HeterosphereEquation(alt, 1.209434e-8f, -9.692458e-6f, 0.003002041f, -0.4523015f, 19.19151f);
    }
    else if(b == 13)
    {
        return HeterosphereEquation(alt, 8.113942e-10f, -9.822568e-7f, 4.687616e-4f, -0.1231710f, 3.067409f);
    }
    else if(b == 14)
    {
        return HeterosphereEquation(alt, 9.814674e-11f, -1.654439e-7f, 1.148115e-4f, -0.05431334f, -2.011365f);
    }
    else if(b == 15)
    {
        return HeterosphereEquation(alt, -7.835161e-11f, 1.964589e-7f, -1.657213e-4f, 0.04305869f, -14.77132f);
    }
    else if(b == 16)
    {
        return HeterosphereEquation(alt, 2.813255e-11f, -1.120689e-7f, 1.695568e-4f, -0.1188941f, 14.56718f);
    }
    else
    {
        return 0;
    }
}

inline float CalculateDensity(float alt, float pressure, float temp, int b)
{
    if(b <= 6)
    {
        // Ideal gas law
        return (pressure * air_molar_mass) / (gas_constant * temp);
    }
    else if(b == 7)
    {
        return HeterosphereEquation(alt, 0.0f, -3.322622E-06f, 9.111460E-04f, -0.2609971f, 5.944694f);
    }
    else if(b == 8)
    {
        return HeterosphereEquation(alt, 0.0f, 2.873405e-05f, -0.008492037f, 0.6541179f, -23.62010f);
    }
    else if(b == 9)
    {
        return HeterosphereEquation(alt, -1.240774e-05f, 0.005162063f, -0.8048342f, 55.55996f, -1443.338f);
    }
    else if(b == 10)
    {
        return HeterosphereEquation(alt, 0.0f, -8.854164e-05f, 0.03373254f, -4.390837f, 176.5294f);
    }
    else if(b == 11)
    {
        return HeterosphereEquation(alt, 3.661771e-07f, -2.154344e-04f, 0.04809214f, -4.884744f, 172.3597f);
    }
    else if(b == 12)
    {
        return HeterosphereEquation(alt, 1.906032e-08f, -1.527799E-05f, 0.004724294f, -0.6992340f, 20.50921f);
    }
    else if(b == 13)
    {
        return HeterosphereEquation(alt, 1.199282e-09f, -1.451051e-06f, 6.910474e-04f, -0.1736220f, -5.321644f);
    }
    else if(b == 14)
    {
        return HeterosphereEquation(alt, 1.140564e-10f, -2.130756e-07f, 1.570762e-04f, -0.07029296f, -12.89844f);
    }
    else if(b == 15)
    {
        return HeterosphereEquation(alt, 8.105631e-12f, -2.358417e-09f, -2.635110e-06f, -0.01562608f, -20.02246f);
    }
    else if(b == 16)
    {
        return HeterosphereEquation(alt, -3.701195e-12f, -8.608611e-09f, 5.118829e-05f, -0.06600998f, -6.137674f);
    }
    else
    {
        return 0;
    }
}

inline float CalculateMach(float temp)
{
    return sqrt((air_gamma * gas_constant * temp) / air_molar_mass);
}

inline EnvironmentVars CalculateEnvironmentVariables(float alt)
{
    EnvironmentVars vars;

    float geo_alt = round(CalculateGeopotentialAltitude(alt));
    vars.g = CalculateGravity(alt);
    vars.tempFunc = CalculateTemperature(alt, geo_alt);
    vars.pressure = CalculatePressure(alt, geo_alt, vars.tempFunc.temp, (int)vars.tempFunc.b);
    vars.density = CalculateDensity(alt, vars.pressure, vars.tempFunc.temp, (int)vars.tempFunc.b);
    vars.c = CalculateMach(vars.tempFunc.temp);

    return vars;
}

#endif