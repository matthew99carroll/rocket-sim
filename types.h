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

const float pi = 3.14159265359;
const float g_0 = 9.80665;
const float air_molar_mass = 0.02896968;
const float gas_constant = 8.314462618;
const float air_gamma = 1.4;
const float air_rho_0 = 1.2252;
const float earth_radius = 6356766;

// struct Vector3f {
//     float x;
//     float y;
//     float z;
// };

// Vector3f Vector3fAddition(Vector3f a, Vector3f b)
// {
//     Vector3f c;
//     c.x = a.x + b.x;
//     c.y = a.y + b.y;
//     c.z = a.z + b.z;

//     return c;
// }

// Vector3f Vector3fSubtraction(Vector3f a, Vector3f b)
// {
//     Vector3f c;
//     c.x = a.x - b.x;
//     c.y = a.y - b.y;
//     c.z = a.z - b.z;

//     return c;
// }

// Vector3f Vector3fMultiply(Vector3f a, Vector3f b)
// {
//     Vector3f c;
//     c.x = a.x * b.x;
//     c.y = a.y * b.y;
//     c.z = a.z * b.z;

//     return c;
// }

// Vector3f Vector3fMultiply(float a, Vector3f b)
// {
//     Vector3f c;
//     c.x = a * b.x;
//     c.y = a * b.y;
//     c.z = a * b.z;

//     return c;
// }

// Vector3f Vector3fDivide(Vector3f a, Vector3f b)
// {
//     Vector3f c;
//     c.x = a.x / b.x;
//     c.y = a.y / b.y;
//     c.z = a.z / b.z;

//     return c;
// }

// Vector3f Vector3fDivide(float a, Vector3f b)
// {
//     Vector3f c;
//     c.x = a / b.x;
//     c.y = a / b.y;
//     c.z = a / b.z;

//     return c;
// }

// Vector3f Vector3fDivide(Vector3f a, float b)
// {
//     Vector3f c;
//     c.x = a.x / b;
//     c.y = a.y / b;
//     c.z = a.z / b;

//     return c;
// }