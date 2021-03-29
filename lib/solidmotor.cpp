#include "engine.h"

SolidMotor::SolidMotor(std::string _name,
                       float _mass,
                       Eigen::Vector3f _com,
                       Eigen::Vector3f _rel_pos,
                       Eigen::Vector3f _moi,
                       Eigen::Vector3f _rel_rot,
                       float _isp,
                       float _avg_thrust,
                       float _burn_time,
                       ThrustCurve _thrust_curve,
                       Eigen::Vector3f _cot,
                       Eigen::Vector3f _gimbal,
                       std::vector<float> _gimbal_limits,
                       float _delay,
                       float _diameter,
                       float _length,
                       float _prop_mass)
{
    Engine(_name,
           _mass,
           _com,
           _rel_pos,
           _moi,
           _rel_rot,
           _isp,
           _avg_thrust,
           _burn_time,
           _thrust_curve,
           _cot,
           _gimbal,
           _gimbal_limits);

    delay = _delay;
    radius = _diameter / 2;
    length = _length;
    prop_mass = _prop_mass;
    initial_prop_mass = prop_mass;
    prop_volume = pi * pow(radius, 2) * length;
    prop_density = prop_mass / prop_volume;
    isp = (avg_thrust / g_0) / (prop_mass / burn_time);
    moi = CalculateMOI();
}

SolidMotor::SolidMotor() {}

void SolidMotor::UpdateSolidMotor(float t, float step)
{
    UpdateEngine(t - delay);
    prop_mass -= mass_flow_rate * step;
    mass -= mass_flow_rate * step;
    moi = CalculateMOI();
    UpdateComponent();
}

Eigen::Vector3f SolidMotor::CalculateMOI()
{
    float mass_delta = initial_prop_mass - prop_mass;
    float bore_radius = sqrt(mass_delta / (prop_density * length * pi));

    moi[0] = (1 / 12) * mass * (3 * pow(radius, 2) + pow(bore_radius, 2) + pow(length, 2));
    moi[1] = (1 / 12) * mass * (3 * pow(radius, 2) + pow(bore_radius, 2) + pow(length, 2));
    moi[2] = (1 / 2) * mass * (3 * pow(radius, 2) + pow(bore_radius, 2));

    return moi;
}

SolidMotor::~SolidMotor() {}