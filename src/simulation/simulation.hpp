#pragma once

#include "cgp/cgp.hpp"

struct particle_structure
{
    cgp::vec3 p; // Position
    cgp::vec3 v; // Speed
    cgp::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
};

struct cylinder_structure
{
    cgp::vec3 c; // Center
    float r;     // Radius
    float h;     // Height
    cgp::vec3 n; // Normal
};

struct elastic_rope_structure
{
    std::vector<cgp::vec3> cur_positions; // The positions of rope nodes
    std::vector<cgp::vec3> ini_positions; // The initial positions of rope nodes
    cgp::vec3 normal;                     // The normal of the rope
    float length;             // The total length of the rope
    float elasticity;         // The elasticity coefficient of the rope
    int collision_index = -1; // The index of the node that collides with the particle
    int prev_collision_index = -1;
};

void simulate(particle_structure& particle, std::vector<float> boundary_list, std::vector<cylinder_structure>& cylinders, std::vector<elastic_rope_structure>& ropes, float dt, float const alpha);

void collision_sphere_plane(particle_structure& particle, std::vector<float> boundary_list, float const alpha);

void collision_sphere_cylinder(particle_structure& particle, cylinder_structure const& cylinder, float const alpha);

void particle_elastic_rope_collision(particle_structure& particle, elastic_rope_structure& rope, float dt);
