#include "simulation.hpp"

using namespace cgp;

bool is_vector_equal(vec3 const& v1, vec3 const& v2)
{
    return v1[0] == v2[0] && v1[1] == v2[1] && v1[2] == v2[2];
}

bool is_vec3_same_direction(vec3 const& normal, vec3 const& v1, vec3 const& v2)
{
    float dot1 = dot(normal, v1);
    float dot2 = dot(normal, v2);
    return dot1 * dot2 > 0;
}

vec3 project_point(vec3 const& p, vec3 const& p1, vec3 const& p2, vec3 const& normal)
{
    if (p1.y == p2.y) {  // 如果直线垂直于 x 轴
        return {p.x, p1.y, p.z};  // 投影点的 x 坐标与直线上任意一点的 x 坐标相同，y 坐标为要投影点的 y 坐标
    } else if (p1.z == p2.z) {  // 如果直线平行于 y 轴
        return {p.x, p.y, p1.z};  // 投影点的 y 坐标与直线上任意一点的 y 坐标相同，x 坐标为要投影点的 x 坐标
    } else
    {
        // 计算直线斜率
        float k = (p2.z - p1.z) / (p2.y - p1.y);
        // 计算垂直直线的斜率
        float k_vertical = -1 / k;
        // 计算投影点到直线的距离
        float dist = (normal.y * (p1.y - p.y) + normal.z * (p1.z - p.z)) / norm(normal);
        // 计算投影点的坐标
        float x = p.x;
        float y = p.y + dist * normal.y / norm(normal);
        float z = p.z + dist * normal.z / norm(normal);
        return {x, y, z};
    }
}

void update_rope_positions(elastic_rope_structure& rope, vec3 const& direction, float const penetration)
{
    // update rope position
    // left side
    float len_left = norm(rope.ini_positions[rope.collision_index] - rope.ini_positions[0]);
    for (int i = rope.collision_index - 1; i > 0; --i) {
        float len_i = norm(rope.ini_positions[i] - rope.ini_positions[0]);
        rope.cur_positions[i] = rope.ini_positions[i] - (penetration * len_i / len_left) * direction;
    }
    // right side
    float len_right = norm(rope.ini_positions[rope.ini_positions.size() - 1] - rope.ini_positions[rope.collision_index]);
    for (int i = rope.collision_index + 1; i < rope.cur_positions.size() - 1; ++i) {
        float len_i = norm(rope.ini_positions[rope.ini_positions.size() - 1] - rope.ini_positions[i]);
        rope.cur_positions[i] = rope.ini_positions[i] - (penetration * len_i / len_right) * direction;
    }
    rope.cur_positions[rope.collision_index] = rope.ini_positions[rope.collision_index] - penetration * direction;
}

void collision_sphere_plane(particle_structure& particle, std::vector<float> boundary_list, float const alpha)
{
    vec3 const& p = particle.p;
    vec3& v = particle.v;
    float const r = particle.r;

    if (p[1] - r < boundary_list[0]) {
        float const penetration = p[1] - r - boundary_list[0];
        if (penetration < 0) {
            float velocity_component = v[1];
            if (velocity_component < 0) {
                v[1] = alpha * (-v[1]);
            }
        }
    }
    if (p[1] + r > boundary_list[1]) {
        float const penetration = p[1] + r - boundary_list[1];
        if (penetration > 0) {
            float velocity_component = v[1];
            if (velocity_component > 0) {
                v[1] = alpha * (-v[1]);
            }
        }
    }
    if (p[2] - r < boundary_list[2]) {
        float const penetration = p[2] - r - boundary_list[2];
        if (penetration < 0) {
            float velocity_component = v[2];
            if (velocity_component < 0) {
                v[2] = alpha * (-v[2]);
            }
        }
    }
    if (p[2] + r > boundary_list[3]) {
        float const penetration = p[2] + r - boundary_list[3];
        if (penetration > 0) {
            float velocity_component = v[2];
            if (velocity_component > 0) {
                v[2] = alpha * (-v[2]);
            }
        }
    }

}

void collision_sphere_cylinder(particle_structure& particle, cylinder_structure const& cylinder, float const alpha)
{
    vec3& p = particle.p;
    vec3& v = particle.v;
    float const r = particle.r;

    vec3 const d = p - cylinder.c;
    vec3 const d_perp = d - dot(d, cylinder.n) * cylinder.n;
    float const d_perp_len = norm(d_perp);

    if (d_perp_len < cylinder.r + r) {
        vec3 const closest_point = cylinder.c + dot(d, cylinder.n) * cylinder.n + (cylinder.r / d_perp_len) * d_perp;
        vec3 const normal = normalize(p - closest_point);

        float const penetration = r - norm(p - closest_point);
        if (penetration > 0) {
            float velocity_component = dot(v, normal);
            if (velocity_component < 0) {
                vec3 const vn = dot(v, normal) * normal;
                vec3 const vt = v - vn;
                v = alpha * (vt - vn);
            }
        }
    }
}

void particle_elastic_rope_collision(particle_structure& particle, elastic_rope_structure& rope, float dt)
{
    // float const alpha = 0.95f; // 碰撞响应系数
    int closest_point_index = -1;
    float min_distance = std::numeric_limits<float>::max();
    for (int i = 0; i < rope.cur_positions.size(); ++i) {
        float distance = norm(particle.p - rope.ini_positions[i]);
        if (distance < min_distance) {
            min_distance = distance;
            closest_point_index = i;
        }
    }
    rope.collision_index = closest_point_index;

    if (rope.collision_index != -1 && rope.collision_index != 0 && rope.collision_index != rope.cur_positions.size() - 1 && rope.prev_collision_index != -1) {
        vec3& closest_point = rope.ini_positions[rope.collision_index];
        vec3 displacement = particle.p - closest_point;
        float distance = norm(displacement);
        vec3 direction = normalize(displacement);
        // std::cout << "closest_point: " << closest_point << " p: " << particle.p << std::endl;

        vec3& prev_collision_point = rope.cur_positions[rope.prev_collision_index];
        float prev_distance = norm(particle.p - prev_collision_point);
        vec3 prev_direction;
        if (!is_vector_equal(prev_collision_point, rope.ini_positions[rope.prev_collision_index])){
            prev_direction = normalize((prev_collision_point - rope.ini_positions[rope.prev_collision_index]) * 1e6);
        }
        else {
            prev_direction = -normalize(particle.p - prev_collision_point);
        }

        // std::cout << "prev_direction: " << prev_direction << "  direction: " << direction << std::endl;
        // std::cout << "norm dist: " << norm(prev_direction - direction) << std::endl;

        bool is_same_direction = is_vec3_same_direction(rope.normal, prev_direction, direction);
        float penetration;
        if (is_same_direction){
            penetration = particle.r + distance;
            direction = -direction;
        }
        else {
            penetration = particle.r - distance;
        }

        // std::cout << "prev_direction: " << prev_direction << "  direction: " << direction << std::endl;
        // std::cout << "prev_distance: " << prev_distance << "  penetration: " << penetration << std::endl;

        // 上一个碰撞点在圆内：发生碰撞
        if (prev_distance <= particle.r) {
            // std::cout << "collision" << std::endl;
            update_rope_positions(rope, direction, penetration);
        }

        // 上一个碰撞点在圆外：回弹或复原
        else if (prev_distance > particle.r) {
            if (!is_same_direction && distance >= particle.r)
            {
                // std::cout << "fuyuan" << std::endl;
                for (int i = 0; i < rope.cur_positions.size(); ++i) 
                {
                    rope.cur_positions[i] = rope.ini_positions[i];
                }
            }
            else 
            {
                // std::cout << "huitan" << std::endl;
                // std::cout << "prev_collision_point: " << prev_collision_point << std::endl;
                // std::cout << "prev_ini_ipoint: " << rope.ini_positions[rope.prev_collision_index] << std::endl;
                update_rope_positions(rope, direction, penetration);
            }
        }

        // 计算弹力
        // float spring_force_magnitude = rope.elasticity * norm(rope.cur_positions[rope.collision_index] - rope.ini_positions[rope.collision_index]);
        // vec3 spring_force = spring_force_magnitude * direction;
        float diff_left = norm(rope.cur_positions[rope.collision_index] - rope.cur_positions[0]) - norm(rope.ini_positions[rope.collision_index] - rope.ini_positions[0]);
        vec3 spring_direction_left = normalize(rope.cur_positions[0] - rope.cur_positions[rope.collision_index]);
        vec3 spring_force_left = rope.elasticity * diff_left * spring_direction_left;
        float diff_right = norm(rope.cur_positions[rope.cur_positions.size() - 1] - rope.cur_positions[rope.collision_index]) - norm(rope.ini_positions[rope.cur_positions.size() - 1] - rope.ini_positions[rope.collision_index]);
        vec3 spring_direction_right = normalize(rope.cur_positions[rope.cur_positions.size() - 1] - rope.cur_positions[rope.collision_index]);
        vec3 spring_force_right = rope.elasticity * diff_right * spring_direction_right;
        vec3 spring_force = spring_force_left + spring_force_right;

        // 更新粒子速度（考虑弹簧力和阻尼）
        vec3 particle_acceleration = spring_force / particle.m;
        particle.v += particle_acceleration * dt;
    }
    rope.prev_collision_index = rope.collision_index;
}

void simulate(particle_structure& particle, std::vector<float> boundary_list, std::vector<cylinder_structure>& cylinders, std::vector<elastic_rope_structure>& ropes, float dt_true, float const alpha)
{
    vec3 const g = { 0,0,-9.81f };
	size_t const N_substep = 10;
	float const dt = dt_true / N_substep;
	for (size_t k_substep = 0; k_substep < N_substep; ++k_substep)
    {
        // Compute gravity force
        // vec3 const f = particle.m * g;

        // particle.v = (1 - 0.9f * dt) * particle.v + dt * f / particle.m;

        // Update particle position
        particle.p += dt * particle.v;

        // Check for collision with cylinders
        for (cylinder_structure& cylinder : cylinders)
        {
            collision_sphere_cylinder(particle, cylinder, alpha);
        }

        // Check for collision with elastic rope
        for (elastic_rope_structure& rope : ropes)
        {
            particle_elastic_rope_collision(particle, rope, dt);
        }

        // Check for collision with boundaries
        collision_sphere_plane(particle, boundary_list, alpha);

        

    }
}
