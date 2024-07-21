#include "scene.hpp"
#include "simulation/simulation.hpp"



// Test scene to check correct compilation and run
//  The code should display a rotating cube and a textured quad on top of a circular ground.

using namespace cgp;

cylinder_structure create_cylinder(vec3 const& c, float r, float h, vec3 const& n)
{
	cylinder_structure cylinder;
	cylinder.c = c;
	cylinder.r = r;
	cylinder.h = h;
	cylinder.n = n;
	return cylinder;
}

void scene_structure::init_cylinders()
{
	float const r_cylinder = 0.05f;
	float const h_cylinder = 0.3f;
	vec3 const n = { 1, 0, 0 };

	cylinder_list.push_back(create_cylinder({ 0, 0.4f,0.4f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, -0.4f,0.4f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 0.4f,-0.4f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, -0.4f,-0.4f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 0,1 }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 0,-1 }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 1.2f,0 }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, -1.2f,0 }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 0,1.8f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 0,-1.8f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 0.7f,2.5f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, -0.7f,2.5f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 0.7f,-2.5f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, -0.7f,-2.5f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 1.0f,1.6f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, -1.0f,1.6f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 1.0f,-1.6f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, -1.0f,-1.6f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 1.5f,1.0f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, -1.5f,1.0f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, 1.5f,-1.0f }, r_cylinder, h_cylinder, n));
	cylinder_list.push_back(create_cylinder({ 0, -1.5f,-1.0f }, r_cylinder, h_cylinder, n));

	cylinder.initialize_data_on_gpu(mesh_primitive_cylinder(r_cylinder, { 0,0,0 }, n*h_cylinder));
	// cylinder.material.color = { 0.8f,0.8f,1 };
	cylinder.material.phong.specular = 0.1f;
	cylinder.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/cylinder.jpg");

	circle.initialize_data_on_gpu(mesh_primitive_disc(r_cylinder, { h_cylinder,0,0 }, n));
	// circle.material.color = { 0.8f,0.8f,1 };
	circle.material.phong.specular = 0.1f;
	circle.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/cylinder.jpg");
}

elastic_rope_structure create_rope(vec3 const& start, vec3 const& end)
{
	elastic_rope_structure rope;
	// Create a line
	numarray<vec3> line_positions;   // the basic structure of a curve is a vector of vec3
	int const N_sample = 101;          // Number of samples of the curve
	for (int k = 0; k < N_sample; ++k)
	{
		const float u = k / (N_sample - 1.0f); // u \in [0,1]

		vec3 direction = end - start;

		// curve oscillating as a cosine
		vec3 p = start + u * direction;

		line_positions.push_back(p);

		// Create a rope
		rope.ini_positions.push_back(p);
		rope.cur_positions.push_back(p);
		rope.normal = normalize(cross(direction, { 1,0,0 }));
		rope.length = norm(direction);
		rope.elasticity = 50.0f;
	}

	return rope;
}

void scene_structure::init_ropes()
{	
	float const r_particle = 0.08f;

	rope_list.push_back(create_rope({r_particle,2.0f, 1.3f}, {r_particle,0.9f, 3.0f}));
	rope_list.push_back(create_rope({r_particle,2.0f, -1.3f}, {r_particle,0.9f, -3.0f}));
	rope_list.push_back(create_rope({r_particle,-2.0f, 1.3f}, {r_particle,-0.9f, 3.0f}));
	rope_list.push_back(create_rope({r_particle,-2.0f, -1.3f}, {r_particle,-0.9f, -3.0f}));


	// rope_list.push_back(create_rope({r_particle,-0.5f, -1.0f}, {r_particle,0.5f, -1.0f}));

	flix_line.initialize_data_on_gpu(rope_list[0].ini_positions);
	flix_line.color = { 1,0,0 };
}

void scene_structure::initialize()
{
	// Set the behavior of the camera and its initial position
	camera_control.initialize(inputs, window);
	camera_control.set_rotation_axis_z();
	camera_control.look_at({ 3.0f, 2.0f, 2.0f }, { 0,0,0 }, { 0,0,1 });

	// Create a visual frame representing the coordinate system
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());

	// initialize the ground
	boundary_list = {-2, 2, -3, 3}; // left, right, bottom, top
	ground.initialize_data_on_gpu(mesh_primitive_quadrangle({ 0,boundary_list[0],boundary_list[3] }, { 0,boundary_list[0],boundary_list[2] }, { 0,boundary_list[1],boundary_list[2] }, { 0, boundary_list[1],boundary_list[3] }));
	ground.material.phong.specular = 0.05f;
	ground.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/background1.jpg");

	// initialize the boundary
	float const h_boundary = 0.3f;
	boundary_left.initialize_data_on_gpu(mesh_primitive_quadrangle({ h_boundary,boundary_list[0],boundary_list[3] }, { h_boundary,boundary_list[0],boundary_list[2] }, { 0,boundary_list[0],boundary_list[2] }, { 0,boundary_list[0],boundary_list[3] }));
	boundary_right.initialize_data_on_gpu(mesh_primitive_quadrangle({ h_boundary,boundary_list[1],boundary_list[3] }, { h_boundary,boundary_list[1],boundary_list[2] }, { 0,boundary_list[1],boundary_list[2] }, { 0,boundary_list[1],boundary_list[3] }));
	boundary_bottom.initialize_data_on_gpu(mesh_primitive_quadrangle({ h_boundary,boundary_list[0],boundary_list[2] }, { h_boundary,boundary_list[1],boundary_list[2] }, { 0,boundary_list[1],boundary_list[2] }, { 0,boundary_list[0],boundary_list[2] }));
	boundary_top.initialize_data_on_gpu(mesh_primitive_quadrangle({ h_boundary,boundary_list[0],boundary_list[3] }, { h_boundary,boundary_list[1],boundary_list[3] }, { 0,boundary_list[1],boundary_list[3] }, { 0,boundary_list[0],boundary_list[3] }));
	boundary_left.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/boundary.jpg");
	boundary_right.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/boundary.jpg");
	boundary_bottom.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/boundary.jpg");
	boundary_top.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/boundary.jpg");

	// initialize the cylinders
	init_cylinders();

	// create a particle
	float const r_particle = 0.08f;
	particle.p = { r_particle,0,-0 };
	particle.v = { 0,-1.0f,-1.2f };
	particle.c = { 0,0,1 };
	particle.r = r_particle;
	particle.m = 1;
	
	// Create a sphere
	sphere.initialize_data_on_gpu(mesh_primitive_sphere());
	// sphere.material.color = { 0,0,1 };
	sphere.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/ball1.jpg");

	// initialize the ropes
	init_ropes();

}

void scene_structure::display_frame()
{
	// Set the light to the current position of the camera
	environment.light = camera_control.camera_model.position();
	
	if (gui.display_frame)
		draw(global_frame, environment);


	float const time = timer.t;
	timer.update();

	// the general syntax to display a mesh is:
	//   draw(mesh_drawable_name, environment);
	//     Note: scene is used to set the uniform parameters associated to the camera, light, etc. to the shader
	draw(ground, environment);
	
	draw(boundary_left, environment);
	draw(boundary_right, environment);
	draw(boundary_bottom, environment);
	draw(boundary_top, environment);

	// display the cylinders
	for(auto& cylin : cylinder_list) {
		cylinder.model.translation = cylin.c;
		draw(cylinder, environment);
		circle.model.translation = cylin.c;
		draw(circle, environment);
	}

	// start the simulation
	float const dt = 0.01f * timer.scale;
	float const alpha = 1.0f;
	simulate(particle, boundary_list, cylinder_list, rope_list, dt, alpha);
		
	// Display sphere
	// ********************************************* //
	sphere.model.scaling = particle.r;
	sphere.model.translation = particle.p;
	draw(sphere, environment);

	// Display line
	// ********************************************* //
	for (auto& rope : rope_list) {
		flix_line.vbo_position.update(rope.cur_positions);
		draw(flix_line, environment);
	}

}

void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::SliderFloat("Time Scale", &timer.scale, 0.0f, 2.0f, "%.1f");
}

void scene_structure::mouse_move_event()
{
	if (!inputs.keyboard.shift)
		camera_control.action_mouse_move(environment.camera_view);
}
void scene_structure::mouse_click_event()
{
	camera_control.action_mouse_click(environment.camera_view);
}
void scene_structure::keyboard_event()
{
	camera_control.action_keyboard(environment.camera_view);
}
void scene_structure::idle_frame()
{
	camera_control.idle_frame(environment.camera_view);
}