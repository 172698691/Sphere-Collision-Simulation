#pragma once

#include "cgp/cgp.hpp"
#include "environment.hpp"
#include "simulation/simulation.hpp"
#include <vector>

using cgp::mesh_drawable;
using cgp::curve_drawable;
using cgp::mesh;
using cgp::numarray;
using cgp::vec3;


struct gui_parameters {
	bool display_frame = true;
};

// Define cylinder_structure inside scene_structure
struct scene_structure : cgp::scene_inputs_generic {

    // ****************************** //
    // Elements and shapes of the scene
    // ****************************** //
    camera_controller_orbit_euler camera_control;
    camera_projection_perspective camera_projection;
    window_structure window;

    mesh_drawable global_frame;          // The standard global frame
    environment_structure environment;   // Standard environment controler
    input_devices inputs;                // Storage for inputs status (mouse, keyboard, window dimension)
    gui_parameters gui;                  // Standard GUI element storage

    // ****************************** //
    // Elements and shapes of the scene
    // ****************************** //

    cgp::timer_basic timer;

    mesh_drawable cube;
    mesh_drawable ground;
    mesh_drawable cylinder;
    mesh_drawable circle;
    mesh_drawable sphere;
    mesh_drawable boundary_left;
    mesh_drawable boundary_right;
    mesh_drawable boundary_bottom;
    mesh_drawable boundary_top;

    particle_structure particle;
    curve_drawable flix_line;
	// elastic_rope_structure rope;

    std::vector<cylinder_structure> cylinder_list;
    std::vector<elastic_rope_structure> rope_list;
    std::vector<float> boundary_list;

    // ****************************** //
    // Functions
    // ****************************** //

    void initialize();    // Standard initialization to be called before the animation loop
    void display_frame(); // The frame display to be called within the animation loop
    void display_gui();   // The display of the GUI, also called within the animation loop

    void mouse_move_event();
    void mouse_click_event();
    void keyboard_event();
    void idle_frame();

    void init_cylinders();
    void init_ropes();
};
