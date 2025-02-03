#ifndef ASSEMBLER_HPP
#define ASSEMBLER_HPP

#include <stdio.h>

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <map>


#include "libslic3r/Model.hpp"


namespace Slic3r
{

   class Print;

}

class Assembl3r
{
    private:

        struct AssemblyNode
        {
            std::vector<AssemblyNode*> parents;

            Slic3r::Model model;

            bool operator <(const AssemblyNode& rhs) const              //TODO this is janky
            {
                for (auto model_object_1 : model.objects)
                {
                    bool id_found = false;

                    for (auto model_object_2 : rhs.model.objects)
                    {
                        if (model_object_1->id().id == model_object_2->id().id)
                            id_found = true;
                    }

                    if (!id_found)
                        return true;
                }

                return false;
            }
        };

    public:

        Assembl3r(const Assembl3r&) = delete;
        Assembl3r& operator=(const Assembl3r&) = delete;

        static Assembl3r& get_instance() {
            static Assembl3r instance;
            return instance;
        }

        std::vector<std::string> simple_rotation(Slic3r::Print &print);

        std::vector<std::string> simple_layer_assembly(Slic3r::Print &print);

        void generate_assembly_sequence(Slic3r::Print &print);

        Slic3r::Model initial_model;

        void AddToGCode(std::string fragment);

        std::vector<AssemblyNode> find_node_neighbours(AssemblyNode node);

    private:

        Assembl3r() {}

        ~Assembl3r() {}

        void generate_model_object_pairs(Slic3r::Print &print);

        std::set<AssemblyNode> m_assembly_nodes;

        struct ModelObjectPairPtrs
        {
            size_t id;

            Slic3r::ModelObject* model_object;

            Slic3r::ModelObject* print_object; 
        };

        std::map<size_t, ModelObjectPairPtrs>  m_object_map;

        double m_safe_height = 100;

        std::vector<std::string> gcode;  //TEMP - this will eleventually have to be replaced when dealing with splittable objects

        std::vector<std::string> m_assembly_commands;

        //Check if a part can be moved arbitrarily in the z direction without colliding with other parts
        bool collisions_on_z_move(Slic3r::ModelObject* model_object);

        //Mesh functions

        bool triangle_line_intersect(std::vector<stl_vertex> triangle, std::vector<stl_vertex> line);

        bool triangles_intersect(std::vector<stl_vertex> triangle_A, std::vector<stl_vertex> triangle_B);

        bool meshes_intersect(Slic3r::TriangleMesh mesh_1, Slic3r::TriangleMesh mesh_2);

        //GCode command generators

        void GCODE_go_to_safe_height();

        void GCODE_go_to_height(double height, double feed_rate = 1000);

        void GCODE_go_to_position(double x_pos, double y_pos, double feed_rate = 1000);

        void GCODE_change_to_gripper();

        void GCODE_change_to_vacuum();

        void GCODE_change_to_extruder();

        void GCODE_open_gripper();

        void GCODE_close_gripper(double close_value);

        void GCODE_vibrate_gripper(int cycles = 100, double amplitude = 0.09, int acceleration = 40000);

        void GCODE_vacuum_on();

        void GCODE_vacuum_off();
};



#endif