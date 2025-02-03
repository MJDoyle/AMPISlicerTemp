#include "Assembler.hpp"
#include "libslic3r/Print.hpp"
#include "yaml-cpp/yaml.h"

#include <fstream>

/*
    Generate a map of object names to object positions in both the model and the print
*/
void Assembl3r::generate_model_object_pairs(Slic3r::Print &print)
{
    std::cout << std::endl << "***Generating model pairs***" << std::endl << std::endl;

    std::cout << "***Model objects***" << std::endl << std::endl;

    for (Slic3r::ModelObject* model_object : initial_model.objects)
    {
        std::cout   << "ID: " << model_object->id().id 
                    << " Name: " << model_object->name 
                    << " Internal: " << model_object->volumes[0]->internal
                    << " Position: "  << model_object->mesh().center().x() << " " << model_object->mesh().center().y() << " " << model_object->mesh().center().z() << std::endl;
                    
    }

    std::cout << std::endl << "***Print objects***" << std::endl << std::endl;

    for (auto print_object : print.objects())
    {
        for (auto print_instance : print_object->instances())
        {
            std::cout   << "ID: " << print_instance.model_instance->get_object()->id().id 
                        << " Name: " << print_instance.model_instance->get_object()->name
                        << " Internal: " << print_instance.model_instance->get_object()->volumes[0]->internal
                        << " Position: "  << print_instance.model_instance->get_object()->mesh().center().x() << " " << print_instance.model_instance->get_object()->mesh().center().y() << " " << print_instance.model_instance->get_object()->mesh().center().z() << std::endl;
        }
    }


    //Iterate through model objects and populate the map
    for (Slic3r::ModelObject* model_object : initial_model.objects)
    {
        m_object_map[model_object->id().id] = ModelObjectPairPtrs();

        m_object_map[model_object->id().id].model_object = model_object;

        m_object_map[model_object->id().id].id = model_object->id().id;
    }

    //Find the corresponding models from the print objects
    for (auto print_object : print.objects())
    {
        for (auto print_instance : print_object->instances())
        {
            size_t object_id = print_instance.model_instance->get_object()->id().id;

            if (m_object_map.count(object_id))
                m_object_map[object_id].print_object = print_instance.model_instance->get_object();

            else
                std::cerr << "Error - no corresponding model object found for print object" << std::endl;
        }       
    }

    //Check that all print objects have been set - note that a model object without a print object could be an external part
    for (auto const& [id, object_pair] : m_object_map)
    {
        if (object_pair.print_object == nullptr)
        {
            if (!object_pair.model_object->volumes[0]->internal)
                std::cout << "External part detected" << std::endl;

            else
                std::cerr << "Error - no corrsponding print object found for model object" << std::endl;
        }
    }   
}

void Assembl3r::AddToGCode(std::string fragment)
{
    std::stringstream ss(fragment);
    std::string line;

    while (std::getline(ss, line)) {
        gcode.push_back(line);
    }
}

void Assembl3r::generate_assembly_sequence(Slic3r::Print &print)
{

    generate_model_object_pairs(print);

    YAML::Node root;

    YAML::Node commands = YAML::Node(YAML::NodeType::Sequence);


    //Add the search-for-external-parts commands to the assembly

    for (auto const& [id, object_pair] : m_object_map)
    {
        if (!object_pair.model_object->volumes[0]->internal)
        {
            YAML::Node detect_part_command;
            detect_part_command["command-type"] = "LOCATE_EXTERNAL_PART";
            detect_part_command["command-properties"]["part-description"] = ""; //TODO need to work out how this interacts with Matheusz' programme
            detect_part_command["command-properties"]["part-name"] = object_pair.model_object->name;
            detect_part_command["command-properties"]["part-id"] = object_pair.model_object->id().id;
            detect_part_command["command-properties"]["part-height"] = object_pair.model_object->mesh().size().z();

            commands.push_back(detect_part_command);
        }
    }

    //Add the internal part designation commands

    for (auto const& [id, object_pair] : m_object_map)
    {
        if (object_pair.model_object->volumes[0]->internal)
        {
            YAML::Node designate_part_command;
            designate_part_command["command-type"] = "DESIGNATE_INTERNAL_PART";
            designate_part_command["command-properties"]["part-description"] = "";
            designate_part_command["command-properties"]["part-name"] = object_pair.model_object->name;
            designate_part_command["command-properties"]["part-id"] = object_pair.model_object->id().id;
            designate_part_command["command-properties"]["x-pos"] = object_pair.print_object->mesh().center().x();
            designate_part_command["command-properties"]["y-pos"] = object_pair.model_object->mesh().center().y();
            designate_part_command["command-properties"]["part-height"] = object_pair.model_object->mesh().size().z();              //TODO eventually this will be replaced by mesh or something in the part-description
        
            commands.push_back(designate_part_command);
        }
    }

    //Add the 3D printing commands to the assembly

    if (gcode.size() > 0)
    {
        YAML::Node print_command;
        print_command["command-type"] = "DIRECT_PRINT";

        // Add "gcode" sequence to the first command
        YAML::Node gcode1 = YAML::Node(YAML::NodeType::Sequence);

        for (std::string gcode_line : gcode)
        {
            gcode1.push_back(gcode_line);
        }

        print_command["command-properties"]["gcode"] = gcode1;

        commands.push_back(print_command);
    }

    //Generate the pnp commands and add them to the assembly

    //Transfer the object pairs into a vector and sort based on model z height

    std::vector<ModelObjectPairPtrs> sorted_object_pairs;

    for (auto const& [id, object_pair] : m_object_map)
        sorted_object_pairs.push_back(object_pair);

    std::sort(sorted_object_pairs.begin(), sorted_object_pairs.end(), [](const auto& a, const auto& b) {return a.model_object->mesh().center().z() < b.model_object->mesh().center().z();});



    // AssemblyNode base_node;

    // base_node.model = initial_model;

    // std::vector<AssemblyNode> neighbours = find_node_neighbours(base_node);

    // std::cout << "NUM NEIGHBOURS: " << neighbours.size() << std::endl;

    //TODO - this doesn't handle an external part being the first object
    //Select the first object as the base object
    size_t base_id = sorted_object_pairs[0].id;

    if (!sorted_object_pairs[0].model_object->volumes[0]->internal)
    {
        std::cerr << "Error - base part is not internal" << std::endl;
        return;
    }


    //Generate vibration commands

    for (ModelObjectPairPtrs object_pair : sorted_object_pairs)
    {
        //Do nothing with the base object
        if (object_pair.id == base_id)
            continue;

        //Only need to vibrate internal parts
        if (!object_pair.model_object->volumes[0]->internal)
            continue;

        YAML::Node vibrate_part_command;

        vibrate_part_command["command-type"] = "VIBRATE_PART";
        vibrate_part_command["command-properties"]["part-id"] = object_pair.model_object->id().id;
        
        commands.push_back(vibrate_part_command);
    }


    //Offset between base object position in print and in model
    Slic3r::Vec3d base_offset = sorted_object_pairs[0].model_object->mesh().center() - sorted_object_pairs[0].print_object->mesh().center();

    //Check that the base offset is zero - the base part must be on the bed in both the model and the print
    if (base_offset.z() != 0)
        std::cerr << "Error - z offset of base part is not zero" << std::endl;

    //Iterate through each of the objects and vibrate them (except for the base object)
    for (ModelObjectPairPtrs object_pair : sorted_object_pairs)
    {
        //Do nothing with the base object
        if (object_pair.id == base_id)
            continue;

        Slic3r::Vec3d target_position = object_pair.model_object->mesh().center() - base_offset;

        YAML::Node place_part_command;

        place_part_command["command-type"] = "PLACE_PART";
        place_part_command["command-properties"]["part-name"] = object_pair.model_object->name;
        place_part_command["command-properties"]["part-id"] = object_pair.model_object->id().id;
        place_part_command["command-properties"]["x-target-pos"] = target_position.x();
        place_part_command["command-properties"]["y-target-pos"] = target_position.y();                    
        place_part_command["command-properties"]["z-target-pos"] = object_pair.model_object->max_z();
        
        commands.push_back(place_part_command);
    }

    root["commands"] = commands;

    std::ofstream fout("assembly_plan.yaml");

    fout << root;

    fout.close();
}

bool Assembl3r::triangle_line_intersect(std::vector<stl_vertex> triangle, std::vector<stl_vertex> line)
{
    std::cout << "Triangle line intersect test" << std::endl;

    std::cout << "Triangle: " << std::endl << triangle[0] << " " << triangle[1] << " " << triangle[2] << std::endl;

    std::cout << "Line: " << std::endl <<  line[0] << " " << line[1] << std::endl;

    //Compute triangle normal
    stl_vertex triangle_normal = (triangle[1] - triangle[0]).cross(triangle[2] - triangle[0]);

    std::cout << "check1" << std::endl;

    //Compute line direction vector
    stl_vertex line_direction = line[1] - line[0];

    std::cout << "check2" << std::endl;

    //Check angle between line and normal
    float triangle_normal_dot_line_direction = triangle_normal.dot(line_direction);

    std::cout << "check3" << std::endl;

    //A value of 0 indicates an angle of 90 -> the line and triangle are parallel and so don't intersect
    if (triangle_normal_dot_line_direction == 0)
        return false; 

    std::cout << "check4" << std::endl;

    //Compute vector from line start point to any triangle point
    stl_vertex line_triangle_delta = triangle[0] - line[0];

    std::cout << "check5" << std::endl;

    //Compute intersection parameter
    float intersection_parameter = triangle_normal.dot(line_triangle_delta) / triangle_normal_dot_line_direction;

    std::cout << "check6" << std::endl;

    //If paramter does not lie within 0 <= t <= 1, there is no intersection
    if (intersection_parameter < 0 || intersection_parameter > 1)
        return false;

    std::cout << "check7" << std::endl;

    //Compute the intersection point
    stl_vertex intersection_point = line[0] + intersection_parameter * line_direction;

    std::cout << "Intersection point: " << intersection_point << std::endl;

    //Check if the intersection point is at one of the triangle vertices - if it is then this does not count as an intersection

    for (auto triangle_vertex : triangle)
    {
        float epsilon = 0.00001;

        if (abs(intersection_point(0) - triangle_vertex(0)) < epsilon && abs(intersection_point(1) - triangle_vertex(1)) < epsilon && abs(intersection_point(2) - triangle_vertex(2)) < epsilon)
            return false;
    }


    //Perform a point-in-triangle test in 3D using the barycentric method
    stl_vertex v_0 = triangle[2] - triangle[0];
    stl_vertex v_1 = triangle[1] - triangle[0];
    stl_vertex v_2 = intersection_point - triangle[0];

    std::cout << "check9" << std::endl;

    float d_00 = v_0.dot(v_0);
    float d_01 = v_0.dot(v_1);
    float d_11 = v_1.dot(v_1);
    float d_20 = v_2.dot(v_0);
    float d_21 = v_2.dot(v_1);

    std::cout << "check10" << std::endl;

    float denom = d_00 * d_11 - d_01 * d_01;

    float u = (d_11 * d_20 - d_01 * d_21) / denom;

    float v = (d_00 * d_21 - d_01 * d_20) / denom;

    if (u >= 0 && v >= 0 and u + v <= 1)
        return true;

    return false;

}

bool Assembl3r::triangles_intersect(std::vector<stl_vertex> triangle_A, std::vector<stl_vertex> triangle_B)
{
    std::cout << "Triangles intersect test" << std::endl;

    std::vector<stl_vertex> line_A_a;
    std::vector<stl_vertex> line_A_b;
    std::vector<stl_vertex> line_A_c;

    line_A_a.push_back(triangle_A[0]);
    line_A_a.push_back(triangle_A[1]);

    line_A_b.push_back(triangle_A[1]);
    line_A_b.push_back(triangle_A[2]);

    line_A_c.push_back(triangle_A[2]);
    line_A_c.push_back(triangle_A[0]);



    std::vector<stl_vertex> line_B_a;
    std::vector<stl_vertex> line_B_b;
    std::vector<stl_vertex> line_B_c;

    line_B_a.push_back(triangle_B[0]);
    line_B_a.push_back(triangle_B[1]);

    line_B_b.push_back(triangle_B[1]);
    line_B_b.push_back(triangle_B[2]);

    line_B_c.push_back(triangle_B[2]);
    line_B_c.push_back(triangle_B[0]);

    if (triangle_line_intersect(triangle_A, line_B_a))
        return true;

    if (triangle_line_intersect(triangle_A, line_B_b))
        return true;

    if (triangle_line_intersect(triangle_A, line_B_c))
        return true;

    if (triangle_line_intersect(triangle_B, line_A_a))
        return true;

    if (triangle_line_intersect(triangle_B, line_A_b))
        return true;

    if (triangle_line_intersect(triangle_B, line_A_c))
        return true;
    
    return false;
}



bool Assembl3r::meshes_intersect(Slic3r::TriangleMesh mesh_1, Slic3r::TriangleMesh mesh_2)
{
    std::cout << "Meshes intersect test" << std::endl;

    std::vector<stl_triangle_vertex_indices> indices_1 = mesh_1.its.indices;

    std::vector<stl_triangle_vertex_indices> indices_2 = mesh_2.its.indices;

    std::vector<stl_vertex> vertices_1 = mesh_1.its.vertices;

    std::vector<stl_vertex> vertices_2 = mesh_2.its.vertices;

    for (auto triangle_1 : indices_1)
    {
        for (auto triangle_2 : indices_2)
        {
            //https://stackoverflow.com/questions/7113344/find-whether-two-triangles-intersect-or-not
            //For each triangle pair there are two ways they could intersect: either two edges of triangle A intersect with triangle B, or one edge of triangle A
            //intersects with triangle B and one edge of triangle B intersects with triangle A
            //This doesn't account for the possiblity of co-planar triangles, but for our cae we're happy to treat this situation as the triangles not intersecting

            //Obtain the triangle vertices
            std::vector<stl_vertex> triangle_A;
            std::vector<stl_vertex> triangle_B;

            triangle_A.push_back((stl_vertex() << vertices_1[triangle_1(0)]).finished());
            triangle_A.push_back((stl_vertex() << vertices_1[triangle_1(1)]).finished());
            triangle_A.push_back((stl_vertex() << vertices_1[triangle_1(2)]).finished());

            triangle_B.push_back((stl_vertex() << vertices_2[triangle_2(0)]).finished());
            triangle_B.push_back((stl_vertex() << vertices_2[triangle_2(1)]).finished());
            triangle_B.push_back((stl_vertex() << vertices_2[triangle_2(2)]).finished());

            std::cout << std::endl << "Triangles to test - A:" << triangle_A[0] << " " << triangle_A[1] << " " << triangle_A[2] << std::endl << " B: " << triangle_B[0] << " " << triangle_B[1] << " " << triangle_B[2] << std::endl;

            //triangle_A.push_back((stl_vertex() << vertices_1[triangle_1(0)], vertices_1[triangle_1(1)], vertices_1[triangle_1(2)]).finished());
            //triangle_B.push_back((stl_vertex() << vertices_2[triangle_2(0)], vertices_2[triangle_2(1)], vertices_2[triangle_2(2)]).finished());

            if (triangles_intersect(triangle_A, triangle_B))
            {
                std::cout << "Triangles intersect" << std::endl;
                return true;             
            }
        }
    }

    return false;
}

std::vector<Assembl3r::AssemblyNode> Assembl3r::find_node_neighbours(Assembl3r::AssemblyNode node)
{
    std::vector<Assembl3r::AssemblyNode> neighbours;

    std::cout << "Finding neighbours" << std::endl;

    //Iterate through each object in the node's model
    //If the object can be removed (no collisions) then create a new node with the new model (minus one object from the original node)
    //and add this as a neighbour
    for (auto model_object : node.model.objects)
    {
        std::cout << "Object to pick: " << model_object->mesh().center().x() << " " << model_object->mesh().center().y() << " " << model_object->mesh().center().z() << std::endl;


        if (!collisions_on_z_move(model_object))
        {
            std::cout << "No collisions on move" << std::endl;

            Assembl3r::AssemblyNode neighbour_node;

            neighbour_node.model = node.model;

            neighbours.push_back(neighbour_node);
        }

        else
            std::cout << "Collisions on move" << std::endl;
    }

    return neighbours;
}

bool Assembl3r::collisions_on_z_move(Slic3r::ModelObject* model_object)
{
    //Step the model object up to some maximum height and check collisions against each of the other model parts

    float translatedDistance = 0;

    for (float height = 0; height < 100; height += 1)
    {
        translatedDistance += 1;

        model_object->translate(0, 0, 1);

        for (auto const& [id, object_pair] : m_object_map)
        {
            //Don't check intersects with self
            if (id == model_object->id().id)
                continue;

            if (meshes_intersect(model_object->mesh(), object_pair.model_object->mesh()))
            {
                model_object->translate(0, 0, -translatedDistance);

                return true;
            }
        }
    }

    model_object->translate(0, 0, -translatedDistance);

    return false;
}

/*
    Generate a GCode string commanding the toolhead to move to the safe height as determined by model size.
    Add the string to the list of assembly commands.
*/
void Assembl3r::GCODE_go_to_safe_height()
{
    std::stringstream command_stream;

    command_stream << "G0 Z" << m_safe_height << " ;Lower bed to safe height";

    m_assembly_commands.push_back(command_stream.str());
}

void Assembl3r::GCODE_go_to_height(double height, double feed_rate /*=1000*/)
{
    std::stringstream command_stream;

    //Default feed rate - G0
    if (feed_rate == double(1000))
        command_stream << "G0 Z" << height << " ;Set bed height";

    //Specified feed rate - G1
    else
        command_stream << "G1 Z" << height << " F" << feed_rate << " ;Set bed height";

    m_assembly_commands.push_back(command_stream.str());
}

void Assembl3r::GCODE_go_to_position(double x_pos, double y_pos, double feed_rate /*=1000*/)
{
    std::stringstream command_stream;

    //Default feed rate - G0
    if (feed_rate == double(1000))
        command_stream << "G0 X" << x_pos << " Y" << y_pos << " ;Move to XY position";

    //Specified feed rate - G1
    else
        command_stream << "G1 X" << x_pos << " Y" << y_pos << " F" << feed_rate << " ;Move to XY position";

    m_assembly_commands.push_back(command_stream.str());
}

void Assembl3r::GCODE_change_to_gripper()
{
    m_assembly_commands.push_back(std::string("TOOL_PICKUP T=3 ;Pick up gripper tool"));
}

void Assembl3r::GCODE_change_to_vacuum()
{
    m_assembly_commands.push_back(std::string("TOOL_PICKUP T=2 ;Pick up vacuum tool"));
}

void Assembl3r::GCODE_change_to_extruder()
{
    m_assembly_commands.push_back(std::string("TOOL_PICKUP T=0 ;Pick up extruder tool"));
}

void Assembl3r::GCODE_open_gripper()
{
    m_assembly_commands.push_back(std::string("GRIPPER_OPEN ;Open the gripper"));
}

void Assembl3r::GCODE_close_gripper(double close_value)
{
    std::stringstream command_stream;

    command_stream << "GRIPPER_CLOSE CLOSURE=" << close_value << " ;Close the gripper";

    m_assembly_commands.push_back(command_stream.str());
}

void Assembl3r::GCODE_vibrate_gripper(int cycles, double amplitude, int acceleration)
{
    std::stringstream command_stream;

    command_stream << "GRIPPER_BUZZ CYCLES=" << cycles << " AMPLITUDE=" << amplitude << " ACCELERATION=" << acceleration << " ;Vibrate the gripper";

    m_assembly_commands.push_back(command_stream.str());
}

void Assembl3r::GCODE_vacuum_on()
{
    m_assembly_commands.push_back(std::string("VACUUM_ON ;Turn vacuum on"));
}

void Assembl3r::GCODE_vacuum_off()
{
    m_assembly_commands.push_back(std::string("VACUUM_OFF ;Turn vacuum off"));
}


std::vector<std::string> Assembl3r::simple_rotation(Slic3r::Print &print)
{
    // generate_model_object_pairs(print);

    // if (m_object_map.size() == 0)
    //     return m_assembly_commands;

    // //Iterate though each part, vibrate it, pick it, rotate it, place it
    // for (auto const& [name, object_pair] : m_object_map)
    // {
    //     object_pair.print_object
    // }

    return m_assembly_commands;
}


std::vector<std::string> Assembl3r::simple_layer_assembly(Slic3r::Print &print)
{
    generate_model_object_pairs(print);

    if (m_object_map.size() == 0)
        return m_assembly_commands;

    //TODO
    return m_assembly_commands;

    //Transfer the object pairs into a vector and sort based on model z height

    std::vector<ModelObjectPairPtrs> sorted_object_pairs;

    for (auto const& [id, object_pair] : m_object_map)
        sorted_object_pairs.push_back(object_pair);

    std::sort(sorted_object_pairs.begin(), sorted_object_pairs.end(), [](const auto& a, const auto& b) {return a.model_object->mesh().center().z() < b.model_object->mesh().center().z();});


    //Look at rotations (ModelInstance)
    // for (ModelObjectPairPtrs object_pair : sorted_object_pairs)
    // {
    //     std::cout << "sorted pair: " << object_pair.id << " " << object_pair.model_object->name << " " << object_pair.model_object->mesh().center().z() << std::endl;

    //     Slic3r::Vec3d model_rotation_instance = object_pair.model_object->instances[0]->get_rotation();

    //     Slic3r::Vec3d print_rotation_instance = object_pair.print_object->instances[0]->get_rotation();

    //     std::cout << "Model instance rotation: " << model_rotation_instance.x() << " " << model_rotation_instance.y() << " " << model_rotation_instance.z() << std::endl;

    //     std::cout << "Print instance rotation: " << print_rotation_instance.x() << " " << print_rotation_instance.y() << " " << print_rotation_instance.z() << std::endl;
    // }

    //Check for intersection
    //if (sorted_object_pairs.size() > 1 && meshes_intersect(sorted_object_pairs[0].model_object->mesh(), sorted_object_pairs[1].model_object->mesh()))
    //    std::cout << "MESHES INTERSECT" << std::endl;

    // //Look at meshes (also in ModelVolume)
    // for (ModelObjectPairPtrs object_pair : sorted_object_pairs)
    // {
    //     std::cout << "Mesh: " << std::endl;

    //     std::vector<stl_triangle_vertex_indices> indices = object_pair.model_object->mesh().its.indices;

    //     std::vector<stl_vertex> vertices = object_pair.model_object->mesh().its.vertices;

    //     std::cout << "Vertices: " << std::endl;

    //     for (stl_vertex v : vertices)
    //         std::cout << v(0) << " " << v(1) << " " << v(2) << std::endl;



    //     std::cout << "Indices: " << std::endl;

    //     for (stl_triangle_vertex_indices tvi : indices)
    //         std::cout << tvi(0) << " " << tvi(1) << " " << tvi(2) << std::endl;
    // }
    


    //Select the first object as the base object
    size_t base_id = sorted_object_pairs[0].id;

    //Offset between base object position in print and in model
    Slic3r::Vec3d base_offset = sorted_object_pairs[0].model_object->mesh().center() - sorted_object_pairs[0].print_object->mesh().center();

    //Check that the base offset is zero - the base part must be on the bed in both the model and the print
    if (base_offset.z() != 0)
        std::cerr << "Error - z offset of base part is not zero" << std::endl;

    std::cout << "Assembly positions:" << std::endl;

    std::cout << "Bed positions:" << std::endl;

    for (ModelObjectPairPtrs object_pair : sorted_object_pairs)
        std::cout << object_pair.id << " " << object_pair.print_object->mesh().center().x() << " " << object_pair.print_object->mesh().center().y() << " " << object_pair.print_object->mesh().center().z() << std::endl;


    std::cout << "Model positions:" << std::endl;

    for (ModelObjectPairPtrs object_pair : sorted_object_pairs)
        std::cout << object_pair.id << " " << object_pair.model_object->mesh().center().x() << " " << object_pair.model_object->mesh().center().y() << " " << object_pair.model_object->mesh().center().z() << std::endl;

    std::cout << "Offset: " << base_offset.x() << " " << base_offset.y() << " " << base_offset.z() << std::endl;

    //Calculate the safe height in millimeters - all objects will be below this height
    //TODO - this is insufficient if initial model gets rotated or if individual parts are printed standing on end etc.
    m_safe_height = initial_model.max_z() + 10;

    //Set to safe height
    GCODE_go_to_safe_height();

    //Change to gripper
    GCODE_change_to_gripper();  

    //Open gripper
    GCODE_open_gripper();   

    //Iterate through each of the objects and vibrate them (except for the base object)
    for (ModelObjectPairPtrs object_pair : sorted_object_pairs)
    {
        //Do nothing with the base object
        if (object_pair.id == base_id)
            continue;

        //Go to the object print position
        GCODE_go_to_position(object_pair.print_object->mesh().center().x(), object_pair.print_object->mesh().center().y());     

        //Lower gripper to bed
        GCODE_go_to_height(0);

        //Close gripper
        GCODE_close_gripper(-10);

        //Vibrate gripper
        GCODE_vibrate_gripper(300, 0.09);

        GCODE_vibrate_gripper(100, 0.11);

        GCODE_vibrate_gripper(5, 0.4, 1000);

        //Open gripper
        GCODE_open_gripper();

        //Return to safe height
        GCODE_go_to_safe_height();      
    }

    //Change to vacuum
    GCODE_change_to_vacuum();

    //Iterate through each of the objects and pick and place them (except for the base object)
    for (ModelObjectPairPtrs object_pair : sorted_object_pairs)
    {
        //Do nothing with the base object
        if (object_pair.id == base_id)
            continue;

        //Go to the object print position
        GCODE_go_to_position(object_pair.print_object->mesh().center().x(), object_pair.print_object->mesh().center().y());

        //Lower vacuum to object top -2 for better pickup
        GCODE_go_to_height(object_pair.print_object->max_z() - 2);

        //Start vacuum
        GCODE_vacuum_on();

        //Go to safe height
        GCODE_go_to_safe_height();

        //Go to object model position
        Slic3r::Vec3d target_position = object_pair.model_object->mesh().center() - base_offset;

        GCODE_go_to_position(target_position.x(), target_position.y());

        //Lower vacuum to object top -2 for better place
        GCODE_go_to_height(object_pair.model_object->max_z() - 2);

        //Stop vacuum
        GCODE_vacuum_off();

        //Go to safe height
        GCODE_go_to_safe_height();
    }

    return m_assembly_commands;
}
