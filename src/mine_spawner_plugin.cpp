#include <fstream>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/ModelDatabase.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace gazebo
{

using Mine = ignition::math::Vector3d;


class MineSpawnerPlugin : public WorldPlugin
{
public:
    ros::NodeHandle public_nh;
    std::vector<Mine> surface_mines;
    std::vector<Mine> buried_mines;
    physics::HeightmapShapePtr heightmap;

    bool mines_loaded_ = false;


    MineSpawnerPlugin() : WorldPlugin()
    {
        std::string surface_mines_file;
        if (!public_nh.getParam("/minesweepers_gazebo/surface_mines_file", surface_mines_file))
        {
            ROS_ERROR("/minesweepers_gazebo/surface_mines_file parameter not found!");
            return;
        }
        surface_mines = loadMinesFromYAML(surface_mines_file);

        // TODO: Add buried mines
        // std::string buried_mines_file;
        // if (!public_nh.getParam<std::string>("/minesweepers_gazebo/buried_mines_file", buried_mines_file))
        // {
        //     ROS_ERROR("/minesweepers_gazebo/buried_mines_file parameter not found!");
        //     return;
        // }
        // buried_mines = loadMinesFromYAML(buried_mines_file);

    }


    // Check if the YAML document containing the robot info is well formed
    bool checkYAMLIntegrity(YAML::Node mine_doc)
    {
        bool res = true;
        // Check file integrity
        if (!mine_doc["x"])
        {
            std::cerr << "ERROR: Mines file: does not include x tag" << std::endl;
            res = false;
        }
        if (!mine_doc["y"])
        {
            std::cerr << "ERROR: Mines file: does not include y tag" << std::endl;
            res = false;
        }
        return res;
    }

    std::vector<Mine> loadMinesFromYAML(std::string file_name)
    {
        std::vector<Mine> mines;
        // Load the YAML file
        std::vector<YAML::Node> root_doc = YAML::LoadAllFromFile(file_name);
        // Iterate through all documents (one for each robot)
        for (const auto& mine_doc : root_doc)
        {
            if (!checkYAMLIntegrity(mine_doc))
            {
                throw std::invalid_argument("YAML file malformed");
            }
            Mine m;
            m.X() = mine_doc["x"].as<double>();
            m.Y() = mine_doc["y"].as<double>();
            mines.push_back(m);
            std::cout << "Loaded mine " << m << std::endl;
        }
        return mines;
    }

    void spawnSurfaceMines(physics::WorldPtr world)
    {
        // Load SDF string from model file
        std::string mine_model_path = common::ModelDatabase::Instance()->GetModelPath("model://surface_mine");
        std::ifstream fs(mine_model_path + "/surface_mine.sdf");
        std::stringstream ss;
        ss << fs.rdbuf();
        std::string mine_sdf_string = ss.str();

        for (int i = 0; i < surface_mines.size(); ++i)
        {
            // Compute terrain height and add 5cm to have the whole mine body out
            surface_mines[i].Z() = getHeightmapHeight(surface_mines[i].X(), surface_mines[i].Y()) + 0.05;
            std::cout << "Spawning surface mine " << surface_mines[i] << std::endl;

            // Load the SDF from the string
            sdf::SDF mine_sdf;
            mine_sdf.SetFromString(mine_sdf_string);

            // Update the model name
            sdf::ElementPtr mine_model = mine_sdf.Root()->GetElement("model");
            mine_model->GetAttribute("name")->SetFromString("surface_mine_" + std::to_string(i));
            // Update the model pose
            mine_model->GetElement("pose")->Set(ignition::math::Pose3d(surface_mines[i].X(),
                                                                       surface_mines[i].Y(),
                                                                       surface_mines[i].Z(),
                                                                       0, 0, 0));
            // Spawn the model in the world
            world->InsertModelSDF(mine_sdf);
            // std::cout << "Mine sdf: " << mine_sdf.ToString() << std::endl;
        }
    }

    double getHeightmapHeight(double x, double y)
    {
        if (this->heightmap == NULL) return 0.0;
        // Coordinate transformation from regular World (x, y) to the HeightmapShape (index_x, index_y) 
        ignition::math::Vector3d size = this->heightmap->Size(); 
        ignition::math::Vector2i vc = this->heightmap->VertexCount();
        int index_x = (x + size.X() / 2) / size.X() * vc.X() - 1;
        int index_y = (-y + size.Y() / 2) / size.Y() * vc.Y() - 1;

        // Get the height at given index
        return this->heightmap->GetHeight(index_x , index_y);
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading MineSpawnerPlugin...");
        // Make sure the ROS node for Gazebo has already been initialized                                                                                    
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        // Get the heightmap model
        std::string heightmap_model_name;
        if (!_sdf->HasElement("heightmap_model_name"))
        {
                ROS_ERROR("Missing parameter <heightmap_model_name> in MineSpawnerPlugin");
                return;
        }
        else heightmap_model_name = _sdf->GetElement("heightmap_model_name")->Get<std::string>();

        std::cout << "heightmap_model_name: " << heightmap_model_name << std::endl;

        physics::ModelPtr heightmap_model = _world->ModelByName(heightmap_model_name);
        if (heightmap_model != NULL)
        {
            physics::CollisionPtr heightmap_collision = heightmap_model->GetLink("link")->GetCollision("collision");
            this->heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(heightmap_collision->GetShape());
        }

        spawnSurfaceMines(_world);

        ROS_INFO("MineSpawnerPlugin Loaded!");
    }

};

GZ_REGISTER_WORLD_PLUGIN(MineSpawnerPlugin)

} // gazebo namespace
