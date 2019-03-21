#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <minesweepers_gazebo/MineMapSolution.h>

#include <yaml-cpp/yaml.h>

namespace gazebo
{

using Mine = ignition::math::Vector3d;
using Vector2i = ignition::math::Vector2i;

class MinesweepersJuryPlugin : public WorldPlugin
{
public:
    std::vector<Mine> surface_mines;
    std::vector<Vector2i> surface_mine_cells;
    std::vector<Mine> buried_mines;
    std::vector<Vector2i> buried_mine_cells;
    
    ros::NodeHandle public_nh;
    ros::Subscriber solution_sub;
    tf::TransformListener tf_listener;

    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;

    common::Time update_time = common::Time(0.1);
    common::Time previous_time;

    Vector2i minefield_origin;
    Vector2i minefield_size;

    MinesweepersJuryPlugin() : WorldPlugin(), tf_listener(public_nh)
    {
        std::string surface_mines_file;
        if (!public_nh.getParam("/minesweepers_gazebo/surface_mines_file", surface_mines_file))
        {
            ROS_FATAL("/minesweepers_gazebo/surface_mines_file parameter not found!");
            return;
        }
        LoadMinesFromYAML(surface_mines_file);

        // TODO: Add buried mines
        // std::string buried_mines_file;
        // if (!public_nh.getParam<std::string>("/minesweepers_gazebo/buried_mines_file", buried_mines_file))
        // {
        //     ROS_FATAL("/minesweepers_gazebo/buried_mines_file parameter not found!");
        //     return;
        // }
        // buried_mines = LoadMinesFromYAML(buried_mines_file);

        solution_sub = public_nh.subscribe("/minesweepers_gazebo/mine_map_solution", 2,
                                           &MinesweepersJuryPlugin::SolutionCallback, this);
    }


    // Check if the YAML document containing the mine info is well formed
    bool CheckMineYAMLIntegrity(YAML::Node mine_doc)
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

    void LoadMinesFromYAML(std::string file_name)
    {
        this->surface_mines.clear();
        // Load the YAML file
        std::vector<YAML::Node> root_doc = YAML::LoadAllFromFile(file_name);
        // Iterate through all documents (one for each robot)
        for (const auto& mine_doc : root_doc)
        {
            if (!CheckMineYAMLIntegrity(mine_doc))
            {
                throw std::invalid_argument("YAML file malformed");
            }

            Mine m;
            m.X() = mine_doc["x"].as<double>();
            m.Y() = mine_doc["y"].as<double>();
            this->surface_mines.push_back(m);

            Vector2i mine_cell;
            mine_cell.X() = (int)std::floor(m.X()) - minefield_origin.X();
            mine_cell.Y() = (int)std::floor(m.Y()) - minefield_origin.Y();
            this->surface_mine_cells.push_back(mine_cell);

            std::cout << "Loaded mine " << m << std::endl;
        }
    }

    void UpdateMines()
    {
        for (int i = 0; i < surface_mines.size(); ++i)
        {
            physics::ModelPtr mine_model = world->ModelByName("surface_mine_" + std::to_string(i));
            if (mine_model != NULL)
            {
                this->surface_mines[i].X() = mine_model->WorldPose().Pos().X();
                this->surface_mines[i].Y() = mine_model->WorldPose().Pos().Y();
                this->surface_mines[i].Z() = mine_model->WorldPose().Pos().Z();
                // std::cout << surface_mines[i] << std::endl;
                this->surface_mine_cells[i].X() = (int)std::floor(surface_mines[i].X()) - minefield_origin.X();
                this->surface_mine_cells[i].Y() = (int)std::floor(surface_mines[i].Y()) - minefield_origin.Y();
            }
        }
    }

    void OnUpdate()
    {
        if (common::Time::GetWallTime() - this->previous_time > this->update_time)
        {
            UpdateMines();

            this->previous_time = common::Time::GetWallTime();
        }
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading MinesweepersJuryPlugin...");

        // Make sure the ROS node for Gazebo has already been initialized                                                                                    
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        // Get field origin and size
        if (!_sdf->HasElement("minefield_origin"))
        {
                ROS_WARN("Missing parameter <minefield_origin> in MetalDetectorPlugin. Default to (0, 0)");
                minefield_origin = Vector2i(0, 0);
        }
        else minefield_origin = _sdf->GetElement("minefield_origin")->Get<Vector2i>();
        ROS_INFO_STREAM("minefield_origin: " << minefield_origin);
        if (!_sdf->HasElement("minefield_size"))
        {
                ROS_WARN("Missing parameter <minefield_size> in MetalDetectorPlugin. Default to (20, 20)");
                minefield_size = Vector2i(20, 20);
        }
        else minefield_size = _sdf->GetElement("minefield_size")->Get<Vector2i>();
        ROS_INFO_STREAM("minefield_size: " << minefield_size);

        this->world = _world;

        // Set the previous update time to the load time
        this->previous_time = common::Time::GetWallTime();

        // Listen to the update event from the simulator
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MinesweepersJuryPlugin::OnUpdate, this));

        ROS_INFO("MinesweepersJuryPlugin Loaded!");
    }

    void SolutionCallback(const minesweepers_gazebo::MineMapSolutionConstPtr& solution_msg)
    {
        ROS_INFO_STREAM("New solution submitted");
        ROS_INFO("Surface mines:");
        double score = 0.0;
        for (const auto& mine_detection : solution_msg->surface_mines)
        {
            ROS_INFO_STREAM("Detection at [" << (int)mine_detection.x << ", " << (int)mine_detection.y << "]");
            if (mine_detection.x < 0 || mine_detection.x >= minefield_size.X())
            {
                ROS_WARN("X coordinate out if field");
            }
            if (mine_detection.y < 0 || mine_detection.y >= minefield_size.Y())
            {
                ROS_WARN("Y coordinate out if field");
            }

            bool good_detection = false;
            for (const auto mine : surface_mine_cells)
            {
                if (mine.X() == mine_detection.x && mine.Y() == mine_detection.y)
                {
                    good_detection = true;
                    break;
                }
            }
            if (good_detection)
            {
                ROS_INFO("Successful detection of surface mine!");
                score += 10;
            }
            else
            {
                ROS_INFO("Wrong detection. False positive!");
                score -= 2;
            }
        }
        ROS_INFO_STREAM("TOTAL SCORE: " << score);
    }
};

GZ_REGISTER_WORLD_PLUGIN(MinesweepersJuryPlugin)

} // gazebo namespace
