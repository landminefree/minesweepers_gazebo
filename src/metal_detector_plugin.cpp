#include <cmath>
#include <random>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <metal_detector_msgs/Coil.h>

#include <yaml-cpp/yaml.h>

namespace gazebo
{

using Mine = ignition::math::Vector3d;


class MetalDetectorPlugin : public WorldPlugin
{
public:
    std::vector<Mine> surface_mines;
    std::vector<Mine> buried_mines;
    std::map<std::string, std::vector<std::string>> coil_frames;
    std::map<std::string, ros::Publisher> coil_pubs;

    // Mine measurement parameters
    double min_mine_distance_threshold = 1.0;
    double measurement_noise_stddev = 0.1;

    // Random engine
    std::random_device rd;
    std::mt19937 random_generator;
    
    ros::NodeHandle public_nh;
    tf::TransformListener tf_listener;

    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;

    common::Time update_time;
    common::Time previous_time;

    MetalDetectorPlugin() : WorldPlugin(), random_generator(rd()), tf_listener(public_nh)
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
        std::string coil_config_file;
        if (!public_nh.getParam("/minesweepers_gazebo/coil_config_file", coil_config_file))
        {
            ROS_FATAL("/minesweepers_gazebo/coil_config_file parameter not found!");
            return;
        }
        LoadCoilConfig(coil_config_file);
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

    // Check if the YAML document containing the coils info is well formed
    bool CheckCoilYAMLIntegrity(YAML::Node coil_doc)
    {
        bool res = true;
        // Check file integrity
        if (!coil_doc["robot_frame"])
        {
            std::cerr << "ERROR: Mines file: does not include robot_frame tag" << std::endl;
            res = false;
        }
        if (!coil_doc["coil_frames"])
        {
            std::cerr << "ERROR: Mines file: does not include coil_frames tag" << std::endl;
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
            std::cout << "Loaded mine " << m << std::endl;
        }
    }

    void LoadCoilConfig(std::string file_name)
    {
        this->coil_frames.clear();
        // Load the YAML file
        std::vector<YAML::Node> root_doc = YAML::LoadAllFromFile(file_name);
        // Iterate through all documents (one for each robot)
        for (const auto& robot_doc : root_doc)
        {
            if (!CheckCoilYAMLIntegrity(robot_doc))
            {
                throw std::invalid_argument("YAML file malformed");
            }
            std::string robot_frame = robot_doc["robot_frame"].as<std::string>();
            std::cout << "Robot frame: " << robot_frame << std::endl;
            coil_frames[robot_frame] = std::vector<std::string>();
            for (const auto& coil_doc : robot_doc["coil_frames"])
            {
                std::string coil_frame = coil_doc.as<std::string>();
                std::cout << "\t" << coil_frame << std::endl;
                // Save the frame
                coil_frames[robot_frame].push_back(coil_frame);
                // Create this coil publisher
                std::string topic = "/metal_detector_simulator/" + robot_frame + "/" + coil_frame;
                coil_pubs[topic] = public_nh.advertise<metal_detector_msgs::Coil>(topic, 5);
            }
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
            }
        }
    }

    double MetalMeasurement(ignition::math::Vector3d coil_pose)
    {
        // Init gaussian random distributions
        std::normal_distribution<> gauss_distribution(0.0, measurement_noise_stddev);

        double measurement = 0.0;
        // Find the closest mine to the coil
        double closest_distance = 9999999.9;
        int closest_index = -1;
        for (int i = 0; i < surface_mines.size(); ++i)
        {
            double distance = coil_pose.Distance(surface_mines[i]);
            if (distance < closest_distance)
            {
                closest_distance = distance;
                closest_index = i;
            }
        }

        // Apply measurement function based on distance
        if (closest_distance < min_mine_distance_threshold)
        {
            measurement = std::exp(- (0.5 * closest_distance * closest_distance) / (2 * measurement_noise_stddev * measurement_noise_stddev)) / (measurement_noise_stddev * std::sqrt(2*M_PI));
        }

        measurement += gauss_distribution(MetalDetectorPlugin::random_generator);
        // Limit value to [0, 1]
        measurement = std::min(measurement, 1.0);
        measurement = std::max(measurement, 0.0);
        return measurement;
    }

    void OnUpdate()
    {
        if (common::Time::GetWallTime() - this->previous_time > this->update_time)
        {
            UpdateMines();
            ros::Time now = ros::Time::now();
            for (const auto& coil_config : coil_frames)
            {
                std::string robot_frame = coil_config.first;
                std::vector<std::string> coil_frames = coil_config.second;

                // Find the model with this robot_frame link
                physics::ModelPtr robot_model;
                for (const auto& model : this->world->Models())
                {
                    for (const auto& link : model->GetLinks())
                    {
                        if (link->GetName() == robot_frame)
                        {
                            robot_model = model;
                            break;
                        }
                    }
                    if (robot_model != NULL) break;
                }
                if (robot_model == NULL)
                {
                    ROS_ERROR_STREAM("Could not find model with link " << robot_frame << ". Skipping robot...");
                    continue;
                }

                for (const auto& coil_frame : coil_frames)
                {
                    tf::StampedTransform transform;
                    try
                    {
                        tf_listener.lookupTransform(robot_frame, coil_frame, ros::Time(0), transform);
                    }
                    catch (tf::TransformException exception)
                    {
                        ROS_ERROR_STREAM(exception.what());
                        ROS_ERROR_STREAM("Cannot transform coil" << coil_frame << ". Skipping measurement...");
                        continue;
                    }
                    ignition::math::Vector3d coil_robot_pose(transform.getOrigin().x(),
                                                       transform.getOrigin().y(),
                                                       transform.getOrigin().z());
                    // ROS_INFO_STREAM("Coil " << coil_frame << " pose: " << coil_robot_pose);
                    ignition::math::Vector3d coil_world_pose = robot_model->WorldPose().Pos() + coil_robot_pose;
                    double measurement = MetalMeasurement(coil_world_pose);
                    // ROS_INFO_STREAM("Coil measurement: " << measurement);

                    // Publish the measurement
                    metal_detector_msgs::Coil coil_msg;
                    coil_msg.header.frame_id = coil_frame;
                    coil_msg.header.stamp = now;
                    coil_msg.left_coil = measurement;
                    coil_msg.right_coil = measurement;
                    coil_pubs["/metal_detector_simulator/" + robot_frame + "/" + coil_frame].publish(coil_msg);
                }
            }

            this->previous_time = common::Time::GetWallTime();
        }
    }

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        ROS_INFO("Loading MetalDetectorPlugin...");
        // Make sure the ROS node for Gazebo has already been initialized                                                                                    
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        // Get update rate
        if (!_sdf->HasElement("update_rate"))
        {
                ROS_WARN("Missing parameter <update_rate> in MetalDetectorPlugin. Default to 30Hz");
                update_time = common::Time(1.0 / 30.0);
                return;
        }
        else update_time = common::Time(1.0 / _sdf->GetElement("update_rate")->Get<double>());
        ROS_INFO_STREAM("update_rate: " << 1.0 / update_time.Double());

        // Set the previous update time to the load time
        this->previous_time = common::Time::GetWallTime();

        // Get minimum mine distance threshold parameter
        if (!_sdf->HasElement("min_mine_distance_threshold"))
        {
                ROS_WARN("Missing parameter <min_mine_distance_threshold> in MetalDetectorPlugin. Default to 1m");
                min_mine_distance_threshold = 1.0;
        }
        else min_mine_distance_threshold = _sdf->GetElement("min_mine_distance_threshold")->Get<double>();
        ROS_INFO_STREAM("min_mine_distance_threshold: " << min_mine_distance_threshold);

        // Get noise stddev
        if (!_sdf->HasElement("measurement_noise_stddev"))
        {
                ROS_WARN("Missing parameter <measurement_noise_stddev> in MetalDetectorPlugin. Default to 0.1m");
                measurement_noise_stddev = 0.1;
        }
        else measurement_noise_stddev = _sdf->GetElement("measurement_noise_stddev")->Get<double>();
        ROS_INFO_STREAM("measurement_noise_stddev: " << measurement_noise_stddev);

        this->world = _world;

        // Listen to the update event from the simulator
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MetalDetectorPlugin::OnUpdate, this));

        ROS_INFO("MetalDetectorPlugin Loaded!");
    }

};

GZ_REGISTER_WORLD_PLUGIN(MetalDetectorPlugin)

} // gazebo namespace
