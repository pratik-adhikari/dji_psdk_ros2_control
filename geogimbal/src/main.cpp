// #include <rclcpp/rclcpp.hpp>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include "geogimbal/geo_gimbal_control.hpp"
// #include "geogimbal/gimbal_transform_node.hpp"

// /**
//  * @file main.cpp
//  * @brief Entry point for running the gimbal transformation and control nodes in a ROS2 environment.
//  */

// /**
//  * @struct TargetXYZ
//  * @brief Stores target coordinates (x, y, z) for gimbal tracking.
//  */
// struct TargetXYZ {
//     double x, y, z;
// };

// /**
//  * @brief Main function initializing ROS2 nodes and reading target positions from a CSV file.
//  *
//  * Initializes the ROS2 system, sets up the GimbalTransformNode and GeoGimbalControl nodes,
//  * and reads target positions from a CSV file to set the first available target.
//  *
//  * @param argc Argument count.
//  * @param argv Argument values.
//  * @return Execution status code.
//  */
// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);

//     // Create an executor
//     auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

//     // Create our nodes
//     auto transformNode = std::make_shared<GimbalTransformNode>();
//     auto gimbalControl = std::make_shared<GeoGimbalControl>();

//     // Add them to the executor
//     exec->add_node(transformNode);
//     exec->add_node(gimbalControl);

//     // Read CSV file for target locations
//     std::string csv_path = "data/target_locations.csv";
//     std::ifstream file(csv_path);
//     std::vector<TargetXYZ> targets;
//     if (file.is_open()) {
//         std::string line;
//         while (std::getline(file, line)) {
//             if (line.empty() || line[0] == '#') // Ignore empty lines and comments
//                 continue;
//             std::stringstream ss(line);
//             double x, y, z;
//             if (!(ss >> x)) break;
//             if (ss.peek() == ',' || ss.peek() == ' ') ss.ignore();
//             if (!(ss >> y)) break;
//             if (ss.peek() == ',' || ss.peek() == ' ') ss.ignore();
//             if (!(ss >> z)) z = 0.0; // Default to zero if no z value provided
//             targets.push_back({x, y, z});
//         }
//         file.close();
//     }

//     // Set the first available target for the gimbal control node
//     if (!targets.empty()) {
//         TargetLocation t{targets[0].x, targets[0].y, targets[0].z};
//         gimbalControl->set_target(t);
//     }

//     RCLCPP_INFO(rclcpp::get_logger("main"), "Spinning with transformNode & gimbalControl...");
//     exec->spin();

//     rclcpp::shutdown();
//     return 0;
// }
// #include <iostream>
// #include "geogimbal/coordinate_transformer.hpp"

// int main() {
//     CoordinateTransformer transformer;
    
//     // Example: Setup for WGS84 UTM zone 15N
//     // If you want 32N (EPSG:25832 or EPSG:32632 for WGS84), pass zone=32, northern=true
//     transformer.setup(FrameType::UTM, 15, true, 0.0, 0.0, 0.0);
    
//     // Test #1: WGS84 -> UTM
//     // We'll treat inWgs as (longitude=-93, latitude=42, altitude=300)
//     Coordinate input_wgs;
//     input_wgs.longitude = -93.0;
//     input_wgs.latitude  =  42.0;
//     input_wgs.altitude  = 300.0;
    
//     Coordinate output_utm;
//     transformer.wgs84ToFrame(input_wgs, output_utm);

//     std::cout << "WGS84 to UTM (Zone 15N) results:\n";
//     std::cout << "  Easting:  " << output_utm.longitude << "\n";
//     std::cout << "  Northing: " << output_utm.latitude  << "\n";
//     std::cout << "  Altitude: " << output_utm.altitude  << "\n";

//     // // Test #2: UTM -> WGS84
//     // Coordinate input_utm;
//     // input_utm.longitude = 500000.0;   // easting
//     // input_utm.latitude  = 4649776.0;  // northing
//     // input_utm.altitude  = 250.0;

//     // Coordinate output_wgs;
//     // transformer.frameToWgs84(input_utm, output_wgs);

//     // std::cout << "UTM to WGS84 results:\n";
//     // std::cout << "  Longitude: " << output_wgs.longitude << "\n";
//     // std::cout << "  Latitude:  " << output_wgs.latitude  << "\n";
//     // std::cout << "  Altitude:  " << output_wgs.altitude  << "\n";

//     // Test #3: UTM -> WGS84
//     Coordinate input_utm;
//     input_utm.longitude = 433269.117;   // easting
//     input_utm.latitude  = 5699714.136;  // northing
//     input_utm.altitude  = 0.0;

//     Coordinate output_wgs;
//     transformer.frameToWgs84(input_utm, output_wgs);

//     std::cout << "UTM to WGS84 results:\n";
//     std::cout << "  Longitude: " << output_wgs.longitude << "\n";
//     std::cout << "  Latitude:  " << output_wgs.latitude  << "\n";
//     std::cout << "  Altitude:  " << output_wgs.altitude  << "\n";
    
//     return 0;
// }
/**
 * @file main.cpp
 * @brief An example ROS2 entry point that loads parameters and runs GimbalTransformNode.
 */
 #include <ament_index_cpp/get_package_share_directory.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include "geogimbal/gimbal_transform_node.hpp" // or your actual header
 
 int main(int argc, char** argv)
 {
     rclcpp::init(argc, argv);
 
     // Retrieve the installed config path
     std::string share_dir = ament_index_cpp::get_package_share_directory("geogimbal");
     std::string params_path = share_dir + "/config/parameters.yaml";
 
     // NodeOptions with path to installed parameters file
     rclcpp::NodeOptions options;
     options.arguments({
       "--ros-args",
       "--params-file", params_path
     });
 
     auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
     auto gimbalNode = std::make_shared<GimbalTransformNode>(options);
     executor->add_node(gimbalNode);
 
     RCLCPP_INFO(rclcpp::get_logger("main"), "Spinning GimbalTransformNode...");
     executor->spin();
 
     rclcpp::shutdown();
     return 0;
 }
 