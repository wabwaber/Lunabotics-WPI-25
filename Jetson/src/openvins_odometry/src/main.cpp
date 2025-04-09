#include <stdlib.h>
#include "/catkin_ws/open_vins/ov_msckf/src/core/VioManager.h"

#include <string.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ov_msckf/src/core/VioManager.h"
#include "ov_msckf/src/ros/ROS2Visualizer.h"
#include "core/VioManagerOptions.h"
#include <iostream>
using namespace ov_msckf;

std::shared_ptr<VioManager> sys; //openVINS system
std::shared_ptr<ROS2Visualizer> viz; //visualization

int main(int argc, char** argv){
    //here down was taken from openVINS's run_subscribe_msckf.cpp file and modified to work for our purposes
    std::string config_path = "unset_path_to_config.yaml";
    if(argc > 1){
        config_path = argv[1];
    }
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("run_subscribe_msckf", options);
    node->get_parameter<std::string>("config_path", config_path);

    auto parser = std::make_shared<ov_core::YamlParser>(config_path);
    parser->set_node(node);
    std::string verbosity = "DEBUG";
    parser->parse_config("Verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);

    VioManagerOptions params;
    params.print_and_load(parser);
    params.use_multi_threading_subs = true;
    sys = std::make_shared<VioManager>(params);
    viz = std::make_shared<ROS2Visualizer>(node, sys);
    viz->setup_subscribers(parser);
    if(!parser->successful){
        PRINT_ERROR(RED "unable to parse all parameters, please fix\n")
        std::exit(EXIT_FAILURE);
    }
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    viz->visualize_final();
    rclcpp::shutdown();
    return EXIT_SUCCESS
    //end of OpenVINS code


}