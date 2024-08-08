#include "parser.hpp"
#include <cstdlib>

Config& config = Config::singleton();

Config::Config() {
    const char* env_filename = getenv("FILEPATH");
    if (!env_filename) {
        std::cerr << "FILEPATH environment variable not set." << std::endl;
        throw std::runtime_error("FILEPATH environment variable not set.");
    }

    auto doc = std::make_unique<pugi::xml_document>();

    try {
        if (doc->load_file(env_filename)) {
            std::cout << "File status\tLoaded XODR" << std::endl;
        } else {
            throw std::runtime_error("Failed to load XML file.");
        }
    } catch (const std::exception& e) {
        std::cerr << "File status\tLoading Failed for XODR: " << e.what() << std::endl;
        throw;
    }
}


void Config::parse() {
    auto start = std::chrono::system_clock::now();
    pugi::xml_document doc;
    const char* env_filename= getenv("FILEPATH");
    #if defined(WIN32)
        doc.load_file("env_filename");
    #else
        doc.load_file(env_filename);
    #endif
    config.num_of_lanes = 0;

    pugi::xml_node root_node = doc.child("OpenDRIVE");
    pugi::xml_node parent_node = root_node.child("road");
    for (pugi::xml_attribute road_attr = parent_node.first_attribute(); road_attr; road_attr = road_attr.next_attribute()) {
        config.road_specs.emplace(road_attr.name(), road_attr.value());
    }

    config.road_length = stod(config.road_specs.at("length"));
    for (pugi::xml_node child = parent_node.first_child(); child; child = child.next_sibling()) {
        std::string map_component = child.name();
        if (map_component == "planView") {
            for (pugi::xml_node grand_child = child.first_child(); grand_child; grand_child = grand_child.next_sibling()) {
                config.num_of_geometeries++;
                std::map<std::string, double> planview_parameters;
                for (pugi::xml_attribute attr = grand_child.first_attribute(); attr; attr = attr.next_attribute()) {
                    planview_parameters.emplace(attr.name(), std::stod(attr.value()));
                }
                config.planeview_data.push_back(planview_parameters);
            }
        } else if (map_component == "lanes") {
            for (pugi::xml_node grand_child = child.first_child(); grand_child; grand_child = grand_child.next_sibling()) {
                for (pugi::xml_node lane_section = grand_child.first_child(); lane_section; lane_section = lane_section.next_sibling()) {
                    std::string lane_position = lane_section.name();
                    if (lane_position == "left") {
                        config.parseLaneSection(lane_section, config.left_lane_attributes, config.left_lane_dimensions_rm, config.left_lanes_frames, 4);
                        config.number_of_left_lanes = config.left_lanes_frames.size();
                    } else if (lane_position == "center") {
                        config.parseLaneSection(lane_section, config.center_lane_attributes, config.center_lane_dimensions_rm);
                    } else if (lane_position == "right") {
                        config.parseLaneSection(lane_section, config.right_lane_attributes, config.right_lane_dimensions_rm, config.right_lanes_frames, 1);
                        config.number_of_right_lanes = config.right_lanes_frames.size();
                    }
                }
            }
        }
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "Computation time of the parser at " << std::ctime(&end_time) << "Elapsed time: " << elapsed_seconds.count() << "s\n";
}


void Config::parseLaneSection(pugi::xml_node& lane_section, 
                              std::vector<std::map<std::string, std::string>>& lane_attributes,
                              std::vector<std::map<std::string, std::string>>& lane_dimensions_rm,
                              std::map<int, std::vector<std::map<std::string, double>>>& lanes_frames, 
                              int start_id) {
    for (pugi::xml_node lane = lane_section.first_child(); lane; lane = lane.next_sibling()) {
        config.num_of_lanes++;
        std::map<std::string, std::string> attributes;
        for (pugi::xml_attribute attr = lane.first_attribute(); attr; attr = attr.next_attribute()) {
            attributes.emplace(attr.name(), attr.value());
        }
        lane_attributes.push_back(attributes);

        std::vector<std::map<std::string, double>> width_tags;
        for (pugi::xml_node lane_feature = lane.first_child(); lane_feature; lane_feature = lane_feature.next_sibling()) {
            std::string feature_name = lane_feature.name();
            if (feature_name == "width") {
                std::map<std::string, double> dimensions;
                for (pugi::xml_attribute attr = lane_feature.first_attribute(); attr; attr = attr.next_attribute()) {
                    dimensions.emplace(attr.name(), std::stod(attr.value()));
                }
                width_tags.push_back(dimensions);
            } else if (feature_name == "roadMark") {
                std::map<std::string, std::string> dimensions;
                for (pugi::xml_attribute attr = lane_feature.first_attribute(); attr; attr = attr.next_attribute()) {
                    dimensions.emplace(attr.name(), attr.value());
                }
                lane_dimensions_rm.push_back(dimensions);
            }
        }
        lanes_frames.emplace(start_id, width_tags);
        start_id--;
    }
}


void Config::parseLaneSection(pugi::xml_node& lane_section, 
                              std::deque<std::map<std::string, std::string>>& lane_attributes,
                              std::map<std::string, std::string>& lane_dimensions_rm) {
    for (pugi::xml_node lane = lane_section.first_child(); lane; lane = lane.next_sibling()) {
        config.num_of_lanes++;
        std::map<std::string, std::string> attributes;
        for (pugi::xml_attribute attr = lane.first_attribute(); attr; attr = attr.next_attribute()) {
            attributes.emplace(attr.name(), attr.value());
        }
        lane_attributes.push_back(attributes);

        for (pugi::xml_node lane_feature = lane.first_child(); lane_feature; lane_feature = lane_feature.next_sibling()) {
            std::string feature_name = lane_feature.name();
            if (feature_name == "roadMark") {
                for (pugi::xml_attribute attr = lane_feature.first_attribute(); attr; attr = attr.next_attribute()) {
                    lane_dimensions_rm.emplace(attr.name(), attr.value());
                }
            }
        }
    }
}


Config& Config::singleton()
{
  static Config instance;
  return instance;
}
