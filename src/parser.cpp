#include "parser.hpp"
#include <cstdlib>

Config& config = Config::singleton();

Config::Config() {
    pugi::xml_document *doc;
    doc = new pugi::xml_document;
    const char* env_filename= getenv("FILEPATH");
    try {
        #if defined(WIN32)
        if(doc->load_file(env_filename)){
            std::cout<<"File status"<<"\t"<<"Loaded XODR"<<std::endl;
            delete doc;
        }
        else{ 
            delete doc;
            throw 505;
        }
        #else
        if(doc->load_file(env_filename)){
                std::cout<<"File status"<<"\t"<<"Loaded XODR"<<std::endl;
                delete doc;
            }
            else{ 
                delete doc;
                throw 505;
            }
        #endif
    } 
    catch (...) {
     std::cerr<<"File status"<<"\t"<<"Loading Failed for XODR"<<std::endl;
     }
}

void Config::parse() {
    std::chrono::time_point<std::chrono::system_clock> start, end;
    pugi::xml_document doc;
    const char* env_filename= getenv("FILEPATH");
    #if defined(WIN32)
        doc.load_file("env_filename");
    #else
        doc.load_file(env_filename);
    #endif
    config.num_of_lanes = 0;

    pugi::xml_node root_node = doc.child("OpenDRIVE");
    pugi::xml_node parent_node= root_node.first_child().next_sibling();
    for (pugi::xml_attribute road_attr = parent_node.first_attribute(); road_attr; road_attr = road_attr.next_attribute()) {
        std::string road_attr_name = road_attr.name();
        std::string road_attr_val = road_attr.value();
        config.road_specs.insert(std::pair<std::string, std::string>(road_attr_name, road_attr_val));
    }
    config.road_length = stod(config.road_specs.at("length"));
    
    pugi::xml_node child_node = parent_node.first_child();
    for (pugi::xml_node child = parent_node.first_child(); child; child = child.next_sibling()) {
        std::string map_components = child.name();
        if (!map_components.compare("planView")) {
            start = std::chrono::system_clock::now();
            for (pugi::xml_node grand_child = child.first_child(); grand_child; grand_child = grand_child.next_sibling()) {
                config.num_of_geometeries++;
                std::map<std::string, double> planview_parameters;
                for (pugi::xml_attribute attr = grand_child.first_attribute(); attr; attr = attr.next_attribute()) {
                    std::string names = attr.name();
                    std::string values = attr.value();
                    planview_parameters.insert(std::pair<std::string, double>(names, stod(values)));
                }
                config.planeview_data.push_back(planview_parameters);
                planview_parameters.clear();
            }
            end = std::chrono::system_clock::now();
        }
        else if(!map_components.compare("lanes")) {
            for (pugi::xml_node grand_child = child.first_child(); grand_child; grand_child = grand_child.next_sibling()) {
                for (pugi::xml_node grand_grand_child = grand_child.first_child(); grand_grand_child; grand_grand_child = grand_grand_child.next_sibling()) {
                    std::string lane_position=grand_grand_child.name();
                    if(!lane_position.compare("left")) {
                        int id_info = 4;
                        for (pugi::xml_node grand_grand_grand_child = grand_grand_child.first_child(); grand_grand_grand_child; grand_grand_grand_child = grand_grand_grand_child.next_sibling()) {
                            config.num_of_lanes++;
                            for (pugi::xml_attribute lane_attr = grand_grand_grand_child.first_attribute(); lane_attr; lane_attr = lane_attr.next_attribute()) {
                                std::string names = lane_attr.name();
                                std::string values = lane_attr.value();
                                std::map<std::string, std::string> left_lane_parameters;
                                left_lane_parameters.insert(std::pair<std::string, std::string>(names, values));
                                config.left_lane_attributes.push_back(left_lane_parameters); 
                            }
                            std::vector<std::map<std::string, double>> width_tages;
                            for (pugi::xml_node grand_4_child = grand_grand_grand_child.first_child(); grand_4_child; grand_4_child = grand_4_child.next_sibling()){
                                std::string road_dims = grand_4_child.name();
                                if(!road_dims.compare("width")) {
                                    std::map<std::string, double> left_lane_dims;
                                    for (pugi::xml_attribute width_attr = grand_4_child.first_attribute(); width_attr; width_attr = width_attr.next_attribute()) {
                                        std::string names = width_attr.name();
                                        std::string values = width_attr.value();
                                        left_lane_dims.insert(std::pair<std::string, double>(names, stod(values)));
                                    }
                                    width_tages.push_back(left_lane_dims);
                                    left_lane_dims.clear();
                                }
                                if(!road_dims.compare("roadMark")) {
                                    std::map<std::string, std::string> left_lane_dims;
                                    for (pugi::xml_attribute roadmark_attr = grand_4_child.first_attribute(); roadmark_attr; roadmark_attr = roadmark_attr.next_attribute()) {
                                        std::string names = roadmark_attr.name();
                                        std::string values = roadmark_attr.value();
                                        left_lane_dims.insert(std::pair<std::string, std::string>(names, values));
                                    }
                                    config.left_lane_dimentions_rm.push_back(left_lane_dims);
                                    left_lane_dims.clear();
                                }
                            }
                            config.left_lanes_frames.insert(std::pair<int, std::vector<std::map<std::string, double>>>(id_info, width_tages));
                            id_info--;
                        }
                        config.number_of_left_lanes = config.left_lanes_frames.size();
                    }
                    if(!lane_position.compare("center")) {
                        for (pugi::xml_node grand_grand_grand_child = grand_grand_child.first_child(); grand_grand_grand_child; grand_grand_grand_child = grand_grand_grand_child.next_sibling()) {
                            config.num_of_lanes++;
                            for (pugi::xml_attribute lane_attr = grand_grand_grand_child.first_attribute(); lane_attr; lane_attr = lane_attr.next_attribute()) {
                                std::string names = lane_attr.name();
                                std::string values = lane_attr.value();
                                std::map<std::string, std::string> center_lane_parameters;
                                center_lane_parameters.insert(std::pair<std::string, std::string>(names, values));
                                config.center_lane_attributes.push_back(center_lane_parameters); 
                            }
                            for (pugi::xml_node grand_4_child = grand_grand_grand_child.first_child(); grand_4_child; grand_4_child = grand_4_child.next_sibling()){
                                std::string road_dims = grand_4_child.name();
                                if(!road_dims.compare("roadMark")) {
                                    for (pugi::xml_attribute roadmark_attr = grand_4_child.first_attribute(); roadmark_attr; roadmark_attr = roadmark_attr.next_attribute()) {
                                        std::string names = roadmark_attr.name();
                                        std::string values = roadmark_attr.value();
                                        config.center_lane_dimentions_rm.insert(std::pair<std::string, std::string>(names, values));
                                    }
                                }
                            }
                        }
                    }
                    if(!lane_position.compare("right")) {
                        int id_info = 1;
                        for (pugi::xml_node grand_grand_grand_child = grand_grand_child.first_child(); grand_grand_grand_child; grand_grand_grand_child = grand_grand_grand_child.next_sibling()) {
                            config.num_of_lanes++;
                            for (pugi::xml_attribute lane_attr = grand_grand_grand_child.first_attribute(); lane_attr; lane_attr = lane_attr.next_attribute()) {
                                std::string names = lane_attr.name();
                                std::string values = lane_attr.value();
                                std::map<std::string, std::string> right_lane_parameters;
                                right_lane_parameters.insert(std::pair<std::string, std::string>(names, values));
                                config.right_lane_attributes.push_back(right_lane_parameters); 
                            }
                            std::vector<std::map<std::string, double>> width_tages;
                            for (pugi::xml_node grand_4_child = grand_grand_grand_child.first_child(); grand_4_child; grand_4_child = grand_4_child.next_sibling()){
                                std::string road_dims = grand_4_child.name();
                                if(!road_dims.compare("width")) {
                                    std::map<std::string, double> right_lane_dims;
                                    for (pugi::xml_attribute width_attr = grand_4_child.first_attribute(); width_attr; width_attr = width_attr.next_attribute()) {
                                        std::string names = width_attr.name();
                                        std::string values = width_attr.value();
                                        right_lane_dims.insert(std::pair<std::string, double>(names, stod(values)));
                                    }
                                    width_tages.push_back(right_lane_dims);
                                    right_lane_dims.clear();
                                }
                                if(!road_dims.compare("roadMark")) {
                                    std::map<std::string, std::string> right_lane_dims;
                                    for (pugi::xml_attribute roadmark_attr = grand_4_child.first_attribute(); roadmark_attr; roadmark_attr = roadmark_attr.next_attribute()) {
                                        std::string names = roadmark_attr.name();
                                        std::string values = roadmark_attr.value();
                                        right_lane_dims.insert(std::pair<std::string, std::string>(names, values));
                                    }
                                    config.right_lane_dimentions_rm.push_back(right_lane_dims);
                                    right_lane_dims.clear();
                                }
                            }
                            config.right_lane_dimentions.push_back(width_tages);
                            std::map<int, int> right_lane_frames;
                            right_lane_frames.insert(std::pair<int, int>((id_info*(-1)), width_tages.size()));
                            config.right_lanes_frames.push_back(right_lane_frames);
                            id_info++;
                        }
                    }
                }
            }
        }
    }
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "Computation time of the parser at " << std::ctime(&end_time) << "Elapsed time: " << elapsed_seconds.count() << "s\n";
}

Config& Config::singleton()
{
  static Config instance;
  return instance;
}

