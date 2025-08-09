#include "GigEManager.hxx"
#include <iostream>
#include <algorithm>
#include <regex>

std::string getNodeName(int argc, char* argv[]){
    const char* target_name{"GigE"};
    char* entry{};
    std::for_each(argv + 1, argv + argc, [target_name, &entry](char* arg)
    {if(std::strstr(arg, target_name) != nullptr) entry = arg;});
    std::regex pattern(std::string(target_name) + "(.*)");
    std::cmatch match;
    return (std::regex_search(entry, match, pattern))?match[0].str():"";
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    std::string node_name(getNodeName(argc, argv));

    std::shared_ptr<GigE> g{std::make_shared<GigE>(std::string(node_name))};
  try {
    g->warmUp();
    g->loopForEver();
    std::clog << "spin" << std::endl;
    rclcpp::spin(g);
  } catch (std::exception &e) {
    std::cerr << "Caught Error " << e.what() << std::endl;
    rclcpp::shutdown();
  }
  rclcpp::shutdown();
  return 0;
}