#include <StateRenderer.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "StateRenderer");
    const std::string urdf_path = (argc > 1) ? std::string(argv[1]) : ros::param::param("~urdf_path", std::string());
    StateRenderer(urdf_path, true, ros::param::param("~visualise", true)).run();
}
