#include <StateRenderer.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "StateRenderer");
    const std::string urdf_path = (argc > 1) ? std::string(argv[1]) : std::string();
    if(ros::param::param("~visualise", true))
        StateRenderer(urdf_path).visualise();
    else
        StateRenderer(urdf_path).spin();
}
