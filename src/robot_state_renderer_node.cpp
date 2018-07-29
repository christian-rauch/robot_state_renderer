#include <StateRenderer.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "StateRenderer");
    if(ros::param::param("~visualise", true))
        StateRenderer(std::to_string(*argv[1])).visualise();
    else
        StateRenderer(std::to_string(*argv[1])).spin();
}
