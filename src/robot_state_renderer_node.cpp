#include <StateRenderer.hpp>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "StateRenderer");
    StateRenderer(std::to_string(*argv[1])).run(ros::param::param("~visualise", true));
}
