// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on

#include <memory>

#include "tbai_ros_mpc/MpcController.hpp"
#include "tbai_ros_static/StaticController.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tbai_core/Logging.hpp>
#include <tbai_core/Utils.hpp>
#include <tbai_core/config/Config.hpp>
#include <tbai_core/control/CentralController.hpp>
#include <tbai_deploy_go2/Go2RobotInterface.hpp>
#include <tbai_ros_core/Publishers.hpp>
#include <tbai_ros_core/Rate.hpp>
#include <tbai_ros_core/Subscribers.hpp>
#include <tbai_ros_deploy_go2_mpc/Go2Joystick.hpp>

class Go2RobotInterfaceWithLidar : public tbai::Go2RobotInterface {
   public:
    Go2RobotInterfaceWithLidar(const tbai::Go2RobotInterfaceArgs &args) : tbai::Go2RobotInterface(args) {
        if (args.subscribeLidar()) {
            ros::NodeHandle nh;
            lidarPublisher = nh.advertise<sensor_msgs::PointCloud2>("unitree_lidar_points", 1);
        }
    }

    void lidarCallback(const void *msg2) override {
        auto *msg = reinterpret_cast<const sensor_msgs::msg::dds_::PointCloud2_ *>(msg2);

        // Create a sensor_msgs::PointCloud2 message from the input
        sensor_msgs::PointCloud2 pc2_msg;

        // Use getter methods to access fields from the DDS message
        pc2_msg.header.stamp = ros::Time(msg->header().stamp().sec(), msg->header().stamp().nanosec());
        pc2_msg.header.frame_id = "radar";
        pc2_msg.height = msg->height();
        pc2_msg.width = msg->width();

        // Convert fields (std::vector-like) to ROS fields
        pc2_msg.fields.resize(msg->fields().size());
        for (size_t i = 0; i < msg->fields().size(); ++i) {
            const auto &field = msg->fields()[i];
            pc2_msg.fields[i].name = field.name();
            pc2_msg.fields[i].offset = field.offset();
            pc2_msg.fields[i].datatype = field.datatype();
            pc2_msg.fields[i].count = field.count();
        }

        pc2_msg.is_bigendian = msg->is_bigendian();
        pc2_msg.point_step = msg->point_step();
        pc2_msg.row_step = msg->row_step();
        pc2_msg.is_dense = msg->is_dense();

        pc2_msg.data = std::move(const_cast<std::vector<uint8_t> &>(msg->data()));

        // Publish the message
        lidarPublisher.publish(pc2_msg);
    }

   private:
    ros::Publisher lidarPublisher;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tbai_ros_deploy_go2_mpc");
    ros::NodeHandle nh;

    // Set zero time
    tbai::writeInitTime(tbai::RosTime::rightNow());

    // Initialize Go2RobotInterface
    std::shared_ptr<tbai::Go2RobotInterface> go2RobotInterface =
        std::shared_ptr<tbai::Go2RobotInterface>(new Go2RobotInterfaceWithLidar(
            tbai::Go2RobotInterfaceArgs()
                .networkInterface(tbai::getEnvAs<std::string>("TBAI_GO2_NETWORK_INTERFACE", true, "eth0"))
                .subscribeLidar(tbai::getEnvAs<bool>("TBAI_GO2_PUBLISH_LIDAR", true, false))));

    std::shared_ptr<tbai::StateSubscriber> stateSubscriber = go2RobotInterface;
    std::shared_ptr<tbai::CommandPublisher> commandPublisher = go2RobotInterface;

    auto changeControllerTopic = tbai::fromGlobalConfig<std::string>("change_controller_topic");

    std::shared_ptr<tbai::ChangeControllerSubscriber> changeControllerSubscriber =
        std::shared_ptr<tbai::ChangeControllerSubscriber>(
            new tbai::RosChangeControllerSubscriber(nh, changeControllerTopic));

    tbai::CentralController<ros::Rate, tbai::RosTime> controller(commandPublisher, changeControllerSubscriber);

    // Add static controller
    controller.addController(std::make_unique<tbai::static_::RosStaticController>(stateSubscriber));

    std::string urdfString = nh.param<std::string>("robot_description", "");

    if (urdfString.empty()) {
        throw std::runtime_error("Failed to get param /robot_description");
    }

    auto referenceGeneratorType = tbai::fromGlobalConfig<std::string>("reference_generator/type");

    std::shared_ptr<tbai::reference::ReferenceVelocityGenerator> referenceVelocityPtr;
    if (referenceGeneratorType == "go2_joystick") {
        auto joystick = tbai::reference::getGo2JoystickShared(nh);
        joystick->Start();
        referenceVelocityPtr = joystick;
    } else if (referenceGeneratorType == "joystick") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else if (referenceGeneratorType == "twist") {
        referenceVelocityPtr = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    } else {
        TBAI_THROW("Invalid reference generator type: {}. Supported types are: go2_joystick, joystick, twist",
                   referenceGeneratorType);
    }

    // Add MPC controller
    controller.addController(std::make_unique<tbai::mpc::MpcController>(stateSubscriber, referenceVelocityPtr));

    // Start controller loop
    controller.start();

    return EXIT_SUCCESS;
}