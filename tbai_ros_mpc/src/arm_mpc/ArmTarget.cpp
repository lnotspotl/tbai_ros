#include <interactive_markers/interactive_marker_server.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ros/ros.h>
#include <tbai_ros_ocs2/common/RosMsgConversions.h>
#include <tbai_ros_ocs2/mpc_observation.h>
#include <tbai_ros_ocs2/mpc_target_trajectories.h>

using namespace ocs2;

class TargetTrajectoriesPublisher {
   public:
    TargetTrajectoriesPublisher(ros::NodeHandle &nodeHandle, const std::string &topicPrefix)
        : server_("target_marker") {
        // Publisher for target trajectories
        targetPublisher_ = nodeHandle.advertise<tbai_ros_ocs2::mpc_target_trajectories>(topicPrefix + "_mpc_target", 1);

        // Subscriber for observation (to get current time)
        observationSubscriber_ = nodeHandle.subscribe(topicPrefix + "_mpc_observation", 1,
                                                      &TargetTrajectoriesPublisher::observationCallback, this);

        // Create interactive marker
        createInteractiveMarker();
    }

    void createInteractiveMarker() {
        visualization_msgs::InteractiveMarker intMarker;
        intMarker.header.frame_id = "panda_link0";
        intMarker.header.stamp = ros::Time::now();
        intMarker.name = "ee_target";
        intMarker.description = "End Effector Target";
        intMarker.scale = 0.2;

        // Initial position
        intMarker.pose.position.x = 0.5;
        intMarker.pose.position.y = 0.0;
        intMarker.pose.position.z = 0.5;
        intMarker.pose.orientation.w = 1.0;

        // Create a sphere marker for visual feedback
        visualization_msgs::Marker sphereMarker;
        sphereMarker.type = visualization_msgs::Marker::SPHERE;
        sphereMarker.scale.x = 0.08;
        sphereMarker.scale.y = 0.08;
        sphereMarker.scale.z = 0.08;
        sphereMarker.color.r = 1.0;
        sphereMarker.color.g = 0.5;
        sphereMarker.color.b = 0.0;
        sphereMarker.color.a = 0.8;

        visualization_msgs::InteractiveMarkerControl sphereControl;
        sphereControl.always_visible = true;
        sphereControl.markers.push_back(sphereMarker);
        intMarker.controls.push_back(sphereControl);

        // Add 6DOF controls
        visualization_msgs::InteractiveMarkerControl control;

        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        intMarker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        intMarker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        intMarker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        intMarker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        intMarker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        intMarker.controls.push_back(control);

        server_.insert(intMarker, boost::bind(&TargetTrajectoriesPublisher::markerCallback, this, _1));
        server_.applyChanges();
    }

    void markerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
            // Extract position and orientation
            Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
            Eigen::Quaterniond orientation(feedback->pose.orientation.w, feedback->pose.orientation.x,
                                           feedback->pose.orientation.y, feedback->pose.orientation.z);

            // Create target state (7D: position + quaternion)
            vector_t target(7);
            target.head(3) = position;
            target.tail(4) = orientation.coeffs();

            // Create zero input (will be inferred by MPC)
            vector_t zeroInput = vector_t::Zero(7);  // 7 DOF arm

            // Create target trajectories
            TargetTrajectories targetTrajectories({latestTime_}, {target}, {zeroInput});

            // Publish
            auto msg = ros_msg_conversions::createTargetTrajectoriesMsg(targetTrajectories);
            targetPublisher_.publish(msg);
        }
    }

    void observationCallback(const tbai_ros_ocs2::mpc_observation::ConstPtr &msg) { latestTime_ = msg->time; }

   private:
    interactive_markers::InteractiveMarkerServer server_;
    ros::Publisher targetPublisher_;
    ros::Subscriber observationSubscriber_;
    scalar_t latestTime_ = 0.0;
};

int main(int argc, char *argv[]) {
    const std::string robotName = "arm";
    ros::init(argc, argv, robotName + "_target");
    ros::NodeHandle nodeHandle;

    TargetTrajectoriesPublisher targetPublisher(nodeHandle, robotName);

    ROS_INFO("Arm Target: Interactive marker ready. Move the marker in RViz to set targets.");

    ros::spin();

    return 0;
}
