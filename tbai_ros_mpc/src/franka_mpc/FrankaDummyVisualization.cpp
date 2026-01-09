#include <geometry_msgs/PoseArray.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>
#include <ros/package.h>
#include <tbai_mpc/franka_mpc/AccessHelperFunctions.h>
#include <tbai_mpc/franka_mpc/FactoryFunctions.h>
#include <tbai_mpc/franka_mpc/FrankaInterface.h>
#include <tbai_mpc/franka_mpc/FrankaModelInfo.h>
#include <tbai_ros_mpc/franka_mpc/FrankaDummyVisualization.h>
#include <tbai_ros_ocs2/common/RosMsgHelpers.h>
#include <tf/tf.h>
#include <urdf/model.h>
#include <visualization_msgs/MarkerArray.h>

namespace ocs2 {
namespace franka {

FrankaDummyVisualization::FrankaDummyVisualization(ros::NodeHandle &nodeHandle, const FrankaInterface &interface)
    : pinocchioInterface_(interface.getPinocchioInterface()), modelInfo_(interface.getFrankaModelInfo()) {
    launchVisualizerNode(nodeHandle);
}

template <typename It>
void assignHeader(It firstIt, It lastIt, const std_msgs::Header &header) {
    for (; firstIt != lastIt; ++firstIt) {
        firstIt->header = header;
    }
}

template <typename It>
void assignIncreasingId(It firstIt, It lastIt, int startId = 0) {
    for (; firstIt != lastIt; ++firstIt) {
        firstIt->id = startId++;
    }
}

void FrankaDummyVisualization::launchVisualizerNode(ros::NodeHandle &nodeHandle) {
    const std::string urdfName = "robot_description";
    urdf::Model model;
    if (!model.initParam(urdfName)) {
        ROS_ERROR("URDF model load was NOT successful");
    }
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        ROS_ERROR("Failed to extract kdl tree from xml robot description");
    }

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(tree));
    robotStatePublisherPtr_->publishFixedTransforms(true);

    stateOptimizedPublisher_ =
        nodeHandle.advertise<visualization_msgs::MarkerArray>("/franka/optimizedStateTrajectory", 1);
    stateOptimizedPosePublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("/franka/optimizedPoseTrajectory", 1);

    std::string taskFile;
    nodeHandle.getParam("/taskFile", taskFile);
    loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames_, false);
}

void FrankaDummyVisualization::update(const SystemObservation &observation, const PrimalSolution &policy,
                                      const CommandData &command) {
    const ros::Time timeStamp = ros::Time::now();

    publishObservation(timeStamp, observation);
    publishTargetTrajectories(timeStamp, command.mpcTargetTrajectories_);
    publishOptimizedTrajectory(timeStamp, policy);
}

void FrankaDummyVisualization::publishObservation(const ros::Time &timeStamp, const SystemObservation &observation) {
    const auto r_world_base = getBasePosition(observation.state, modelInfo_);
    const Eigen::Quaternion<scalar_t> q_world_base = getBaseOrientation(observation.state, modelInfo_);

    geometry_msgs::TransformStamped base_tf;
    base_tf.header.stamp = timeStamp;
    base_tf.header.frame_id = "panda_link0";
    base_tf.child_frame_id = modelInfo_.baseFrame;
    base_tf.transform.translation = ros_msg_helpers::getVectorMsg(r_world_base);
    base_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(q_world_base);
    tfBroadcaster_.sendTransform(base_tf);

    const auto j_arm = getArmJointAngles(observation.state, modelInfo_);
    std::map<std::string, scalar_t> jointPositions;
    for (size_t i = 0; i < modelInfo_.dofNames.size(); i++) {
        jointPositions[modelInfo_.dofNames[i]] = j_arm(i);
    }
    for (const auto &name : removeJointNames_) {
        jointPositions[name] = 0.0;
    }
    robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
}

void FrankaDummyVisualization::publishTargetTrajectories(const ros::Time &timeStamp,
                                                         const TargetTrajectories &targetTrajectories) {
    const Eigen::Vector3d eeDesiredPosition = targetTrajectories.stateTrajectory.back().head(3);
    Eigen::Quaterniond eeDesiredOrientation;
    eeDesiredOrientation.coeffs() = targetTrajectories.stateTrajectory.back().tail(4);
    geometry_msgs::TransformStamped command_tf;
    command_tf.header.stamp = timeStamp;
    command_tf.header.frame_id = "panda_link0";
    command_tf.child_frame_id = "command";
    command_tf.transform.translation = ros_msg_helpers::getVectorMsg(eeDesiredPosition);
    command_tf.transform.rotation = ros_msg_helpers::getOrientationMsg(eeDesiredOrientation);
    tfBroadcaster_.sendTransform(command_tf);
}

void FrankaDummyVisualization::publishOptimizedTrajectory(const ros::Time &timeStamp, const PrimalSolution &policy) {
    const scalar_t TRAJECTORYLINEWIDTH = 0.005;
    const std::array<scalar_t, 3> red{0.6350, 0.0780, 0.1840};
    const std::array<scalar_t, 3> blue{0, 0.4470, 0.7410};
    const auto &mpcStateTrajectory = policy.stateTrajectory_;

    visualization_msgs::MarkerArray markerArray;

    std::vector<geometry_msgs::Point> baseTrajectory;
    baseTrajectory.reserve(mpcStateTrajectory.size());
    geometry_msgs::PoseArray poseArray;
    poseArray.poses.reserve(mpcStateTrajectory.size());

    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();

    std::vector<geometry_msgs::Point> endEffectorTrajectory;
    endEffectorTrajectory.reserve(mpcStateTrajectory.size());
    std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const Eigen::VectorXd &state) {
        pinocchio::forwardKinematics(model, data, state);
        pinocchio::updateFramePlacements(model, data);
        const auto eeIndex = model.getBodyId(modelInfo_.eeFrame);
        const vector_t eePosition = data.oMf[eeIndex].translation();
        endEffectorTrajectory.push_back(ros_msg_helpers::getPointMsg(eePosition));
    });

    markerArray.markers.emplace_back(
        ros_msg_helpers::getLineMsg(std::move(endEffectorTrajectory), blue, TRAJECTORYLINEWIDTH));
    markerArray.markers.back().ns = "EE Trajectory";

    std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t &state) {
        const auto r_world_base = getBasePosition(state, modelInfo_);
        const Eigen::Quaternion<scalar_t> q_world_base = getBaseOrientation(state, modelInfo_);

        geometry_msgs::Pose pose;
        pose.position = ros_msg_helpers::getPointMsg(r_world_base);
        pose.orientation = ros_msg_helpers::getOrientationMsg(q_world_base);
        baseTrajectory.push_back(pose.position);
        poseArray.poses.push_back(std::move(pose));
    });

    markerArray.markers.emplace_back(ros_msg_helpers::getLineMsg(std::move(baseTrajectory), red, TRAJECTORYLINEWIDTH));
    markerArray.markers.back().ns = "Base Trajectory";

    assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
                 ros_msg_helpers::getHeaderMsg("panda_link0", timeStamp));
    assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());
    poseArray.header = ros_msg_helpers::getHeaderMsg("panda_link0", timeStamp);

    stateOptimizedPublisher_.publish(markerArray);
    stateOptimizedPosePublisher_.publish(poseArray);
}

}  // namespace franka
}  // namespace ocs2
