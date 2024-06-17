#include "tbai_mpc/reference/ReferenceTrajectoryGenerator.hpp"

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/terrain/PlaneFitting.h>
#include <ocs2_switched_model_msgs/local_terrain.h>
#include <ocs2_anymal_commands/TerrainAdaptation.h>
#include <grid_map_filters_rsl/lookup.hpp>
#include <tbai_core/Throws.hpp>
#include <tbai_core/config/YamlConfig.hpp>
namespace tbai {
namespace mpc {
namespace reference {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
LocalTerrainEstimator::LocalTerrainEstimator() {
    // Get ros node handle
    ros::NodeHandle nodeHandle;

    // Load robot description - urdf
    std::string urdfString;
    TBAI_ROS_THROW_IF(!nodeHandle.getParam("/robot_description", urdfString),
                      "Failed to get robot description from parameter server.");

    // load frame declaration file
    std::string frameDeclarationFile;
    TBAI_ROS_THROW_IF(!nodeHandle.getParam("/frame_declaration_file", frameDeclarationFile),
                      "Failed to get frame declaration file from parameter server.");

    // Load kinematics model
    kinematicsModel_ = getAnymalKinematics(anymal::frameDeclarationFromFile(frameDeclarationFile), urdfString);

    lastFootholds_.resize(4);
    for (size_t i = 0; i < 4; i++) {
        lastFootholds_[i] = vector3_t::Zero();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void LocalTerrainEstimator::updateFootholds(const ocs2::SystemObservation &observation) {
    // Base position
    auto basePose = getBasePose(observation.state);

    // Joint positions
    auto jointPositions = getJointPositions(observation.state);

    // Compute forward kinematics
    auto footholds = kinematicsModel_->feetPositionsInOriginFrame(basePose, jointPositions);

    // contact flags
    contact_flag_t contactFlags = modeNumber2StanceLeg(observation.mode);

    // Update last footholds
    for (size_t i = 0; i < 4; i++) {
        if (contactFlags[i]) {
            lastFootholds_[i] = footholds[i];
        }
    }

    updateLocalTerrainEstimate(lastFootholds_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void LocalTerrainEstimator::updateLocalTerrainEstimate(const std::vector<vector3_t> &footholds) {
    const auto normalAndPosition = estimatePlane(footholds);
    terrainPlane_ = TerrainPlane(normalAndPosition.position,
                                 orientationWorldToTerrainFromSurfaceNormalInWorld(normalAndPosition.normal));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ReferenceTrajectoryGenerator::ReferenceTrajectoryGenerator(const std::string &targetCommandFile, ros::NodeHandle &nh)
    : firstObservationReceived_(false) {
    using tbai::core::fromRosConfig;

    defaultJointState_.setZero(12);
    loadSettings(targetCommandFile);

    ros::NodeHandle nodeHandle;

    // Load robot description - urdf
    std::string urdfString;
    TBAI_ROS_THROW_IF(!nodeHandle.getParam("/robot_description", urdfString),
                      "Failed to get robot description from parameter server.");

    // load frame declaration file
    std::string frameDeclarationFile;
    TBAI_ROS_THROW_IF(!nodeHandle.getParam("/frame_declaration_file", frameDeclarationFile),
                      "Failed to get frame declaration file from parameter server.");

    // Load kinematics model
    kinematicsModel_ = getAnymalKinematics(anymal::frameDeclarationFromFile(frameDeclarationFile), urdfString);

    trajdt_ = fromRosConfig<scalar_t>("mpc_controller/reference_trajectory/traj_dt");
    trajKnots_ = fromRosConfig<size_t>("mpc_controller/reference_trajectory/traj_knots");

    // Setup ROS subscribers
    auto observationTopic = fromRosConfig<std::string>("mpc_controller/reference_trajectory/observation_topic");
    observationSubscriber_ =
        nh.subscribe(observationTopic, 1, &ReferenceTrajectoryGenerator::observationCallback, this);

    auto terrainTopic = fromRosConfig<std::string>("mpc_controller/reference_trajectory/terrain_topic");
    terrainSubscriber_ = nh.subscribe(terrainTopic, 1, &ReferenceTrajectoryGenerator::terrainCallback, this);

    // Setup ROS publishers
    auto referenceTopic = fromRosConfig<std::string>("mpc_controller/reference_trajectory/reference_topic");
    referencePublisher_ = nh.advertise<ocs2_msgs::mpc_target_trajectories>(referenceTopic, 1, false);

    blind_ = fromRosConfig<bool>("mpc_controller/reference_trajectory/blind");

    velocityGeneratorPtr_ = tbai::reference::getReferenceVelocityGeneratorUnique(nh);
    terrainPublisher_ = nh.advertise<ocs2_switched_model_msgs::local_terrain>("/local_terrain", 1, false);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/

void ReferenceTrajectoryGenerator::publishReferenceTrajectory() {
    if (!firstObservationReceived_) {
        ROS_WARN_THROTTLE(1.0, "No observation received yet. Cannot publish reference trajectory.");
        return;
    }

    // Generate reference trajectory
    ocs2::TargetTrajectories referenceTrajectory = generateReferenceTrajectory(ros::Time::now().toSec(), 0.0);

    // Publish reference trajectory
    referencePublisher_.publish(ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(referenceTrajectory));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceHorizon ReferenceTrajectoryGenerator::getBaseReferenceHorizon() {
    return {trajdt_, trajKnots_};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceCommand ReferenceTrajectoryGenerator::getBaseReferenceCommand(scalar_t time) {
    auto velocityCommand = velocityGeneratorPtr_->getReferenceVelocity(time, trajdt_);
    return {velocityCommand.velocity_x, velocityCommand.velocity_y, velocityCommand.yaw_rate, comHeight_};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceState ReferenceTrajectoryGenerator::getBaseReferenceState() {
    std::lock_guard<std::mutex> lock(observationMutex_);
    scalar_t observationTime = latestObservation_.time;
    Eigen::Vector3d positionInWorld = latestObservation_.state.segment<3>(3);
    Eigen::Vector3d eulerXyz = latestObservation_.state.head<3>();
    return {observationTime, positionInWorld, eulerXyz};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
const TerrainPlane &ReferenceTrajectoryGenerator::getTerrainPlane() const {
    return localTerrainEstimator_.getPlane();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
Base2dReferenceTrajectory generate2DExtrapolatedBaseReferencem(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                              const BaseReferenceCommand& command) {
  const double dt = horizon.dt;

  Base2dReferenceTrajectory baseRef;
  baseRef.time.reserve(horizon.N);
  baseRef.yaw.reserve(horizon.N);
  baseRef.positionInWorld.reserve(horizon.N);

  baseRef.time.push_back(initialState.t0);

  // Project position and orientation to horizontal plane
  baseRef.positionInWorld.emplace_back(initialState.positionInWorld.x(), initialState.positionInWorld.y());
  baseRef.yaw.emplace_back(alignDesiredOrientationToTerrain(initialState.eulerXyz, TerrainPlane()).z());

  for (int k = 1; k < horizon.N; ++k) {
    baseRef.time.push_back(baseRef.time.back() + dt);

    scalar_t alpha = 1.0/static_cast<scalar_t>(horizon.N) * static_cast<scalar_t>(k);

    // Advance position
    vector2_t velocity = {command.headingVelocity, command.lateralVelocity};
    baseRef.positionInWorld.push_back(baseRef.positionInWorld.front() + alpha * velocity);

    // Advance orientation
    baseRef.yaw.push_back(baseRef.yaw.front() + alpha * command.yawRate);
  }

  return baseRef;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void addVelocitiesFromFiniteDifferencem(BaseReferenceTrajectory& baseRef) {
  auto N = baseRef.time.size();
  if (N <= 1) {
    return;
  }

  baseRef.linearVelocityInWorld.clear();
  baseRef.angularVelocityInWorld.clear();
  baseRef.linearVelocityInWorld.reserve(N);
  baseRef.angularVelocityInWorld.reserve(N);

  for (int k = 0; (k + 1) < baseRef.time.size(); ++k) {
    auto dt = baseRef.time[k + 1] - baseRef.time[k];
    baseRef.angularVelocityInWorld.push_back(rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[k + 1], baseRef.eulerXyz[k]) / dt);
    baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[k + 1] - baseRef.positionInWorld[k]) / dt);
  }

  auto dt = baseRef.time[N - 1] - baseRef.time[N - 2];
  baseRef.angularVelocityInWorld.push_back(rotationErrorInWorldEulerXYZ(baseRef.eulerXyz[N - 1], baseRef.eulerXyz[N - 2]) / dt);
  baseRef.linearVelocityInWorld.push_back((baseRef.positionInWorld[N - 1] - baseRef.positionInWorld[N - 2]) / dt);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceTrajectory generateExtrapolatedBaseReferencem(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                          const BaseReferenceCommand& command, const grid_map::GridMap& gridMap,
                                                          double nominalStanceWidthInHeading, double nominalStanceWidthLateral) {
  const auto& baseReferenceLayer = gridMap.get("smooth_planar");

  // Helper to get a projected heading frame derived from the terrain.
  auto getLocalHeadingFrame = [&](const vector2_t& baseXYPosition, scalar_t yaw) {
    vector2_t lfOffset(nominalStanceWidthInHeading / 2.0, nominalStanceWidthLateral / 2.0);
    vector2_t rfOffset(nominalStanceWidthInHeading / 2.0, -nominalStanceWidthLateral / 2.0);
    vector2_t lhOffset(-nominalStanceWidthInHeading / 2.0, nominalStanceWidthLateral / 2.0);
    vector2_t rhOffset(-nominalStanceWidthInHeading / 2.0, -nominalStanceWidthLateral / 2.0);
    // Rotate from heading to world frame
    rotateInPlace2d(lfOffset, yaw);
    rotateInPlace2d(rfOffset, yaw);
    rotateInPlace2d(lhOffset, yaw);
    rotateInPlace2d(rhOffset, yaw);
    // shift by base center
    lfOffset += baseXYPosition;
    rfOffset += baseXYPosition;
    lhOffset += baseXYPosition;
    rhOffset += baseXYPosition;

    auto interp = [&](const vector2_t& offset) -> vector3_t {
      auto projection = grid_map::lookup::projectToMapWithMargin(gridMap, grid_map::Position(offset.x(), offset.y()));

      try {
        auto z = gridMap.atPosition("smooth_planar", projection, grid_map::InterpolationMethods::INTER_NEAREST);
        return vector3_t(offset.x(), offset.y(), z);
      } catch (std::out_of_range& e) {
        double interp = gridMap.getResolution() / (projection - gridMap.getPosition()).norm();
        projection = (1.0 - interp) * projection + interp * gridMap.getPosition();
        auto z = gridMap.atPosition("smooth_planar", projection, grid_map::InterpolationMethods::INTER_NEAREST);
        return vector3_t(offset.x(), offset.y(), z);
      }
    };

    vector3_t lfVerticalProjection = interp(lfOffset);
    vector3_t rfVerticalProjection = interp(rfOffset);
    vector3_t lhVerticalProjection = interp(lhOffset);
    vector3_t rhVerticalProjection = interp(rhOffset);

    const auto normalAndPosition = estimatePlane({lfVerticalProjection, rfVerticalProjection, lhVerticalProjection, rhVerticalProjection});

    TerrainPlane terrainPlane(normalAndPosition.position, orientationWorldToTerrainFromSurfaceNormalInWorld(normalAndPosition.normal));
    return getProjectedHeadingFrame({0.0, 0.0, yaw}, terrainPlane);
  };


  auto reference2d = generate2DExtrapolatedBaseReferencem(horizon, initialState, command);

  BaseReferenceTrajectory baseRef;
  baseRef.time = std::move(reference2d.time);
  baseRef.eulerXyz.reserve(horizon.N);
  baseRef.positionInWorld.reserve(horizon.N);

  // Adapt poses
  for (int k = 0; k < horizon.N; ++k) {
    const auto projectedHeadingFrame = getLocalHeadingFrame(reference2d.positionInWorld[k], reference2d.yaw[k]);

    baseRef.positionInWorld.push_back(
        adaptDesiredPositionHeightToTerrain(reference2d.positionInWorld[k], projectedHeadingFrame, command.baseHeight));
    baseRef.eulerXyz.emplace_back(alignDesiredOrientationToTerrain({0.0, 0.0, reference2d.yaw[k]}, projectedHeadingFrame));
  }

  addVelocitiesFromFiniteDifferencem(baseRef);
  return baseRef;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ocs2::TargetTrajectories ReferenceTrajectoryGenerator::generateReferenceTrajectory(scalar_t time, scalar_t dt) {
    // Get base reference trajectory
    BaseReferenceTrajectory baseReferenceTrajectory;
    
    if (!terrainMapPtr_) {
        baseReferenceTrajectory = generateExtrapolatedBaseReference(getBaseReferenceHorizon(), getBaseReferenceState(),
                                                                    getBaseReferenceCommand(time), getTerrainPlane());

    } else {
        auto b = getBaseReferenceState();

        {
            std::lock_guard<std::mutex> lock(observationMutex_);
            auto observation = latestObservation_;
            vector3_t com = vector3_t::Zero();
            auto footPositions = kinematicsModel_->feetPositionsInOriginFrame(getBasePose(observation.state), getJointPositions(observation.state));
            for (size_t i = 0; i < 4; i++) {
                com += footPositions[i];
            }
            com /= 4.0;
            b.positionInWorld.head<2>() = com.head<2>();
        }


        baseReferenceTrajectory =
            generateExtrapolatedBaseReferencem(getBaseReferenceHorizon(), b,
                                              getBaseReferenceCommand(time), *terrainMapPtr_, 0.5, 0.3);
    }

    // Generate target trajectory
    ocs2::scalar_array_t desiredTimeTrajectory = std::move(baseReferenceTrajectory.time);
    const size_t N = desiredTimeTrajectory.size();
    ocs2::vector_array_t desiredStateTrajectory(N);
    ocs2::vector_array_t desiredInputTrajectory(N, ocs2::vector_t::Zero(INPUT_DIM));
    for (size_t i = 0; i < N; ++i) {
        ocs2::vector_t state = ocs2::vector_t::Zero(STATE_DIM);

        // base orientation
        state.head<3>() = baseReferenceTrajectory.eulerXyz[i];

        auto Rt = switched_model::rotationMatrixOriginToBase(baseReferenceTrajectory.eulerXyz[i]);

        // base position
        state.segment<3>(3) = baseReferenceTrajectory.positionInWorld[i];

        // base angular velocity
        state.segment<3>(6) = Rt * baseReferenceTrajectory.angularVelocityInWorld[i];

        // base linear velocity
        state.segment<3>(9) = Rt * baseReferenceTrajectory.linearVelocityInWorld[i];

        // joint angles
        state.segment<12>(12) = defaultJointState_;

        desiredStateTrajectory[i] = std::move(state);
    }

    return ocs2::TargetTrajectories(std::move(desiredTimeTrajectory), std::move(desiredStateTrajectory),
                                    std::move(desiredInputTrajectory));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceTrajectoryGenerator::loadSettings(const std::string &targetCommandFile) {
    // Load target COM height
    ocs2::loadData::loadCppDataType<scalar_t>(targetCommandFile, "comHeight", comHeight_);

    // Load default joint angles
    ocs2::loadData::loadEigenMatrix(targetCommandFile, "defaultJointState", defaultJointState_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceTrajectoryGenerator::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(observationMutex_);
    latestObservation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);

    // Update local terrain estimate if no terrain map is available
    if (!terrainMapPtr_) {
        localTerrainEstimator_.updateFootholds(latestObservation_);

        // Publish local terrain estimate
        ocs2_switched_model_msgs::local_terrain localTerrainMsg;

        vector_t position = localTerrainEstimator_.getPlane().positionInWorld;
        localTerrainMsg.position.push_back(position[0]);
        localTerrainMsg.position.push_back(position[1]);
        localTerrainMsg.position.push_back(position[2]);

        matrix_t rotation = localTerrainEstimator_.getPlane().orientationWorldToTerrain;
        localTerrainMsg.rotation.push_back(rotation(0, 0));
        localTerrainMsg.rotation.push_back(rotation(0, 1));
        localTerrainMsg.rotation.push_back(rotation(0, 2));
        localTerrainMsg.rotation.push_back(rotation(1, 0));
        localTerrainMsg.rotation.push_back(rotation(1, 1));
        localTerrainMsg.rotation.push_back(rotation(1, 2));
        localTerrainMsg.rotation.push_back(rotation(2, 0));
        localTerrainMsg.rotation.push_back(rotation(2, 1));
        localTerrainMsg.rotation.push_back(rotation(2, 2));

        terrainPublisher_.publish(localTerrainMsg);
    }

    firstObservationReceived_ = true;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceTrajectoryGenerator::terrainCallback(const grid_map_msgs::GridMap &msg) {
    if (!blind_) {
        // Convert ROS message to grid map
        std::unique_ptr<grid_map::GridMap> mapPtr(new grid_map::GridMap);
        std::vector<std::string> layers = {"smooth_planar"};
        grid_map::GridMapRosConverter::fromMessage(msg, *mapPtr, layers, false, false);

        // Swap terrain map pointers
        terrainMapPtr_.swap(mapPtr);
    }
}

std::unique_ptr<ReferenceTrajectoryGenerator> getReferenceTrajectoryGeneratorUnique(ros::NodeHandle &nh) {
    std::string configPath;
    TBAI_ROS_THROW_IF(!nh.getParam("target_command_config_file", configPath),
                      "Failed to get config file path from parameter server.");
    return std::make_unique<ReferenceTrajectoryGenerator>(configPath, nh);
}

std::shared_ptr<ReferenceTrajectoryGenerator> getReferenceTrajectoryGeneratorShared(ros::NodeHandle &nh) {
    return std::shared_ptr<ReferenceTrajectoryGenerator>(getReferenceTrajectoryGeneratorUnique(nh).release());
}

}  // namespace reference
}  // namespace mpc
}  // namespace tbai
