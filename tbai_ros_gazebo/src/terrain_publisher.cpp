#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/common/SystemPaths.hh>

#include <string>

using Pose = ignition::math::Pose3d;

Pose readPose(const sdf::ElementPtr& element) {
    if (element->HasElement("pose")) {
        return element->Get<Pose>("pose");
    }
    return Pose::Zero;
}

Pose compose(const Pose& parent, const Pose& child) {
    return Pose(
        parent.Pos() + parent.Rot().RotateVector(child.Pos()),
        parent.Rot() * child.Rot()
    );
}

void collectBoxes(
    const sdf::ElementPtr& model,
    const Pose& parent,
    const std::string& frame,
    visualization_msgs::MarkerArray& output
) {
    const Pose modelPose = compose(parent, readPose(model));

    // Visit every link in this model.
    for (auto link = model->HasElement("link")
                         ? model->GetElement("link") : nullptr;
         link;
         link = link->GetNextElement("link")) {

        const Pose linkPose = compose(modelPose, readPose(link));

        // Visit every collision in this link.
        for (auto collision = link->HasElement("collision")
                                  ? link->GetElement("collision") : nullptr;
             collision;
             collision = collision->GetNextElement("collision")) {

            auto geometry = collision->GetElement("geometry");

            if (!geometry->HasElement("box")) {
                ROS_WARN_STREAM(
                    "Skipping non-box collision: "
                    << collision->Get<std::string>("name")
                );
                continue;
            }

            const Pose pose = compose(linkPose, readPose(collision));
            const auto size = geometry->GetElement("box")
                                  ->Get<ignition::math::Vector3d>("size");

            visualization_msgs::Marker marker;

            marker.header.frame_id = frame;
            marker.header.stamp = ros::Time(0);
            marker.ns = "terrain_ground_truth";
            marker.id = static_cast<int>(output.markers.size());
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = pose.Pos().X();
            marker.pose.position.y = pose.Pos().Y();
            marker.pose.position.z = pose.Pos().Z();

            marker.pose.orientation.x = pose.Rot().X();
            marker.pose.orientation.y = pose.Rot().Y();
            marker.pose.orientation.z = pose.Rot().Z();
            marker.pose.orientation.w = pose.Rot().W();

            marker.scale.x = size.X();
            marker.scale.y = size.Y();
            marker.scale.z = size.Z();

            marker.color.r = 0.6;
            marker.color.g = 0.6;
            marker.color.b = 0.6;
            marker.color.a = 0.3;

            marker.lifetime = ros::Duration(0);
            marker.frame_locked = true;

            output.markers.push_back(marker);
        }
    }

    // Handle models contained inside other models.
    for (auto nested = model->HasElement("model")
                           ? model->GetElement("model") : nullptr;
         nested;
         nested = nested->GetNextElement("model")) {
        collectBoxes(nested, modelPose, frame, output);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "terrain_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    std::string worldFile;
    std::string frame;

    privateNh.param<std::string>("frame_id", frame, "world");

    if (!privateNh.getParam("world_file", worldFile)) {
        ROS_ERROR("Set the private parameter ~world_file.");
        return 1;
    }

    // This standalone node needs Gazebo's model paths to resolve SDF includes.
    for (const auto& path : gazebo::common::SystemPaths::Instance()->GetModelPaths()) {
        sdf::addURIPath("model://", path);
    }

    sdf::SDFPtr document(new sdf::SDF());
    sdf::init(document);

    if (!sdf::readFile(worldFile, document) ||
        !document->Root()->HasElement("world")) {
        ROS_ERROR_STREAM("Cannot read world: " << worldFile);
        return 1;
    }

    visualization_msgs::MarkerArray markers;
    auto world = document->Root()->GetElement("world");

    for (auto model = world->HasElement("model")
                          ? world->GetElement("model") : nullptr;
         model;
         model = model->GetNextElement("model")) {
        collectBoxes(model, Pose::Zero, frame, markers);
    }

    if (markers.markers.empty()) {
        ROS_ERROR("No box collisions found.");
        return 1;
    }

    auto publisher =
        nh.advertise<visualization_msgs::MarkerArray>(
            "/terrain_ground_truth", 1, true
        );

    publisher.publish(markers);

    ROS_INFO_STREAM(
        "Published " << markers.markers.size()
        << " terrain boxes in " << frame
    );

    ros::spin();
    return 0;
}
