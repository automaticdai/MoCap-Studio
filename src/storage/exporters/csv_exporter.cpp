#include "storage/exporters/csv_exporter.h"
#include <fstream>
#include <stdexcept>
#include <iomanip>

namespace mocap {

void CsvExporter::exportRaw2D(const std::string& path,
    const std::vector<std::pair<double, std::vector<Raw2DPose>>>& frames)
{
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open CSV file: " + path);

    f << std::fixed << std::setprecision(6);
    f << "timestamp,person_id,confidence,bbox_x,bbox_y,bbox_w,bbox_h,"
         "keypoint_name,keypoint_index,kp_x,kp_y,kp_conf\n";

    for (const auto& [timestamp, poses] : frames) {
        for (const auto& pose : poses) {
            for (const auto& kp : pose.keypoints) {
                f << timestamp << ","
                  << pose.person_id << ","
                  << pose.confidence << ","
                  << pose.bbox.x << "," << pose.bbox.y << ","
                  << pose.bbox.width << "," << pose.bbox.height << ","
                  << kp.name << "," << kp.index << ","
                  << kp.x << "," << kp.y << "," << kp.conf << "\n";
            }
        }
    }
}

void CsvExporter::exportPose3D(const std::string& path,
    const std::vector<std::pair<double, std::vector<Pose3D>>>& frames)
{
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open CSV file: " + path);

    f << std::fixed << std::setprecision(6);
    f << "timestamp,person_id,marker_name,marker_index,x,y,z,confidence,n_views\n";

    for (const auto& [timestamp, poses] : frames) {
        for (const auto& pose : poses) {
            for (const auto& m : pose.markers) {
                f << timestamp << ","
                  << pose.global_person_id << ","
                  << m.name << "," << m.index << ","
                  << m.position.x() << "," << m.position.y() << "," << m.position.z() << ","
                  << m.confidence << "," << m.n_views << "\n";
            }
        }
    }
}

void CsvExporter::exportSkeleton(const std::string& path,
    const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames)
{
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open CSV file: " + path);

    f << std::fixed << std::setprecision(6);
    f << "timestamp,person_id,root_x,root_y,root_z,"
         "root_qw,root_qx,root_qy,root_qz,"
         "joint_name,joint_index,qw,qx,qy,qz,euler_x,euler_y,euler_z\n";

    for (const auto& [timestamp, poses] : frames) {
        for (const auto& pose : poses) {
            for (const auto& jr : pose.joint_rotations) {
                f << timestamp << ","
                  << pose.global_person_id << ","
                  << pose.root_position.x() << ","
                  << pose.root_position.y() << ","
                  << pose.root_position.z() << ","
                  << pose.root_rotation.w() << ","
                  << pose.root_rotation.x() << ","
                  << pose.root_rotation.y() << ","
                  << pose.root_rotation.z() << ","
                  << jr.joint_name << "," << jr.joint_index << ","
                  << jr.rotation.w() << "," << jr.rotation.x() << ","
                  << jr.rotation.y() << "," << jr.rotation.z() << ","
                  << jr.euler_xyz.x() << "," << jr.euler_xyz.y() << ","
                  << jr.euler_xyz.z() << "\n";
            }
        }
    }
}

}  // namespace mocap
