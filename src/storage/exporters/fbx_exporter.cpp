#include "storage/exporters/fbx_exporter.h"
#include <spdlog/spdlog.h>
#include <stdexcept>

#ifdef MOCAP_HAS_ASSIMP
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#endif

namespace mocap {

bool FbxExporter::isAvailable() {
#ifdef MOCAP_HAS_ASSIMP
    return true;
#else
    return false;
#endif
}

void FbxExporter::exportSkeleton(
    const std::string& path,
    const SkeletonDefinition& skeleton,
    const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames,
    double fps,
    int person_id)
{
#ifdef MOCAP_HAS_ASSIMP
    Assimp::Exporter exporter;

    // Build an Assimp scene with skeleton hierarchy and animation
    auto scene = std::make_unique<aiScene>();
    scene->mRootNode = new aiNode("RootNode");

    // Build skeleton node hierarchy
    std::function<aiNode*(int, aiNode*)> buildNode = [&](int joint_idx, aiNode* parent) -> aiNode* {
        const auto& jd = skeleton.joint(joint_idx);
        aiNode* node = new aiNode(jd.name.c_str());
        node->mParent = parent;

        // Set transform from rest offset
        aiMatrix4x4& t = node->mTransformation;
        t = aiMatrix4x4();
        t.a4 = jd.rest_offset.x();
        t.b4 = jd.rest_offset.y();
        t.c4 = jd.rest_offset.z();

        auto children = skeleton.childrenOf(joint_idx);
        node->mNumChildren = static_cast<unsigned int>(children.size());
        if (!children.empty()) {
            node->mChildren = new aiNode*[children.size()];
            for (size_t i = 0; i < children.size(); ++i) {
                node->mChildren[i] = buildNode(children[i], node);
            }
        }
        return node;
    };

    // Find root
    int root_idx = -1;
    for (int i = 0; i < skeleton.jointCount(); ++i) {
        if (skeleton.joint(i).parent == -1) { root_idx = i; break; }
    }

    if (root_idx >= 0) {
        aiNode* skel_root = buildNode(root_idx, scene->mRootNode);
        scene->mRootNode->mNumChildren = 1;
        scene->mRootNode->mChildren = new aiNode*[1];
        scene->mRootNode->mChildren[0] = skel_root;
    }

    // Create animation
    scene->mNumAnimations = 1;
    scene->mAnimations = new aiAnimation*[1];
    auto anim = new aiAnimation();
    anim->mName = aiString("MoCapAnimation");
    anim->mDuration = frames.empty() ? 0.0 : frames.back().first;
    anim->mTicksPerSecond = fps;

    // Create channels for each joint
    anim->mNumChannels = static_cast<unsigned int>(skeleton.jointCount());
    anim->mChannels = new aiNodeAnim*[skeleton.jointCount()];

    for (int j = 0; j < skeleton.jointCount(); ++j) {
        auto channel = new aiNodeAnim();
        channel->mNodeName = aiString(skeleton.joint(j).name.c_str());

        // Count frames for this person
        int frame_count = 0;
        for (const auto& [ts, poses] : frames) {
            for (const auto& pose : poses) {
                if (pose.global_person_id == person_id) { frame_count++; break; }
            }
        }

        channel->mNumRotationKeys = frame_count;
        channel->mRotationKeys = new aiQuatKey[frame_count];

        if (skeleton.joint(j).parent == -1) {
            channel->mNumPositionKeys = frame_count;
            channel->mPositionKeys = new aiVectorKey[frame_count];
        } else {
            channel->mNumPositionKeys = 0;
            channel->mPositionKeys = nullptr;
        }

        int fi = 0;
        for (const auto& [ts, poses] : frames) {
            for (const auto& pose : poses) {
                if (pose.global_person_id != person_id) continue;

                if (j < static_cast<int>(pose.joint_rotations.size())) {
                    const auto& jr = pose.joint_rotations[j];
                    channel->mRotationKeys[fi] = aiQuatKey(
                        ts, aiQuaternion(jr.rotation.w(), jr.rotation.x(),
                                         jr.rotation.y(), jr.rotation.z())
                    );
                }

                if (skeleton.joint(j).parent == -1 && channel->mPositionKeys) {
                    channel->mPositionKeys[fi] = aiVectorKey(
                        ts, aiVector3D(pose.root_position.x(),
                                       pose.root_position.y(),
                                       pose.root_position.z())
                    );
                }
                fi++;
                break;
            }
        }

        anim->mChannels[j] = channel;
    }

    scene->mAnimations[0] = anim;

    // Need at least one mesh/material for valid scene
    scene->mNumMeshes = 0;
    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial*[1];
    scene->mMaterials[0] = new aiMaterial();

    auto result = exporter.Export(scene.get(), "fbx", path);
    if (result != AI_SUCCESS) {
        throw std::runtime_error("FBX export failed: " + std::string(exporter.GetErrorString()));
    }

    spdlog::info("Exported {} frames to FBX: {}", frames.size(), path);

#else
    (void)path; (void)skeleton; (void)frames; (void)fps; (void)person_id;
    spdlog::error("FBX export not available — Assimp not found at build time");
    throw std::runtime_error("FBX export requires Assimp library (not available)");
#endif
}

}  // namespace mocap
