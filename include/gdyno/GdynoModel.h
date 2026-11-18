#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "gdyno/GdynoFormat.h"

namespace gdyno {

    struct Vec3 {
        float x = 0.0f, y = 0.0f, z = 0.0f;
    };

    struct Quat {
        float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;
    };

    struct Submesh {
        std::string name;
        uint32_t indexOffset = 0;
        uint32_t indexCount = 0;
        Vec3 baseColor{1.0f, 1.0f, 1.0f};
        float emission = 0.0f;
    };

    struct Socket {
        std::string name;
        int32_t parentBone = -1;
        Vec3 position{0.0f, 0.0f, 0.0f};
        Quat rotation{0.0f, 0.0f, 0.0f, 1.0f};
        Vec3 scale{1.0f, 1.0f, 1.0f};
    };

    struct Bone {
        std::string name;
        int32_t parentIndex = -1;
        std::array<float, 16> inverseBindMatrix{};
        Vec3 bindLocalPos{0.0f, 0.0f, 0.0f};
        Quat bindLocalRot{0.0f, 0.0f, 0.0f, 1.0f};
        Vec3 bindLocalScale{1.0f, 1.0f, 1.0f};
    };

    struct VecKey {
        float time = 0.0f;
        Vec3 value{};
    };

    struct QuatKey {
        float time = 0.0f;
        Quat value{};
    };

    struct AnimChannel {
        int32_t boneIndex = -1;
        std::vector<VecKey> posKeys;
        std::vector<QuatKey> rotKeys;
        std::vector<VecKey> scaleKeys;
    };

    struct AnimClip {
        std::string name;
        float durationSec = 0.0f;
        std::vector<AnimChannel> channels;
    };

    struct Model {
        uint32_t flags = FLAG_NONE;
        Vec3 aabbMin{0.0f, 0.0f, 0.0f};
        Vec3 aabbMax{0.0f, 0.0f, 0.0f};

        std::vector<PackedVertex> vertices;
        std::vector<PackedSkinWeight> skinWeights;
        std::vector<uint32_t> indices;
        std::vector<Submesh> submeshes;
        std::vector<Socket> sockets;
        std::vector<Bone> bones;
        std::vector<AnimClip> animations;

        bool hasSkinning() const { return (flags & FLAG_SKINNING) != 0 && !skinWeights.empty(); }
        bool hasSkeleton() const { return (flags & FLAG_SKELETON) != 0 && !bones.empty(); }
        bool hasAnimations() const { return (flags & FLAG_ANIMATIONS) != 0 && !animations.empty(); }

        const Submesh* findSubmesh(const std::string& subName) const {
            for (const auto& s : submeshes) {
                if (s.name == subName) return &s;
            }
            return nullptr;
        }

        const Socket* findSocket(const std::string& sockName) const {
            for (const auto& s : sockets) {
                if (s.name == sockName) return &s;
            }
            return nullptr;
        }

        int32_t findBoneIndex(const std::string& boneName) const {
            for (size_t i = 0; i < bones.size(); ++i) {
                if (bones[i].name == boneName) return static_cast<int32_t>(i);
            }
            return -1;
        }

        // Вычисляет локальные матрицы костей на момент времени t (с зацикливанием)
        void sampleAnimation(size_t clipIndex, float timeSec, std::vector<std::array<float, 16>>& outBoneMatrices) const;
    };

} // namespace gdyno
