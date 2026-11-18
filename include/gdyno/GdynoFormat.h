#pragma once

#include <cstdint>

namespace gdyno {

    constexpr uint32_t MAGIC = 0x4E594447u; // "GDYN" в little-endian
    constexpr uint16_t FORMAT_VERSION = 1;
    constexpr size_t MAX_NAME_LEN = 32;
    constexpr size_t MAX_BONES_PER_VERTEX = 4;
    constexpr size_t MAX_SKELETON_BONES = 64;

    enum AssetFlags : uint32_t {
        FLAG_NONE       = 0,
        FLAG_SKINNING   = 1u << 0, // Присутствует буфер весов костей
        FLAG_SOCKETS    = 1u << 1, // Присутствуют точки крепления (колёса, фары, руки)
        FLAG_SKELETON   = 1u << 2, // Присутствует иерархия костей
        FLAG_ANIMATIONS = 1u << 3  // Присутствуют клипы скелетных анимаций
    };

    #pragma pack(push, 1)

    struct FileHeader {
        uint32_t magic = MAGIC;
        uint16_t version = FORMAT_VERSION;
        uint16_t reserved = 0;
        uint32_t flags = FLAG_NONE;

        uint32_t vertexCount = 0;
        uint32_t indexCount = 0;
        uint32_t submeshCount = 0;
        uint32_t socketCount = 0;
        uint32_t boneCount = 0;
        uint32_t animClipCount = 0;

        float aabbMin[3] = {0.0f, 0.0f, 0.0f};
        float aabbMax[3] = {0.0f, 0.0f, 0.0f};
    };

    // Единая вершина, напрямую совместимая с VBO игровых движков
    struct PackedVertex {
        float position[3];
        float normal[3];
        float uv[2];
        float color[3];
        float emission;
    };

    // Опциональный буфер скиннинга (записывается только если установлен FLAG_SKINNING)
    struct PackedSkinWeight {
        uint8_t boneIds[MAX_BONES_PER_VERTEX];
        uint8_t weights[MAX_BONES_PER_VERTEX]; // Нормализованы в 0..255
    };

    struct PackedSubmesh {
        char name[MAX_NAME_LEN];
        uint32_t indexOffset;
        uint32_t indexCount;
        float baseColor[3];
        float emission;
    };

    struct PackedSocket {
        char name[MAX_NAME_LEN];
        int32_t parentBone; // -1 если привязан к корню модели
        float position[3];
        float rotationQuat[4]; // x, y, z, w
        float scale[3];
    };

    struct PackedBone {
        char name[MAX_NAME_LEN];
        int32_t parentIndex; // -1 для корневой кости
        float inverseBindMatrix[16]; // Column-major 4x4
        float bindLocalPos[3];
        float bindLocalRot[4]; // x, y, z, w
        float bindLocalScale[3];
    };

    struct PackedAnimClipHeader {
        char name[MAX_NAME_LEN];
        float durationSec;
        uint32_t channelCount;
    };

    struct PackedAnimChannelHeader {
        int32_t boneIndex;
        uint32_t posKeyCount;
        uint32_t rotKeyCount;
        uint32_t scaleKeyCount;
    };

    struct PackedVecKey {
        float time;
        float value[3];
    };

    struct PackedQuatKey {
        float time;
        float value[4]; // x, y, z, w
    };

    #pragma pack(pop)

} // namespace gdyno
