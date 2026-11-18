#include "gdyno/GdynoLoader.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

namespace gdyno {

    namespace {

        std::string unpackName(const char raw[MAX_NAME_LEN]) {
            size_t len = 0;
            while (len < MAX_NAME_LEN && raw[len] != '\0') {
                ++len;
            }
            return std::string(raw, len);
        }

        void packName(const std::string& src, char dst[MAX_NAME_LEN]) {
            std::memset(dst, 0, MAX_NAME_LEN);
            size_t n = std::min(src.size(), MAX_NAME_LEN - 1);
            std::memcpy(dst, src.data(), n);
        }

        Vec3 lerpVec3(const Vec3& a, const Vec3& b, float t) {
            return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
        }

        Quat slerpQuat(const Quat& qa, const Quat& qb, float t) {
            Quat b = qb;
            float dot = qa.x * b.x + qa.y * b.y + qa.z * b.z + qa.w * b.w;
            if (dot < 0.0f) {
                dot = -dot;
                b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
            }
            Quat res;
            if (dot > 0.9995f) {
                res = {
                    qa.x + t * (b.x - qa.x),
                    qa.y + t * (b.y - qa.y),
                    qa.z + t * (b.z - qa.z),
                    qa.w + t * (b.w - qa.w)
                };
            } else {
                float theta0 = std::acos(std::clamp(dot, -1.0f, 1.0f));
                float theta = theta0 * t;
                float sinTheta = std::sin(theta);
                float sinTheta0 = std::sin(theta0);
                float s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
                float s1 = sinTheta / sinTheta0;
                res = {
                    s0 * qa.x + s1 * b.x,
                    s0 * qa.y + s1 * b.y,
                    s0 * qa.z + s1 * b.z,
                    s0 * qa.w + s1 * b.w
                };
            }
            float len = std::sqrt(res.x * res.x + res.y * res.y + res.z * res.z + res.w * res.w);
            if (len > 1e-6f) {
                res.x /= len; res.y /= len; res.z /= len; res.w /= len;
            }
            return res;
        }

        std::array<float, 16> composeTRS(const Vec3& t, const Quat& r, const Vec3& s) {
            float xx = r.x * r.x, yy = r.y * r.y, zz = r.z * r.z;
            float xy = r.x * r.y, xz = r.x * r.z, yz = r.y * r.z;
            float wx = r.w * r.x, wy = r.w * r.y, wz = r.w * r.z;

            std::array<float, 16> m{};
            m[0]  = (1.0f - 2.0f * (yy + zz)) * s.x;
            m[1]  = (2.0f * (xy + wz)) * s.x;
            m[2]  = (2.0f * (xz - wy)) * s.x;
            m[3]  = 0.0f;

            m[4]  = (2.0f * (xy - wz)) * s.y;
            m[5]  = (1.0f - 2.0f * (xx + zz)) * s.y;
            m[6]  = (2.0f * (yz + wx)) * s.y;
            m[7]  = 0.0f;

            m[8]  = (2.0f * (xz + wy)) * s.z;
            m[9]  = (2.0f * (yz - wx)) * s.z;
            m[10] = (1.0f - 2.0f * (xx + yy)) * s.z;
            m[11] = 0.0f;

            m[12] = t.x;
            m[13] = t.y;
            m[14] = t.z;
            m[15] = 1.0f;
            return m;
        }

        std::array<float, 16> mulMat4(const std::array<float, 16>& a, const std::array<float, 16>& b) {
            std::array<float, 16> out{};
            for (int col = 0; col < 4; ++col) {
                for (int row = 0; row < 4; ++row) {
                    out[col * 4 + row] =
                    a[0 * 4 + row] * b[col * 4 + 0] +
                    a[1 * 4 + row] * b[col * 4 + 1] +
                    a[2 * 4 + row] * b[col * 4 + 2] +
                    a[3 * 4 + row] * b[col * 4 + 3];
                }
            }
            return out;
        }

        Vec3 evalVecTrack(const std::vector<VecKey>& keys, float t, const Vec3& fallback) {
            if (keys.empty()) return fallback;
            if (keys.size() == 1 || t <= keys.front().time) return keys.front().value;
            if (t >= keys.back().time) return keys.back().value;
            for (size_t i = 0; i + 1 < keys.size(); ++i) {
                if (t >= keys[i].time && t <= keys[i + 1].time) {
                    float span = keys[i + 1].time - keys[i].time;
                    float alpha = (span > 1e-6f) ? (t - keys[i].time) / span : 0.0f;
                    return lerpVec3(keys[i].value, keys[i + 1].value, alpha);
                }
            }
            return keys.back().value;
        }

        Quat evalQuatTrack(const std::vector<QuatKey>& keys, float t, const Quat& fallback) {
            if (keys.empty()) return fallback;
            if (keys.size() == 1 || t <= keys.front().time) return keys.front().value;
            if (t >= keys.back().time) return keys.back().value;
            for (size_t i = 0; i + 1 < keys.size(); ++i) {
                if (t >= keys[i].time && t <= keys[i + 1].time) {
                    float span = keys[i + 1].time - keys[i].time;
                    float alpha = (span > 1e-6f) ? (t - keys[i].time) / span : 0.0f;
                    return slerpQuat(keys[i].value, keys[i + 1].value, alpha);
                }
            }
            return keys.back().value;
        }

    } // namespace

    void Model::sampleAnimation(size_t clipIndex, float timeSec, std::vector<std::array<float, 16>>& outBoneMatrices) const {
        const size_t n = bones.size();
        outBoneMatrices.resize(n);
        if (n == 0) return;

        std::vector<Vec3> localPos(n);
        std::vector<Quat> localRot(n);
        std::vector<Vec3> localScale(n);
        for (size_t i = 0; i < n; ++i) {
            localPos[i] = bones[i].bindLocalPos;
            localRot[i] = bones[i].bindLocalRot;
            localScale[i] = bones[i].bindLocalScale;
        }

        if (clipIndex < animations.size()) {
            const auto& clip = animations[clipIndex];
            float t = (clip.durationSec > 1e-5f) ? std::fmod(std::max(0.0f, timeSec), clip.durationSec) : 0.0f;
            for (const auto& ch : clip.channels) {
                if (ch.boneIndex < 0 || static_cast<size_t>(ch.boneIndex) >= n) continue;
                size_t bi = static_cast<size_t>(ch.boneIndex);
                localPos[bi] = evalVecTrack(ch.posKeys, t, localPos[bi]);
                localRot[bi] = evalQuatTrack(ch.rotKeys, t, localRot[bi]);
                localScale[bi] = evalVecTrack(ch.scaleKeys, t, localScale[bi]);
            }
        }

        std::vector<std::array<float, 16>> globalTransforms(n);
        for (size_t i = 0; i < n; ++i) {
            std::array<float, 16> localM = composeTRS(localPos[i], localRot[i], localScale[i]);
            int32_t parent = bones[i].parentIndex;
            if (parent >= 0 && static_cast<size_t>(parent) < i) {
                globalTransforms[i] = mulMat4(globalTransforms[static_cast<size_t>(parent)], localM);
            } else {
                globalTransforms[i] = localM;
            }
            outBoneMatrices[i] = mulMat4(globalTransforms[i], bones[i].inverseBindMatrix);
        }
    }

    bool Loader::saveToFile(const std::filesystem::path& filePath, const Model& model, std::string* outError) {
        std::ofstream out(filePath, std::ios::binary);
        if (!out.is_open()) {
            if (outError) *outError = "Failed to open output file: " + filePath.string();
            return false;
        }

        FileHeader hdr{};
        hdr.magic = MAGIC;
        hdr.version = FORMAT_VERSION;
        hdr.flags = model.flags;
        if (!model.skinWeights.empty()) hdr.flags |= FLAG_SKINNING;
        if (!model.sockets.empty()) hdr.flags |= FLAG_SOCKETS;
        if (!model.bones.empty()) hdr.flags |= FLAG_SKELETON;
        if (!model.animations.empty()) hdr.flags |= FLAG_ANIMATIONS;

        hdr.vertexCount = static_cast<uint32_t>(model.vertices.size());
        hdr.indexCount = static_cast<uint32_t>(model.indices.size());
        hdr.submeshCount = static_cast<uint32_t>(model.submeshes.size());
        hdr.socketCount = static_cast<uint32_t>(model.sockets.size());
        hdr.boneCount = static_cast<uint32_t>(model.bones.size());
        hdr.animClipCount = static_cast<uint32_t>(model.animations.size());

        hdr.aabbMin[0] = model.aabbMin.x; hdr.aabbMin[1] = model.aabbMin.y; hdr.aabbMin[2] = model.aabbMin.z;
        hdr.aabbMax[0] = model.aabbMax.x; hdr.aabbMax[1] = model.aabbMax.y; hdr.aabbMax[2] = model.aabbMax.z;

        out.write(reinterpret_cast<const char*>(&hdr), sizeof(FileHeader));

        if (!model.vertices.empty()) {
            out.write(reinterpret_cast<const char*>(model.vertices.data()),
                      static_cast<std::streamsize>(model.vertices.size() * sizeof(PackedVertex)));
        }
        if ((hdr.flags & FLAG_SKINNING) && !model.skinWeights.empty()) {
            out.write(reinterpret_cast<const char*>(model.skinWeights.data()),
                      static_cast<std::streamsize>(model.skinWeights.size() * sizeof(PackedSkinWeight)));
        }
        if (!model.indices.empty()) {
            out.write(reinterpret_cast<const char*>(model.indices.data()),
                      static_cast<std::streamsize>(model.indices.size() * sizeof(uint32_t)));
        }

        for (const auto& sm : model.submeshes) {
            PackedSubmesh p{};
            packName(sm.name, p.name);
            p.indexOffset = sm.indexOffset;
            p.indexCount = sm.indexCount;
            p.baseColor[0] = sm.baseColor.x; p.baseColor[1] = sm.baseColor.y; p.baseColor[2] = sm.baseColor.z;
            p.emission = sm.emission;
            out.write(reinterpret_cast<const char*>(&p), sizeof(PackedSubmesh));
        }

        for (const auto& sk : model.sockets) {
            PackedSocket p{};
            packName(sk.name, p.name);
            p.parentBone = sk.parentBone;
            p.position[0] = sk.position.x; p.position[1] = sk.position.y; p.position[2] = sk.position.z;
            p.rotationQuat[0] = sk.rotation.x; p.rotationQuat[1] = sk.rotation.y;
            p.rotationQuat[2] = sk.rotation.z; p.rotationQuat[3] = sk.rotation.w;
            p.scale[0] = sk.scale.x; p.scale[1] = sk.scale.y; p.scale[2] = sk.scale.z;
            out.write(reinterpret_cast<const char*>(&p), sizeof(PackedSocket));
        }

        for (const auto& bn : model.bones) {
            PackedBone p{};
            packName(bn.name, p.name);
            p.parentIndex = bn.parentIndex;
            std::memcpy(p.inverseBindMatrix, bn.inverseBindMatrix.data(), 16 * sizeof(float));
            p.bindLocalPos[0] = bn.bindLocalPos.x; p.bindLocalPos[1] = bn.bindLocalPos.y; p.bindLocalPos[2] = bn.bindLocalPos.z;
            p.bindLocalRot[0] = bn.bindLocalRot.x; p.bindLocalRot[1] = bn.bindLocalRot.y;
            p.bindLocalRot[2] = bn.bindLocalRot.z; p.bindLocalRot[3] = bn.bindLocalRot.w;
            p.bindLocalScale[0] = bn.bindLocalScale.x; p.bindLocalScale[1] = bn.bindLocalScale.y; p.bindLocalScale[2] = bn.bindLocalScale.z;
            out.write(reinterpret_cast<const char*>(&p), sizeof(PackedBone));
        }

        for (const auto& clip : model.animations) {
            PackedAnimClipHeader chdr{};
            packName(clip.name, chdr.name);
            chdr.durationSec = clip.durationSec;
            chdr.channelCount = static_cast<uint32_t>(clip.channels.size());
            out.write(reinterpret_cast<const char*>(&chdr), sizeof(PackedAnimClipHeader));

            for (const auto& ch : clip.channels) {
                PackedAnimChannelHeader cch{};
                cch.boneIndex = ch.boneIndex;
                cch.posKeyCount = static_cast<uint32_t>(ch.posKeys.size());
                cch.rotKeyCount = static_cast<uint32_t>(ch.rotKeys.size());
                cch.scaleKeyCount = static_cast<uint32_t>(ch.scaleKeys.size());
                out.write(reinterpret_cast<const char*>(&cch), sizeof(PackedAnimChannelHeader));

                for (const auto& k : ch.posKeys) {
                    PackedVecKey pk{ k.time, { k.value.x, k.value.y, k.value.z } };
                    out.write(reinterpret_cast<const char*>(&pk), sizeof(PackedVecKey));
                }
                for (const auto& k : ch.rotKeys) {
                    PackedQuatKey pk{ k.time, { k.value.x, k.value.y, k.value.z, k.value.w } };
                    out.write(reinterpret_cast<const char*>(&pk), sizeof(PackedQuatKey));
                }
                for (const auto& k : ch.scaleKeys) {
                    PackedVecKey pk{ k.time, { k.value.x, k.value.y, k.value.z } };
                    out.write(reinterpret_cast<const char*>(&pk), sizeof(PackedVecKey));
                }
            }
        }

        return out.good();
    }

    bool Loader::loadFromFile(const std::filesystem::path& filePath, Model& outModel, std::string* outError) {
        std::ifstream in(filePath, std::ios::binary);
        if (!in.is_open()) {
            if (outError) *outError = "Cannot open .gdyno file: " + filePath.string();
            return false;
        }

        FileHeader hdr{};
        in.read(reinterpret_cast<char*>(&hdr), sizeof(FileHeader));
        if (!in.good() || hdr.magic != MAGIC) {
            if (outError) *outError = "Invalid .gdyno magic header";
            return false;
        }
        if (hdr.version != FORMAT_VERSION) {
            if (outError) *outError = "Unsupported .gdyno version";
            return false;
        }

        outModel = Model{};
        outModel.flags = hdr.flags;
        outModel.aabbMin = { hdr.aabbMin[0], hdr.aabbMin[1], hdr.aabbMin[2] };
        outModel.aabbMax = { hdr.aabbMax[0], hdr.aabbMax[1], hdr.aabbMax[2] };

        outModel.vertices.resize(hdr.vertexCount);
        if (hdr.vertexCount > 0) {
            in.read(reinterpret_cast<char*>(outModel.vertices.data()),
                    static_cast<std::streamsize>(hdr.vertexCount * sizeof(PackedVertex)));
        }

        if (hdr.flags & FLAG_SKINNING) {
            outModel.skinWeights.resize(hdr.vertexCount);
            if (hdr.vertexCount > 0) {
                in.read(reinterpret_cast<char*>(outModel.skinWeights.data()),
                        static_cast<std::streamsize>(hdr.vertexCount * sizeof(PackedSkinWeight)));
            }
        }

        outModel.indices.resize(hdr.indexCount);
        if (hdr.indexCount > 0) {
            in.read(reinterpret_cast<char*>(outModel.indices.data()),
                    static_cast<std::streamsize>(hdr.indexCount * sizeof(uint32_t)));
        }

        outModel.submeshes.reserve(hdr.submeshCount);
        for (uint32_t i = 0; i < hdr.submeshCount; ++i) {
            PackedSubmesh p{};
            in.read(reinterpret_cast<char*>(&p), sizeof(PackedSubmesh));
            Submesh sm;
            sm.name = unpackName(p.name);
            sm.indexOffset = p.indexOffset;
            sm.indexCount = p.indexCount;
            sm.baseColor = { p.baseColor[0], p.baseColor[1], p.baseColor[2] };
            sm.emission = p.emission;
            outModel.submeshes.push_back(std::move(sm));
        }

        outModel.sockets.reserve(hdr.socketCount);
        for (uint32_t i = 0; i < hdr.socketCount; ++i) {
            PackedSocket p{};
            in.read(reinterpret_cast<char*>(&p), sizeof(PackedSocket));
            Socket sk;
            sk.name = unpackName(p.name);
            sk.parentBone = p.parentBone;
            sk.position = { p.position[0], p.position[1], p.position[2] };
            sk.rotation = { p.rotationQuat[0], p.rotationQuat[1], p.rotationQuat[2], p.rotationQuat[3] };
            sk.scale = { p.scale[0], p.scale[1], p.scale[2] };
            outModel.sockets.push_back(std::move(sk));
        }

        outModel.bones.reserve(hdr.boneCount);
        for (uint32_t i = 0; i < hdr.boneCount; ++i) {
            PackedBone p{};
            in.read(reinterpret_cast<char*>(&p), sizeof(PackedBone));
            Bone bn;
            bn.name = unpackName(p.name);
            bn.parentIndex = p.parentIndex;
            std::memcpy(bn.inverseBindMatrix.data(), p.inverseBindMatrix, 16 * sizeof(float));
            bn.bindLocalPos = { p.bindLocalPos[0], p.bindLocalPos[1], p.bindLocalPos[2] };
            bn.bindLocalRot = { p.bindLocalRot[0], p.bindLocalRot[1], p.bindLocalRot[2], p.bindLocalRot[3] };
            bn.bindLocalScale = { p.bindLocalScale[0], p.bindLocalScale[1], p.bindLocalScale[2] };
            outModel.bones.push_back(std::move(bn));
        }

        outModel.animations.reserve(hdr.animClipCount);
        for (uint32_t c = 0; c < hdr.animClipCount; ++c) {
            PackedAnimClipHeader chdr{};
            in.read(reinterpret_cast<char*>(&chdr), sizeof(PackedAnimClipHeader));
            AnimClip clip;
            clip.name = unpackName(chdr.name);
            clip.durationSec = chdr.durationSec;
            clip.channels.reserve(chdr.channelCount);

            for (uint32_t chIdx = 0; chIdx < chdr.channelCount; ++chIdx) {
                PackedAnimChannelHeader cch{};
                in.read(reinterpret_cast<char*>(&cch), sizeof(PackedAnimChannelHeader));
                AnimChannel ch;
                ch.boneIndex = cch.boneIndex;

                ch.posKeys.resize(cch.posKeyCount);
                for (uint32_t k = 0; k < cch.posKeyCount; ++k) {
                    PackedVecKey pk{};
                    in.read(reinterpret_cast<char*>(&pk), sizeof(PackedVecKey));
                    ch.posKeys[k] = { pk.time, { pk.value[0], pk.value[1], pk.value[2] } };
                }
                ch.rotKeys.resize(cch.rotKeyCount);
                for (uint32_t k = 0; k < cch.rotKeyCount; ++k) {
                    PackedQuatKey pk{};
                    in.read(reinterpret_cast<char*>(&pk), sizeof(PackedQuatKey));
                    ch.rotKeys[k] = { pk.time, { pk.value[0], pk.value[1], pk.value[2], pk.value[3] } };
                }
                ch.scaleKeys.resize(cch.scaleKeyCount);
                for (uint32_t k = 0; k < cch.scaleKeyCount; ++k) {
                    PackedVecKey pk{};
                    in.read(reinterpret_cast<char*>(&pk), sizeof(PackedVecKey));
                    ch.scaleKeys[k] = { pk.time, { pk.value[0], pk.value[1], pk.value[2] } };
                }

                clip.channels.push_back(std::move(ch));
            }
            outModel.animations.push_back(std::move(clip));
        }

        return in.good() || in.eof();
    }

} // namespace gdyno
