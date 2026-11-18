#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

#include "gdyno/GdynoLoader.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

    struct ConvertOptions {
        std::string inputPath;
        std::string outputPath;
        float targetHeight = -1.0f; // > 0: авто-масштаб по высоте (напр. 1.75 для пешеходов)
        float targetLength = -1.0f; // > 0: авто-масштаб по длине Z (напр. 4.4 для авто)
        bool groundCenter = true;   // Ставит модель центром X/Z в 0 и низом на Y = 0
    };

    void transformPoint(const float m[16], const float p[3], float out[3]) {
        out[0] = m[0] * p[0] + m[4] * p[1] + m[8]  * p[2] + m[12];
        out[1] = m[1] * p[0] + m[5] * p[1] + m[9]  * p[2] + m[13];
        out[2] = m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14];
    }

    void transformDir(const float m[16], const float d[3], float out[3]) {
        out[0] = m[0] * d[0] + m[4] * d[1] + m[8]  * d[2];
        out[1] = m[1] * d[0] + m[5] * d[1] + m[9]  * d[2];
        out[2] = m[2] * d[0] + m[6] * d[1] + m[10] * d[2];
        float len = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
        if (len > 1e-6f) {
            out[0] /= len; out[1] /= len; out[2] /= len;
        }
    }

    void extractNodeAndChildren(const cgltf_data* data,
                                const cgltf_node* node,
                                const std::unordered_map<const cgltf_node*, int32_t>& boneNodeMap,
                                gdyno::Model& outModel,
                                bool& anySkinned) {
        float worldMat[16];
        cgltf_node_transform_world(node, worldMat);

        std::string nodeName = node->name ? node->name : "part";

        // Узлы с префиксом socket_ или пустые дочерние узлы колёс/фар сохраняем как сокеты
        if (!node->mesh && (nodeName.rfind("socket_", 0) == 0 || nodeName.rfind("Socket_", 0) == 0)) {
            gdyno::Socket sk;
            sk.name = nodeName;
            sk.parentBone = -1;
            if (node->parent) {
                auto it = boneNodeMap.find(node->parent);
                if (it != boneNodeMap.end()) sk.parentBone = it->second;
            }
            sk.position = { worldMat[12], worldMat[13], worldMat[14] };
            cgltf_float rot[4];
            if (node->has_rotation) {
                sk.rotation = { node->rotation[0], node->rotation[1], node->rotation[2], node->rotation[3] };
            }
            if (node->has_scale) {
                sk.scale = { node->scale[0], node->scale[1], node->scale[2] };
            }
            outModel.sockets.push_back(std::move(sk));
        }

        if (node->mesh) {
            const cgltf_mesh* mesh = node->mesh;
            for (cgltf_size pIdx = 0; pIdx < mesh->primitives_count; ++pIdx) {
                const cgltf_primitive& prim = mesh->primitives[pIdx];
                if (prim.type != cgltf_primitive_type_triangles) continue;

                const cgltf_accessor* posAcc = nullptr;
                const cgltf_accessor* normAcc = nullptr;
                const cgltf_accessor* uvAcc = nullptr;
                const cgltf_accessor* colAcc = nullptr;
                const cgltf_accessor* jointsAcc = nullptr;
                const cgltf_accessor* weightsAcc = nullptr;

                for (cgltf_size aIdx = 0; aIdx < prim.attributes_count; ++aIdx) {
                    const cgltf_attribute& attr = prim.attributes[aIdx];
                    if (attr.type == cgltf_attribute_type_position) posAcc = attr.data;
                    else if (attr.type == cgltf_attribute_type_normal) normAcc = attr.data;
                    else if (attr.type == cgltf_attribute_type_texcoord && attr.index == 0) uvAcc = attr.data;
                    else if (attr.type == cgltf_attribute_type_color && attr.index == 0) colAcc = attr.data;
                    else if (attr.type == cgltf_attribute_type_joints && attr.index == 0) jointsAcc = attr.data;
                    else if (attr.type == cgltf_attribute_type_weights && attr.index == 0) weightsAcc = attr.data;
                }
                if (!posAcc) continue;

                // Запекаем базовый цвет и эмиссию glTF-материала прямо в вершины
                float matColor[3] = { 0.78f, 0.78f, 0.78f };
                float matEmission = 0.0f;
                if (prim.material) {
                    if (prim.material->has_pbr_metallic_roughness) {
                        const float* bc = prim.material->pbr_metallic_roughness.base_color_factor;
                        matColor[0] = bc[0]; matColor[1] = bc[1]; matColor[2] = bc[2];
                    }
                    const float* em = prim.material->emissive_factor;
                    float emMax = std::max({ em[0], em[1], em[2] });
                    if (emMax > 0.01f) {
                        matEmission = emMax * 2.0f;
                        matColor[0] = std::max(matColor[0], em[0]);
                        matColor[1] = std::max(matColor[1], em[1]);
                        matColor[2] = std::max(matColor[2], em[2]);
                    }
                }

                uint32_t baseVertex = static_cast<uint32_t>(outModel.vertices.size());
                uint32_t baseIndex = static_cast<uint32_t>(outModel.indices.size());
                cgltf_size vCount = posAcc->count;

                bool primSkinned = (jointsAcc != nullptr && weightsAcc != nullptr && node->skin != nullptr);
                if (primSkinned) anySkinned = true;

                for (cgltf_size v = 0; v < vCount; ++v) {
                    float rawPos[3] = { 0.0f, 0.0f, 0.0f };
                    cgltf_accessor_read_float(posAcc, v, rawPos, 3);

                    float rawNorm[3] = { 0.0f, 1.0f, 0.0f };
                    if (normAcc) cgltf_accessor_read_float(normAcc, v, rawNorm, 3);

                    float uv[2] = { 0.0f, 0.0f };
                    if (uvAcc) cgltf_accessor_read_float(uvAcc, v, uv, 2);

                    float vCol[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
                    if (colAcc) cgltf_accessor_read_float(colAcc, v, vCol, 4);

                    gdyno::PackedVertex pv{};
                    if (primSkinned) {
                        // Для скиннинга вершины хранятся в bind-пространстве меша
                        pv.position[0] = rawPos[0]; pv.position[1] = rawPos[1]; pv.position[2] = rawPos[2];
                        pv.normal[0] = rawNorm[0]; pv.normal[1] = rawNorm[1]; pv.normal[2] = rawNorm[2];
                    } else {
                        transformPoint(worldMat, rawPos, pv.position);
                        transformDir(worldMat, rawNorm, pv.normal);
                    }

                    pv.uv[0] = uv[0];
                    pv.uv[1] = uv[1];
                    pv.color[0] = matColor[0] * vCol[0];
                    pv.color[1] = matColor[1] * vCol[1];
                    pv.color[2] = matColor[2] * vCol[2];
                    pv.emission = matEmission;
                    outModel.vertices.push_back(pv);

                    gdyno::PackedSkinWeight sw{};
                    if (primSkinned) {
                        cgltf_uint j[4] = { 0, 0, 0, 0 };
                        float w[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
                        cgltf_accessor_read_uint(jointsAcc, v, j, 4);
                        cgltf_accessor_read_float(weightsAcc, v, w, 4);

                        float wSum = w[0] + w[1] + w[2] + w[3];
                        if (wSum > 1e-5f) {
                            for (int k = 0; k < 4; ++k) w[k] /= wSum;
                        } else {
                            w[0] = 1.0f;
                        }

                        for (int k = 0; k < 4; ++k) {
                            int32_t mappedBone = 0;
                            if (j[k] < node->skin->joints_count) {
                                const cgltf_node* jointNode = node->skin->joints[j[k]];
                                auto it = boneNodeMap.find(jointNode);
                                if (it != boneNodeMap.end()) mappedBone = it->second;
                            }
                            sw.boneIds[k] = static_cast<uint8_t>(std::clamp(mappedBone, 0, 255));
                            sw.weights[k] = static_cast<uint8_t>(std::round(std::clamp(w[k], 0.0f, 1.0f) * 255.0f));
                        }
                    }
                    outModel.skinWeights.push_back(sw);
                }

                if (prim.indices) {
                    for (cgltf_size i = 0; i < prim.indices->count; ++i) {
                        cgltf_uint idx = static_cast<cgltf_uint>(cgltf_accessor_read_index(prim.indices, i));
                        outModel.indices.push_back(baseVertex + idx);
                    }
                } else {
                    for (uint32_t i = 0; i < static_cast<uint32_t>(vCount); ++i) {
                        outModel.indices.push_back(baseVertex + i);
                    }
                }

                gdyno::Submesh sm;
                sm.name = (mesh->primitives_count > 1) ? (nodeName + "_" + std::to_string(pIdx)) : nodeName;
                sm.indexOffset = baseIndex;
                sm.indexCount = static_cast<uint32_t>(outModel.indices.size()) - baseIndex;
                sm.baseColor = { matColor[0], matColor[1], matColor[2] };
                sm.emission = matEmission;
                outModel.submeshes.push_back(std::move(sm));
            }
        }

        for (cgltf_size c = 0; c < node->children_count; ++c) {
            extractNodeAndChildren(data, node->children[c], boneNodeMap, outModel, anySkinned);
        }
                                }

                                void printUsage() {
                                    std::cout << "Usage: glb2gdyno <input.glb|.gltf> <output.gdyno> [options]\n"
                                    << "Options:\n"
                                    << "  --height <meters>   Auto-scale model so Y-height equals <meters> (e.g. 1.75 for pedestrians)\n"
                                    << "  --length <meters>   Auto-scale model so Z-length equals <meters> (e.g. 4.40 for cars)\n"
                                    << "  --no-center         Keep original origin instead of grounding Y=0 and centering X/Z\n";
                                }

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        printUsage();
        return 1;
    }

    ConvertOptions opts;
    opts.inputPath = argv[1];
    opts.outputPath = argv[2];

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--height" && i + 1 < argc) {
            opts.targetHeight = std::stof(argv[++i]);
        } else if (arg == "--length" && i + 1 < argc) {
            opts.targetLength = std::stof(argv[++i]);
        } else if (arg == "--no-center") {
            opts.groundCenter = false;
        }
    }

    cgltf_options cgltfOpts{};
    cgltf_data* data = nullptr;
    if (cgltf_parse_file(&cgltfOpts, opts.inputPath.c_str(), &data) != cgltf_result_success) {
        std::cerr << "[glb2gdyno] Failed to parse glTF/GLB: " << opts.inputPath << "\n";
        return 1;
    }
    if (cgltf_load_buffers(&cgltfOpts, data, opts.inputPath.c_str()) != cgltf_result_success) {
        std::cerr << "[glb2gdyno] Failed to load glTF buffers\n";
        cgltf_free(data);
        return 1;
    }

    gdyno::Model model;
    std::unordered_map<const cgltf_node*, int32_t> boneNodeMap;

    // 1. Извлечение скелета (если есть skin)
    if (data->skins_count > 0) {
        const cgltf_skin& skin = data->skins[0];
        size_t bCount = std::min(skin.joints_count, gdyno::MAX_SKELETON_BONES);
        for (size_t i = 0; i < bCount; ++i) {
            boneNodeMap[skin.joints[i]] = static_cast<int32_t>(i);
        }

        std::vector<float> ibmFlat(bCount * 16, 0.0f);
        if (skin.inverse_bind_matrices) {
            cgltf_accessor_unpack_floats(skin.inverse_bind_matrices, ibmFlat.data(), bCount * 16);
        }

        for (size_t i = 0; i < bCount; ++i) {
            const cgltf_node* jNode = skin.joints[i];
            gdyno::Bone bn;
            bn.name = jNode->name ? jNode->name : ("bone_" + std::to_string(i));
            bn.parentIndex = -1;
            if (jNode->parent) {
                auto it = boneNodeMap.find(jNode->parent);
                if (it != boneNodeMap.end()) bn.parentIndex = it->second;
            }
            if (skin.inverse_bind_matrices) {
                std::memcpy(bn.inverseBindMatrix.data(), &ibmFlat[i * 16], 16 * sizeof(float));
            } else {
                bn.inverseBindMatrix = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
            }
            if (jNode->has_translation) {
                bn.bindLocalPos = { jNode->translation[0], jNode->translation[1], jNode->translation[2] };
            }
            if (jNode->has_rotation) {
                bn.bindLocalRot = { jNode->rotation[0], jNode->rotation[1], jNode->rotation[2], jNode->rotation[3] };
            }
            if (jNode->has_scale) {
                bn.bindLocalScale = { jNode->scale[0], jNode->scale[1], jNode->scale[2] };
            }
            model.bones.push_back(std::move(bn));
        }
        if (!model.bones.empty()) model.flags |= gdyno::FLAG_SKELETON;
    }

    // 2. Обход сцены и сбор мешей/саб-мешей/сокетов
    bool anySkinned = false;
    const cgltf_scene* scene = data->scene ? data->scene : (data->scenes_count > 0 ? &data->scenes[0] : nullptr);
    if (scene) {
        for (cgltf_size i = 0; i < scene->nodes_count; ++i) {
            extractNodeAndChildren(data, scene->nodes[i], boneNodeMap, model, anySkinned);
        }
    }

    if (!anySkinned) {
        model.skinWeights.clear();
    } else {
        model.flags |= gdyno::FLAG_SKINNING;
    }

    // 3. Извлечение скелетных анимаций (если есть)
    if (!model.bones.empty() && data->animations_count > 0) {
        for (cgltf_size a = 0; a < data->animations_count; ++a) {
            const cgltf_animation& srcAnim = data->animations[a];
            gdyno::AnimClip clip;
            clip.name = srcAnim.name ? srcAnim.name : ("anim_" + std::to_string(a));
            clip.durationSec = 0.0f;

            std::unordered_map<int32_t, size_t> boneToChannel;

            for (cgltf_size c = 0; c < srcAnim.channels_count; ++c) {
                const cgltf_animation_channel& srcCh = srcAnim.channels[c];
                if (!srcCh.target_node || !srcCh.sampler) continue;
                auto it = boneNodeMap.find(srcCh.target_node);
                if (it == boneNodeMap.end()) continue;

                int32_t bIdx = it->second;
                size_t chIdx = 0;
                auto chIt = boneToChannel.find(bIdx);
                if (chIt == boneToChannel.end()) {
                    chIdx = clip.channels.size();
                    boneToChannel[bIdx] = chIdx;
                    gdyno::AnimChannel newCh;
                    newCh.boneIndex = bIdx;
                    clip.channels.push_back(std::move(newCh));
                } else {
                    chIdx = chIt->second;
                }

                const cgltf_animation_sampler* samp = srcCh.sampler;
                if (!samp->input || !samp->output) continue;
                cgltf_size keyCount = samp->input->count;

                for (cgltf_size k = 0; k < keyCount; ++k) {
                    float timeVal = 0.0f;
                    cgltf_accessor_read_float(samp->input, k, &timeVal, 1);
                    clip.durationSec = std::max(clip.durationSec, timeVal);

                    if (srcCh.target_path == cgltf_animation_path_type_translation) {
                        float val[3] = { 0, 0, 0 };
                        cgltf_accessor_read_float(samp->output, k, val, 3);
                        clip.channels[chIdx].posKeys.push_back({ timeVal, { val[0], val[1], val[2] } });
                    } else if (srcCh.target_path == cgltf_animation_path_type_rotation) {
                        float val[4] = { 0, 0, 0, 1 };
                        cgltf_accessor_read_float(samp->output, k, val, 4);
                        clip.channels[chIdx].rotKeys.push_back({ timeVal, { val[0], val[1], val[2], val[3] } });
                    } else if (srcCh.target_path == cgltf_animation_path_type_scale) {
                        float val[3] = { 1, 1, 1 };
                        cgltf_accessor_read_float(samp->output, k, val, 3);
                        clip.channels[chIdx].scaleKeys.push_back({ timeVal, { val[0], val[1], val[2] } });
                    }
                }
            }

            if (!clip.channels.empty()) {
                model.animations.push_back(std::move(clip));
            }
        }
        if (!model.animations.empty()) model.flags |= gdyno::FLAG_ANIMATIONS;
    }

    // 4. Вычисление AABB и авто-нормализация масштаба/опорной точки
    if (!model.vertices.empty()) {
        float minP[3] = {  1e9f,  1e9f,  1e9f };
        float maxP[3] = { -1e9f, -1e9f, -1e9f };
        for (const auto& v : model.vertices) {
            for (int k = 0; k < 3; ++k) {
                minP[k] = std::min(minP[k], v.position[k]);
                maxP[k] = std::max(maxP[k], v.position[k]);
            }
        }

        float scaleFactor = 1.0f;
        float curH = maxP[1] - minP[1];
        float curL = maxP[2] - minP[2];
        if (opts.targetHeight > 0.0f && curH > 1e-5f) {
            scaleFactor = opts.targetHeight / curH;
        } else if (opts.targetLength > 0.0f && curL > 1e-5f) {
            scaleFactor = opts.targetLength / curL;
        }

        float offsetX = (opts.groundCenter && !anySkinned) ? -0.5f * (minP[0] + maxP[0]) * scaleFactor : 0.0f;
        float offsetY = (opts.groundCenter && !anySkinned) ? -minP[1] * scaleFactor : 0.0f;
        float offsetZ = (opts.groundCenter && !anySkinned) ? -0.5f * (minP[2] + maxP[2]) * scaleFactor : 0.0f;

        float finalMin[3] = {  1e9f,  1e9f,  1e9f };
        float finalMax[3] = { -1e9f, -1e9f, -1e9f };
        for (auto& v : model.vertices) {
            v.position[0] = v.position[0] * scaleFactor + offsetX;
            v.position[1] = v.position[1] * scaleFactor + offsetY;
            v.position[2] = v.position[2] * scaleFactor + offsetZ;
            for (int k = 0; k < 3; ++k) {
                finalMin[k] = std::min(finalMin[k], v.position[k]);
                finalMax[k] = std::max(finalMax[k], v.position[k]);
            }
        }
        for (auto& sk : model.sockets) {
            sk.position.x = sk.position.x * scaleFactor + offsetX;
            sk.position.y = sk.position.y * scaleFactor + offsetY;
            sk.position.z = sk.position.z * scaleFactor + offsetZ;
        }

        model.aabbMin = { finalMin[0], finalMin[1], finalMin[2] };
        model.aabbMax = { finalMax[0], finalMax[1], finalMax[2] };
    }

    cgltf_free(data);

    std::string saveErr;
    if (!gdyno::Loader::saveToFile(opts.outputPath, model, &saveErr)) {
        std::cerr << "[glb2gdyno] " << saveErr << "\n";
        return 1;
    }

    std::cout << "[glb2gdyno] Saved " << opts.outputPath
    << " | verts=" << model.vertices.size()
    << " tris=" << (model.indices.size() / 3)
    << " submeshes=" << model.submeshes.size()
    << " sockets=" << model.sockets.size()
    << " bones=" << model.bones.size()
    << " anims=" << model.animations.size() << "\n";
    return 0;
}
