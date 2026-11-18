#pragma once

#include <filesystem>
#include <string>
#include "gdyno/GdynoModel.h"

namespace gdyno {

    class Loader {
    public:
        static bool loadFromFile(const std::filesystem::path& filePath, Model& outModel, std::string* outError = nullptr);
        static bool saveToFile(const std::filesystem::path& filePath, const Model& model, std::string* outError = nullptr);
    };

} // namespace gdyno
