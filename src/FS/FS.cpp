#include "FS.h" 
namespace SlimeVR {
    bool FS::exists(const char* filepath) {
        return LittleFS.exists(filepath);
    }

    bool FS::deleteFile(const char* filepath) {
        return LittleFS.remove(filepath);
    }

    bool FS::createFile(const char* filepath) {
        File file = LittleFS.open(filepath, "w");
        if (!file) return false;
        file.close();
        return true;
    }

    bool FS::saveJSON(const char* filepath, const JsonDocument& document) {
        File file = LittleFS.open(filepath, "w");
        if (!file) return false;
        bool success = serializeJson(document, file) > 0;
        file.close();
        return success;
    }

    bool FS::readJSON(const char* filepath, JsonDocument& document) {
        File file = LittleFS.open(filepath, "r");
        if (!file) return false;
        ArduinoJson::DeserializationError error = deserializeJson(document, file);
        file.close();
        return error == ArduinoJson::DeserializationError::Ok;
    }

    std::vector<String> FS::listFiles(const char* path, bool isRecursive) {
        std::vector<String> files;
        Dir dir = LittleFS.openDir(path);
        while (dir.next()) {
            if (dir.isFile()) {
                files.push_back(dir.fileName());
            } else if (isRecursive && dir.isDirectory()) {
                std::vector<String> subFiles = listFiles((String(path) + "/" + dir.fileName()).c_str(), true);
                files.insert(files.end(), subFiles.begin(), subFiles.end());
            }
        }
        return files;
    }
}