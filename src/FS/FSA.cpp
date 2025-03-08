#include "FSA.h"
#include <LittleFS.h>

FSA::FSA() {
    // Initialize LittleFS
    if (!LittleFS.begin()) {
        Serial.println("An Error has occurred while mounting LittleFS");
    }
}

bool FSA::begin() {
    return LittleFS.begin();
}

bool FSA::saveJson(const char* filename, JsonDocument& json) {
    if (!fileExists(filename)) {
        if (!createFile(filename)) {
            return false;
        }
    }

    String jsonString;
    serializeJson(json, jsonString);
    return writeFile(filename, jsonString.c_str());
}

bool FSA::readJson(const char* filename, JsonDocument& json) {
    if (!fileExists(filename)) {
        return false;
    }

    String content;
    if (readFile(filename, content)) {
        DeserializationError error = deserializeJson(json, content);
        return error == DeserializationError::Ok;
    }
    return false;
}

bool FSA::fileExists(const char* filename) {
    return LittleFS.exists(filename);
}

bool FSA::createFile(const char* filename) {
    FILE file = lfs_file_open();
    if (!file) {
        return false;
    }
    file.close();
    return true;
}

bool FSA::writeFile(const char* filename, const char* content) {
    File file = LittleFS.open(filename, FILE_WRITE);
    if (!file) {
        return false;
    }
    file.print(content);
    file.close();
    return true;
}

bool FSA::readFile(const char* filename, String& content) {
    File file = LittleFS.open(filename, FILE_READ);
    if (!file) {
        return false;
    }
    content = file.readString();
    file.close();
    return true;
}