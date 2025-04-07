#ifndef __SFS_H__
#define __SFS_H__
#include "ArduinoJson.hpp"
#include <LittleFS.h>
#include "GlobalVars.h"
#include <vector>
namespace SlimeVR {
    typedef ArduinoJson::JsonDocument JsonDocument;
    typedef ArduinoJson::DynamicJsonDocument DynJsonDocument;
    typedef ArduinoJson::JsonArray JsonArray;
    /// @brief File system class, used for reading and writing JSON files to the file system.
    /// This is an improvement over SlimeVR's current FileSystem class, as previously everything is hard coded (no dynamic saving/loading on request) and instead not correctly
    /// abstracted to my liking, or rather, not useful enough for the future, so this class is going to basically let anyone
    /// save and read json files from the filesystem, which allows for dynamic data saving and loading, without even needing to touch this abstraction,
    /// compared to SlimeVR's stock abstraction.
    struct FS {
        SlimeVR::Logger logger = SlimeVR::Logger(Serial, "SlimeVR", "FS");
        
        bool safeFileOperation(const char* filepath, const char* mode, std::function<bool(File&)> op) {
            ESP.wdtDisable();
            noInterrupts();
            bool result = false;
            if(ESP.getFreeHeap() > 25000) {
                File file = LittleFS.open(filepath, mode);
                if(file) {
                    result = op(file);
                    file.close();
                }
            }
            interrupts();
            ESP.wdtEnable(100);
            return result;
        }

        FS() {
            if (!LittleFS.begin()) {
                logger.error("Failed to mount LittleFS filesystem. Reformatting?");
                // If LittleFS isn't setup, set it up here.
                /// TODO: uncomment this line after testing. LittleFS.format();
            }
        }

        // Check if a file exists in the filesystem.
        bool exists(const char* filepath);

        // Delete a file from the filesystem
        bool deleteFile(const char* filepath);

        // Create a new file in the filesystem
        bool createFile(const char* filepath); 

        // Save JSON data to a file.
        bool saveJSON(const char* filepath, const JsonDocument& document);

        // Read JSON data from a file. The data from the file is deserialized into the provided JsonDocument.
        // Returns true if successful, false otherwise.
        bool readJSON(const char* filepath, JsonDocument& document);

        // List all files that exist from the rootpath `path`.
        std::vector<String> listFiles(const char* path, bool isRecursive = false);
    };
}
#endif