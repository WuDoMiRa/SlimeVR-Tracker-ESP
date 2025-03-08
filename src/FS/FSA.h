#ifndef FS_H
#define FS_H

#include <ArduinoJson.h>

class FSA {
public:
    FSA();
    bool begin();
    bool saveJson(const char* filename, JsonDocument& json);
    bool readJson(const char* filename, JsonDocument& json);

private:
    bool fileExists(const char* filename);
    bool createFile(const char* filename);
    bool writeFile(const char* filename, const char* content);
    bool readFile(const char* filename, String& content);
};

#endif // FS_H