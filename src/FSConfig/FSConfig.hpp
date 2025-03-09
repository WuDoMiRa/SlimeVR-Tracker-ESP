#ifndef FS_H
#define FS_H

#include "../GlobalVars.h"
namespace SlimeVR {
    /// @brief Filesystem wrapper. Can save/read/delete `json`. Upgrade from the last Configuration.h/cpp files.
    struct FSConfig {
        SlimeVR::Logger logger = SlimeVR::Logger(Serial, "SlimeVR", "FSConfig");
        FSConfig(){
            //int err = lfs_mount(&lfs, &cfg);
            //if (err) { // cant mount so reformat
            //    lfs_format(&lfs, &cfg);
            //    lfs_mount(&lfs, &cfg);
            //}
        }
        //lfs_t lfs;
        //lfs_file_t file;
        //lfs_config cfg;
    
        //void SaveJSON(char* filename, JsonDocument &doc){
            
        //    lfs_file_open(&lfs, &file, filename, LFS_O_RDWR | LFS_O_CREAT);
            //serializeJson(doc, file);
        //    lfs_file_close(&lfs, &file);
        //}
    
        //void ReadJSON(char* filename, JsonDocument &doc){
        //    int exists=lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY);
        //    if(exists<0){
        //        logger.error("File %s does not exist, and thus this will crash the program. :(", filename);
        //        return;
        //    }
            //deserializeJson(doc, file);
        //    lfs_file_close(&lfs, &file);
        //}
    
        //void deleteFile(char* filename){lfs_remove(&lfs, filename);}
        //bool fileExists(char* filename){return lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY)>=0;}
    };
}
#endif // FS_H