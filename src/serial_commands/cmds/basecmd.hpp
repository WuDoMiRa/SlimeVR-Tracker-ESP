#include <string>
#ifndef BASE_CMD_H 
#define BASE_CMD_H
struct BaseCommand {
    SlimeVR::Logger logger = SlimeVR::Logger(Serial, "SlimeVR", "UnnamedCommand"); // the logger.
    std::string name=""; // The name of the command.
    std::string description=""; // Description of command.
    std::string usage="examplecmd arg1(string)=\"example string\" arg2(bool)=false"; // Help of the usage of command.

    virtual void setup(){}; // Setup function for the command. Probably not required to run.
    virtual void run(std::vector<String> &arguments){}; // Run function for the command.
};
#endif