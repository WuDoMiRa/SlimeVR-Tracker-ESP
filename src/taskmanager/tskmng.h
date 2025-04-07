/// Task manager is a class that aims to battle the inherent limitations of the ESP8266 board.
// What it will do is, it will make sure that anything that is required to be updated, will be done when its turn is up,
// and then the function that was just called removed from the queue, depending on how long it wants to stay.

// This battles the design concepts of the stock slimevr firmware, and overall should be much cleaner,
// rather than have a single class have its own .h and .cpp file for an unimportant task.

#ifndef __TASK_MANAGER_H
#define __TASK_MANAGER_H
#include "GlobalVars.h"
#include <functional>
#include <vector>
#include <any>

namespace SlimeVR {
    enum TaskType {
        RUN_ONCE, // runs once
        RUN_EVERY_CYCLE, // runs every cycle
        RUN_EVERY_CYCLE_WITH_INTERVAL, // runs every cycle with an interval (in milliseconds)
    };

    /// @brief This is a state that can hold dynamic data. You can assign anything to it like State['abc']=3, but you must cast it back to the original type, or base class.
    struct TaskState {
        std::map<std::string, std::any> data;
        std::any& operator[](const std::string& key) {
            return data[key];
        }
        const std::any& operator[](const std::string& key) const {
            return data.at(key);
        }
        bool contains(const std::string& key) const {
            return data.find(key) != data.end();
        }
        void erase(const std::string& key) {
            data.erase(key);
        }
        void clear() {
            data.clear();
        }
        size_t size() const {
            return data.size();
        }
        // add == capability
        bool operator==(const TaskState& other) const {
            if (data.size() != other.data.size()) return false;
            for (const auto& pair : data) {
                if (!other.contains(pair.first)) return false;
            }
            return true;
        }
    };

    
    typedef std::function<void(TaskState &state)> TaskFunc; // A function that takes a JsonDocument and returns void.
    /// @brief This is a task.
    struct Task {
        Task() : type(RUN_ONCE), interval(0), logger(Serial, "SlimeVR", "Task") {}  // Default constructor
        Task(TaskType t, TaskFunc f, unsigned long i = 0) 
            : type(t), func(f), interval(i), logger(Serial, "SlimeVR", "Task") {}
        virtual void init() = 0;
        //Task() : type(RUN_ONCE), interval(0), logger {} // Default constructor.
        TaskState state; // Data for the task to store stuff in. You can also store things in this arbitrarily. i.e inputting imu1, imu2, etc.
        TaskType type; // The type of task.
        TaskFunc func; // The function to run.
        unsigned long interval; // The interval to run the task at. This is only used for RUN_EVERY_CYCLE_WITH_INTERVAL tasks. is in milliseconds.
        SlimeVR::Logger logger; // The logger for the task.
        //Task(TaskType type, std::function<void()> func, unsigned long interval) : type(type), func(func) {}
    };

    /// @brief TaskManager is a class that manages tasks.
    struct TaskManager {
        SlimeVR::Logger logger = SlimeVR::Logger(Serial,"SlimeVR","TaskManager"); // The logger for the task.
        std::vector<std::shared_ptr<Task>> tasks; // Managed task pointers
        unsigned long lastRunTime; // The last time the task manager ran. This is used to calculate the time since the last run.

        /// TODO: use-after-free is occuring because, when a task is created out of scope `main.cpp` and `addTask` is called,
        /// the variable is now deleted as soon as it leaves scope, and therefore
        /// now we have a freed variable that wont do anything inside of the list of tasks. 
        void addTask(std::shared_ptr<Task> task) {
            tasks.push_back(task);
        };
        void removeTask(std::shared_ptr<Task> task) {
            tasks.erase(
                std::remove_if(tasks.begin(), tasks.end(),
                    [&task](const auto& t) { return t->state == task->state; }),
                tasks.end()
            );
        };
        
        ~TaskManager() {
            tasks.clear();
        }

        void runTasks() {
            // Enhanced stack protection
            volatile uint32_t stackCanary;
            __asm__ __volatile__ ("mov %0, sp" : "=r" (stackCanary));
            if(stackCanary < 0x3FFE8000) {
                logger.error("STACK SMASH DETECTED! SP: 0x%08x", stackCanary);
                ESP.reset();
            }
            
            if(stackCanary < 0x3FFE8000) { // Check stack pointer is in valid range
                logger.error("Stack corruption detected!");
                ESP.reset();
            }

            unsigned long currentTime = millis();
            ESP.wdtFeed();
            for (auto& task : tasks) {
                ESP.wdtDisable();
                if (task->type == RUN_ONCE) {
                    task->func(task->state); // Run the task.
                    removeTask(task); // Remove the task after running it.
                } else if (task->type == RUN_EVERY_CYCLE) {
                    task->func(task->state); // Run the task.
                } else if (task->type == RUN_EVERY_CYCLE_WITH_INTERVAL) {
                    if (currentTime - lastRunTime >= task->interval) {
                        task->func(task->state); // Run the task.
                    }
                }
                ESP.wdtEnable(100); // Re-enable WDT after task execution
            }
            lastRunTime = currentTime; // Update the last run time.
        };
    };
}
#endif