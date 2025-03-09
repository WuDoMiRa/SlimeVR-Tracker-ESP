#include "IMU.h"
#include "ICM42688.h"
namespace SlimeVR {
    struct ICM42688_DRIVER : IMUObj<ICM42688_FIFO> {
        ICM42688_DRIVER() = default;
        char* name = "ICM42688";
        //int id = 6;  --- I don't know the id.
        
        bool imu_init() {
            imu_drv = std::make_unique<ICM42688_FIFO>(ICM42688_FIFO(Wire, 0x68, PIN_IMU_SDA, PIN_IMU_SCL));
            int status = imu_drv->begin();
            if (!(status > 0)) {
                return false; // failed to initialize
            }
            status=imu_drv->enableFifo(true,true,true);
            if (!(status > 0)) {
                return false; // failed to initialize
            }
            return true;
        };

        void update() {
            if (!(imu_drv->getAGT()>0)){
                // we cannot get new data.
                logger.error("Failed to get new data from the IMU.");
            }
            acceleration={imu_drv->accX(),imu_drv->accY(),imu_drv->accZ()};
            rotation={imu_drv->gyrX(),imu_drv->gyrY(),imu_drv->gyrZ()};
        };
    };    
}