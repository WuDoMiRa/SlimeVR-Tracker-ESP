#include "IMU.h"
#include "ICM42688.h"
namespace SlimeVR {
    struct ICM42688_DRIVER : IMUObj<ICM42688_FIFO> {
        ICM42688_DRIVER(){
            /// TODO: warning! setting parameters like this may require double defines based on what filter we'd like!
            // set parameters
            vqf_parameters.tauAcc = 0.5f;                // Faster response for limbs/body parts
            vqf_parameters.restThGyr = 0.15f;             // Avoid false rest detection
            vqf_parameters.restThAcc = 0.2f;
            vqf_parameters.restMinT = 0.5f;               // Short rest detection window
            vqf_parameters.motionBiasEstEnabled = true;   // Enable dynamic bias updates
            vqf_parameters.biasForgettingTime = 50.0f;    // Faster bias adaptation
            vqf_parameters.biasSigmaInit = 0.1f;          // Higher initial uncertainty
            vqf_filter = VQF(vqf_parameters, 0.01);
        };
        char* name = "ICM42688";
        //int id = 6;  --- I don't know the id.
        
        bool imu_init() override {
            imu_drv = std::make_unique<ICM42688_FIFO>(ICM42688_FIFO(Wire, 0x68, PIN_IMU_SDA, PIN_IMU_SCL));
            // configuration imu driver side
            return ( // all of this should return true if all of these are successful. did this to avoid checking if status is error every single one of these calls.
                !(imu_drv->begin()<0) && 
                !(imu_drv->enableFifo(true,true,true)<0) &&
                !(imu_drv->setGyroODR(ICM42688::ODR::odr2k)<0) && 
                !(imu_drv->setAccelODR(ICM42688::ODR::odr1k)<0) && 
                !(imu_drv->setAccelFS(ICM42688::gpm16)<0)
            );
        };

        void update() override {
            if (!(imu_drv->getAGT()>0)){
                // we cannot get new data.
                logger.error("Failed to get new data from the IMU.");
            }
            acceleration={imu_drv->accX(),imu_drv->accY(),imu_drv->accZ()}; // update accel
            gyro={imu_drv->gyrX(),imu_drv->gyrY(),imu_drv->gyrZ()}; // update gyro
            VQF_update(); // after calling this, we get an updated quaternion.
            /// TODO: developer note, don't even try using things like .get_euler_xyz() on the `quaternion` class. The function isn't implemented in the math library (cpp, not h) side.
            /// Because who-ever added the library took it from the godot engine.
            //rotation=quat.get_euler_xyz(); // update rotation
            /// I just removed 'math' and just added eigen. Still not a perfect solution but, whatever. I totally understand,
            /// Wanting a smaller binary rather than using a useless big library where you're mainly going to be using vector3's and quaternions.
            /// It's not even compiling because 'boost' doesn't exist
            /// instead replaced with 'gmath'.
            rotation=quat.normalized().toRotationMatrix().eulerAngles(0,1,2);
        };
    };    
}