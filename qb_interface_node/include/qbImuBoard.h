#ifndef QBIMUBOARD_H
#define QBIMUBOARD_H

#include <iostream>

#include "qbInterface.h"
#include "imuboard_communications.h"

class qbImuBoard : public qbInterface
{
    public:
        //------------------------------ qbImuBoard
        // Costructor of qbImuBoard class
        qbImuBoard(const int id = 1);

        //------------------------------ qbImuBoard
        // External costructor of qbImuBoard class
        qbImuBoard(comm_settings*, const int id = 1);

        //------------------------------ ~qbImuBoard
        // Distructor of qbImuBoard class
        ~qbImuBoard();

        // ------------------------------
        //      Other Functions
        // ------------------------------
        
        //------------------------------ initImuBoard
        // Get useful structure and data from IMU board
        void initImuBoard();

        //------------------------------ getImuReadings
        // Retrieve accelerometers, gyroscopes and magnetometers readings
        void getImuReadings();


		float*   imu_values_;
        int      n_imu_;
        uint8_t* ids_;
        uint8_t* imu_table_;
        uint8_t* mag_cal_;

    protected:

    private:

};

#endif // QBIMUBOARD_H