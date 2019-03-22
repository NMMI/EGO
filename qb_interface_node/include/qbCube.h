#ifndef QBCUBE_H
#define QBCUBE_H

#include <iostream>

#include "qbInterface.h"

class qbCube : public qbInterface
{
    public:

        //------------------------------ qbCube
        // Costructor of qbCube class
        qbCube(const int id = 1);

        //------------------------------ qbCube
        // External costructor of qbCube class
        qbCube(comm_settings*, const int id = 1);

        //------------------------------ ~qbCube
        // Distructor of qbCube class
        ~qbCube();

        // ------------------------------
        //      Other Functions
        // ------------------------------

        //------------------------------ setPosAndPreset
        // Set position and stiffness preset of the cube, default input is set to radiants
        void setPosAndPreset(float, float, angular_unit unit = RAD);

        //------------------------------ getPosAndPreset
        // Get position and stiffness preset of the cube, default input is set to radiants
        bool getPosAndPreset(float*, float*, angular_unit unit = RAD);

        //------------------------------ setPosAndPresetPerc
        // Set position and stiffness percentage of the cube, default input is set to radiants
        void setPosAndPresetPerc(float, float, angular_unit unit = RAD);

        //------------------------------ setPosAndPresetPerc
        // Get position and stiffness percentage of the cube, default input is set to radiants
        bool getPosAndPresetPerc(float*, float*, angular_unit unit = RAD);

        //------------------------------ getPos
        // Get position angle in the chosen unit of measure
        bool getPos(float*, angular_unit unit = RAD);

        //------------------------------ getPreset
        // Get preset of the cube
        bool getPreset(float*, angular_unit = RAD);

        //------------------------------ getPresetPerc
        // Get preset of the cube
        bool getPresetPerc(float*);

        //------------------------------ getPosAndCurr
        // Get P1/P2/PL measurements and currents of the cube
        bool getPosAndCurr(float*, float*, angular_unit = RAD);

        //------------------------------ getPPAndCurr
        // Get Eq. position/Preset and currents of the cube
        bool getPPAndCurr(float*, float*, float*, angular_unit = RAD);


    protected:

    private:

};

#endif // QBCUBE_H