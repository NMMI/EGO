#ifndef QBHAND_H
#define QBHAND_H

#include <qbInterface.h>

#define NUM_OF_TRIALS 5


class qbHand : public qbInterface
{
    public:
        //------------------------------ qbHand
        // Costructor of qbHand class
        qbHand(int);

        //------------------------------ qbHand
        // External costructor of qbHand class
        qbHand(comm_settings*, int);

        //------------------------------ ~qbHand
        // Destructor of qbHand class
        ~qbHand();


        // ------------------------------
        //      Other Functions
        // ------------------------------

        //------------------------------ setPosition
        // Set closure of the hand
        bool setPosPerc(float);

        //------------------------------ getAngle
        // Get Measurement in angle of the hand closure
        bool getPosPerc(float*);

        //------------------------------ getAngle
        // Get Position and Current of the hand closure
        bool getPosAndCurr(float*, float*, angular_unit);

        //------------------------------ getPIDparams
        // Get position PID params
        bool getPIDparams(float*);

        //------------------------------ init
        // Inizialize default limits for the hand
        void retrieveLimits();

        // Inizialize default values for the PID
        void retrievePID();

        // Set values for the PID
        void setPID(float, float, float);

    protected:

        // Position limit for motors
        int POS_LIMIT_M1_[2], POS_LIMIT_M2_[2];


        // Actual hand PID parameters
        float position_PID[3];

    private:

};

#endif // QBHAND_H