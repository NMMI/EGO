#include "qbHand.h"

//-----------------------------------------------------
//                                               qbHand
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbHand class
/ *****************************************************
/   parameters:
/               id, ID of the cube
/   return:
/
*/

qbHand::qbHand(int id) : qbInterface(id) {}

//-----------------------------------------------------
//                                               qbHand
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbHand class
/ *****************************************************
/   parameters:
/       cs, communications channel
/       id, ID of the cube
/   return:
/
*/

qbHand::qbHand(comm_settings* cs, int id) : qbInterface(cs, id) {
    retrieveLimits();
    retrievePID();
}


//-----------------------------------------------------
//                                              ~qbHand
//-----------------------------------------------------

/*
/ *****************************************************
/ Destructor of qbHand class
/ *****************************************************
/   parameters:
/   return:
/
*/

qbHand::~qbHand() {}


//====================== OTHER FUNCTIONS =======================================

//-----------------------------------------------------
//                                           setPosPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Set closure of the hand
/ *****************************************************
/   parameters:
/       - rateC, close hand rate
/   return:
/       [state]
/
*/

bool qbHand::setPosPerc(float rateC) {

    // Check input value

    if (rateC < 0.0)
        rateC = 0;

    if (rateC > 1.0)
        rateC = 1.0;

    short int curr_ref[2];

    // Motor of the hand position

    curr_ref[0] = rateC * POS_LIMIT_M1_[1] / 4.0 * axis_dir_;
    curr_ref[1] = rateC * POS_LIMIT_M1_[1] / 4.0 * axis_dir_;

    // Call cube position

    commSetInputs(cube_comm_, id_, curr_ref);

    return true;
}


//-----------------------------------------------------
//                                           getPosPerc
//-----------------------------------------------------

/*
/ *****************************************************
/ Get Measurement in angle of the hand closure
/ *****************************************************
/   parameters:
/   return:
/               value in radiants of hand closure
/
*/

bool qbHand::getPosPerc(float* angle) {

    // Get measurment in tic

    short int meas[3];

    if (!getMeas(meas))
        return false;

    // Trasform in perc and return measured value

    *angle = (((float) meas[0]) / DEG_TICK_MULTIPLIER) * (M_PI/180.0) / (POS_LIMIT_M1_[1] / 4) * axis_dir_;

    return true;
}

//-----------------------------------------------------
//                                         getPIDparams
//-----------------------------------------------------

/*
/ *****************************************************
/ Get position PID params
/ *****************************************************
/   parameters:
/   return:
/               values of hand PID parameters
/
*/

bool qbHand::getPIDparams(float* params) {

    for (int i=0; i<3; i++) {
        params[i] = position_PID[i];
    }

    return true;
}

//-----------------------------------------------------
//                                       retrieveLimits
//-----------------------------------------------------

/*
/ *****************************************************
/ Inizialize default values for the cube
/ *****************************************************
/   parameters:
/   return:
/
*/

void qbHand::retrieveLimits() {

    int pos_limits[4];
    int PARAM_POS_LIMIT = 10; // Position limits index

    POS_LIMIT_M1_[0] = POS_LIMIT_M1_[1] = 0;
    POS_LIMIT_M2_[0] = POS_LIMIT_M2_[1] = 0;

    unsigned char parameter_buffer[3000];
    
    // Retrieve information
    for (int i = 0; i < NUM_OF_TRIALS; i++) {
        if(!commGetParamList(cube_comm_, id_, 0, NULL, 0, 0, parameter_buffer)) {
            // Save limits
            for (int i=0; i<2; i++) {
              int parameter_field = 0;
              for (int j=0; j<sizeof(int); j++) {
                POS_LIMIT_M1_[i] += parameter_buffer[PARAM_POS_LIMIT*PARAM_BYTE_SLOT + 8 + i*sizeof(int) + sizeof(int) - j - 1] << (8 * j);
              }
            }

            POS_LIMIT_M1_[0] = POS_LIMIT_M1_[0] / 2;
            POS_LIMIT_M1_[1] = POS_LIMIT_M1_[1] / 2;
            POS_LIMIT_M2_[0] = 0;
            POS_LIMIT_M2_[1] = 0;
            return;
        }
    }
   std::cerr << "Unable to retrieve hand params. ID: " << id_ << std::endl;
}

//-----------------------------------------------------
//                                          retrievePID
//-----------------------------------------------------

/*
/ *****************************************************
/ Inizialize default values for the cube
/ *****************************************************
/   parameters:
/   return:
/
*/

void qbHand::retrievePID() {

    int PARAM_PID = 1; // position PID parametere index

    position_PID[0] = position_PID[1] = position_PID[2] = 0.0;
    float aux_fl;

    unsigned char parameter_buffer[3000];
    
    // Retrieve information
    for (int i = 0; i < NUM_OF_TRIALS; i++) {
        if(!commGetParamList(cube_comm_, id_, 0, NULL, 0, 0, parameter_buffer)) {
            // Save limits
            for (int i=0; i<3; i++) {
              for (int j=0; j<sizeof(float); j++) {
                ((char *) &aux_fl)[0] = parameter_buffer[PARAM_PID*PARAM_BYTE_SLOT + 8 + i*sizeof(float) +3];
                ((char *) &aux_fl)[1] = parameter_buffer[PARAM_PID*PARAM_BYTE_SLOT + 8 + i*sizeof(float) +2];
                ((char *) &aux_fl)[2] = parameter_buffer[PARAM_PID*PARAM_BYTE_SLOT + 8 + i*sizeof(float) +1];
                ((char *) &aux_fl)[3] = parameter_buffer[PARAM_PID*PARAM_BYTE_SLOT + 8 + i*sizeof(float)];
                position_PID[i] = (float) (aux_fl);
              }
            }
            std::cout << "PID params. ID " << id_  << " : "<< position_PID[0] << " " << position_PID[1] << " " << position_PID[2] << std::endl;

            return;
        }
    }
   std::cerr << "Unable to retrieve hand PID params. ID: " << id_ << std::endl;

}

//-----------------------------------------------------
//                                          setPID
//-----------------------------------------------------

/*
/ *****************************************************
/ Set values for the cube
/ *****************************************************
/   parameters:
/        values of the hand PID
/   return:
/
*/

void qbHand::setPID(float k_p, float k_i, float k_d) {

    int PARAM_SET_PID = 2; // position PID parametere index
    float aux_float[3] = {0,0,0}; 

    aux_float[0] = k_p;
    aux_float[1] = k_i;
    aux_float[2] = k_d;

    commGetParamList(cube_comm_, id_, PARAM_SET_PID, aux_float, sizeof(float), 3, NULL);
    usleep(100000);
    commStoreParams(cube_comm_, id_);
    usleep(100000);
}

//-----------------------------------------------------
//                                        getPosAndCurr
//-----------------------------------------------------

/*
/ *****************************************************
/ Get positions and current for each motor, default 
/ input is set to radiants
/ *****************************************************
/   arguments:
/       - position, measured angle
/       - current, motor current
/       - unit, measurement unit for position
/               and stiffness [rad or deg]
/   return:
/       true  on success
/       false on failure
/
*/

bool qbHand::getPosAndCurr(float* position, float* current, angular_unit unit) {
     
    short int meas[3], curr[2];     

    // Get position measurements and currents
    if(!getMeasAndCurr(meas, curr))
        return false;

    // Return position in the right unit

    if (unit == DEG)
        position[0] = (((float) meas[0]) / DEG_TICK_MULTIPLIER);
    else
        if (unit == RAD)
            position[0] = (((float) meas[0]) / DEG_TICK_MULTIPLIER) * (M_PI / 180);
        else
            position[0] = (float) meas[0];

    // save Currents
    current[0] = (float) curr[0];
    current[1] = (float) curr[1];

    return true;
}

