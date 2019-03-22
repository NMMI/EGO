#include <iostream>

#include "qbInterface.h"



//-----------------------------------------------------
//                                          qbInterface
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbInterface class
/ *****************************************************
/   argument:
/       - id, ID of the board
/   return:
/
*/

qbInterface::qbInterface(const int id) {

    // Set axis direction

    if (id >= 0)
        axis_dir_ = 1;
    else
        axis_dir_ = -1;

    id_ = abs(id);

    cube_comm_ = NULL;
}


//-----------------------------------------------------
//                                          qbInterface
//-----------------------------------------------------

/*
/ *****************************************************
/ External costructor of qbInterface class
/ *****************************************************
/   argument:
/       - cs, serial communication pointer
/       - id, ID of the board
/   return:
/
*/

qbInterface::qbInterface(comm_settings* cs, const int id) {

    // Set axis direction

    if (id >= 0)
        axis_dir_ = 1;
    else
        axis_dir_ = -1;

    id_ = abs(id);

    cube_comm_ = cs;
}


//-----------------------------------------------------
//                                         ~qbInterface
//-----------------------------------------------------

/*
/ *****************************************************
/ Distructor of qbInterface class
/ *****************************************************
/
/   arguments:
/   return:
/
*/

qbInterface::~qbInterface() {

    close();

}


//======================= OTHER FUNCTIONS ======================================

//-----------------------------------------------------
//                                                 open
//-----------------------------------------------------

/*
/ *****************************************************
/ Open serial communication with cube
/ *****************************************************
/   arguments:
/       port, the communication port
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::open(const char* port) {

    // Check Connection State

    if(cube_comm_ != NULL) {
        cerr << "WARNING: Port already opened" << endl;
        return false;
    }

    cube_comm_ = new comm_settings;

    // Establish serial connection

    openRS485(cube_comm_, port);

    if (cube_comm_->file_handle == INVALID_HANDLE_VALUE) {
        cerr << "ERROR: Unable to open port" << endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------
//                                                close
//-----------------------------------------------------

/*
/ *****************************************************
/ Close serial communication
/ *****************************************************
/   arguments:
/   return:
/
*/

void qbInterface::close() {

    deactivate();

    // close commnication
    if (cube_comm_ != NULL) {
        closeRS485(cube_comm_);
        delete cube_comm_;
        cube_comm_ = NULL;
    }
}


//-----------------------------------------------------
//                                             activate
//-----------------------------------------------------

/*
/ *****************************************************
/ Activate the cube
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::activate() {

    

    char status = 0;

    short int inputs[2];
    inputs[0] = 0;
    inputs[1] = 0;

    commSetInputs(cube_comm_, id_, inputs);

    // Check connection status
    if (cube_comm_ == NULL) {
        cerr << "ERROR:[qbInterface activate()] Port not opened" << endl;
        return false;
    }

    // Activate board
    commActivate(cube_comm_, id_, 1);

    // Wait setting time

    usleep(1000);

    // Check if board is active
    commGetActivate(cube_comm_, id_, &status);

    // Check status
    if (!status){
        cerr << "Unable to activate ID: " << id_ <<  endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------
//                                           deactivate
//-----------------------------------------------------

/*
/ *****************************************************
/ Active the cube
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::deactivate() {

    char status = 0;

    // Check connection status
    if (cube_comm_ == NULL){
        //cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    // Deactivate board
    commActivate(cube_comm_, id_, 0);

    // Wait setting time
    usleep(1000);

    // Check if the board is inactive
    commGetActivate(cube_comm_, id_, &status);

    if (status) {
        cerr << "Unable to deactivate" << endl;
        return false;
    }

    return true;
}


//-----------------------------------------------------
//                                              getMeas
//-----------------------------------------------------

/*
/ *****************************************************
/ Get measurement of positions [1, 2, 3]  in ticks
/ *****************************************************
/   arguments:
/       - meas, 3 elements array pointer for measurements
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::getMeas(short int* meas) {

    if (cube_comm_ == NULL) {
        cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    if (commGetMeasurements(cube_comm_, id_, meas) < 0)
        return false;

    // Axis direction
    
    meas[0] *= axis_dir_;
    meas[1] *= axis_dir_;
    meas[2] *= axis_dir_;

    return true;
}


//-----------------------------------------------------
//                                            setInputs
//-----------------------------------------------------

/*
/ *****************************************************
/ Set board inputs in ticks [1, 2]
/ *****************************************************
/   arguments:
/       - inputs, 2 elements array pointer for inputs
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::setInputs(short int* inputs) {

    if (cube_comm_ == NULL) {
        cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    // Axis direction

    inputs[0] *= axis_dir_;
    inputs[1] *= axis_dir_;

    commSetInputs(cube_comm_, id_, inputs);

    return true;
}

//-----------------------------------------------------
//                                                getID
//-----------------------------------------------------

/*
/ *****************************************************
/ Getter for variable ID
/ *****************************************************
/   arguments:
/   return:
/       ID
/
*/

int qbInterface::getID() {

    return id_;
}

//-----------------------------------------------------
//                                       getMeasAndCurr
//-----------------------------------------------------

/*
/ *****************************************************
/ Get measurement of positions [1, 2, 3]  in ticks and
/ currents of [1, 2] motors
/ *****************************************************
/   arguments:
/       - curr, 3 elements array pointer for measurements
/       - meas, 3 elements array pointer for measurements
/   return:
/       true  on success
/       false on failure
/
*/

bool qbInterface::getMeasAndCurr(short int* meas, short int* curr) {

    short int aux[5];

    if (cube_comm_ == NULL) {
        cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    if (commGetCurrAndMeas(cube_comm_, id_, aux) < 0)
        return false;

    // Current

    curr[0] = aux[0];
    curr[1] = aux[1];

    // Motor pos and axis direction
    
    meas[0] = aux[2] * axis_dir_;
    meas[1] = aux[3] * axis_dir_;
    meas[2] = aux[4] * axis_dir_;

    return true;
}


/* END OF FILE */
