//-----------------------------------------------------
//                                          getCurrents
//-----------------------------------------------------

/*
/ *****************************************************
/ Get currents of motors [1, 2]
/ *****************************************************
/   arguments:
/       - curr, 2 elements array pointer for currents
/   return:
/       true  on success
/       false on failure
/
*/
template <typename T>
bool qbInterface::getCurrents(T* curr) {

    if (cube_comm_ == NULL) {
        cerr << "ERROR: Port not opened" << endl;
        return false;
    }

    short int currBuf[2];

    if (commGetCurrents(cube_comm_, id_, currBuf))
        return false;

    curr[0] = currBuf[0];
    curr[1] = currBuf[1];

    return true;
}
