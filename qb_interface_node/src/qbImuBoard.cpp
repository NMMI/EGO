#include "qbImuBoard.h"

using std::cout;
using std::cerr;
using std::cin;
using std::endl;

//-----------------------------------------------------
//                                           qbImuBoard
//-----------------------------------------------------

/*
/ *****************************************************
/ Costructor of qbImuBoard class
/ *****************************************************
/   argument:
/       - id, ID of the cube
/   return:
/
*/

qbImuBoard::qbImuBoard(const int id) : qbInterface(id) {}

//-----------------------------------------------------
//                                           qbImuBoard
//-----------------------------------------------------

/*
/ *****************************************************
/ External costructor of qbImuBoard class
/ *****************************************************
/   argument:
/       - cs, serial communication pointer to the cube
/       - id, ID of the cube
/   return:
/
*/

qbImuBoard::qbImuBoard(comm_settings* cs, const int id) : qbInterface(cs, id) {
	
	initImuBoard();
	
}


//-----------------------------------------------------
//                                          ~qbImuBoard
//-----------------------------------------------------

/*
/ *****************************************************
/ Distructor of qbImuBoard class
/ *****************************************************
/
/   arguments:
/   return:
/
*/

qbImuBoard::~qbImuBoard() {}


//========================== OTHER FUNCTIONS ===================================


//-----------------------------------------------------
//                                         initImuBoard
//-----------------------------------------------------
void qbImuBoard::initImuBoard() {

	uint8_t aux_string[2000];
	uint8_t PARAM_SLOT_BYTES = 50;
	uint8_t num_imus_id_params = 6;
	uint8_t num_mag_cal_params = 0;
	uint8_t first_imu_parameter = 2;
	int v = 0;
	int num_of_params;
	
	commGetParamList(cube_comm_, id_, 0, NULL, 0, 0, aux_string);
	
	num_of_params = aux_string[5];
	
	//aux_string[6] <-> packet_data[2] on the firmware
	n_imu_ = aux_string[8];
	printf("Number of connected IMUs: %d\n", n_imu_);
	
	// Compute number of read parameters depending on global_args.n_imu and
	// update packet_length
	num_mag_cal_params = (n_imu_ / 2);
	if ( (n_imu_ - num_mag_cal_params*2) > 0 ) num_mag_cal_params++;
	
	ids_ = (uint8_t *) calloc(n_imu_, sizeof(uint8_t));
	v = 0;
	for (int k = 1; k <= num_imus_id_params; k++){
		if (aux_string[k*PARAM_SLOT_BYTES + 8] != 255) {
			ids_[v] = aux_string[k*PARAM_SLOT_BYTES + 8];
			v++;
		}
		if (aux_string[k*PARAM_SLOT_BYTES + 9] != 255) {
			ids_[v] = aux_string[k*PARAM_SLOT_BYTES + 9];
			v++;
		}
		if (aux_string[k*PARAM_SLOT_BYTES + 10] != 255) {
			ids_[v] = aux_string[k*PARAM_SLOT_BYTES + 10];
			v++;
		}
	}
/*	
	for (int i=0; i< n_imu_; i++){
		ids_[i] = aux_string[2*PARAM_SLOT_BYTES + 8 + i];
	}
	*/
	// Retrieve magnetometer calibration parameters
	mag_cal_ = (uint8_t *) calloc(n_imu_, 3*sizeof(uint8_t));
	v = 0;
	for (int k=1; k <= num_mag_cal_params; k++) {
		mag_cal_[3*v + 0] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 8];
		mag_cal_[3*v + 1] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 9];
		mag_cal_[3*v + 2] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 10];
		printf("MAG PARAM: %d %d %d\n", mag_cal_[3*v + 0], mag_cal_[3*v + 1], mag_cal_[3*v + 2]);
		v++;
		
		if (aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 7] == 6) {
			mag_cal_[3*v + 0] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 11];
			mag_cal_[3*v + 1] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 12];
			mag_cal_[3*v + 2] = aux_string[num_imus_id_params*PARAM_SLOT_BYTES + k*PARAM_SLOT_BYTES + 13];
			printf("MAG PARAM: %d %d %d\n", mag_cal_[3*v + 0], mag_cal_[3*v + 1], mag_cal_[3*v + 2]);
			v++;
		}
	}
/*	
	for (int i=0; i< n_imu_; i++){
		mag_cal_[3*i + 0] = aux_string[3*PARAM_SLOT_BYTES + 8 + 3*i];
		mag_cal_[3*i + 1] = aux_string[3*PARAM_SLOT_BYTES + 9 + 3*i];
		mag_cal_[3*i + 2] = aux_string[3*PARAM_SLOT_BYTES + 10 + 3*i];
		printf("MAG PARAM: %d %d %d\n", mag_cal_[3*i + 0], mag_cal_[3*i + 1], mag_cal_[3*i + 2]);
		
	}
*/	
	first_imu_parameter = 1 + num_imus_id_params + num_mag_cal_params + 1;
	imu_table_ = (uint8_t *) calloc(n_imu_, 5*sizeof(uint8_t));
	for (int i=0; i< n_imu_; i++){
		imu_table_[5*i + 0] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 8 + 50*i];
		imu_table_[5*i + 1] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES+ 9 + 50*i];
		imu_table_[5*i + 2] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 10 + 50*i];
		imu_table_[5*i + 3] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 11 + 50*i];
		imu_table_[5*i + 4] = aux_string[first_imu_parameter*PARAM_SLOT_BYTES + 12 + 50*i];
		printf("ID: %d  - %d, %d, %d, %d, %d\n", ids_[i], imu_table_[5*i + 0], imu_table_[5*i + 1], imu_table_[5*i + 2], imu_table_[5*i + 3], imu_table_[5*i + 4]);
		
	}

	
	// Imu values is a 3 sensors x 3 axes x n_imu_ values
	imu_values_ = (float *) calloc(n_imu_, 3*3*sizeof(float)+4*sizeof(float)+sizeof(float));
		
}



//-----------------------------------------------------
//                                  	 getImuReadings
//-----------------------------------------------------
void qbImuBoard::getImuReadings() {
	
	commGetImuReadings(cube_comm_, id_, imu_table_, mag_cal_, n_imu_, imu_values_);
			
	 // for (int i = 0; i < n_imu_; i++) {
		
	 // 	printf("IMU: %d\n", ids_[i]);
	
	 // 	if (imu_table_[5*i + 0]){
	 // 		printf("Accelerometer\n");
	 // 		printf("%f, %f, %f\n", imu_values_[3*3*i], imu_values_[3*3*i+1], imu_values_[3*3*i+2]);
	 // 	}
	 // 	if (imu_table_[5*i + 1]){
	 // 		printf("Gyroscope\n");
		// 	printf("%f, %f, %f\n", imu_values_[3*3*i+3], imu_values_[3*3*i+4], imu_values_[3*3*i+5]);
		// }
		// if (imu_table_[5*i + 2] ){
		// 	printf("Magnetometer\n");
		// 	printf("%f, %f, %f\n", imu_values_[3*3*i+6], imu_values_[3*3*i+7], imu_values_[3*3*i+8]);
	 // 	}
		
		// printf("\n");
	 // }
	
}

/* END OF FILE */
