///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**********************************
 ** Using the ZED with Oculus Rift
 **********************************/

#define NOMINMAX

#include "stdafx.h"

//flag images available 
bool flag_image_L_ = 0;
bool flag_image_R_ = 0;
//flag exit threads
bool flag_left = 0;
bool flag_right = 0;
bool flag1 = 0;
bool flag2 = 0;
bool flag3 = 0;
//mutex images manipulation
std::mutex image_mtx_L_, image_mtx_R_;
//Images to send to the oculus
cv::Mat frame_right_, frame_left_;

geometry_msgs::Vector3 myo_emg_R1_msg, myo_emg_R2_msg, myo_emg_R3_msg, myo_emg_L1_msg, myo_emg_L2_msg, myo_emg_L3_msg;

ros::NodeHandle nh;

//oculus session
ovrSession session;
ovrGraphicsLuid luid;
ovrResult result;

int main(int argc, char **argv) {

    // Initialize SDL2's context
    SDL_Init(SDL_INIT_VIDEO);
	
    // Initialize Oculus' context
    result = ovr_Initialize(nullptr);
    if (OVR_FAILURE(result)) {
        std::cout << "ERROR: Failed to initialize libOVR" << std::endl;
        SDL_Quit();
        return -1;
    }

    
    // Connect to the Oculus headset
    result = ovr_Create(&session, &luid);
    if (OVR_FAILURE(result)) {
        std::cout << "ERROR: Oculus Rift not detected" << std::endl;
        ovr_Shutdown();
        SDL_Quit();
        return -1;
    }

	//char *ros_master = "172.26.5.245:11411";
	char *ros_master = "192.168.0.77:11411";
	//char *ros_master = "192.168.192.120:11411";
	//char *ros_master = "192.168.0.150:11411";
	nh.initNode(ros_master);
	printf("Connecting to server at %s\n", ros_master);

	boost::thread task_left_Th, task_right_Th, task1Th, task2Th, task3Th;
	//rcv images left and right threads
	task_left_Th = boost::thread(left_image_task);
	task_right_Th = boost::thread(right_image_task);
	//rendering oculus thread
	task1Th = boost::thread(oculus_images_task);
	//send joy and head tracking thread
	task2Th = boost::thread(tracking_task);
	//connect to myo
	//task3Th = boost::thread(myo_task);

	task_left_Th.detach();
	task_right_Th.detach();
	task1Th.detach();
	task2Th.detach();
	//task3Th.detach();

	std::cout << "Threads start\n\n";
    
    // Main loop
	//while (!(flag_left || flag_right || flag1 || flag2 || flag3))
	while (!(flag_left || flag_right || flag1 || flag2 ))
	{

	}

	//end oculus session
	ovr_Destroy(session);
	ovr_Shutdown();
    // Quit
    return 0;
}
