#include <myrobot/myrobot_hardware_interface.h>

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

using namespace std;

myrobot::myrobot(ros::NodeHandle& nh, uint8_t type, double rate) : nh_(nh) {

// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

    ros::Duration update_freq = ros::Duration(1.0/rate);

	rs232Handle_ = open("/dev/ttyACM0", O_RDWR); //Open the rs232 file descriptor in read/write mode
	//rs232Handle_ = open("/tmp/ttyV0", O_RDWR); //Open the rs232 file descriptor in read/write mode
	if (rs232Handle_ < 0) {
		printf("Error %i from open: %s\n", errno, strerror(errno));
		return;
	}
	else
	{
		printf("Opened success\n");
	}

	// Create new termios struc, we call it 'tty' for convention
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Read in existing settings, and handle any error
	if(tcgetattr(rs232Handle_, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return;
	}
	tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
	cfmakeraw(&tty);         // sets all the things to make it RAW

	tty.c_cc[VTIME] = 0;     // Wait for up to 100ms (1 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;
	// Set in/out baud rate to be 115200
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(rs232Handle_, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}

	rs232_buffer_[0] = type;//'Z';
	rs232_buffer_[1] = 13;
	int qty_written = ::write(rs232Handle_,rs232_buffer_,2);
   	printf("Request photon get ready for ROS: ");
	if(type == 'Y')
	{
	   	printf("velocity");
	}
	else
	{
	   	printf("position");
	}
   	printf(" type at %fHz\n", rate);
   	printf("update freq %fsec\n", update_freq.toSec());
	int qty_read = 0;
	ros::Duration(0.1).sleep();

	rs232_buffer_[0]=0;
	while(rs232_buffer_[0] != type)
	{
		rs232_buffer_[0]=0;
		qty_read = ::read(rs232Handle_, rs232_buffer_, 1);
		//printf("waiting for response (read%u) %c\n", qty_read, rs232_buffer_[0]);
		ros::Duration(0.05).sleep();
		if(qty_read == 0 || rs232_buffer_[0] == 10)
		{
			rs232_buffer_[0] = type;
			rs232_buffer_[1] = 13;
			qty_written = ::write(rs232Handle_,rs232_buffer_,2);
		   	//printf("Request photon get ready for ROS again\n");
		}
	}

	// clear buffer first
	::tcflush(rs232Handle_,TCIOFLUSH);

   	printf("\nPhoton in ROS mode\n");

//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &myrobot::update, this);
}


myrobot::~myrobot() {
}


void myrobot::init() {

/////////////////////////////////////////////////        
// Create joint_state_interface for JointA
    hardware_interface::JointStateHandle jointStateHandleA("jt1_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandleA);
// Create joint_state_interface for JointB
    hardware_interface::JointStateHandle jointStateHandleB("jt2_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandleB);
// Create joint_state_interface for JointC
    hardware_interface::JointStateHandle jointStateHandleC("jt3_joint", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandleC);
// Create joint_state_interface for JointD
    hardware_interface::JointStateHandle jointStateHandleD("jt4_joint", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
    joint_state_interface_.registerHandle(jointStateHandleD);
// Create joint_state_interface for JointE
    hardware_interface::JointStateHandle jointStateHandleE("jt5_joint", &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
    joint_state_interface_.registerHandle(jointStateHandleE);


// Create position joint interface as JointA accepts position command.
    //hardware_interface::JointHandle jointPositionHandleA(jointStateHandleA, &joint_position_command_[0]);
    //position_joint_interface_.registerHandle(jointPositionHandleA);
// Create velocity joint interface as JointA accepts velocity command.
    hardware_interface::JointHandle jointVelocityHandleA(jointStateHandleA, &joint_velocity_command_[0]);
    velocity_joint_interface_.registerHandle(jointVelocityHandleA); 
// Create Joint Limit interface for JointA
    joint_limits_interface::getJointLimits("jt1_joint", nh_, limits);
    //joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleA(jointPositionHandleA, limits);
    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandleA(jointVelocityHandleA, limits);
    //positionJointSaturationInterface.registerHandle(jointLimitsHandleA);
	velocityJointSaturationInterface.registerHandle(jointLimitsHandleA);
    
// Create position joint interface as JointB accepts position command.
    //hardware_interface::JointHandle jointPositionHandleB(jointStateHandleB, &joint_position_command_[1]);
    //position_joint_interface_.registerHandle(jointPositionHandleB);
// Create velocity joint interface as JointB accepts velocity command.
    hardware_interface::JointHandle jointVelocityHandleB(jointStateHandleB, &joint_velocity_command_[1]);
    velocity_joint_interface_.registerHandle(jointVelocityHandleB); 
// Create Joint Limit interface for JointB
    joint_limits_interface::getJointLimits("jt2_joint", nh_, limits);
    //joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleB(jointPositionHandleB, limits);
    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandleB(jointVelocityHandleB, limits);
    //positionJointSaturationInterface.registerHandle(jointLimitsHandleB);
	velocityJointSaturationInterface.registerHandle(jointLimitsHandleB);

// Create position joint interface as JointC accepts position command.
    //hardware_interface::JointHandle jointPositionHandleC(jointStateHandleC, &joint_position_command_[2]);
    //position_joint_interface_.registerHandle(jointPositionHandleC);
// Create velocity joint interface as JointC accepts velocity command.
    hardware_interface::JointHandle jointVelocityHandleC(jointStateHandleC, &joint_velocity_command_[2]);
    velocity_joint_interface_.registerHandle(jointVelocityHandleC); 
// Create Joint Limit interface for JointC
    joint_limits_interface::getJointLimits("jt3_joint", nh_, limits); 	
    //joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleC(jointPositionHandleC, limits);
    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandleC(jointVelocityHandleC, limits);
    //positionJointSaturationInterface.registerHandle(jointLimitsHandleC);    
    velocityJointSaturationInterface.registerHandle(jointLimitsHandleC);    

// Create position joint interface as JointD accepts position command.
    //hardware_interface::JointHandle jointPositionHandleD(jointStateHandleD, &joint_position_command_[3]);
    //position_joint_interface_.registerHandle(jointPositionHandleD);
// Create velocity joint interface as JointD accepts velocity command.
	hardware_interface::JointHandle jointVelocityHandleD(jointStateHandleD, &joint_velocity_command_[3]);
    velocity_joint_interface_.registerHandle(jointVelocityHandleD); 
// Create Joint Limit interface for JointD
    joint_limits_interface::getJointLimits("jt4_joint", nh_, limits);
    //joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleD(jointPositionHandleD, limits);
    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandleD(jointVelocityHandleD, limits);
    //positionJointSaturationInterface.registerHandle(jointLimitsHandleD);
    velocityJointSaturationInterface.registerHandle(jointLimitsHandleD);

// Create position joint interface as JointE accepts position command.
    hardware_interface::JointHandle jointPositionHandleE(jointStateHandleE, &joint_position_command_[4]);
    position_joint_interface_.registerHandle(jointPositionHandleE);
// Create Joint Limit interface for JointE
    joint_limits_interface::getJointLimits("jt5_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleE(jointPositionHandleE, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandleE);


// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&effortJointSaturationInterface);
    registerInterface(&velocityJointSaturationInterface);
    registerInterface(&positionJointSaturationInterface);

}



//This is the control loop
void myrobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    comms_to_controller(elapsed_time_);
    controller_manager_->update(ros::Time::now(), elapsed_time_);
}


void myrobot::comms_to_controller(ros::Duration elapsed_time) {
    // Safety
    positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for all joints

	// need to convert from radians to position counts
	//  so if we have 8389pulse per 180deg or 3.141592654radian, then it's factored by 2670.301635196
	uint16_t joint_target[5];

	rs232_buffer_[0] = 0;
	rs232_buffer_[1] = 0;

	if(joint_position_command_[0] > 0) { joint_target[0] = (uint16_t)((3.141592654 + joint_position_command_[0]) * 2670.301635196); }
	else { joint_target[0] = 8389 - (uint16_t)(abs(joint_position_command_[0]) * 2670.301635196); }
	rs232_buffer_[2] = joint_target[0] & 0xff;
	rs232_buffer_[3] = joint_target[0] >> 8 & 0xff;

	if(joint_position_command_[1] > 0) { joint_target[1] = (uint16_t)((3.141592654 + joint_position_command_[1]) * 2670.301635196); }
	else { joint_target[1] = 8389 - (uint16_t)(abs(joint_position_command_[1]) * 2670.301635196); }
	rs232_buffer_[4] = joint_target[1] & 0xff;
	rs232_buffer_[5] = joint_target[1] >> 8 & 0xff;

	if(joint_position_command_[2] > 0) { joint_target[2] = (uint16_t)((3.141592654 + joint_position_command_[2]) * 2670.301635196); }
	else { joint_target[2] = 8389 - (uint16_t)(abs(joint_position_command_[2]) * 2670.301635196); }
	rs232_buffer_[6] = joint_target[2] & 0xff;
	rs232_buffer_[7] = joint_target[2] >> 8 & 0xff;

	if(joint_position_command_[3] > 0) { joint_target[3] = (uint16_t)((3.141592654 + joint_position_command_[3]) * 2670.301635196); }
	else { joint_target[3] = 8389 - (uint16_t)(abs(joint_position_command_[3]) * 2670.301635196); }
	rs232_buffer_[8] = joint_target[3] & 0xff;
	rs232_buffer_[9] = joint_target[3] >> 8 & 0xff;

	// Joint 5 is prismatic (aka linear from 0 to 300 counts equals 0 to 0.030m)
	joint_target[4] = (uint16_t)(joint_position_command_[4] * 10000);
	rs232_buffer_[10] = joint_target[4] & 0xff;
	rs232_buffer_[11] = joint_target[4] >> 8 & 0xff;

	// add joint speed
	// need to convert from radians/sec to PWM output value
	//  so if we have 8389pulse per 180deg or 3.141592654radian, then it's factored by 2670.301635196/radian
	// Photon accepts 0-127 as rev, 128 as stop, 129 to 255 as fwd
	//
	// Might need to deadband around zero
	double deadband = 0.001;
	double speed_value;
	rs232_buffer_[12] = 0; // there is no joint0
	for(int i=0; i<4;i++)
	{
		// test for forward velocity
		if(joint_velocity_command_[i] > deadband)
		{
			// calculate the required speed by factoring the velocity command by some magic number
			speed_value = (joint_velocity_command_[i] * 12700) + 45;
			if(speed_value > 255) speed_value = 255; // full speed forward
			if(speed_value < 129) speed_value = 128;
			rs232_buffer_[13 + i] = (uint8_t)speed_value;
		}
		// test for reverse velocity
		if(joint_velocity_command_[i] < (-1 * deadband))
		{
			speed_value = (abs(joint_velocity_command_[i]) * 12700) + 45;
			if(speed_value < 0) speed_value = 0;     // full speed reverse
			if(speed_value > 127) speed_value = 128;
			rs232_buffer_[13 + i] = 128 - (uint8_t)speed_value;
		}
		// test for zero velocity request (or within deadband zone)
		if((joint_velocity_command_[i] <= deadband) && (joint_velocity_command_[i] >= (-1 * deadband)))
		{
			rs232_buffer_[13 + i] = 128; // or zero velocity
		}
	}
	// joint 5 is a position joint so doesnt require speed, so its fixed
	rs232_buffer_[17] = 150; // joint 5 speed

	//ros::Time time_now;
	//time_now = ros::Time::now();
	//uint64_t time_diff = time_now.toNSec() - time_nsec;
	//time_nsec = time_now.toNSec();
	//printf("JT3 speed:%03u - %lu\n", rs232_buffer_[15], time_diff / 1000000);
	//system("clear");
	printf("JT1 JT2 JT3 JT4 JT5\n");
	printf("%03u %03u %03u %03u %03u\n", rs232_buffer_[13], rs232_buffer_[14], rs232_buffer_[15], rs232_buffer_[16], rs232_buffer_[17]);

	//if(joint_target[3] != pre_targ[3])
	//{
	//	printf("new target JT4:%u\n", joint_target[3]);
	//	pre_targ[3] = joint_target[3];
	//}

	// clear buffer first, VERY important! screws comms otherwise
	::tcflush(rs232Handle_,TCIOFLUSH);

	int qty_written;
	uint8_t crc = 0x83;
	for(int i=0; i<18;i++)
	{
		crc ^= rs232_buffer_[i];
	}

	rs232_buffer_[18] = crc; // put crc at the end for termination
	qty_written = ::write(rs232Handle_,rs232_buffer_,19);
	if (qty_written != 19) {
		/* ERROR HANDLING: rs232 transaction failed */
		//printf("Failed to write to the rs232 bus (%u : %u). %d\n", write_success, write_failure, errno);
		++write_failure;
	}
	else {
		++write_success;
	}
	//printf("Write finished:");
	//for(int i=0; i <= 18; i++)
	//{
	//	printf("%02x,",rs232_buffer_[i]);
	//}
	//printf("\n");

	char rx_char[256];
	int rx_ptr = 0;
	int qty_read;
	qty_read = 0;
	bool success=false;

	while(!success) //qty_read == 0 && rx_ptr < 255 && !success)
	{
		//printf("about to read\n");
		qty_read = ::read(rs232Handle_, rx_char, 1);
		if((qty_read > 0 || rx_ptr == 15) && rx_ptr < 255)
		{
/*			printf("read=%u,", qty_read);
			for(int transfer=0; transfer<qty_read; transfer++)
			{
				rs232_buffer_[rx_ptr] = (uint8_t)rx_char[transfer] & 0xff;
				printf("buf[%u]=0x%02x,", rx_ptr, (int)rs232_buffer_[rx_ptr]);
				++rx_ptr;
				if(rx_ptr > 255) break;
			}
			printf("\n");*/
			rs232_buffer_[rx_ptr] = (uint8_t)rx_char[0] & 0xff;
			++rx_ptr;

			if(rx_ptr > 14)
			{
				//printf("\n");
				crc = 0x83;
				for(int i=0; i<14;i++)
				{
					crc ^= rs232_buffer_[i];
				}

				//printf("HB  PKT PO0     PO1     PO2     PO3     PO4     PO5\n");
				//for(int readread=0; readread<rx_ptr; readread++)
				//{
				//	printf("%03u ", (int)rs232_buffer_[readread]);
				//}
				//printf("\n");

				if (crc == rs232_buffer_[14]) {
					// need to work out pulses per radian
					// so if we have 8389pulse per 180deg or 3.14159radian, then its a factor of 0.00037449
					uint16_t positions[5];

					positions[0] = (((uint16_t)rs232_buffer_[5] << 8) + rs232_buffer_[4]);
					//printf("JT1:%u",positions[0]);
					positions[1] = (((uint16_t)rs232_buffer_[7] << 8) + rs232_buffer_[6]);
					//printf("JT2:%u",positions[1]);
					positions[2] = (((uint16_t)rs232_buffer_[9] << 8) + rs232_buffer_[8]);
					//printf("JT3:%u",positions[2]);
					positions[3] = (((uint16_t)rs232_buffer_[11] << 8) + rs232_buffer_[10]);
					//printf("JT4:%u",positions[3]);
					positions[4] = (((uint16_t)rs232_buffer_[13] << 8) + rs232_buffer_[12]);
					//printf("JT5:%u\n",positions[4]);

					joint_position_[0] = (float)positions[0] * 0.00037449 - 3.141592654;
					joint_position_[1] = (float)positions[1] * 0.00037449 - 3.141592654;
					joint_position_[2] = (float)positions[2] * 0.00037449 - 3.141592654;
					joint_position_[3] = (float)positions[3] * 0.00037449 - 3.141592654;

					/*if(positions[0] >= 8389){ joint_position_[0] = (float)positions[0] * 0.00037449; }
					else 					{ joint_position_[0] = (float)positions[0] * 0.00037449 - 3.141592654; }
					if(positions[1] >= 8389){ joint_position_[1] = (float)positions[1] * 0.00037449; }
					else 					{ joint_position_[1] = (float)positions[1] * 0.00037449 - 3.141592654; }
					if(positions[2] >= 8389){ joint_position_[2] = (float)positions[2] * 0.00037449; }
					else 					{ joint_position_[2] = (float)positions[2] * 0.00037449 - 3.141592654; }
					if(positions[3] >= 8389){ joint_position_[3] = (float)positions[3] * 0.00037449; }
					else 					{ joint_position_[3] = (float)positions[3] * 0.00037449 - 3.141592654; }*/
					// Joint is prismatic aka linear between 0 and 300counts or 0 to 0.030m
					joint_position_[4] = (float)positions[4] / 10000;
					//printf("Receiving finished\n");
					rx_ptr = 0;
					success=true;
				}
				else
				{
					//printf("rec crc failed\n");
					// remove one char from the start and shift pointer back to align packet
					for(int i=0; i < sizeof(rs232_buffer_)-1; i++)
					{
						//printf("%02x,",rs232_buffer_[i]);
						rs232_buffer_[i] = rs232_buffer_[i+1];
					}
					//rs232_buffer_[14] = 0;
					--rx_ptr;
					//printf("\n");
				}
			}
		}
		else
		{
			//printf("no more to read\n");
			//ros::Duration(0.01).sleep();
		}
	}
}



int main(int argc, char** argv)
{
	uint8_t control_type;
	double loop_hz_arg;
    printf("Jaycar Robot Arm about to run\n");
    
	cout << "You have entered " << argc 
         << " arguments:" << "\n"; 
  
    for (int i = 0; i < argc; ++i) 
        cout << argv[i] << "\n"; 
  
	if(argc > 3)
	{
		control_type = argv[1][0];
	}
	else
	{
		control_type = 'Z';
	}

	//Set the frequency of the control loop.
    loop_hz_arg= atof(argv[2]);
	printf("rate:%f\n",loop_hz_arg);

    //Initialze the ROS node.
    ros::init(argc, argv, "Jaycar_Robot_Arm_hardware_inerface_node");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(2);

    // Create the object of the robot hardware_interface class and spin the thread. 
    myrobot ROBOT(nh, control_type, loop_hz_arg);
    spinner.spin();
    
    return 0;
}

