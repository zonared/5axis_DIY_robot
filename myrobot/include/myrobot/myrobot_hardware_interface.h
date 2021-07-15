#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <errno.h>

class myrobot : public hardware_interface::RobotHW 
{
    public:
        myrobot(ros::NodeHandle& nh, uint8_t type, double rate);
        ~myrobot();
        void init();
        int selectDevice(int fd, int addr, char *name);
        void update(const ros::TimerEvent& e);
        void comms_to_controller(ros::Duration elapsed_time);
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::EffortJointInterface effort_joint_interface_;
		hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::SoftJointLimits soft_limits;
        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        
        double joint_position_[5];
        double joint_velocity_[5];
        double joint_effort_[5];
        double joint_velocity_command_[5];
        double joint_position_command_[5];
        
        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
		uint64_t time_nsec;
        int fd, result;
        int rs232Handle_;
        uint8_t rs232_buffer_[256];
        int8_t heartbeat;
        uint16_t write_success;
        uint16_t write_failure;
		uint16_t pre_targ[6];
        bool rs232_busy;
		uint64_t debug_print;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

