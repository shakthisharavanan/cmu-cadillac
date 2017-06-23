
#ifndef _GAZEBO_ACKERMANN_ROS_PLUGIN_H_
#define _GAZEBO_ACKERMANN_ROS_PLUGIN_H_
#include <ros/ros.h>
#include <string>
#include <vector>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h> 
#include <std_msgs/String.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//#include <gazebo/util/system.hh>

namespace gazebo
{
  class  AckermannROSPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: AckermannROSPlugin();
    
    public: virtual ~AckermannROSPlugin();
    
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void OnUpdate();

    private: void receive_ackermann_data(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
	private: void receive_joints_data(const sensor_msgs::JointState::ConstPtr& msg);
   // private: std::vector<event::ConnectionPtr> connections;

	private: ros::NodeHandle* rosNode;
    private: ros::CallbackQueue queue;
    private: void QueueThread();
    private: boost::thread callbackQueueThread;
    private: ros::Publisher joint_pub;
    private: ros::Subscriber subAckermannCmd, jointStateSub;
    private: physics::ModelPtr model;
    private: transport::NodePtr node;
    private: physics::LinkPtr chassis;
    private: transport::SubscriberPtr velSub;
    ackermann_msgs::AckermannDriveStamped ack_msg;	
    float ack_steering_angle,ack_speed,ack_acc;		
	
	private: double steeringRatio;
    private: double tireAngleRange;
	
	private: double wheelRadius;
	private: double maxGas, maxBrake;
	private: double frontPower, rearPower;
    private: double maxSpeed;
    
    private: double aeroLoad;
    private: double swayForce;
    
	private: math::Vector3 velocity;
	
	private: sdf::ElementPtr _sdf;
	private: physics::JointPtr front_left_steering_joint, front_right_steering_joint;
	private: physics::JointPtr front_left_wheel_joint,front_right_wheel_joint,rear_left_wheel_joint,rear_right_wheel_joint;
	private: physics::JointPtr gasJoint, brakeJoint, steeringJoint;

   /*
    private: physics::LinkPtr chassis;
    private: std::vector<physics::JointPtr> joints;
    private: physics::JointPtr gasJoint, brakeJoint;
    private: physics::JointPtr steeringJoint;

    private: math::Vector3 velocity;

    

    private: double frontPower, rearPower;
    private: double maxSpeed;
    private: double wheelRadius;

    private: double steeringRatio;
    private: double tireAngleRange;
    private: double maxGas, maxBrake;

    private: double aeroLoad;
    private: double swayForce;*/
  };
}
#endif //__GAZEBO_ACKERMANN_ROS_PLUGIN_H_
