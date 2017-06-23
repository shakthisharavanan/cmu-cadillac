/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "AckermannROSPlugin.h"
#include "std_msgs/String.h"
#include <string>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(AckermannROSPlugin)


/////////////////////////////////////////////////
AckermannROSPlugin::AckermannROSPlugin()
{


}

// Destructor
AckermannROSPlugin::~AckermannROSPlugin()
{
    this->rosNode->shutdown();
    this->queue.clear();
    this->queue.disable();
    this->callbackQueueThread.join();
    delete this->rosNode;
}

void AckermannROSPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	
	//ros stuff
    this->rosNode = new ros::NodeHandle("");

    // Get the world name
    this->model = _model;

    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());

	//Parameters and Joints from Gazebo
    //steering
    this->front_left_steering_joint = this->model->GetJoint(_sdf->Get<std::string>("front_left_steering_joint"));
    this->front_right_steering_joint = this->model->GetJoint(_sdf->Get<std::string>("front_right_steering_joint"));

    //Wheels
    this->front_left_wheel_joint = this->model->GetJoint(_sdf->Get<std::string>("front_left_wheel_joint"));
    this->front_right_wheel_joint = this->model->GetJoint(_sdf->Get<std::string>("front_right_wheel_joint"));
    this->rear_left_wheel_joint = this->model->GetJoint(_sdf->Get<std::string>("rear_left_wheel_joint"));
    this->rear_right_wheel_joint = this->model->GetJoint(_sdf->Get<std::string>("rear_right_wheel_joint"));

    //gas and brake
    //this->gasJoint = this->model->GetJoint(_sdf->Get<std::string>("gas"));
    //this->brakeJoint = this->model->GetJoint(_sdf->Get<std::string>("brake"));

    //this->steeringJoint = this->model->GetJoint(_sdf->Get<std::string>("steering"));

    this->frontPower = _sdf->Get<double>("front_power");
    this->rearPower = _sdf->Get<double>("rear_power");


    ros::SubscribeOptions ackermann_cmd_so =
        ros::SubscribeOptions::create<ackermann_msgs::AckermannDriveStamped>(
            this->model->GetName() + "/ackermann_cmd", 100,
            boost::bind(&AckermannROSPlugin::receive_ackermann_data, this, _1),
            ros::VoidPtr(), &this->queue);
    this->subAckermannCmd = this->rosNode->subscribe(ackermann_cmd_so);

    // ros callback queue for processing subscription
    this->callbackQueueThread = boost::thread(
        boost::bind(&AckermannROSPlugin::QueueThread, this));

}
/////////////////////////////////////////////////
void AckermannROSPlugin::receive_ackermann_data(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{
    ack_msg.drive=msg->drive;
    //ack_steering_angle.data=msg->drive::steering_angle;
    double wheelAngle =msg->drive.steering_angle;
    ack_speed = ack_msg.drive.speed;

    //this->maxSpeed = ack_speed;
    double jointVel = ack_speed;

    ack_acc = ack_msg.drive.acceleration;
    //printf("Steering angle [%f]\n", ack_steering_angle);

   // this->steeringJoint->SetAngle(0,wheelAngle /* *this->steeringRatio */);

    //double wheelAngle = this->steeringJoint->GetAngle(0).Radian();

    printf("Steering Angle [%f]\n", wheelAngle);
    //ROS_ERROR("receive_ackermann_data");




    // Compute the rotational velocity of the wheels
    //double jointVel = (std::max(0.0, gas-brake) * this->maxSpeed) / this->wheelRadius;
    jointVel = jointVel / this->wheelRadius;
    double torque=10;
    //double jointVel=10.0;
    //gas=10.0;

    // Set velocity and max force for each wheel
     #if GAZEBO_MAJOR_VERSION >= 5 && GAZEBO_MINOR_VERSION >= 0
     
		 this->front_left_wheel_joint->SetVelocityLimit(1, jointVel);
		this->front_left_wheel_joint->SetForce(1, torque* this->frontPower*fabs(jointVel));

		this->front_right_wheel_joint->SetVelocityLimit(1, jointVel);
		this->front_right_wheel_joint->SetForce(1, torque * this->frontPower*fabs(jointVel));

		this->rear_left_wheel_joint->SetVelocityLimit(1, jointVel);
		this->rear_left_wheel_joint->SetForce(1, torque * this->rearPower*fabs(jointVel));

		this->rear_right_wheel_joint->SetVelocityLimit(1, jointVel);
		this->rear_right_wheel_joint->SetForce(1, torque * this->rearPower*fabs(jointVel));
		
		

		
     #else
		this->front_left_wheel_joint->SetVelocity(1, jointVel);
		this->front_left_wheel_joint->SetMaxForce(1, torque* this->frontPower);

		this->front_right_wheel_joint->SetVelocity(1, jointVel);
		this->front_right_wheel_joint->SetMaxForce(1, torque * this->frontPower);

		this->rear_left_wheel_joint->SetVelocity(1, jointVel);
		this->rear_left_wheel_joint->SetMaxForce(1, torque * this->rearPower);

		this->rear_right_wheel_joint->SetVelocity(1, jointVel);
		this->rear_right_wheel_joint->SetMaxForce(1, torque * this->rearPower);
	#endif

    // Set wheel angles; formula is atan(2Ltan(psi)/(2L +(or)- Btan(psi))) where L is wheelbase and B is axle length
    // Set the front-left wheel angle; 
    #if (wheelAngle>=0)
        this->front_left_steering_joint->SetLowStop(0, atan(5.614*tan(wheelAngle)/(5.614-1.627*tan(wheelAngle))));
        this->front_left_steering_joint->SetHighStop(0, atan(5.614*tan(wheelAngle)/(5.614-1.627*tan(wheelAngle))));
        this->front_left_steering_joint->SetLowStop(0, atan(5.614*tan(wheelAngle)/(5.614-1.627*tan(wheelAngle))));
        this->front_left_steering_joint->SetHighStop(0, atan(5.614*tan(wheelAngle)/(5.614-1.627*tan(wheelAngle))));

        // Set the front-right wheel angle
        this->front_right_steering_joint->SetHighStop(0, atan(5.614*tan(wheelAngle)/(5.614+1.627*tan(wheelAngle))));
        this->front_right_steering_joint->SetLowStop(0, atan(5.614*tan(wheelAngle)/(5.614+1.627*tan(wheelAngle))));
        this->front_right_steering_joint->SetHighStop(0, atan(5.614*tan(wheelAngle)/(5.614+1.627*tan(wheelAngle))));
        this->front_right_steering_joint->SetLowStop(0, atan(5.614*tan(wheelAngle)/(5.614+1.627*tan(wheelAngle))));

    #else
        this->front_left_steering_joint->SetLowStop(0, atan(5.614*tan(wheelAngle)/(5.614-1.627*tan(wheelAngle))));
        this->front_left_steering_joint->SetHighStop(0, atan(5.614*tan(wheelAngle)/(5.614-1.627*tan(wheelAngle))));
        this->front_left_steering_joint->SetLowStop(0, atan(5.614*tan(wheelAngle)/(5.614-1.627*tan(wheelAngle))));
        this->front_left_steering_joint->SetHighStop(0, atan(5.614*tan(wheelAngle)/(5.614-1.627*tan(wheelAngle))));

        // Set the front-right wheel angle
        this->front_right_steering_joint->SetHighStop(0, atan(5.614*tan(wheelAngle)/(5.614+1.627*tan(wheelAngle))));
        this->front_right_steering_joint->SetLowStop(0, atan(5.614*tan(wheelAngle)/(5.614+1.627*tan(wheelAngle))));
        this->front_right_steering_joint->SetHighStop(0, atan(5.614*tan(wheelAngle)/(5.614+1.627*tan(wheelAngle))));
        this->front_right_steering_joint->SetLowStop(0, atan(5.614*tan(wheelAngle)/(5.614+1.627*tan(wheelAngle))));       
    // Get the current velocity of the car
    #endif
    this->velocity = this->chassis->GetWorldLinearVel();
	printf("Current velocity of the car:[%f][%f][%f]\n",this->velocity.x,this->velocity.y,this->velocity.z);
}

void AckermannROSPlugin::QueueThread()
{
    static const double timeout = 0.01;

    while (this->rosNode->ok())
    {
        this->queue.callAvailable(ros::WallDuration(timeout));
    }
}

void AckermannROSPlugin::Init()
{
    this->chassis = this->front_left_wheel_joint->GetParent();

    // This assumes that the largest dimension of the wheel is the diameter
    physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
        this->front_left_wheel_joint->GetChild());
    math::Box bb = parent->GetBoundingBox();
    this->wheelRadius = bb.GetSize().GetMax() * 0.5;
    printf("Wheel radius : [%f]\n",this->wheelRadius);

}
void AckermannROSPlugin::OnUpdate()
{

}


