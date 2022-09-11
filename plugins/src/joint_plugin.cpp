#ifndef _JOINT_PLUGIN_HH_
#define _JOINT_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <sstream>


namespace gazebo
{
  /// 
  class jointVelocityPlugin : public ModelPlugin
  {
    /// Constructor
    public: jointVelocityPlugin() {}
    

    
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {

      std::cerr << "\nThe velocity joint plugin is attached as[" <<
        _model->GetName() << "]\n";

        
      // Safety check
      if (_model->GetJointCount() == 0)
      {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
      }
        
      //model pointer
      this->model = _model;
      
      this->joint = _model->GetJoints()[0];
      
      // Initialize ros
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // ROS Node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Subscriber Options
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float64MultiArray>(
            "/" + this->model->GetName() + "/vel_cmd",
            1,
            boost::bind(&jointVelocityPlugin::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      
      // Subscriber Pointer      
      this->rosSub = this->rosNode->subscribe(so);
      
      
      // Publisher Pointer      
      this->rosPub = this->rosNode->advertise<std_msgs::Float64MultiArray>("/" + this->model->GetName() + "/joint_vel", 1000);

      // Spin up the queue helper thread.
      this->rosQueueThread =
      std::thread(std::bind(&jointVelocityPlugin::QueueThread, this));
      
      
      if (_sdf->HasElement("motor1")){
        motor1_ = _sdf->Get<std::string>("motor1");
      }
      if (_sdf->HasElement("motor2")){
        motor2_ = _sdf->Get<std::string>("motor2");
      }
      if (_sdf->HasElement("motor3")){
        motor3_ = _sdf->Get<std::string>("motor3");
      }
      if (_sdf->HasElement("motor4")){
        motor4_ = _sdf->Get<std::string>("motor4");
      }
      if (_sdf->HasElement("motor5")){
        motor5_ = _sdf->Get<std::string>("motor5");
      }    
    }
    
    
    public: void SetVelocity(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      this->model->GetJoint(motor1_)->SetVelocity(0, msg->data.at(0));
      this->model->GetJoint(motor2_)->SetVelocity(0, msg->data.at(1));
      this->model->GetJoint(motor3_)->SetVelocity(0, msg->data.at(2));
      this->model->GetJoint(motor4_)->SetVelocity(0, msg->data.at(3));
      this->model->GetJoint(motor5_)->SetVelocity(0, msg->data.at(4));
    }


    public: void OnRosMsg(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
      this->SetVelocity(msg);
      this->rosPub.publish(msg);
    }
    

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    
    /// Pointer to the model.
    private: physics::ModelPtr model;

    ///Pointer to the joint.
    private: physics::JointPtr joint;
        
    /// Node Pointer
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// Subscriber Pointer
    private: ros::Subscriber rosSub;
    
    /// Publisher Pointer
    private: ros::Publisher rosPub;

    /// A ROS callbackqueue 
    private: ros::CallbackQueue rosQueue;

    /// Thread 
    private: std::thread rosQueueThread;
    
    /// Joint Names    
    private: std::string motor1_, motor2_, motor3_, motor4_, motor5_;

  };


  GZ_REGISTER_MODEL_PLUGIN(jointVelocityPlugin)
}
#endif
