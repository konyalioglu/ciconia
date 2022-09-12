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
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include <string>
#include <sstream>
#include <ignition/math.hh>


namespace gazebo
{
  /// 
  class gazeboStatePublisher : public ModelPlugin
  {
    /// Constructor
    public: gazeboStatePublisher() {}
    

    
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
        ros::init(argc, argv, "gazebo_statesss",
            ros::init_options::NoSigintHandler);
      }
      
      //this->rosNode.reset(new ros::NodeHandle("gazebo_statesss"));
      
      if (_sdf->HasElement("topic")){
        this->topic_name = _sdf->Get<std::string>("topic");
      }
      if (_sdf->HasElement("rate")){
        this->rate = _sdf->Get<float>("rate");
      }
       
      if (_sdf->HasElement("link_name")){
        link_name_ = _sdf->Get<std::string>("link_name");
        link = _model->GetLink(link_name_);
      } 
      else{
        link = _model->GetLink();
        link_name_ = link->GetName();
        
      }
      std::cerr << "\nThe velocity joint plugin is attached as[" <<
        link_name_ << "]\n";
      
      
      gazeboStatePublisher c_pointer;
      
      //this->rosTimer = this->rosNode.createTimer(ros::Duration(1/this->rate), Foo());
      this->rosTimer = this->rosNode.createTimer(ros::Duration(1/this->rate), &gazeboStatePublisher::callback, this);
      
      /// Position Publisher Pointer      
      this->rosPubAngularVel = this->rosNode.advertise<geometry_msgs::Vector3>("/" + this->topic_name + "/angular_velocity", 3);
           
      /// Linear Velocity Publisher Pointer      
      this->rosPubLinearAccel = this->rosNode.advertise<geometry_msgs::Vector3>("/" + this->topic_name + "/linear_acceleration", 3);
      
      /// Quaternion Publisher Pointer      
      this->rosPubLinearVel = this->rosNode.advertise<geometry_msgs::Vector3>("/" + this->topic_name + "/linear_velocity", 3);
      
      /// Publisher Pointer      
      this->rosPubRobotQuaternion = this->rosNode.advertise<geometry_msgs::Quaternion>("/" + this->topic_name + "/quaternion", 3);
      
      /// Publisher Pointer      
      this->rosPubRobotPosition = this->rosNode.advertise<geometry_msgs::Vector3>("/" + this->topic_name + "/position", 3);

      /// Spin up the queue helper thread.
      this->rosQueueThread =
      std::thread(std::bind(&gazeboStatePublisher::QueueThread, this));
      
    }
    


    public: void callback(const ros::TimerEvent& event)
    {
      ignition::math::Pose3d Pose = this->link->WorldPose();   
      ignition::math::Vector3d Pos = Pose.Pos();
      ignition::math::Quaterniond Quat = Pose.Rot();

      this->robotQuaternion.w  = Quat.W();
      this->robotQuaternion.x  = Quat.X();
      this->robotQuaternion.y  = Quat.Y();
      this->robotQuaternion.z  = Quat.Z();
      this->rosPubRobotQuaternion.publish(this->robotQuaternion);
      
      this->robotPosition.x  = Pos.X();
      this->robotPosition.y  = Pos.Y();
      this->robotPosition.z  = Pos.Z(); 
      this->rosPubRobotPosition.publish(this->robotPosition);
      
      
      ignition::math::Vector3d AngularVel = this->link->WorldAngularVel();      

      this->robotAngularVel.x  = AngularVel.X();
      this->robotAngularVel.y  = AngularVel.Y();
      this->robotAngularVel.z  = AngularVel.Z();
      this->rosPubAngularVel.publish(this->robotAngularVel);
      
      
      ignition::math::Vector3d LinearAccel = this->link->WorldLinearAccel();   
      
      this->robotLinearAccel.x  = LinearAccel.X();
      this->robotLinearAccel.y  = LinearAccel.Y();
      this->robotLinearAccel.z  = LinearAccel.Z();
      this->rosPubLinearAccel.publish(this->robotLinearAccel);


      ignition::math::Vector3d LinearVel = this->link->WorldLinearVel();   
      
      this->robotLinearVel.x  = LinearVel.X();
      this->robotLinearVel.y  = LinearVel.Y();
      this->robotLinearVel.z  = LinearVel.Z();
      this->rosPubLinearVel.publish(this->robotLinearVel);

    }
    

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode.ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    
    /// Pointer to the model.
    private: physics::ModelPtr model;

    ///Pointer to the joint.
    private: physics::JointPtr joint;
    
    private: physics::LinkPtr link;
        
    /// Node Pointer
    public: ros::NodeHandle rosNode;

    
    /// Publisher Pointer
    private: ros::Publisher rosPubAngularVel;
    private: ros::Publisher rosPubLinearAccel;
    private: ros::Publisher rosPubLinearVel;
    private: ros::Publisher rosPubRobotPosition;
    private: ros::Publisher rosPubRobotQuaternion;
    
    /// Publisher Pointer
    private: ros::Timer rosTimer;
    
    /// A ROS callbackqueue 
    private: ros::CallbackQueue rosQueue;

    /// Thread 
    private: std::thread rosQueueThread;
    
    /// Topic Names    
    private: std::string topic_name;
    
    /// Node rate
    private: float rate;
    
    private: ros::Duration period;
    
    std::string link_name_;

    private: geometry_msgs::Vector3 robotAngularVel;
    private: geometry_msgs::Vector3 robotLinearAccel;
    private: geometry_msgs::Vector3 robotLinearVel;
    private: geometry_msgs::Vector3 robotPosition;
    private: geometry_msgs::Quaternion robotQuaternion;
    
    

  };


  GZ_REGISTER_MODEL_PLUGIN(gazeboStatePublisher)
}
#endif
