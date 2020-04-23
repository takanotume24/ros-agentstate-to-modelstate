/*
Created on Mon Dec  2

@author: mahmoud
*/

#include <gazebo_msgs/ModelState.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <ros/ros.h>

#include <gazebo-9/gazebo/physics/physics.hh>
#include <gazebo-9/gazebo/util/system.hh>
#include <gazebo/common/Plugin.hh>
#include <thread>

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

ros::Publisher rosPub;
std::unique_ptr<ros::NodeHandle> rosNode;
ros::CallbackQueue rosQueue;

void callback(const pedsim_msgs::AgentStatesConstPtr msg) {
  auto agent_states = msg->agent_states;

  for (auto agent_state = agent_states.begin();
       agent_state != agent_states.end(); ++agent_state) {
    gazebo_msgs::ModelState model_state;
    model_state.model_name = std::to_string(agent_state->id);
    model_state.pose = agent_state->pose;
    model_state.twist = agent_state->twist;
    rosPub.publish(model_state);
  }
}

void QueueThread() {
  static const double timeout = 0.1;
  while (rosNode->ok()) {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_publisher");
  ros::NodeHandle n;
  ros::Subscriber rosSub;
  std::thread rosQueueThread;

  rosNode.reset(new ros::NodeHandle("gazebo_client"));
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<pedsim_msgs::AgentStates>(
          "/pedsim_simulator/simulated_agents", 1, &callback, ros::VoidPtr(),
          &rosQueue);

  rosSub = rosNode->subscribe(so);
  rosQueueThread = std::thread(&QueueThread);
  rosPub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1);
  ros::spin();
}

namespace gazebo {
class ActorPosesPlugin : public WorldPlugin {
 public:
  ActorPosesPlugin() : WorldPlugin() {}

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    this->world_ = _world;
    if (!ros::isInitialized()) {
      ROS_ERROR("ROS not initialized");
      return;
    }
    rosNode.reset(new ros::NodeHandle("gazebo_client"));
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<pedsim_msgs::AgentStates>(
            "/pedsim_simulator/simulated_agents", 1,
            boost::bind(&ActorPosesPlugin::OnRosMsg, this, _1), ros::VoidPtr(),
            &rosQueue);

    rosSub = rosNode->subscribe(so);
    rosQueueThread =
        std::thread(std::bind(&ActorPosesPlugin::QueueThread, this));
    rosPub = n.advertise<gazebo_msgs::ModelState>("pose_publisher", 1);
    // in case you need to change/modify model on update
    // this->updateConnection_ =
    // event::Events::ConnectWorldUpdateBegin(std::bind(&ActorPosesPlugin::OnUpdate,
    // this));
  }

 public:
  // call back function when receive rosmsg
  void OnRosMsg(const pedsim_msgs::AgentStatesConstPtr msg) {
    //              ROS_INFO ("OnRosMsg ... ");
    std::string model_name;
    auto models = world_->Models();
    for (auto model = models.begin(); model != models.end(); ++model) {
      physics::ModelPtr tmp_model = *model;
      std::string frame_id = tmp_model->GetName();
      auto agent_states = msg->agent_states;

      for (auto agent_state = agent_states.begin();
           agent_state != agent_states.end(); ++agent_state) {
        if (frame_id != std::to_string(agent_state->id)) {
          continue;
        }
        gazebo_msgs::ModelState model_state;
        model_state.model_name = frame_id;
        model_state.pose = agent_state->pose;
        model_state.twist = agent_state->twist;
        rosPub.publish(model_state);
      }
    }
  }

  // ROS helper function that processes messages
 private:
  void QueueThread() {
    static const double timeout = 0.1;
    while (rosNode->ok()) {
      rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

 private:
  ros::NodeHandle n;
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber rosSub;
  ros::Publisher rosPub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConnection_;
};
// GZ_REGISTER_WORLD_PLUGIN(ActorPosesPlugin)
}  // namespace gazebo
