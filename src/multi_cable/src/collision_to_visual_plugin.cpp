#include <gazebo/gazebo.hh>
#include <ros/ros.h>

#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>

#include <sdf/sdf.hh> // Add this line
#include <sdf/Element.hh>

namespace gazebo
{
  class CollisionToVisualPlugin : public ModelPlugin
  {
  private:
      physics::ModelPtr model;
      // physics::LinkPtr link;
      event::ConnectionPtr updateConnection;


  public:
    // Initialization
    CollisionToVisualPlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      // Store the model pointer for convenience
      this->model = _model;

      // Check if ROS is initialized
      if (!ros::isInitialized())
      {
          ROS_FATAL("ROS is not initialized. Plugin will not work.");
          return;
      }

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CollisionToVisualPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // Loop to get cylinder links
      for (auto link : this->model->GetLinks())
      {
        // Add a if to determine whether to skip this link
        if(link->GetName.find("cylinder") != std::string::npos){
          // Loop to get target link's collisions
          for (auto collision : link->GetCollisions())
          {
            auto collisionGeometry = collision->GetShape()->GetGeometry();
            auto visualGeometry = link->visual->GetShape()->GetGeometry();
            auto visual = link->GetVisual(collision->GetName());

            if (visual)
            {
              visual->SetShape(collision->GetShape());
            }
            else
            {
              ROS_FATAL("Cylinder link <visual> lable not found. Failed to update");
              // auto newVisual = link->CreateVisual(collision->GetName());
              // newVisual->SetShape(collision->GetShape());
            }
          }
        }
        else {
              ROS_INFO("Not 'xx_cylinder' link, skip.");
              continue;
        }

      }
    }

  };

  GZ_REGISTER_MODEL_PLUGIN(CollisionToVisualPlugin)
}
