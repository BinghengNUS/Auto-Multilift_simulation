#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <functional>
#include <ros/ros.h>

namespace gazebo
{
  class SpringDamperPlugin : public ModelPlugin
  {
  public:
    SpringDamperPlugin() {}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      if (!_sdf->HasElement("joint_name"))
      {
        gzerr << "SpringDamperPlugin missing <joint_name> element\n";
        return;
      }

      std::string jointName = _sdf->Get<std::string>("joint_name");
      
      this->joint = _model->GetJoint(jointName);
      if (!this->joint)
      {
        gzerr << "Joint " << jointName << " not found in model\n";
        return;
      }

      if (_sdf->HasElement("spring_stiffness")){
        this->springStiffness = _sdf->Get<double>("spring_stiffness");
        ROS_INFO("Stiffness of the cable: %.5f", springStiffness);}
      else{
        this->springStiffness = 100.0; // 默认值100
        ROS_INFO("Stiffness of the cable: %.5f", springStiffness);}

      if (_sdf->HasElement("damping_coefficient")){
        this->dampingCoefficient = _sdf->Get<double>("damping_coefficient");
        ROS_INFO("damping of the cable: %.5f", dampingCoefficient);}
      else{
        this->dampingCoefficient = 10.0; // 默认值10
        ROS_INFO("damping of the cable: %.5f", dampingCoefficient);}

      this->restPosition = this->joint->Position(0);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&SpringDamperPlugin::OnUpdate, this));
    }

void OnUpdate()
{
  double velocity = this->joint->GetVelocity(0);

  double extension = this->joint->Position(0) - this->restPosition;

  double springForce = -this->springStiffness * extension;

  double dampingForce = -this->dampingCoefficient * velocity;

  double maxDampingForce = 10000.0; 
  if (dampingForce > maxDampingForce)
    dampingForce = maxDampingForce;
  else if (dampingForce < -maxDampingForce)
    dampingForce = -maxDampingForce;

  if (std::isnan(dampingForce) || std::isinf(dampingForce))
  {
    gzerr << "Damping force is invalid (NaN or Inf)" << std::endl;
    dampingForce = 0.0;     //val should be different for nan and inf!!!!!!
  }

  double totalForce = springForce + dampingForce;

  double maxTotalForce = 1000000.0; // 最大总力
  if (totalForce > maxTotalForce)
    totalForce = maxTotalForce;
  else if (totalForce < -maxTotalForce)
    totalForce = -maxTotalForce;

  if (std::isnan(totalForce) || std::isinf(totalForce))
  {
    gzerr << "Total force is invalid (NaN or Inf)" << std::endl;
    totalForce = 0.0;     //val should be different for nan and inf!!!!!!
  }

  this->joint->SetForce(0, totalForce);
}

  private:
    // 指向关节的指针
    physics::JointPtr joint;

    double springStiffness;

    double dampingCoefficient;

    double restPosition;

    event::ConnectionPtr updateConnection;  //update ration and step?
  };

  // 注册插件
  GZ_REGISTER_MODEL_PLUGIN(SpringDamperPlugin)
}