// 文件名：link_connector.cc
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class LinkConnector : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // 获取链接名称
      if (_sdf->HasElement("link1"))
        this->link1Name = _sdf->Get<std::string>("link1");
      else
        gzerr << "Plugin missing <link1> element\n";

      if (_sdf->HasElement("link2"))
        this->link2Name = _sdf->Get<std::string>("link2");
      else
        gzerr << "Plugin missing <link2> element\n";

      // 获取偏移
      if (_sdf->HasElement("offset"))
      {
        sdf::ElementPtr offsetElem = _sdf->GetElement("offset");
        if (offsetElem->HasElement("position"))
          this->offsetPos = offsetElem->Get<ignition::math::Vector3d>("position");
        if (offsetElem->HasElement("orientation"))
          this->offsetRot = offsetElem->Get<ignition::math::Vector3d>("orientation");
      }

      // 获取链接指针
      this->link1 = _model->GetLink(this->link1Name);
      this->link2 = _model->GetLink(this->link2Name);

      if (!this->link1 || !this->link2)
      {
        gzerr << "Unable to find specified links\n";
        return;
      }

      // 连接到世界更新事件
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&LinkConnector::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // 获取当前链接姿态
      ignition::math::Pose3d pose1 = this->link1->WorldPose();
      ignition::math::Pose3d pose2 = this->link2->WorldPose();

      // 计算目标姿态
      ignition::math::Pose3d targetPose = pose1 + ignition::math::Pose3d(this->offsetPos, ignition::math::Quaterniond(this->offsetRot));

      // 计算位置和角度误差
      ignition::math::Vector3d posError = targetPose.Pos() - pose2.Pos();
      ignition::math::Quaterniond rotError = targetPose.Rot() * pose2.Rot().Inverse();

      // 计算力和力矩（简单的比例控制器）
      double kp_pos = 1000.0;
      double kp_rot = 1000.0;

      ignition::math::Vector3d force = kp_pos * posError;
      ignition::math::Vector3d torque = kp_rot * rotError.Euler();

      // 施加力和力矩到 link2
      this->link2->AddForce(force);
      this->link2->AddTorque(torque);
    }

    private: physics::LinkPtr link1;
    private: physics::LinkPtr link2;
    private: std::string link1Name;
    private: std::string link2Name;
    private: ignition::math::Vector3d offsetPos;
    private: ignition::math::Vector3d offsetRot;
    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_MODEL_PLUGIN(LinkConnector)
}
