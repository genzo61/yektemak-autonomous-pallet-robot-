// Yektemak palet baglayici Gazebo plugin'i
//
// Neden gerekli?
//   Robotu gazebo_ros_planar_move suruyor. Bu plugin linklerin hizini disaridan
//   zorladigi icin ODE'nin surtunme cozucusu ustteki paleti tasiyamiyor; palet
//   robotun ustunden kayiyor. Gercek robotta palet lift tablasina yuk agirligi
//   ve surtunmeyle kilitlenir; simulasyonda bunu sabit bir eklemle taklit ederiz.
//
// Kullanim (robot URDF icinde):
//   <plugin name="pallet_attacher" filename="libyektemak_pallet_attacher.so">
//     <parent_link>lift_platform_link</parent_link>
//     <target_prefix>pallet</target_prefix>
//   </plugin>
//
// Servis:  /pallet_attacher/attach  (std_srvs/SetBool)
//   data=true  -> lift tablasinin tam ustundeki paleti bagla
//   data=false -> bagli paleti birak
// Konu:    /pallet_attacher/attached (std_msgs/String, bagli paletin adi ya da "")

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace yektemak
{

class PalletAttacher : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    ros_node_ = gazebo_ros::Node::Get(sdf);

    parent_link_name_ = sdf->Get<std::string>("parent_link", "lift_platform_link").first;
    target_prefix_ = sdf->Get<std::string>("target_prefix", "pallet").first;
    max_xy_ = sdf->Get<double>("max_xy_distance", 0.25).first;
    max_yaw_ = sdf->Get<double>("max_yaw_error", 0.35).first;

    parent_link_ = model_->GetLink(parent_link_name_);
    if (!parent_link_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Link bulunamadi: %s", parent_link_name_.c_str());
      return;
    }

    auto qos = rclcpp::QoS(1).transient_local();
    status_pub_ = ros_node_->create_publisher<std_msgs::msg::String>("pallet_attacher/attached", qos);
    PublishStatus();

    service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "pallet_attacher/attach",
      std::bind(&PalletAttacher::OnRequest, this, std::placeholders::_1, std::placeholders::_2));

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&PalletAttacher::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(), "Palet baglayici hazir (link: %s)", parent_link_name_.c_str());
  }

private:
  struct Pending
  {
    bool attach;
    std::promise<std::pair<bool, std::string>> result;
  };

  // Servis cagrisi fizik dongusu disinda gelir; asil isi OnUpdate icinde yapariz.
  void OnRequest(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    std::future<std::pair<bool, std::string>> fut;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = std::make_unique<Pending>();
      pending_->attach = req->data;
      fut = pending_->result.get_future();
    }
    if (fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      res->success = false;
      res->message = "Zaman asimi (simulasyon duraklatilmis olabilir)";
      return;
    }
    auto r = fut.get();
    res->success = r.first;
    res->message = r.second;
  }

  void OnUpdate()
  {
    std::unique_ptr<Pending> p;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_) {
        return;
      }
      p = std::move(pending_);
    }
    p->result.set_value(p->attach ? Attach() : Detach());
  }

  std::pair<bool, std::string> Attach()
  {
    if (joint_) {
      return {true, "Zaten bagli: " + attached_name_};
    }
    const auto parent_pose = parent_link_->WorldPose();
    gazebo::physics::LinkPtr best;
    std::string best_name;
    double best_dist = max_xy_;

    for (const auto & m : world_->Models()) {
      if (m == model_ || m->GetName().rfind(target_prefix_, 0) != 0) {
        continue;
      }
      auto link = m->GetLink();  // kanonik link
      if (!link) {
        continue;
      }
      auto rel = link->WorldPose() - parent_pose;  // parent cercevesinde
      double dxy = std::hypot(rel.Pos().X(), rel.Pos().Y());
      double yaw = std::remainder(rel.Rot().Yaw(), M_PI);  // palet iki yonlu simetrik
      if (dxy < best_dist && std::fabs(yaw) < max_yaw_) {
        best_dist = dxy;
        best = link;
        best_name = m->GetName();
      }
    }
    if (!best) {
      return {false, "Lift ustunde baglanacak palet bulunamadi"};
    }

    // gazebo_ros_link_attacher ile ayni yontem: limitleri sifir olan mafsal,
    // mevcut goreli konumu kilitler.
    joint_ = world_->Physics()->CreateJoint("revolute", model_);
    joint_->Attach(parent_link_, best);
    joint_->Load(parent_link_, best, ignition::math::Pose3d());
    joint_->SetModel(model_);
    joint_->SetUpperLimit(0, 0.0);
    joint_->SetLowerLimit(0, 0.0);
    joint_->Init();
    attached_name_ = best_name;
    PublishStatus();
    RCLCPP_INFO(ros_node_->get_logger(), "Palet baglandi: %s (sapma %.3f m)",
      best_name.c_str(), best_dist);
    return {true, "Baglandi: " + best_name};
  }

  std::pair<bool, std::string> Detach()
  {
    if (!joint_) {
      return {true, "Bagli palet yok"};
    }
    joint_->Detach();
    joint_->Fini();
    joint_.reset();
    std::string name = attached_name_;
    attached_name_.clear();
    PublishStatus();
    RCLCPP_INFO(ros_node_->get_logger(), "Palet birakildi: %s", name.c_str());
    return {true, "Birakildi: " + name};
  }

  void PublishStatus()
  {
    std_msgs::msg::String msg;
    msg.data = attached_name_;
    status_pub_->publish(msg);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr parent_link_;
  gazebo::physics::JointPtr joint_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::mutex mutex_;
  std::unique_ptr<Pending> pending_;
  std::string parent_link_name_, target_prefix_, attached_name_;
  double max_xy_{0.25}, max_yaw_{0.35};
};

GZ_REGISTER_MODEL_PLUGIN(PalletAttacher)

}  // namespace yektemak
