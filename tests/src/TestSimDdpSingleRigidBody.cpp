/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>

#include <CCC/Constants.h>
#include <CCC/DdpSingleRigidBody.h>

#include "ContactManager.h"
#include "SimModels.h"

using std::placeholders::_1;
using std::placeholders::_2;

class TestSimDdpSingleRigidBody
{
public:
  TestSimDdpSingleRigidBody()
  {
    // Setup ROS
    control_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>("control", 1);
    state_sub_ = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "state", 1, std::bind(&TestSimDdpSingleRigidBody::stateCallback, this, _1));

    forward_srv_ = nh_->create_service<std_srvs::srv::Empty>(
        "/forward", std::bind(&TestSimDdpSingleRigidBody::forwardCallback, this, _1, _2));
    jump_srv_ = nh_->create_service<std_srvs::srv::Empty>(
        "/jump", std::bind(&TestSimDdpSingleRigidBody::jumpCallback, this, _1, _2));
    tilt_srv_ = nh_->create_service<std_srvs::srv::Empty>(
        "/tilt", std::bind(&TestSimDdpSingleRigidBody::tiltCallback, this, _1, _2));
  }

  void run()
  {
    double horizon_dt = 0.03; // [sec]
    double horizon_duration = 3.0; // [sec]
    int horizon_steps = static_cast<int>(horizon_duration / horizon_dt);

    // Setup DDP
    CCC::DdpSingleRigidBody::WeightParam weight_param;
    weight_param.running_pos << 1.0, 1.0, 10.0;
    weight_param.running_ori << 0.5, 0.5, 0.5;
    weight_param.terminal_pos << 1.0, 1.0, 10.0;
    weight_param.terminal_ori << 0.5, 0.5, 0.5;
    CCC::DdpSingleRigidBody ddp(mass_, horizon_dt, horizon_steps, weight_param);
    ddp.ddp_solver_->config().max_iter = 1;
    initial_param_.pos = Eigen::Vector3d(0.0, 0.0, 1.0);

    // Setup contact
    std::function<CCC::DdpSingleRigidBody::MotionParam(double)> motion_param_func = [this](double t)
    {
      CCC::DdpSingleRigidBody::MotionParam motion_param;
      Eigen::Vector2d contact_pos = Eigen::Vector2d::Zero();
      if(forward_duration_ && (*forward_duration_)[0] <= t && t <= (*forward_duration_)[1])
      {
        contact_pos.x() += forward_dist_;
      }
      if(!(jump_duration_ && (*jump_duration_)[0] <= t && t <= (*jump_duration_)[1]))
      {
        motion_param.contact_list.push_back(
            makeContactFromRect({contact_pos + Eigen::Vector2d(-0.5, -0.5), contact_pos + Eigen::Vector2d(0.5, 0.5)}));
      }
      motion_param.inertia_mat.diagonal() = moment_of_inertia_;
      return motion_param;
    };
    std::function<CCC::DdpSingleRigidBody::RefData(double)> ref_data_func = [this](double t)
    {
      CCC::DdpSingleRigidBody::RefData ref_data;
      ref_data.pos << 0.0, 0.0, 1.0;
      if(forward_duration_ && (*forward_duration_)[0] <= t && t <= (*forward_duration_)[1])
      {
        ref_data.pos.x() += forward_dist_;
      }
      if(jump_duration_ && (*jump_duration_)[0] <= t && t <= (*jump_duration_)[1])
      {
        ref_data.pos.z() += jump_height_;
      }
      ref_data.ori.setZero();
      if(tilt_duration_ && (*tilt_duration_)[0] <= t && t <= (*tilt_duration_)[1])
      {
        ref_data.ori.x() += tilt_angle_;
        ref_data.ori.y() += tilt_angle_;
      }
      return ref_data;
    };

    // Run control loop
    rclcpp::Rate rate(200);
    while(rclcpp::ok())
    {
      // rclcpp::Clock clock;
      // rclcpp::Time time_now = clock.now();
      // double t_ = time_now.seconds();
      t_ = nh_->get_clock()->now().seconds();

      if(forward_duration_ && (*forward_duration_)[1] < t_)
      {
        forward_duration_.reset();
      }
      if(jump_duration_ && (*jump_duration_)[1] < t_)
      {
        jump_duration_.reset();
      }
      if(tilt_duration_ && (*tilt_duration_)[1] < t_)
      {
        tilt_duration_.reset();
      }

      rclcpp::spin_some(nh_);

      // Plan
      if(!initial_param_.u_list.empty())
      {
        for(int i = 0; i < ddp.ddp_solver_->config().horizon_steps; i++)
        {
          double tmp_time = t_ + i * ddp.ddp_problem_->dt();
          int input_dim = ddp.ddp_problem_->inputDim(tmp_time);
          if(initial_param_.u_list[i].size() != input_dim)
          {
            initial_param_.u_list[i].setZero(input_dim);
          }
        }
      }
      Eigen::VectorXd planned_force_scales = ddp.planOnce(motion_param_func, ref_data_func, initial_param_, t_);

      // Publish
      std_msgs::msg::Float64MultiArray msg;
      const auto & motion_param = motion_param_func(t_);
      for(const auto & contact : motion_param.contact_list)
      {
        int wrenchRatioIdx = 0;
        for(const auto & vertexWithRidge : contact->vertexWithRidgeList_)
        {
          const Eigen::Vector3d & vertex = vertexWithRidge.vertex;
          const std::vector<Eigen::Vector3d> & ridgeList = vertexWithRidge.ridgeList;

          Eigen::Vector3d vertexForce = Eigen::Vector3d::Zero();
          for(const auto & ridge : ridgeList)
          {
            vertexForce += planned_force_scales(wrenchRatioIdx) * ridge;
            wrenchRatioIdx++;
          }

          std::vector<double> vertexVec(vertex.data(), vertex.data() + 3);
          std::vector<double> vertexForceVec(vertexForce.data(), vertexForce.data() + 3);
          msg.data.insert(msg.data.end(), vertexVec.begin(), vertexVec.end());
          msg.data.insert(msg.data.end(), vertexForceVec.begin(), vertexForceVec.end());
        }
      }
      RCLCPP_INFO(nh_->get_logger(), "Publishing Float64MultiArray: ");
      for (size_t i = 0; i < msg.data.size(); ++i) {
          RCLCPP_INFO(nh_->get_logger(), "  data[%zu] = %f", i, msg.data[i]);
      }
      control_pub_->publish(msg);

      rate.sleep();
    }
  }

protected:
  void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    RCLCPP_INFO(nh_->get_logger(), "stateCallback called");
    const CCC::DdpSingleRigidBody::DdpProblem::StateDimVector & state =
        Eigen::Map<const CCC::DdpSingleRigidBody::DdpProblem::StateDimVector>(msg->data.data());
    initial_param_.pos = state.segment<3>(0);
    initial_param_.ori = state.segment<3>(3);
    initial_param_.linear_vel = state.segment<3>(6);
    initial_param_.angular_vel = state.segment<3>(9);
  }

  void forwardCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, // req
                       std::shared_ptr<std_srvs::srv::Empty::Response> response // res
  )
  {
    (void)request;
    (void)response;

    if(forward_duration_)
    {
      return;
    }

    forward_duration_ = std::make_shared<std::array<double, 2>>();
    (*forward_duration_)[0] = t_ + 2.0;
    (*forward_duration_)[1] = t_ + 4.0;
  }

  bool jumpCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, // req
                    std::shared_ptr<std_srvs::srv::Empty::Response> response // res
  )
  {
    (void)request;
    (void)response;
    RCLCPP_INFO(nh_->get_logger(), "jumpCallback called 1");
    if(jump_duration_)
    {
      RCLCPP_INFO(nh_->get_logger(), "jumpCallback called 2");

      return false;
    }
    RCLCPP_INFO(nh_->get_logger(), "jumpCallback called 3");

    jump_duration_ = std::make_shared<std::array<double, 2>>();
    (*jump_duration_)[0] = t_ + 2.0;
    (*jump_duration_)[1] = t_ + 2.4;
    RCLCPP_INFO(nh_->get_logger(), "jumpCallback called 4");

    return true;
  }

  void tiltCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request, // req
                    std::shared_ptr<std_srvs::srv::Empty::Response> response // res
  )
  {
    (void)request;
    (void)response;

    if(tilt_duration_)
    {
      return;
    }

    tilt_duration_ = std::make_shared<std::array<double, 2>>();
    (*tilt_duration_)[0] = t_ + 2.0;
    (*tilt_duration_)[1] = t_ + 5.0;
  }

protected:
  double t_ = 0.0;

  double mass_ = 10.0; // [kg]
  Eigen::Vector3d moment_of_inertia_ =
      Eigen::Vector3d(0.6166666666666667, 0.48333333333333334, 0.2833333333333333); // [kg m^2]

  CCC::DdpSingleRigidBody::InitialParam initial_param_;

  const double forward_dist_ = 1.5; // [m]
  const double jump_height_ = 0.4; // [m]
  const double tilt_angle_ = 1.0; // [rad]
  std::shared_ptr<std::array<double, 2>> forward_duration_;
  std::shared_ptr<std::array<double, 2>> jump_duration_;
  std::shared_ptr<std::array<double, 2>> tilt_duration_;

  std::shared_ptr<rclcpp::Node> nh_ = rclcpp::Node::make_shared("test_sim_ddp_single_rigid_body");
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr forward_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr jump_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr tilt_srv_;
};

TEST(TestSimDdpSingleRigidBody, Test1)
{
  TestSimDdpSingleRigidBody test;

  test.run();
}

int main(int argc, char ** argv)
{
  // Setup ROS
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
