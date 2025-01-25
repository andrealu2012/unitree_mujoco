#include <cmath>
#include <memory>
#include <mutex>
#include <shared_mutex>



// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_STATE_TOPIC = "rt/lowstate";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

//template <typename T>



// const int G1_NUM_MOTOR = 29;
// struct ImuState {
//   std::array<float, 3> rpy = {};
//   std::array<float, 3> omega = {};
// };
// struct MotorCommand {
//   std::array<float, G1_NUM_MOTOR> q_target = {};
//   std::array<float, G1_NUM_MOTOR> dq_target = {};
//   std::array<float, G1_NUM_MOTOR> kp = {};
//   std::array<float, G1_NUM_MOTOR> kd = {};
//   std::array<float, G1_NUM_MOTOR> tau_ff = {};
// };
// struct MotorState {
//   std::array<float, G1_NUM_MOTOR> q = {};
//   std::array<float, G1_NUM_MOTOR> dq = {};
// };

// // Stiffness for all G1 Joints
// std::array<float, G1_NUM_MOTOR> Kp{
//     60, 60, 60, 100, 40, 40,      // legs
//     60, 60, 60, 100, 40, 40,      // legs
//     60, 40, 40,                   // waist
//     40, 40, 40, 40,  40, 40, 40,  // arms
//     40, 40, 40, 40,  40, 40, 40   // arms
// };

// // Damping for all G1 Joints
// std::array<float, G1_NUM_MOTOR> Kd{
//     1, 1, 1, 2, 1, 1,     // legs
//     1, 1, 1, 2, 1, 1,     // legs
//     1, 1, 1,              // waist
//     1, 1, 1, 1, 1, 1, 1,  // arms
//     1, 1, 1, 1, 1, 1, 1   // arms
// };

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

enum G1JointIndex {
  LeftHipPitch = 0,
  LeftHipRoll = 1,
  LeftHipYaw = 2,
  LeftKnee = 3,
  LeftAnklePitch = 4,
  LeftAnkleB = 4,
  LeftAnkleRoll = 5,
  LeftAnkleA = 5,
  RightHipPitch = 6,
  RightHipRoll = 7,
  RightHipYaw = 8,
  RightKnee = 9,
  RightAnklePitch = 10,
  RightAnkleB = 10,
  RightAnkleRoll = 11,
  RightAnkleA = 11,
  WaistYaw = 12,
  WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
  WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
  LeftShoulderPitch = 15,
  LeftShoulderRoll = 16,
  LeftShoulderYaw = 17,
  LeftElbow = 18,
  LeftWristRoll = 19,
  LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
  LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
  RightShoulderPitch = 22,
  RightShoulderRoll = 23,
  RightShoulderYaw = 24,
  RightElbow = 25,
  RightWristRoll = 26,
  RightWristPitch = 27,  // NOTE INVALID for g1 23dof
  RightWristYaw = 28     // NOTE INVALID for g1 23dof
};



void LowStateHandler(const void *msg)
{
    //     LowState_ low_state = *(const LowState_ *)msg;
        std::cout<<"state"<<std::endl;
   
}




int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_ankle_swing_example network_interface" << std::endl;
    exit(0);
  }

  std::string networkInterface = argv[1];
  std::cout<<"Net : "<<networkInterface<<std::endl;



ChannelFactory::Instance()->Init(1, "lo");
std::cout<<"A: "<<std::endl;
    ChannelPublisher<LowCmd_> lowcmd_publisher_(HG_CMD_TOPIC);
    lowcmd_publisher_.InitChannel();

    std::cout<<"B: "<<std::endl;

    ChannelSubscriber<LowState_> lowstate_suber(HG_STATE_TOPIC);
    lowstate_suber.InitChannel(LowStateHandler);

    std::cout<<"C: "<<std::endl;

    while (true)
    {   
        LowCmd_ low_cmd{};
        for (int i = 0; i < 13; i++)
        {
            // low_cmd.motor_cmd()[i].q() = 0.523;
            // low_cmd.motor_cmd()[i].kp() = 0;
            // low_cmd.motor_cmd()[i].dq() = 0;
            // low_cmd.motor_cmd()[i].kd() = 0;
            // low_cmd.motor_cmd()[i].tau() = 0;

            low_cmd.motor_cmd()[i].q() = 0;
            low_cmd.motor_cmd()[i].kp() = 0;
            low_cmd.motor_cmd()[i].dq() = 0;
            low_cmd.motor_cmd()[i].kd() = 0;
            low_cmd.motor_cmd()[i].tau() = 5;
        }
        lowcmd_publisher_.Write(low_cmd);
        std::cout<<"q = "<<low_cmd.motor_cmd()[0].q()<<std::endl;
        usleep(2000);
    }


  return 0;
}
