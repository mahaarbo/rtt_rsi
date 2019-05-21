#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <kdl/frames.hpp>
#include <rtt_rsi/rsi_command.h>
#include <rtt_rsi/rsi_state.h>
#include <rtt_rsi/udp_server.h>

namespace rtt_rsi
{
static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;

class RSIComponent : public RTT::TaskContext
{
public:
  RSIComponent(const std::string& name)
    : RTT::TaskContext(name, PreOperational)
    , local_host_("127.0.0.1")
    , local_port_(49152)
    , joint_position_(12, 0.0)
    , joint_position_command_(12, 0.0)
    , joint_names_(12)
    , rsi_initial_joint_positions_(12, 0.0)
    , rsi_joint_position_corrections_(12, 0.0)
    , ipoc_(0)
    , n_dof_(6)
    , external_axes_(false)
    , use_force_torque_sensor_(false)
  {
    RTT::log(RTT::Info) << "Constructing ! " << RTT::endlog();

    this->ports()->addPort("joint_position_command", port_joint_position_command_).doc("The joint position command to execute.");
    this->ports()->addPort("joint_position", port_joint_position_).doc("The current joint position.");
    this->ports()->addPort("force_torque_sensor", port_ftc_sensor_).doc("If use_force_torque_sensor is true, gives the KDL::Wrench for the force torque sensor.");
    this->addProperty("local_host", local_host_).doc("The IP to connect to the robot.");
    this->addProperty("local_port", local_port_).doc("The port to connect to the robot.");
    this->addProperty("external_axes", external_axes_).doc("If true, allows for control of external axes E1, E2, E3.");
    this->addProperty("use_force_torque_sensor", use_force_torque_sensor_).doc("Whether to use the force-torque sensor or not.");

    in_buffer_.resize(1024);
    out_buffer_.resize(1024);
    remote_host_.resize(1024);
    remote_port_.resize(1024);
  }

  bool configureHook()
  {
    RTT::log(RTT::Info) << "Configuring  ! " << RTT::endlog();
    // Set the vectors to n_dof length
    joint_position_.resize(n_dof_, 0.0);
    joint_position_command_.resize(n_dof_, 0.0);
    port_joint_position_.setDataSample(joint_position_);
    KDL::SetToZero(sensor_wrench_);
    port_ftc_sensor_.setDataSample(sensor_wrench_);
    
    RTT::log(RTT::Info) << "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")" << RTT::endlog();
    server_.reset(new UDPServer(local_host_, local_port_));
    return true;
  }

  bool startHook()
  {
    RTT::log(RTT::Info) << "Starting  ! " << RTT::endlog();
    RTT::log(RTT::Info) << "Waiting for robot!" << RTT::endlog();

    int bytes = server_->recv(in_buffer_);

    // Drop empty <rob> frame with RSI <= 2.3
    if (bytes < 100)
    {
      bytes = server_->recv(in_buffer_);
    }

    rsi_state_ = RSIState(in_buffer_);
    // Set initial joint positions of arm
    for (std::size_t i = 0; i < n_dof_; ++i)
    {
      joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
      joint_position_command_[i] = joint_position_[i];
      rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
    }
    // Set initial joint positions of external axes
    for (std::size_t i = 6; i < n_dof_; ++i)
    {
      joint_position_[i] = DEG2RAD * rsi_state_.positions[i] / 1000;
      joint_position_command_[i] = joint_position_[i];
      rsi_initial_joint_positions_[i] = joint_position_[i];
    }
    // Set initial sensor_wrench
    if (use_force_torque_sensor_)
    {
      for (std::size_t i = 0; i < 3; ++i)
      {
	sensor_wrench_.force[i] = rsi_state_.force[i];
	sensor_wrench_.torque[i] = rsi_state_.torque[i];
      }
    }
    ipoc_ = rsi_state_.ipoc;
    out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_, rsi_tcp_position_corrections_, external_axes_).xml_doc;
    server_->send(out_buffer_);

    // Set receive timeout to 1 second
    server_->set_timeout(1000);
    RTT::log(RTT::Info) << "Got connection from robot!" << RTT::endlog();

    return true;
  }

  void updateHook()
  {
    // RTT::log(RTT::Info) << "Updating ! " << RTT::endlog();

    if (this->isRunning())
    {
      // Read
      in_buffer_.resize(1024);

      if (server_->recv(in_buffer_) == 0)
      {
        this->error();
      }

      rsi_state_ = RSIState(in_buffer_);
      // Update joint position
      for (std::size_t i = 0; i < n_dof_; ++i)
      {
        joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
      }
      // Update joint position for external axes
      for (std::size_t i = 6; i < n_dof_; ++i)
      {
	// Linear axes from KRC comes as [mm*RAD2DEG]
	joint_position_[i] = DEG2RAD * rsi_state_.positions[i] / 1000;
      }
      // Update sensor_wrench
      if (use_force_torque_sensor_)
      {
	for (std::size_t i = 0; i < 3; ++i)
	{
	  sensor_wrench_.force[i] = rsi_state_.force[i];
	  sensor_wrench_.torque[i] = rsi_state_.torque[i];
	}
      }
      ipoc_ = rsi_state_.ipoc;

      port_joint_position_command_.read(joint_position_command_);

      // Write
      out_buffer_.resize(1024);
      // Joint position commands
      for (std::size_t i = 0; i < n_dof_; ++i)
      {
        rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
      }
      // Joint position commands for external axes
      // With mathematically linked external axes, KRC will change robot joints to keep TCP static
      // Update TCP by same amount as linear axes to avoid this. Units in [mm]
      if (external_axes_)
      {
	//E1 & X-axis
	rsi_joint_position_corrections_[6] = 1000*(joint_position_command_[6] - rsi_initial_joint_positions_[6]);
	rsi_tcp_position_corrections_[0] = -rsi_joint_position_corrections_[6];
	// E2 & Y-axis
	rsi_joint_position_corrections_[7] = 1000 * (joint_position_command_[7] - rsi_initial_joint_positions_[7]);
	rsi_tcp_position_corrections_[1] = -rsi_joint_position_corrections_[7];
	// E3 & Z-axis
	rsi_joint_position_corrections_[8] = 1000 * (joint_position_command_[8] - rsi_initial_joint_positions_[8]);
	rsi_tcp_position_corrections_[2] = rsi_joint_position_corrections_[8];
      }
      out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_, rsi_tcp_position_corrections_, external_axes_).xml_doc;
      server_->send(out_buffer_);
      // Give sensed values on ports
      port_joint_position_.write(joint_position_);
      if (use_force_torque_sensor_)
      {
	port_ftc_sensor_.write(sensor_wrench_);
      }
    }
    this->trigger();
  }

private:
  RTT::InputPort<std::vector<double>> port_joint_position_command_;
  RTT::OutputPort<std::vector<double>> port_joint_position_;
  RTT::OutputPort<KDL::Wrench> port_ftc_sensor_;

  unsigned int n_dof_;
  bool external_axes_;
  bool use_force_torque_sensor_;

  std::vector<std::string> joint_names_;

  std::vector<double> joint_position_;
  std::vector<double> joint_position_command_;
  KDL::Wrench sensor_wrench_;

  // RSI
  RSIState rsi_state_;
  RSICommand rsi_command_;
  std::vector<double> rsi_initial_joint_positions_;
  std::vector<double> rsi_joint_position_corrections_;
  std::vector<double> rsi_tcp_position_corrections_;
  unsigned long long ipoc_;

  std::unique_ptr<UDPServer> server_;
  std::string local_host_;
  int local_port_;
  std::string remote_host_;
  std::string remote_port_;
  std::string in_buffer_;
  std::string out_buffer_;
};

}  // namespace rtt_rsi

ORO_CREATE_COMPONENT(rtt_rsi::RSIComponent)
