#include "interfaz_hardware_omni_v1/hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <sstream>
#include <iostream>

namespace omni_robot_hardware
{

hardware_interface::CallbackReturn OmniRobotHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  arduino_port_ = info.hardware_parameters["arduino_port"];
  esp_port_ = info.hardware_parameters["esp_port"];
  baud_rate_ = std::stoul(info.hardware_parameters["baud_rate"]);
  serial_timeout_ = std::stoi(info.hardware_parameters["serial_timeout"]);
  publish_frequency_ = std::stod(info.hardware_parameters["publish_frequency"]);

  node_name_ = info.hardware_parameters["node_name"];
  velocity_topic_ = info.hardware_parameters["velocity_topic"];
  state_topic_ = info.hardware_parameters["state_topic"];

  min_rps_ = std::stod(info.hardware_parameters["wheel_limits.min_rps"]);
  max_rps_ = std::stod(info.hardware_parameters["wheel_limits.max_rps"]);

  // Configuración de factores de ajuste de los encoders
  encoder_adjustments_.resize(4);
  encoder_adjustments_[0] = std::stod(info.hardware_parameters["encoder_adjustments.wheel_0"]);
  encoder_adjustments_[1] = std::stod(info.hardware_parameters["encoder_adjustments.wheel_1"]);
  encoder_adjustments_[2] = std::stod(info.hardware_parameters["encoder_adjustments.wheel_2"]);
  encoder_adjustments_[3] = std::stod(info.hardware_parameters["encoder_adjustments.wheel_3"]);

  velocity_commands_.resize(4, 0.0);
  encoder_readings_.resize(4, 0.0);

  try {
    serial_arduino_.setPort(arduino_port_);
    serial_arduino_.setBaudrate(baud_rate_);
    serial_arduino_.setTimeout(serial::Timeout::simpleTimeout(serial_timeout_));
    serial_arduino_.open();

    serial_esp_.setPort(esp_port_);
    serial_esp_.setBaudrate(baud_rate_);
    serial_esp_.setTimeout(serial::Timeout::simpleTimeout(serial_timeout_));
    serial_esp_.open();
  } catch (const serial::IOException &e) {
    RCLCPP_ERROR(rclcpp::get_logger("OmniRobotHardwareInterface"), "Unable to open serial ports.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OmniRobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface("joint_wheel_1", hardware_interface::HW_IF_POSITION, &encoder_readings_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("joint_wheel_2", hardware_interface::HW_IF_POSITION, &encoder_readings_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("joint_wheel_3", hardware_interface::HW_IF_POSITION, &encoder_readings_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("joint_wheel_4", hardware_interface::HW_IF_POSITION, &encoder_readings_[3]));

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> OmniRobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Asignar los nombres correctos para cada rueda según el controlador
  command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_wheel_1", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_wheel_2", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_wheel_3", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[2]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("joint_wheel_4", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[3]));

  return command_interfaces;
}


hardware_interface::CallbackReturn OmniRobotHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OmniRobotHardwareInterface"), "Activating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniRobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("OmniRobotHardwareInterface"), "Deactivating...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmniRobotHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Enviar comando para leer encoders a ambos controladores
  serial_arduino_.write("e\n");
  serial_esp_.write("e\n");

  // Leer respuestas
  std::string arduino_response = serial_arduino_.readline();
  std::string esp_response = serial_esp_.readline();

  // Procesar respuesta del Arduino y aplicar ajuste
  std::stringstream arduino_stream(arduino_response);
  double raw_encoder_0, raw_encoder_1;
  arduino_stream >> raw_encoder_0 >> raw_encoder_1;
  encoder_readings_[0] = raw_encoder_0 * encoder_adjustments_[0];
  encoder_readings_[1] = raw_encoder_1 * encoder_adjustments_[1];

  // Procesar respuesta del ESP32 y aplicar ajuste
  std::stringstream esp_stream(esp_response);
  double raw_encoder_2, raw_encoder_3;
  esp_stream >> raw_encoder_2 >> raw_encoder_3;
  encoder_readings_[2] = raw_encoder_2 * encoder_adjustments_[2];
  encoder_readings_[3] = raw_encoder_3 * encoder_adjustments_[3];

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OmniRobotHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Limitar las velocidades según los valores mínimos y máximos permitidos
  for (auto &velocity : velocity_commands_) {
    velocity = clamp_velocity(velocity);
  }

  // Crear comandos para el Arduino (ruedas 0 y 1) y el ESP32 (ruedas 2 y 3)
  std::string arduino_command = "m " + std::to_string(velocity_commands_[0]) + " " + std::to_string(velocity_commands_[1]) + "\n";
  std::string esp_command = "m " + std::to_string(velocity_commands_[2]) + " " + std::to_string(velocity_commands_[3]) + "\n";

  // Enviar comandos
  serial_arduino_.write(arduino_command);
  serial_esp_.write(esp_command);

  return hardware_interface::return_type::OK;
}

double OmniRobotHardwareInterface::clamp_velocity(double velocity)
{
  return std::max(min_rps_, std::min(velocity, max_rps_));
}

}  // namespace omni_robot_hardware
