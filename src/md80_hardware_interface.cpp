// Copyright (c) 2024, Jakub Delicat
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
// (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "md80_hardware_interface/md80_hardware_interface.hpp"

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace md80_hardware_interface {
hardware_interface::CallbackReturn
MD80HardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto motors_size = info_.joints.size();
  RCLCPP_INFO_STREAM(rclcpp::get_logger(get_name()),
                     "Requesting " << motors_size << " motors.");
  md80_info_.resize(motors_size);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD80HardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  add_candle_instances();

  try {
    try_to_initialize_motors();

  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(get_name()),
                        "Got error: " << e.what());
    return CallbackReturn::FAILURE;
  }

  set_modes();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MD80HardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &md80_info_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &md80_info_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &md80_info_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MD80HardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (info_.joints[i].command_interfaces.size() != 1) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(get_name()),
                          "Too many command interfaced defines!");
      return {};
    }
    const auto control_mode = info_.joints[i].command_interfaces[0].name;

    if (control_mode == "position") {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &md80_info_[i].command.position));
      md80_info_[i].control_mode = mab::Md80Mode_E::POSITION_PID;
    } else if (control_mode == "velocity") {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
          &md80_info_[i].command.velocity));
      md80_info_[i].control_mode = mab::Md80Mode_E::VELOCITY_PID;
    } else if (control_mode == "effort") {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
          &md80_info_[i].command.effort));
      // FIXME: Is it even the same?
      md80_info_[i].control_mode = mab::Md80Mode_E::RAW_TORQUE;
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MD80HardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  enable_motors();
  //  TODO: Add service to zero encoders
  // zero_encoders();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MD80HardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  disable_motors();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MD80HardwareInterface::read(const rclcpp::Time & /*time*/,
                            const rclcpp::Duration & /*period*/) {
  std::size_t i = 0;
  for (auto candle : candle_instances) {
    for (auto &md : candle->md80s) {
      md80_info_[i].state.position = md.getPosition();
      md80_info_[i].state.velocity = md.getVelocity();
      md80_info_[i].state.effort = md.getTorque();
      ++i;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MD80HardwareInterface::write(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration & /*period*/) {
  std::size_t i = 0;
  for (auto candle : candle_instances) {
    for (auto &md : candle->md80s) {
      const auto &control_mode = md80_info_[i].control_mode;
      if (control_mode == mab::Md80Mode_E::POSITION_PID) {
        md.setTargetPosition(md80_info_[i].command.position);
      } else if (control_mode == mab::Md80Mode_E::VELOCITY_PID) {
        md.setTargetVelocity(md80_info_[i].command.position);
      } else if (control_mode == mab::Md80Mode_E::IMPEDANCE) {
        md.setTargetTorque(md80_info_[i].command.position);
      }

      ++i;
    }
  }

  return hardware_interface::return_type::OK;
}

std::shared_ptr<mab::Candle>
MD80HardwareInterface::find_candle_by_motor_can_id(uint16_t can_id) {
  for (auto &candle : candle_instances) {
    for (auto id : candle->md80s) {
      if (id.getId() == can_id)
        return candle;
    }
  }
  return nullptr;
}

void MD80HardwareInterface::add_candle_instances() {
  const mab::BusType_E bus = mab::BusType_E::USB;
  const mab::CANdleBaudrate_E baud = mab::CAN_BAUD_1M;

  while (bus == mab::BusType_E::USB) {
    try {
      candle_instances.emplace_back(
          std::make_shared<mab::Candle>(baud, true, bus));
      RCLCPP_INFO_STREAM(
          rclcpp::get_logger(get_name()),
          "Found CANdle with ID: " << candle_instances.back()->getDeviceId());
    } catch (const char *eMsg) {
      break;
    }
  }
}

void MD80HardwareInterface::try_to_initialize_motors() {
  unsigned int not_found_devices = 0;
  unsigned int found_devices = 0;
  for (const auto &joint_info : info_.joints) {
    int can_id = -1;

    try {
      can_id = std::stoi(joint_info.parameters.at("can_id"));
    } catch (const std::out_of_range &e) {
      throw std::runtime_error("can_id param is not defined in joint " +
                               joint_info.name +
                               ". Use <param name=\"can_id\"></param>.");
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger(get_name()),
                       "Check connection for can_id: " << can_id);

    for (auto &candle : candle_instances) {
      if (candle->addMd80(can_id, false)) {
        md80_info_[found_devices].can_id = can_id;
        RCLCPP_INFO_STREAM(rclcpp::get_logger(get_name()),
                           "Found device at f: "
                               << found_devices << " but id "
                               << md80_info_[found_devices].can_id);
        found_devices++;
      } else
        not_found_devices++;
    }

    if (not_found_devices == candle_instances.size()) {
      throw std::runtime_error(
          "Cannot find device on ID: " + std::to_string(can_id) + "!");
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger(get_name()),
                       "Initialized motor at can_id: " << can_id);
  }
}

void MD80HardwareInterface::set_modes() {
  for (auto &md80 : md80_info_) {
    auto candle = find_candle_by_motor_can_id(md80.can_id);

    RCLCPP_INFO_STREAM(rclcpp::get_logger(get_name()),
                       "Nitializing motor at can_id: " << md80.can_id);

    if (candle == nullptr) {
      throw std::runtime_error(
          std::string("\nCould not find instance for can_id: ") +
          std::to_string(md80.can_id));
    }

    candle->controlMd80Mode(md80.can_id, md80.control_mode);
    RCLCPP_INFO_STREAM(rclcpp::get_logger(get_name()),
                       "Mode is set motor at can_id: " << md80.can_id);
  }
}

void MD80HardwareInterface::zero_encoders() {
  for (auto &md80 : md80_info_) {
    auto candle = find_candle_by_motor_can_id(md80.can_id);
    candle->controlMd80SetEncoderZero(md80.can_id);
  }
}

void MD80HardwareInterface::enable_motors() {
  for (auto &md80 : md80_info_) {
    auto candle = find_candle_by_motor_can_id(md80.can_id);
    candle->controlMd80Enable(md80.can_id, true);
  }

  for (auto &candle : candle_instances) {
    candle->begin();
  }
}

void MD80HardwareInterface::disable_motors() {
  for (auto &md80 : md80_info_) {
    auto candle = find_candle_by_motor_can_id(md80.can_id);
    candle->controlMd80Enable(md80.can_id, false);
  }
}

} // namespace md80_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(md80_hardware_interface::MD80HardwareInterface,
                       hardware_interface::SystemInterface)
