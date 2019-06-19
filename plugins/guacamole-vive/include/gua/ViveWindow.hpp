/***************************************************************************************
 * guacamole - delicious VR *
 *                                                                                      *
 * Copyright: (c) 2011-2016 Bauhaus-Universität Weimar * Contact:
 *felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de
 *                                                                                      *
 * This program is free software: you can redistribute it and/or modify it *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option) *
 * any later version. *
 *                                                                                      *
 * This program is distributed in the hope that it will be useful, but * WITHOUT
 *ANY WARRANTY; without even the implied warranty of MERCHANTABILITY * or
 *FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License * for
 *more details. *
 *                                                                                      *
 * You should have received a copy of the GNU General Public License along *
 * with this program. If not, see <http://www.gnu.org/licenses/>. *
 *                                                                                      *
 ***************************************************************************************/

#ifndef GUA_VIVE_WINDOW_HPP
#define GUA_VIVE_WINDOW_HPP

#if defined(_MSC_VER)
#if defined(GUA_VIVE_LIBRARY)
#define GUA_VIVE_DLL __declspec(dllexport)
#else
#define GUA_VIVE_DLL __declspec(dllimport)
#endif
#else
#define GUA_VIVE_DLL
#endif

// guacamole headers
#include <gua/node/Node.hpp>
#include <gua/renderer/GlfwWindow.hpp>
// for the OpenVR members
#include <openvr.h>

namespace gua {

struct TrackedDevice {
  TrackedDevice() : state_(false), id_(-1), pose_(), node_ptr_(nullptr){};

  void set_pose(vr::HmdMatrix34_t const& hmd_matrix_t_pose) {
    pose_ = gua::math::mat4(
        hmd_matrix_t_pose.m[0][0], hmd_matrix_t_pose.m[1][0],
        hmd_matrix_t_pose.m[2][0], 0.0, hmd_matrix_t_pose.m[0][1],
        hmd_matrix_t_pose.m[1][1], hmd_matrix_t_pose.m[2][1], 0.0,
        hmd_matrix_t_pose.m[0][2], hmd_matrix_t_pose.m[1][2],
        hmd_matrix_t_pose.m[2][2], 0.0, hmd_matrix_t_pose.m[0][3],
        hmd_matrix_t_pose.m[1][3], hmd_matrix_t_pose.m[2][3], 1.0);
    if (nullptr != node_ptr_) {
      node_ptr_->set_transform(pose_);
    }
  };
  bool state_;
  short id_;
  gua::math::mat4 pose_;
  std::shared_ptr<node::Node> node_ptr_;
};

struct ControllerDevice : TrackedDevice {
  ControllerDevice()
      : TrackedDevice(),
        active_button_states_(0x0),
        pad_x_value_(0.0f),
        pad_y_value_(0.0f),
        trigger_value_(0.0f) {}

  uint32_t active_button_states_;
  float pad_x_value_;
  float pad_y_value_;
  float trigger_value_;
};

class GUA_VIVE_DLL ViveWindow : public GlfwWindow {
 public:  // typedefs, enums
  enum DeviceID {
    HMD = 1 << 0,
    CONTROLLER_0 = 1 << 1,
    CONTROLLER_1 = 1 << 2,
    TRACKING_REFERENCE_0 = 1 << 3,
    TRACKING_REFERENCE_1 = 1 << 4
  };

  enum ControllerBinaryStates {
    APP_MENU_BUTTON = 1 << 0,
    GRIP_BUTTON = 1 << 1,
    PAD_TOUCH = 1 << 2,
    PAD_BUTTON = 1 << 3,
    TRIGGER_BUTTON = 1 << 4
  };

  enum ControllerContinuousStates {
    PAD_X_VALUE = 1 << 0,
    PAD_Y_VALUE = 1 << 1,
    TRIGGER_VALUE = 1 << 2
  };

 public:
  ViveWindow(std::string const& display = ":0.0");
  virtual ~ViveWindow();

  float const get_IPD() const;
  math::vec2ui get_window_resolution() const;
  bool get_controller_button_active(
      DeviceID device_id,
      ControllerBinaryStates controller_binary_state);
  float get_controller_value(
      DeviceID device_id,
      ControllerContinuousStates controller_continuous_state);
  math::mat4 get_sensor_orientation(DeviceID device_id = DeviceID::HMD) const;

  // steppo
  bool register_node(std::shared_ptr<node::Node> node_ptr, DeviceID device_id = DeviceID::HMD);

  /*virtual*/ math::mat4 get_latest_matrices(unsigned id) const override;



  math::vec2 const& get_left_screen_size() const;
  math::vec2 const& get_right_screen_size() const;
  math::vec3 const& get_left_screen_translation() const;
  math::vec3 const& get_right_screen_translation() const;

  // needs to be called in order to update controller buttons and senor
  // orientations
  void update_sensor_orientations();

  void display(scm::gl::texture_2d_ptr const& texture, bool is_left) override;
  void trigger_haptic_pulse(unsigned int controller_id, float strength) const;

  void open() override;
  void init_context() override;
  void start_frame() override;
  void finish_frame() override;

  void adjust_clipping(const float near_clipping, const float far_clipping);

 private:
  void update_and_predict_sensor_orientations();
  void initialize_hmd_environment();
  void calculate_viewing_setup(const float near_clipping,
                               const float far_clipping);

  std::string display_name_;
  vr::IVRSystem* p_vr_system_ = nullptr;

  math::vec2 screen_size_[2];
  math::vec3 screen_translation_[2];

  // GL Frame Buffers
  unsigned int blit_fbo_read_;
  unsigned int blit_fbo_write_;

  unsigned left_tex_id_ = 0;
  unsigned right_tex_id_ = 0;

  scm::gl::texture_2d_ptr left_texture_ = nullptr;
  scm::gl::texture_2d_ptr right_texture_ = nullptr;

  /*tracked devices associated with the ViveWindow*/
  TrackedDevice hmd_device_;
  std::vector<ControllerDevice> known_controller_devices_;
  std::vector<TrackedDevice> known_tracking_reference_devices_;

  unsigned int number_of_tracked_devices_ = 0;
  std::vector<vr::TrackedDevicePose_t> tracked_devices_handles_;


  // steppo -> should be placed in ViveCameraNode.hpp and handled there
  math::mat4 mat4eyePosLeft_;
  math::mat4 mat4eyePosRight_;

  math::mat4 mat4ProjectionLeft_;
  math::mat4 mat4ProjectionRight_;
};

}  // namespace gua

#endif  // GUA_VIVE_WINDOW_HPP