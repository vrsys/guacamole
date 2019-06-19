/*******************************************************************************
 * guacamole - delicious VR *
 *                                                                              *
 * Copyright: (c) 2011-2016 Bauhaus-Universität Weimar *
 *                                                                              *
 * This program is free software: you can redistribute it and/or modify it *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option) *
 * any later version. *
 *                                                                              *
 * This program is distributed in the hope that it will be useful, but * WITHOUT
 *ANY WARRANTY; without even the implied warranty of MERCHANTABILITY   * or
 *FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License     *
 * for more details. *
 *                                                                              *
 * You should have received a copy of the GNU General Public License along *
 * with this program. If not, see <http://www.gnu.org/licenses/>. *
 *                                                                              *
 *******************************************************************************/

// class header
#include <gua/ViveWindow.hpp>

// gua headers
#include <scm/gl_core/render_device/opengl/gl_core.h>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

ViveWindow::ViveWindow(std::string const& display)
    : GlfwWindow(), display_name_(display) {
  // initialize hmd and texture sizes
  initialize_hmd_environment();

  // calculate screen size, translation and eye distance
  calculate_viewing_setup(0.5, 500.0);
}

ViveWindow::~ViveWindow() {
  vr::VR_Shutdown();
}

void ViveWindow::initialize_hmd_environment() {
  vr::EVRInitError eError = vr::VRInitError_None;
  p_vr_system_ = vr::VR_Init(&eError, vr::VRApplication_Scene);

  if (eError != vr::VRInitError_None) {
    p_vr_system_ = nullptr;
    Logger::LOG_WARNING << "Failed to initialize OpenVR environment."
                        << "Errorcode:" << (int)eError << std::endl;
  }

  uint32_t width, height;
  p_vr_system_->GetRecommendedRenderTargetSize(&width, &height);
  config.set_size(math::vec2ui(width, height));

  config.set_left_resolution(math::vec2ui(width, height));
  config.set_left_position(math::vec2ui(0, 0));
  config.set_right_resolution(math::vec2ui(width, height));
  config.set_right_position(math::vec2ui(width, 0));

  // steppo
  tracked_devices_handles_.resize(vr::k_unMaxTrackedDeviceCount);
  update_sensor_orientations();
}

// get interpupillary distance
float const ViveWindow::get_IPD() const {
  vr::HmdMatrix34_t left_eye =
      p_vr_system_->GetEyeToHeadTransform(vr::EVREye::Eye_Left);
  vr::HmdMatrix34_t right_eye =
      p_vr_system_->GetEyeToHeadTransform(vr::EVREye::Eye_Right);
  return std::fabs(left_eye.m[0][3]) + std::fabs(right_eye.m[0][3]);
}

void ViveWindow::adjust_clipping(const float near_clipping,
                                 const float far_clipping) {
  calculate_viewing_setup(near_clipping, far_clipping);
}

void ViveWindow::calculate_viewing_setup(const float near_clipping,
                                         const float far_clipping) {
  std::cout << "ViveWindow::calculate_viewing_setup()" << std::endl;
  // use just any near and far plane values, since they get reset by guacamole
  float near_distance = near_clipping;
  float far_distance = far_clipping;

  // eye to screen distance can be choosen relatively arbitrary as well,
  // since the virtual screen is enlarged, such that it fits into the frustum
  // at this distance
  float eye_to_screen_distance = 0.08;

  // do the viewing setup calculations for both eyes
  for (unsigned eye_num = 0; eye_num < 2; ++eye_num) {
    vr::EVREye eEye =
        eye_num == 0 ? vr::EVREye::Eye_Left : vr::EVREye::Eye_Right;
    // retreive the correct projection matrix from OpenVR
    auto const& hmd_eye_projection =
        p_vr_system_->GetProjectionMatrix(eEye, near_distance, far_distance);

    // convert the matrix to a gua compatible one
    scm::math::mat4 scm_eye_proj_matrix;
    for (int outer = 0; outer < 4; ++outer) {
      for (int inner = 0; inner < 4; ++inner) {
        scm_eye_proj_matrix[outer * 4 + inner] =
            hmd_eye_projection.m[outer][inner];
      }
    }

    // unproject one frustum corner defining one clipping plane
    scm::math::mat4 inv_hmd_eye_proj_matrix =
        scm::math::inverse(scm_eye_proj_matrix);
    scm::math::vec4 back_right_top_frustum_corner =
        scm::math::vec4(-1.0, -1.0, -1.0, 1.0) * inv_hmd_eye_proj_matrix;
    scm::math::vec4 back_left_bottom_frustum_corner =
        scm::math::vec4(1.0, 1.0, -1.0, 1.0) * inv_hmd_eye_proj_matrix;

    scm::math::vec4 normalized_back_left_bottom_frustum_corner =
        back_left_bottom_frustum_corner / back_left_bottom_frustum_corner[3];
    scm::math::vec4 normalized_back_right_top_frustum_corner =
        back_right_top_frustum_corner / back_right_top_frustum_corner[3];

    // note: eye is in (0,0,0 relative to the frustum)

    // solve for t1 along the ray to the upper right corner for our screen
    // distance
    float t_to_screen_along_eye_to_right_upper_corner_vec =
        (eye_to_screen_distance) / normalized_back_right_top_frustum_corner[2];

    // retreive the upper right screen corner
    scm::math::vec4 screen_right_upper_corner_relative_to_eye =
        scm::math::vec4(t_to_screen_along_eye_to_right_upper_corner_vec *
                            normalized_back_right_top_frustum_corner[0],
                        t_to_screen_along_eye_to_right_upper_corner_vec *
                            normalized_back_right_top_frustum_corner[1],
                        t_to_screen_along_eye_to_right_upper_corner_vec *
                            normalized_back_right_top_frustum_corner[2],
                        1.0);

    // solve for t2 along the ray to the lower yleft corner for our screen
    // distance
    float t_to_screen_along_eye_to_left_lower_corner_vec =
        (eye_to_screen_distance) /
        normalized_back_left_bottom_frustum_corner[2];

    // retreive the lower left screen corner
    scm::math::vec4 screen_left_lower_corner_relative_to_eye =
        scm::math::vec4(t_to_screen_along_eye_to_left_lower_corner_vec *
                            normalized_back_left_bottom_frustum_corner[0],
                        t_to_screen_along_eye_to_left_lower_corner_vec *
                            normalized_back_left_bottom_frustum_corner[1],
                        t_to_screen_along_eye_to_left_lower_corner_vec *
                            normalized_back_left_bottom_frustum_corner[2],
                        1.0);

    // calculate the horizontal and vertical additional offsets of the eye based
    // on the position of the screen corners in relation to the eye
    float horizontal_eye_distortion_offset =
        (std::fabs(screen_left_lower_corner_relative_to_eye[0]) -
         std::fabs(screen_right_upper_corner_relative_to_eye[0])) /
        2.0;

    float vertical_eye_distortion_offset =
        (std::fabs(screen_left_lower_corner_relative_to_eye[1]) -
         std::fabs(screen_right_upper_corner_relative_to_eye[1])) /
        2.0;

    // get default horizontal offset for both eyes
    float half_IPD = get_IPD() / 2.0;

    // choose the correct initial eye offste
    float standard_horizontal_eye_offset = eye_num == 0 ? -half_IPD : half_IPD;

    // calculate the final screen translation
    screen_translation_[eye_num] = gua::math::vec3(
        standard_horizontal_eye_offset + horizontal_eye_distortion_offset,
        0.0 + vertical_eye_distortion_offset, -eye_to_screen_distance);

    // calculate the final screen size
    screen_size_[eye_num] =
        gua::math::vec2f((screen_right_upper_corner_relative_to_eye -
                          screen_left_lower_corner_relative_to_eye));
  }

  // use the same method as in original openvr example // vr::EVREye::Eye_Left :
  // vr::EVREye::Eye_Right;

  vr::HmdMatrix34_t mat4eyePosLeft =
      p_vr_system_->GetEyeToHeadTransform(vr::EVREye::Eye_Left);
  gua::math::mat4 tmp_mat =
      gua::math::mat4(mat4eyePosLeft.m[0][0], mat4eyePosLeft.m[1][0],
                      mat4eyePosLeft.m[2][0], 0.0, mat4eyePosLeft.m[0][1],
                      mat4eyePosLeft.m[1][1], mat4eyePosLeft.m[2][1], 0.0,
                      mat4eyePosLeft.m[0][2], mat4eyePosLeft.m[1][2],
                      mat4eyePosLeft.m[2][2], 0.0, mat4eyePosLeft.m[0][3],
                      mat4eyePosLeft.m[1][3], mat4eyePosLeft.m[2][3], 1.0);
  mat4eyePosLeft_ = scm::math::inverse(tmp_mat);

  vr::HmdMatrix34_t mat4eyePosRight =
      p_vr_system_->GetEyeToHeadTransform(vr::EVREye::Eye_Right);
  tmp_mat =
      gua::math::mat4(mat4eyePosRight.m[0][0], mat4eyePosRight.m[1][0],
                      mat4eyePosRight.m[2][0], 0.0, mat4eyePosRight.m[0][1],
                      mat4eyePosRight.m[1][1], mat4eyePosRight.m[2][1], 0.0,
                      mat4eyePosRight.m[0][2], mat4eyePosRight.m[1][2],
                      mat4eyePosRight.m[2][2], 0.0, mat4eyePosRight.m[0][3],
                      mat4eyePosRight.m[1][3], mat4eyePosRight.m[2][3], 1.0);
  mat4eyePosRight_ = scm::math::inverse(tmp_mat);

  vr::HmdMatrix44_t mat4ProjectionLeft = p_vr_system_->GetProjectionMatrix(
      vr::EVREye::Eye_Left, near_distance, far_distance);
  for (int outer = 0; outer < 4; ++outer) {
    for (int inner = 0; inner < 4; ++inner) {
      tmp_mat[outer * 4 + inner] = mat4ProjectionLeft.m[outer][inner];
    }
  }
  mat4ProjectionLeft_ = scm::math::transpose(tmp_mat);

  vr::HmdMatrix44_t mat4ProjectionRight = p_vr_system_->GetProjectionMatrix(
      vr::EVREye::Eye_Right, near_distance, far_distance);
  for (int outer = 0; outer < 4; ++outer) {
    for (int inner = 0; inner < 4; ++inner) {
      tmp_mat[outer * 4 + inner] = mat4ProjectionRight.m[outer][inner];
    }
  }
  mat4ProjectionRight_ = scm::math::transpose(tmp_mat);
}

void ViveWindow::init_context() {
  WindowBase::init_context();

  auto const& glapi = ctx_.render_context->opengl_api();

  glapi.glGenFramebuffers(1, &blit_fbo_read_);
  glapi.glGenFramebuffers(1, &blit_fbo_write_);

  unsigned int width = 0, height = 0;
  if (p_vr_system_) {
    p_vr_system_->GetRecommendedRenderTargetSize(&width, &height);
    left_texture_ = ctx_.render_device->create_texture_2d(
        scm::math::vec2ui(width, height), scm::gl::FORMAT_RGBA_8, 1, 1, 1);
    right_texture_ = ctx_.render_device->create_texture_2d(
        scm::math::vec2ui(width, height), scm::gl::FORMAT_RGBA_8, 1, 1, 1);

    // createEyeTexture(glapi, width, height);
    left_tex_id_ = left_texture_->object_id();
    // createEyeTexture(glapi, width, height);
    right_tex_id_ = right_texture_->object_id();
  } else {
    std::cerr << "ERROR: p_vr_system_ is nullptr." << std::endl;
  }
}

void ViveWindow::open() {
  config.set_title("guacamole");
  config.set_display_name(display_name_);
  config.set_stereo_mode(StereoMode::SIDE_BY_SIDE);

  GlfwWindow::open();
}

math::vec2ui ViveWindow::get_window_resolution() const {
  uint32_t width, height;
  p_vr_system_->GetRecommendedRenderTargetSize(&width, &height);
  return math::vec2ui(width, height);
}

void ViveWindow::update_sensor_orientations() {

  const unsigned queried_number_of_tracked_devices(
      vr::k_unMaxTrackedDeviceCount);

  if (number_of_tracked_devices_ < queried_number_of_tracked_devices) {
    number_of_tracked_devices_ = queried_number_of_tracked_devices;
    tracked_devices_handles_.resize(number_of_tracked_devices_);
  }
  // vr::TrackedDevicePose_t trackedDevicesHandles[numberOfTrackedDevices];

  float fSecondsSinceLastVsync;
  p_vr_system_->GetTimeSinceLastVsync(&fSecondsSinceLastVsync, NULL);
  float fDisplayFrequency = p_vr_system_->GetFloatTrackedDeviceProperty(
      vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
  float fFrameDuration = 1.0f / fDisplayFrequency;
  float fVsyncToPhotons = p_vr_system_->GetFloatTrackedDeviceProperty(
      vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SecondsFromVsyncToPhotons_Float);
  float predictedSecondsFromNow =
      fFrameDuration - fSecondsSinceLastVsync + fVsyncToPhotons;

  // vr::HmdMatrix34_t pose;
  p_vr_system_->GetDeviceToAbsoluteTrackingPose(
      vr::TrackingUniverseStanding, predictedSecondsFromNow,
      &tracked_devices_handles_[0], number_of_tracked_devices_);

  hmd_device_.state_ = false;
  for (auto& controller : known_controller_devices_) {
    controller.state_ = false;
    // controller.pad_touched_ = false;
    controller.active_button_states_ = 0x0;
  }
  for (auto& tracking_reference_device : known_tracking_reference_devices_) {
    tracking_reference_device.state_ = false;
  }

  unsigned current_controller_count = 0;
  unsigned current_tracking_references_count = 0;

  for (unsigned int device_idx = 0; device_idx < number_of_tracked_devices_;
       ++device_idx) {
    auto const tracked_device_class =
        p_vr_system_->GetTrackedDeviceClass(device_idx);

    auto const& tracked_pose =
        tracked_devices_handles_[device_idx].mDeviceToAbsoluteTracking;
    if (tracked_device_class == vr::TrackedDeviceClass_HMD) {
      hmd_device_.state_ = true;
      hmd_device_.set_pose(tracked_pose);
    } else if (tracked_device_class ==
               vr::TrackedDeviceClass_TrackingReference) {
      ++current_tracking_references_count;

      if (current_tracking_references_count >
          known_tracking_reference_devices_.size()) {
        known_tracking_reference_devices_.resize(
            current_tracking_references_count);
      }

      auto& current_tracking_reference_device =
          known_tracking_reference_devices_[current_tracking_references_count -
                                            1];
      current_tracking_reference_device.state_ = true;
      current_tracking_reference_device.id_ =
          current_tracking_references_count - 1;
      current_tracking_reference_device.set_pose(tracked_pose);
      

    } else if (tracked_device_class == vr::TrackedDeviceClass_Controller) {
      ++current_controller_count;
      if (current_controller_count > known_controller_devices_.size()) {
        known_controller_devices_.resize(current_controller_count);
      }
      auto& current_controller =
          known_controller_devices_[current_controller_count - 1];
      current_controller.state_ = true;
      current_controller.id_ = current_controller_count - 1;

      current_controller.set_pose(tracked_pose);
      
      vr::VRControllerState_t controller_state;
      if (!p_vr_system_->GetControllerState(device_idx, &controller_state,
                                            sizeof(vr::VRControllerState_t))) {
        continue;
      }

      current_controller.trigger_value_ = controller_state.rAxis[1].x;

      if (controller_state.ulButtonTouched &
          vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::GRIP_BUTTON;
        current_controller.pad_x_value_ = controller_state.rAxis[0].x;
        current_controller.pad_y_value_ = controller_state.rAxis[0].y;
      }

      if (controller_state.ulButtonPressed &
          vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::APP_MENU_BUTTON;
      }

      if (controller_state.ulButtonPressed &
          vr::ButtonMaskFromId(vr::k_EButton_Grip)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::GRIP_BUTTON;
      }

      if (controller_state.ulButtonPressed &
          vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::TRIGGER_BUTTON;
      }

      if (controller_state.ulButtonPressed &
          vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::PAD_TOUCH;
      }
    }
  }
  std::cout << "ViveWindow::update_sensor_orientations() finished with known_controller_devices_.size(): "
            << known_controller_devices_.size() << "´and known_tracking_reference_devices_.size(): "
            << known_tracking_reference_devices_.size() << std::endl;
}

void ViveWindow::update_and_predict_sensor_orientations() {
  if (!p_vr_system_)
    return;

  const unsigned queried_number_of_tracked_devices(
      vr::k_unMaxTrackedDeviceCount);

  if (number_of_tracked_devices_ < queried_number_of_tracked_devices) {
    number_of_tracked_devices_ = queried_number_of_tracked_devices;
    tracked_devices_handles_.resize(number_of_tracked_devices_);
  }

  vr::VRCompositor()->WaitGetPoses(&tracked_devices_handles_[0],
                                   number_of_tracked_devices_, NULL, 0.0f);

  // implement as in OpenVR example

  hmd_device_.state_ = false;
  for (auto& controller : known_controller_devices_) {
    controller.state_ = false;
    // controller.pad_touched_ = false;
    controller.active_button_states_ = 0x0;
  }
  for (auto& tracking_reference_device : known_tracking_reference_devices_) {
    tracking_reference_device.state_ = false;
  }

  unsigned current_controller_count = 0;
  unsigned current_tracking_references_count = 0;

  for (unsigned int device_idx = 0; device_idx < number_of_tracked_devices_;
       ++device_idx) {
    auto const tracked_device_class =
        p_vr_system_->GetTrackedDeviceClass(device_idx);

    auto const& tracked_pose =
        tracked_devices_handles_[device_idx].mDeviceToAbsoluteTracking;
    if (tracked_device_class == vr::TrackedDeviceClass_HMD) {
      hmd_device_.state_ = true;
      hmd_device_.set_pose(tracked_pose);
      //std::cout << "hmd_device_.set_pose(tracked_pose);" << std::endl;
    } else if (tracked_device_class ==
               vr::TrackedDeviceClass_TrackingReference) {
      ++current_tracking_references_count;

      if (current_tracking_references_count >
          known_tracking_reference_devices_.size()) {
        known_tracking_reference_devices_.resize(
            current_tracking_references_count);
      }

      auto& current_tracking_reference_device =
          known_tracking_reference_devices_[current_tracking_references_count -
                                            1];
      current_tracking_reference_device.state_ = true;
      current_tracking_reference_device.id_ =
          current_tracking_references_count - 1;
      current_tracking_reference_device.set_pose(tracked_pose);
      //std::cout << "current_tracking_reference_device.set_pose(tracked_pose);" << std::endl;
    } else if (tracked_device_class == vr::TrackedDeviceClass_Controller) {
      ++current_controller_count;
      if (current_controller_count > known_controller_devices_.size()) {
        known_controller_devices_.resize(current_controller_count);
      }
      auto& current_controller =
          known_controller_devices_[current_controller_count - 1];
      current_controller.state_ = true;
      current_controller.id_ = current_controller_count - 1;

      current_controller.set_pose(tracked_pose);
      //std::cout << "current_controller.set_pose(tracked_pose);" << std::endl;
      // steppo do not process controller states from here any more


      vr::VRControllerState_t controller_state;
      if (!p_vr_system_->GetControllerState(device_idx, &controller_state,
                                            sizeof(vr::VRControllerState_t))) {
        continue;
      }

      current_controller.trigger_value_ = controller_state.rAxis[1].x;

      if (controller_state.ulButtonTouched &
          vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::GRIP_BUTTON;
        current_controller.pad_x_value_ = controller_state.rAxis[0].x;
        current_controller.pad_y_value_ = controller_state.rAxis[0].y;
      }

      if (controller_state.ulButtonPressed &
          vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::APP_MENU_BUTTON;
      }

      if (controller_state.ulButtonPressed &
          vr::ButtonMaskFromId(vr::k_EButton_Grip)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::GRIP_BUTTON;
      }

      if (controller_state.ulButtonPressed &
          vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::TRIGGER_BUTTON;
      }

      if (controller_state.ulButtonPressed &
          vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad)) {
        current_controller.active_button_states_ |=
            ControllerBinaryStates::PAD_TOUCH;
      }
    }
  }

}

bool ViveWindow::get_controller_button_active(
    DeviceID device_id,
    ControllerBinaryStates controller_binary_state) {
  if (DeviceID::CONTROLLER_0 != device_id &&
      DeviceID::CONTROLLER_1 != device_id) {
    // gua::Logger::LOG_WARNING << "DeviceID for requested Button does not
    // belong to a controller. Returning 'false'\n";
    return false;
  }

  unsigned int requested_device_idx =
      (device_id == DeviceID::CONTROLLER_0) ? 0 : 1;

  if (requested_device_idx >= known_controller_devices_.size()) {
    // gua::Logger::LOG_WARNING << "Requested Controller is not active.
    // Returning 'false'\n";
    return false;
  }

  return known_controller_devices_[requested_device_idx].active_button_states_ &
         controller_binary_state;
}

float ViveWindow::get_controller_value(
    DeviceID device_id,
    ControllerContinuousStates controller_continuous_state) {
  if (DeviceID::CONTROLLER_0 != device_id &&
      DeviceID::CONTROLLER_1 != device_id) {
    // gua::Logger::LOG_WARNING << "DeviceID for requested Button does not
    // belong to a controller. Returning 'false'\n";
    return false;
  }

  unsigned int requested_device_idx =
      (device_id == DeviceID::CONTROLLER_0) ? 0 : 1;

  if (requested_device_idx >= known_controller_devices_.size()) {
    // gua::Logger::LOG_WARNING << "Requested Controller is not active.
    // Returning 'false'\n";
    return false;
  }

  auto const& current_controller =
      known_controller_devices_[requested_device_idx];

  float requested_value(0.0f);

  switch (controller_continuous_state) {
    case (ControllerContinuousStates::PAD_X_VALUE):
      requested_value = current_controller.pad_x_value_;
      break;
    case (ControllerContinuousStates::PAD_Y_VALUE):
      requested_value = current_controller.pad_y_value_;
      break;
    case (ControllerContinuousStates::TRIGGER_VALUE):
      requested_value = current_controller.trigger_value_;
      break;
    default:
      break;
  }

  return requested_value;
}

math::mat4 ViveWindow::get_sensor_orientation(DeviceID device_id) const {
  math::mat4 device_orientation;

  switch (device_id) {
    case DeviceID::HMD:
      device_orientation = hmd_device_.pose_;
      break;
    case DeviceID::CONTROLLER_0:
      if (known_controller_devices_.size() > 0) {
        device_orientation = known_controller_devices_[0].pose_;
      }
      break;
    case DeviceID::CONTROLLER_1:
      if (known_controller_devices_.size() > 1) {
        device_orientation = known_controller_devices_[1].pose_;
      }
      break;
    case DeviceID::TRACKING_REFERENCE_0:
      if (known_tracking_reference_devices_.size() > 0) {
        device_orientation = known_tracking_reference_devices_[0].pose_;
      }
      break;
    case DeviceID::TRACKING_REFERENCE_1:
      if (known_tracking_reference_devices_.size() > 1) {
        device_orientation = known_tracking_reference_devices_[1].pose_;
      }
      break;

    default:
      break;
  }
  return device_orientation;
}

bool ViveWindow::register_node(std::shared_ptr<node::Node> node_ptr, DeviceID device_id){

	bool success = false;
  switch (device_id) {
    case DeviceID::HMD:
      hmd_device_.node_ptr_ = node_ptr;
      success = true;
      break;
    case DeviceID::CONTROLLER_0:
      if (known_controller_devices_.size() > 0) {
        known_controller_devices_[0].node_ptr_ = node_ptr;
        success = true;
      }
      break;
    case DeviceID::CONTROLLER_1:
      if (known_controller_devices_.size() > 1) {
        known_controller_devices_[1].node_ptr_ = node_ptr;
        success = true;
      }
      break;
    case DeviceID::TRACKING_REFERENCE_0:
      if (known_tracking_reference_devices_.size() > 0) {
        known_tracking_reference_devices_[0].node_ptr_ = node_ptr;
        success = true;
      }
      break;
    case DeviceID::TRACKING_REFERENCE_1:
      if (known_tracking_reference_devices_.size() > 1) {
        known_tracking_reference_devices_[1].node_ptr_ = node_ptr;
        success = true;
      }
      break;

    default:
      break;
  }
  return success;
}



/*virtual*/ math::mat4 ViveWindow::get_latest_matrices(unsigned id) const {
  math::mat4 result;
  switch (id) {
    case 0:
      result = mat4eyePosLeft_;
      break;
    case 1:
      result = mat4ProjectionLeft_;
      break;
    case 2:
      result = mat4eyePosRight_;
      break;
    case 3:
      result = mat4ProjectionRight_;
      break;
    case 4:
      result = hmd_device_.pose_;
      break;
    case 5:
      if (known_controller_devices_.size() > 0) {
        result = known_controller_devices_[0].pose_;
      }
      break;
    case 6:
      if (known_controller_devices_.size() > 1) {
        result = known_controller_devices_[1].pose_;
      }
      break;

    default:
      break;
  }
  return result;
}

math::vec2 const& ViveWindow::get_left_screen_size() const {
  return screen_size_[0];
}

math::vec2 const& ViveWindow::get_right_screen_size() const {
  return screen_size_[1];
}

math::vec3 const& ViveWindow::get_left_screen_translation() const {
  return screen_translation_[0];
}

math::vec3 const& ViveWindow::get_right_screen_translation() const {
  return screen_translation_[1];
}

void ViveWindow::start_frame() {
  GlfwWindow::start_frame();
  //ctx_.render_context->opengl_api().glFlush();
  //ctx_.render_context->opengl_api().glFinish();

  // Variante A
  // update_and_predict_sensor_orientations();

  // Variante B
  // update_sensor_orientations();
  // vr::TrackedDevicePose_t devices[vr::k_unMaxTrackedDeviceCount];
  // vr::VRCompositor()->WaitGetPoses(devices, vr::k_unMaxTrackedDeviceCount,
  // NULL,
  //                                 0.0f);
  // update_sensor_orientations();
}

void ViveWindow::finish_frame() {
  if (left_tex_id_ && right_tex_id_) {
    vr::Texture_t leftEyeTexture{(void*)left_tex_id_, vr::TextureType_OpenGL,
                                 vr::ColorSpace_Gamma};
    vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);

    vr::Texture_t rightEyeTexture{(void*)right_tex_id_, vr::TextureType_OpenGL,
                                  vr::ColorSpace_Gamma};
    vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);
  }

#if 1
  //ctx_.render_context->opengl_api().glFinish();
  update_and_predict_sensor_orientations();
#else
  vr::TrackedDevicePose_t devices[vr::k_unMaxTrackedDeviceCount];
  vr::VRCompositor()->WaitGetPoses(devices, vr::k_unMaxTrackedDeviceCount, NULL,
                                   0.0f);
  update_sensor_orientations();
#endif

  GlfwWindow::finish_frame();
}

void ViveWindow::display(scm::gl::texture_2d_ptr const& texture, bool is_left) {
  auto const& glapi = ctx_.render_context->opengl_api();

  if (!left_tex_id_) {
    return;
  }

  GLuint tex_id = is_left ? left_tex_id_ : right_tex_id_;

  // Copy ~texture~ to ~tex_id~
  glapi.glBindTexture(GL_TEXTURE_2D, tex_id);
  glapi.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, blit_fbo_write_);
  glapi.glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, tex_id, 0);

  GLenum status = glapi.glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE) {
    gua::Logger::LOG_WARNING << "Draw Framebuffer Incomplete.\n";
  }

  // setup read buffer
  glapi.glBindTexture(GL_TEXTURE_2D, texture->object_id());
  glapi.glBindFramebuffer(GL_READ_FRAMEBUFFER, blit_fbo_read_);
  glapi.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, texture->object_id(), 0);

  status = glapi.glCheckFramebufferStatus(GL_READ_FRAMEBUFFER);
  if (status != GL_FRAMEBUFFER_COMPLETE) {
    gua::Logger::LOG_WARNING << "Read Framebuffer Incomplete.\n";
  }

  scm::math::vec2ui const tex_dimensions = texture->dimensions();

  glapi.glBlitFramebuffer(
      0, 0, tex_dimensions[0], tex_dimensions[1], config.left_position().x,
      config.left_position().y, config.left_resolution().x,
      config.left_resolution().y, GL_COLOR_BUFFER_BIT, GL_NEAREST);

  glapi.glBindTexture(GL_TEXTURE_2D, 0);
  glapi.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  glapi.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

  WindowBase::display(texture, is_left);
}

void ViveWindow::trigger_haptic_pulse(unsigned int controller_id,
                                      float strength) const {
  // get device depending on controller id (DeviceID::CONTROLLER_0 not working)
  unsigned int current_controller_count = 0;
  unsigned int device_id_to_pulse = 0;

  for (unsigned int device_idx = 0; device_idx < number_of_tracked_devices_;
       ++device_idx) {
    auto const tracked_device_class =
        p_vr_system_->GetTrackedDeviceClass(device_idx);

    if (tracked_device_class == vr::TrackedDeviceClass_Controller) {
      if (current_controller_count == controller_id) {
        device_id_to_pulse = device_idx;
        break;
      }
      ++current_controller_count;
    }
  }

  if (device_id_to_pulse > 0) {
    // map percentage to absolute value in [0; 3999]
    int absolutePulse = std::min(std::max((int)(strength * 3999), 0), 3999);
    p_vr_system_->TriggerHapticPulse(device_id_to_pulse, 0, absolutePulse);
  }
}

}  // namespace gua