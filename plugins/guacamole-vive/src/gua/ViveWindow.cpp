/*******************************************************************************
* guacamole - delicious VR                                                     *
*                                                                              *
* Copyright: (c) 2011-2016 Bauhaus-Universität Weimar                          *
*                                                                              *
* This program is free software: you can redistribute it and/or modify it      *
* under the terms of the GNU General Public License as published by the Free   *
* Software Foundation, either version 3 of the License, or (at your option)    *
* any later version.                                                           *
*                                                                              *
* This program is distributed in the hope that it will be useful, but          *
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY   *
* or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License     *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU General Public License along      *
* with this program. If not, see <http://www.gnu.org/licenses/>.               *
*                                                                              *
*******************************************************************************/

// class header
#include <gua/ViveWindow.hpp>


// gua headers
#include <gua/utils/Logger.hpp>
#include <gua/databases/Resources.hpp>
#include <scm/gl_core/render_device/opengl/gl_core.h>


namespace gua {

ViveWindow::ViveWindow(std::string const& display)
    : GlfwWindow()
    , display_name_(display)
{
    // initialize hmd and texture sizes
    initialize_vive_environment();

    // calculate screen size, translation and eye distance
    calculate_viewing_setup();
}

ViveWindow::~ViveWindow() {
    vr::VR_Shutdown();
}

void ViveWindow::initialize_vive_environment() {
    vr::EVRInitError eError = vr::VRInitError_None;
    pVRSystem = vr::VR_Init(&eError, vr::VRApplication_Scene);

    if (eError != vr::VRInitError_None) {
        pVRSystem = nullptr;
        Logger::LOG_WARNING << "Failed to initialize OpenVR environment."
            << "Errorcode:" << (int)eError << std::endl;
    }

    uint32_t width, height;
    pVRSystem->GetRecommendedRenderTargetSize(&width, &height);
    config.set_size(math::vec2ui(width, height));

    config.set_left_resolution(math::vec2ui(width, height));
    config.set_left_position(math::vec2ui(0, 0));
    config.set_right_resolution(math::vec2ui(width, height));
    config.set_right_position(math::vec2ui(width, 0));
}

// get interpupillary distance
float const ViveWindow::get_IPD() const {
    vr::HmdMatrix34_t left_eye = pVRSystem->GetEyeToHeadTransform(vr::EVREye::Eye_Left);
    vr::HmdMatrix34_t right_eye = pVRSystem->GetEyeToHeadTransform(vr::EVREye::Eye_Right);
    return std::fabs(left_eye.m[0][3]) + std::fabs(right_eye.m[0][3]);
}

void ViveWindow::calculate_viewing_setup() {
    // use just any near and far plane values, since they get reset by guacamole
    float near_distance = 0.5;
    float far_distance = 500.0;

    // eye to screen distance can be choosen relatively arbitrary as well,
    // since the virtual screen is enlarged, such that it fits into the frustum
    // at this distance
    float eye_to_screen_distance = 0.08;

    // do the viewing setup calculations for both eyes
    for (unsigned eye_num = 0; eye_num < 2; ++eye_num) {
        vr::EVREye eEye = eye_num == 0 ? vr::EVREye::Eye_Left : vr::EVREye::Eye_Right;
        //retreive the correct projection matrix from the oculus API
        auto const& vive_eye_projection = pVRSystem->GetProjectionMatrix(
            eEye, near_distance, far_distance, vr::EGraphicsAPIConvention::API_OpenGL
        );

        //convert the matrix to a gua compatible one
        scm::math::mat4 scm_eye_proj_matrix;
        for (int outer = 0; outer < 4; ++outer) {
            for (int inner = 0; inner < 4; ++inner) {
                scm_eye_proj_matrix[outer*4 + inner] = vive_eye_projection.m[outer][inner];
            }
        }

        // unproject one frustum corner defining one clipping plane
        scm::math::mat4 inv_oculus_eye_proj_matrix = scm::math::inverse(scm_eye_proj_matrix);
        scm::math::vec4 back_right_top_frustum_corner = scm::math::vec4(-1.0, -1.0, -1.0,  1.0) *
            inv_oculus_eye_proj_matrix;
        scm::math::vec4 back_left_bottom_frustum_corner = scm::math::vec4(1.0,  1.0, -1.0,  1.0) *
            inv_oculus_eye_proj_matrix;

        scm::math::vec4 normalized_back_left_bottom_frustum_corner =
            back_left_bottom_frustum_corner / back_left_bottom_frustum_corner[3];
        scm::math::vec4 normalized_back_right_top_frustum_corner =
            back_right_top_frustum_corner / back_right_top_frustum_corner[3];

        //note: eye is in (0,0,0 relative to the frustum)

        //solve for t1 along the ray to the upper right corner for our screen distance
        float t_to_screen_along_eye_to_right_upper_corner_vec =
            (eye_to_screen_distance) / normalized_back_right_top_frustum_corner[2];

        //retreive the upper right screen corner
        scm::math::vec4 screen_right_upper_corner_relative_to_eye = scm::math::vec4(
            t_to_screen_along_eye_to_right_upper_corner_vec *
            normalized_back_right_top_frustum_corner[0],
            t_to_screen_along_eye_to_right_upper_corner_vec *
            normalized_back_right_top_frustum_corner[1],
            t_to_screen_along_eye_to_right_upper_corner_vec *
            normalized_back_right_top_frustum_corner[2],
            1.0
        );

        //solve for t2 along the ray to the lower yleft corner for our screen distance
        float t_to_screen_along_eye_to_left_lower_corner_vec = (eye_to_screen_distance) /
            normalized_back_left_bottom_frustum_corner[2];

        //retreive the lower left screen corner
        scm::math::vec4 screen_left_lower_corner_relative_to_eye = scm::math::vec4(
            t_to_screen_along_eye_to_left_lower_corner_vec *
            normalized_back_left_bottom_frustum_corner[0],
            t_to_screen_along_eye_to_left_lower_corner_vec *
            normalized_back_left_bottom_frustum_corner[1],
            t_to_screen_along_eye_to_left_lower_corner_vec *
            normalized_back_left_bottom_frustum_corner[2],
            1.0
        );

        // calculate the horizontal and vertical additional offsets of the eye based on the position of the
        // screen corners in relation to the eye
        float horizontal_eye_distortion_offset =
            (std::fabs(screen_left_lower_corner_relative_to_eye[0]) -
            std::fabs(screen_right_upper_corner_relative_to_eye[0])) / 2.0;

        float vertical_eye_distortion_offset =
            (std::fabs(screen_left_lower_corner_relative_to_eye[1]) -
            std::fabs(screen_right_upper_corner_relative_to_eye[1])) / 2.0;

        // get default horizontal offset for both eyes
        float half_IPD = get_IPD() / 2.0;

        // choose the correct initial eye offste
        float standard_horizontal_eye_offset = eye_num == 0 ? -half_IPD : half_IPD;

        // calculate the final screen translation
        screen_translation_[eye_num] = gua::math::vec3(
            standard_horizontal_eye_offset + horizontal_eye_distortion_offset,
            0.0 + vertical_eye_distortion_offset, -eye_to_screen_distance
        );

        // calculate the final screen size
        screen_size_[eye_num] = gua::math::vec2f(
            (screen_right_upper_corner_relative_to_eye - screen_left_lower_corner_relative_to_eye)
        );
    }
}

void ViveWindow::init_context() {
    WindowBase::init_context();

    auto const& glapi = ctx_.render_context->opengl_api();

    glapi.glGenFramebuffers(1, &blit_fbo_read_);
    glapi.glGenFramebuffers(1, &blit_fbo_write_);

    unsigned int width = 0, height = 0;
    if (pVRSystem) {
        pVRSystem->GetRecommendedRenderTargetSize(&width, &height);
        left_texture_ = ctx_.render_device->create_texture_2d(
            scm::math::vec2ui(width, height),
            scm::gl::FORMAT_RGBA_8, 1, 1, 1);
        right_texture_ = ctx_.render_device->create_texture_2d(
            scm::math::vec2ui(width, height),
            scm::gl::FORMAT_RGBA_8, 1, 1, 1);

        // createEyeTexture(glapi, width, height);
        left_tex_id_ = left_texture_->object_id();
        // createEyeTexture(glapi, width, height);
        right_tex_id_ = right_texture_->object_id();
    } else {
        std::cerr << "ERROR: pVRSystem is nullptr." << std::endl;
    }
}

void ViveWindow::open() {
    // open side-by-side debug window which shows exactly the same as oculus
    config.set_title("guacamole");
    config.set_display_name(display_name_);
    config.set_stereo_mode(StereoMode::SIDE_BY_SIDE);

    GlfwWindow::open();
}

math::vec2ui ViveWindow::get_window_resolution() const {
    uint32_t width, height;
    pVRSystem->GetRecommendedRenderTargetSize(&width, &height);
    return math::vec2ui(width, height);
}

math::mat4 const& ViveWindow::get_vive_sensor_orientation() const {
    return vive_sensor_orientation_;
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

    float fSecondsSinceLastVsync;
    pVRSystem->GetTimeSinceLastVsync(&fSecondsSinceLastVsync, NULL);

    float fDisplayFrequency = pVRSystem->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_DisplayFrequency_Float);
    float fFrameDuration = 1.0f / fDisplayFrequency;
    float fVsyncToPhotons = pVRSystem->GetFloatTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SecondsFromVsyncToPhotons_Float);

    float ftiming = fFrameDuration - fSecondsSinceLastVsync + fVsyncToPhotons;

    vr::TrackedDevicePose_t devices[vr::k_unMaxTrackedDeviceCount];
    vr::HmdMatrix34_t pose;
    vr::VRCompositor()->WaitGetPoses(devices, vr::k_unMaxTrackedDeviceCount, NULL, 0);
    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        if (devices[i].bPoseIsValid) {
            if (pVRSystem->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_HMD) {
                pose = devices[i].mDeviceToAbsoluteTracking;
                break;
            }
        }
    }
    math::mat4 orientation(
        pose.m[0][0], pose.m[1][0], pose.m[2][0], 0.0,
        pose.m[0][1], pose.m[1][1], pose.m[2][1], 0.0,
        pose.m[0][2], pose.m[1][2], pose.m[2][2], 0.0,
        pose.m[0][3], pose.m[1][3], pose.m[2][3], 1.0
    );
    vive_sensor_orientation_ = orientation;
}

void ViveWindow::finish_frame() {
    if (left_tex_id_ && right_tex_id_) {
        vr::Texture_t leftEyeTexture{ (void*)left_tex_id_, vr::API_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);

        vr::Texture_t rightEyeTexture{ (void*)right_tex_id_, vr::API_OpenGL, vr::ColorSpace_Gamma };
        vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);
    }

    GlfwWindow::finish_frame();
}

void ViveWindow::display(std::shared_ptr<Texture> const& texture, bool is_left) {
    auto const& glapi = ctx_.render_context->opengl_api();

    if (!left_tex_id_) {
        return;
    }

    GLuint tex_id = is_left ? left_tex_id_ : right_tex_id_;

    // Copy ~texture~ to ~tex_id~
    glapi.glBindTexture(GL_TEXTURE_2D, tex_id);
    glapi.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, blit_fbo_write_);
    glapi.glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex_id, 0);

    GLenum status = glapi.glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        gua::Logger::LOG_WARNING << "Draw Framebuffer Incomplete.\n";
    }

    // setup read buffer
    glapi.glBindTexture(GL_TEXTURE_2D, texture->get_buffer(ctx_)->object_id());
    glapi.glBindFramebuffer(GL_READ_FRAMEBUFFER, blit_fbo_read_);
    glapi.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture->get_buffer(ctx_)->object_id(), 0);

    status = glapi.glCheckFramebufferStatus(GL_READ_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        gua::Logger::LOG_WARNING << "Read Framebuffer Incomplete.\n";
    }

    glapi.glBlitFramebuffer(0, 0, texture->width(), texture->height(),
        config.left_position().x, config.left_position().y,
        config.left_resolution().x, config.left_resolution().y,
        GL_COLOR_BUFFER_BIT, GL_NEAREST);

    glapi.glBindTexture(GL_TEXTURE_2D, 0);
    glapi.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glapi.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    WindowBase::display(texture, is_left);
}

}  // namespace gua
