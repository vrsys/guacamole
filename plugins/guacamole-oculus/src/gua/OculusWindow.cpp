/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/OculusWindow.hpp>

// gua headers
#include <gua/utils/Logger.hpp>
#include <gua/databases/Resources.hpp>
#include <scm/gl_core/render_device/opengl/gl_core.h>

// external headers
#include <iostream>

namespace gua
{
OculusWindow::OculusWindow(std::string const& display) : GlfwWindow(), display_name_(display)
{
    // initialize hmd and texture sizes
    initialize_hmd_environment();

    // calculate screen size, translation and eye distance
    calculate_viewing_setup();
}

////////////////////////////////////////////////////////////////////////////////

OculusWindow::~OculusWindow()
{
    ovr_Shutdown();
    ovr_Destroy(hmd_session_);
}

////////////////////////////////////////////////////////////////////////////////
void OculusWindow::open()
{
    // open side-by-side debug window which shows exactly the same as oculus
    config.set_title("guacamole");
    config.set_display_name(display_name_);
    config.set_stereo_mode(StereoMode::SIDE_BY_SIDE);

    GlfwWindow::open();
}

////////////////////////////////////////////////////////////////////////////////

void OculusWindow::init_context()
{
    WindowBase::init_context();

    auto const& glapi = ctx_.render_context->opengl_api();

    glapi.glGenFramebuffers(1, &blit_fbo_read_);
    glapi.glGenFramebuffers(1, &blit_fbo_write_);

    ovrSizei recommended_size_r = ovr_GetFovTextureSize(hmd_session_, ovrEye_Left, hmd_desc_.DefaultEyeFov[ovrEye_Left], 1.0f);
    ovrSizei recommended_size_l = ovr_GetFovTextureSize(hmd_session_, ovrEye_Right, hmd_desc_.DefaultEyeFov[ovrEye_Right], 1.0f);

    texture_swap_chain_desc_.Type = ovrTexture_2D;
    texture_swap_chain_desc_.ArraySize = 1;
    texture_swap_chain_desc_.Width = recommended_size_l.w + recommended_size_r.w;
    texture_swap_chain_desc_.Height = recommended_size_l.h;
    texture_swap_chain_desc_.MipLevels = 1;
    texture_swap_chain_desc_.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
    texture_swap_chain_desc_.SampleCount = 1;
    texture_swap_chain_desc_.StaticImage = ovrFalse;

    auto result = ovr_CreateTextureSwapChainGL(hmd_session_, &texture_swap_chain_desc_, &texture_swap_chain_);
    if(result != ovrSuccess)
    {
        ovrErrorInfo info;
        ovr_GetLastErrorInfo(&info);
        gua::Logger::LOG_WARNING << "Failed to create swap textures for Oculus Rift.\n"
                                 << "Error Code: " << info.ErrorString << std::endl;
    }

    color_layer_.ColorTexture[0] = texture_swap_chain_;
    color_layer_.ColorTexture[1] = texture_swap_chain_;
}

////////////////////////////////////////////////////////////////////////////////

void OculusWindow::display(std::shared_ptr<Texture> const& texture, bool is_left)
{
    auto const& glapi = ctx_.render_context->opengl_api();

    // setup draw buffer
    GLuint tex_id;
    int current_index;

    ovr_GetTextureSwapChainCurrentIndex(hmd_session_, texture_swap_chain_, &current_index);
    ovr_GetTextureSwapChainBufferGL(hmd_session_, texture_swap_chain_, current_index, &tex_id);

    glapi.glBindTexture(GL_TEXTURE_2D, tex_id);
    glapi.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, blit_fbo_write_);
    glapi.glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex_id, 0);

    GLenum status = glapi.glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
    if(status != GL_FRAMEBUFFER_COMPLETE)
    {
        gua::Logger::LOG_WARNING << "Incomplete.\n";
    }

    // setup read buffer
    glapi.glBindTexture(GL_TEXTURE_2D, texture->get_buffer(ctx_)->object_id());
    glapi.glBindFramebuffer(GL_READ_FRAMEBUFFER, blit_fbo_read_);
    glapi.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture->get_buffer(ctx_)->object_id(), 0);
    // glapi.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);

    status = glapi.glCheckFramebufferStatus(GL_READ_FRAMEBUFFER);
    if(status != GL_FRAMEBUFFER_COMPLETE)
    {
        gua::Logger::LOG_WARNING << "Incomplete.\n";
    }

    if(is_left)
    {
        glapi.glBlitFramebuffer(
            0, texture->height(), texture->width(), 0, config.left_position().x, config.left_position().y, config.left_resolution().x, config.left_resolution().y, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
    else
    {
        glapi.glBlitFramebuffer(0,
                                texture->height(),
                                texture->width(),
                                0,
                                config.right_position().x,
                                config.right_position().y,
                                config.right_position().x + config.right_resolution().x,
                                config.right_resolution().y,
                                GL_COLOR_BUFFER_BIT,
                                GL_NEAREST);
    }

    glapi.glBindTexture(GL_TEXTURE_2D, 0);
    glapi.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glapi.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    WindowBase::display(texture, is_left);
}

math::vec2 const& OculusWindow::get_left_screen_size() const { return screen_size_[0]; }

math::vec2 const& OculusWindow::get_right_screen_size() const { return screen_size_[1]; }

math::vec3 const& OculusWindow::get_left_screen_translation() const { return screen_translation_[0]; }

math::vec3 const& OculusWindow::get_right_screen_translation() const { return screen_translation_[1]; }

float const OculusWindow::get_IPD() const
{
    ovrEyeRenderDesc eyeRenderDesc[2];

    eyeRenderDesc[0] = ovr_GetRenderDesc(hmd_session_, ovrEye_Left, hmd_desc_.DefaultEyeFov[0]);
    eyeRenderDesc[1] = ovr_GetRenderDesc(hmd_session_, ovrEye_Right, hmd_desc_.DefaultEyeFov[1]);

    ovrVector3f hmdToEyeViewOffset[2];

    hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeOffset;
    hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeOffset;

    return std::fabs(hmdToEyeViewOffset[0].x) + std::fabs(hmdToEyeViewOffset[1].x);
}

math::vec2ui OculusWindow::get_window_resolution() const { return config.get_size(); }

gua::math::mat4 const& OculusWindow::get_hmd_sensor_orientation() const { return hmd_sensor_orientation_; }

void OculusWindow::start_frame()
{
    GlfwWindow::start_frame();

    auto ftiming = ovr_GetPredictedDisplayTime(hmd_session_, 0);

    ovrTrackingState hmdState = ovr_GetTrackingState(hmd_session_, ftiming, true);

    ovrEyeRenderDesc eyeRenderDesc[2];
    ovrVector3f hmdToEyeViewOffset[2];
    eyeRenderDesc[0] = ovr_GetRenderDesc(hmd_session_, ovrEye_Left, hmd_desc_.DefaultEyeFov[0]);
    eyeRenderDesc[1] = ovr_GetRenderDesc(hmd_session_, ovrEye_Right, hmd_desc_.DefaultEyeFov[1]);

    hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeOffset;
    hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeOffset;

    ovr_CalcEyePoses(hmdState.HeadPose.ThePose, hmdToEyeViewOffset, color_layer_.RenderPose);

    if(hmdState.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked))
    {
        auto pose = hmdState.HeadPose.ThePose;

        scm::math::quat<double> rot_quat(pose.Orientation.w, pose.Orientation.x, pose.Orientation.y, pose.Orientation.z);

        hmd_sensor_orientation_ = scm::math::make_translation((double)pose.Position.x, (double)pose.Position.y, (double)pose.Position.z) * rot_quat.to_matrix();
    }
}

void OculusWindow::finish_frame()
{
    ovr_CommitTextureSwapChain(hmd_session_, texture_swap_chain_);

    ovrLayerHeader* layers = &color_layer_.Header;
    ovrResult result = ovr_SubmitFrame(hmd_session_, framecount_, nullptr, &layers, 1);

    if(!OVR_SUCCESS(result))
    {
        gua::Logger::LOG_WARNING << "Failed to submit frame to the oculus rift.\n";
    }

    GlfwWindow::finish_frame();
}

void OculusWindow::initialize_hmd_environment()
{
    try
    {
        auto result = ovr_Initialize(nullptr);

        if(!OVR_SUCCESS(result))
        {
            Logger::LOG_WARNING << "Failed to initialize oculus environment!"
                                << "Errorcode:" << (int)result << std::endl;
        }

        ovrGraphicsLuid luid;
        result = ovr_Create(&hmd_session_, &luid);

        if(!OVR_SUCCESS(result))
        {
            throw std::runtime_error("Unable to create HMD.");
        }

        hmd_desc_ = ovr_GetHmdDesc(hmd_session_);

        // get optimal texture size for rendering
        ovrSizei ideal_texture_size_left = ovr_GetFovTextureSize(hmd_session_, ovrEyeType(0), hmd_desc_.DefaultEyeFov[0], 1);
        ovrSizei ideal_texture_size_right = ovr_GetFovTextureSize(hmd_session_, ovrEyeType(1), hmd_desc_.DefaultEyeFov[1], 1);

        math::vec2ui window_size(ideal_texture_size_left.w + ideal_texture_size_right.w, std::max(ideal_texture_size_left.h, ideal_texture_size_right.h));

        // initialize window => resolution is independent of rendering resolution!
        config.set_size(window_size);

        config.set_left_resolution(math::vec2ui(ideal_texture_size_left.w, ideal_texture_size_left.h));
        config.set_left_position(math::vec2ui(0, 0));
        config.set_right_resolution(math::vec2ui(ideal_texture_size_right.w, ideal_texture_size_right.h));
        config.set_right_position(math::vec2ui(ideal_texture_size_left.w, 0));

        // Initialize VR structures, filling out description.
        ovrEyeRenderDesc eyeRenderDesc[2];
        ovrVector3f hmdToEyeViewOffset[2];

        eyeRenderDesc[0] = ovr_GetRenderDesc(hmd_session_, ovrEye_Left, hmd_desc_.DefaultEyeFov[0]);
        eyeRenderDesc[1] = ovr_GetRenderDesc(hmd_session_, ovrEye_Right, hmd_desc_.DefaultEyeFov[1]);

        hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeOffset;
        hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeOffset;

        // Initialize our single full screen Fov layer.
        color_layer_.Header.Type = ovrLayerType_EyeFov;
        color_layer_.Header.Flags = 0;
        color_layer_.Fov[0] = eyeRenderDesc[0].Fov;
        color_layer_.Fov[1] = eyeRenderDesc[1].Fov;

        ovrRecti left_viewport;
        left_viewport.Size = {int(config.left_resolution().x), int(config.left_resolution().y)};
        left_viewport.Pos = {int(config.left_position().x), int(config.left_position().y)};

        ovrRecti right_viewport;
        right_viewport.Size = {int(config.right_resolution().x), int(config.right_resolution().y)};
        right_viewport.Pos = {int(config.right_position().x), int(config.right_position().y)};

        color_layer_.Viewport[0] = left_viewport;
        color_layer_.Viewport[1] = right_viewport;
    }
    catch(std::exception& e)
    {
        gua::Logger::LOG_WARNING << "Failed to initialize oculus rift.\n" << e.what() << std::endl;
    }
}

void OculusWindow::calculate_viewing_setup()
{
    // use just any near and far plane values, since they get reset by guacamole
    float near_distance = 0.5;
    float far_distance = 500.0;

    // eye to screen distance can be choosen relatively arbitrary as well, since
    // the virtual screen is enlarged, such that it fits into the frustum at this
    // distance
    float eye_to_screen_distance = 0.08;

    // do the viewing setup calculations for both eyes
    for(unsigned eye_num = 0; eye_num < 2; ++eye_num)
    {
        // retreive the correct projection matrix from the oculus API

        auto const& hmd_eye_projection = ovrMatrix4f_Projection(hmd_desc_.DefaultEyeFov[eye_num], near_distance, far_distance, 0);

        // convert the matrix to a gua compatible one
        scm::math::mat4 scm_eye_proj_matrix;
        for(int outer = 0; outer < 4; ++outer)
        {
            for(int inner = 0; inner < 4; ++inner)
            {
                scm_eye_proj_matrix[outer * 4 + inner] = hmd_eye_projection.M[outer][inner];
            }
        }

        // unproject one frustum corner defining one clipping plane
        scm::math::mat4 inv_hmd_eye_proj_matrix = scm::math::inverse(scm_eye_proj_matrix);
        scm::math::vec4 back_right_top_frustum_corner = scm::math::vec4(-1.0, -1.0, -1.0, 1.0) * inv_hmd_eye_proj_matrix;
        scm::math::vec4 back_left_bottom_frustum_corner = scm::math::vec4(1.0, 1.0, -1.0, 1.0) * inv_hmd_eye_proj_matrix;

        scm::math::vec4 normalized_back_left_bottom_frustum_corner = back_left_bottom_frustum_corner / back_left_bottom_frustum_corner[3];
        scm::math::vec4 normalized_back_right_top_frustum_corner = back_right_top_frustum_corner / back_right_top_frustum_corner[3];

        // note: eye is in (0,0,0 relative to the frustum)

        // solve for t1 along the ray to the upper right corner for our screen distance
        float t_to_screen_along_eye_to_right_upper_corner_vec = (eye_to_screen_distance) / normalized_back_right_top_frustum_corner[2];

        // retreive the upper right screen corner
        scm::math::vec4 screen_right_upper_corner_relative_to_eye = scm::math::vec4(t_to_screen_along_eye_to_right_upper_corner_vec * normalized_back_right_top_frustum_corner[0],
                                                                                    t_to_screen_along_eye_to_right_upper_corner_vec * normalized_back_right_top_frustum_corner[1],
                                                                                    t_to_screen_along_eye_to_right_upper_corner_vec * normalized_back_right_top_frustum_corner[2],
                                                                                    1.0);

        // solve for t2 along the ray to the lower left corner for our screen distance
        float t_to_screen_along_eye_to_left_lower_corner_vec = (eye_to_screen_distance) / normalized_back_left_bottom_frustum_corner[2];

        // retreive the lower left screen corner
        scm::math::vec4 screen_left_lower_corner_relative_to_eye = scm::math::vec4(t_to_screen_along_eye_to_left_lower_corner_vec * normalized_back_left_bottom_frustum_corner[0],
                                                                                   t_to_screen_along_eye_to_left_lower_corner_vec * normalized_back_left_bottom_frustum_corner[1],
                                                                                   t_to_screen_along_eye_to_left_lower_corner_vec * normalized_back_left_bottom_frustum_corner[2],
                                                                                   1.0);

        // calculate the horizontal and vertical additional offsets of the eye based on the position of the
        // screen corners in relation to the eye
        float horizontal_eye_distortion_offset = (std::fabs(screen_left_lower_corner_relative_to_eye[0]) - std::fabs(screen_right_upper_corner_relative_to_eye[0])) / 2.0;

        float vertical_eye_distortion_offset = (std::fabs(screen_left_lower_corner_relative_to_eye[1]) - std::fabs(screen_right_upper_corner_relative_to_eye[1])) / 2.0;

        // get default horizontal offset for both eyes
        float half_IPD = get_IPD() / 2.0;

        // choose the correct initial eye offste
        float standard_horizontal_eye_offset = eye_num == 0 ? -half_IPD : half_IPD;

        // calculate the final screen translation
        screen_translation_[eye_num] = gua::math::vec3(standard_horizontal_eye_offset + horizontal_eye_distortion_offset, 0.0 + vertical_eye_distortion_offset, -eye_to_screen_distance);

        // calculate the final screen size
        screen_size_[eye_num] = gua::math::vec2f((screen_right_upper_corner_relative_to_eye - screen_left_lower_corner_relative_to_eye));
    }
}

} // namespace gua
