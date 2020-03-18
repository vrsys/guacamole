#include <gua/config.hpp>

#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/databases/TextureDatabase.hpp>

namespace gua
{
CameraUniformBlock::CameraUniformBlock(scm::gl::render_device_ptr const& device) : noise_texture_(math::vec2ui(0)) { uniform_block_ = scm::gl::make_uniform_block<CameraBlock>(device); }

CameraUniformBlock::~CameraUniformBlock() { uniform_block_.reset(); }

void CameraUniformBlock::update(
    RenderContext const& context, Frustum const& cam, math::vec3 const& cyclops_position, std::vector<math::vec4> const& clipping_planes, int view_id, math::vec2ui const& screen_resolution)
{
    if(noise_texture_ == math::vec2ui(0))
    {
        noise_texture_ = TextureDatabase::instance()->lookup("gua_noise_texture")->get_handle(context);
    }

    auto camera_position(cam.get_camera_position());
    auto projection(cam.get_projection());
    auto view(cam.get_view());
    auto projection_inv(scm::math::inverse(projection));

    auto view_projection = projection * view;
    uniform_block_.begin_manipulation(context.render_context);
    {
        uniform_block_->view = view;
        uniform_block_->projection = projection;
        uniform_block_->view_projection = view_projection;
        uniform_block_->projection_inverse = projection_inv;
        uniform_block_->view_projection_inverse = scm::math::inverse(view_projection);
        uniform_block_->position = math::vec4(camera_position, 1.0);
        uniform_block_->resolution = screen_resolution;
        uniform_block_->noise_texture = noise_texture_;
        for(unsigned i(0); i < 64 && i < clipping_planes.size(); ++i)
        {
            uniform_block_->clipping_planes[i] = math::vec4f(clipping_planes[i]);
        }
        uniform_block_->clipping_plane_count = clipping_planes.size();
        uniform_block_->cyclops_position = math::vec4(cyclops_position, 1.0);
        uniform_block_->clip_near = cam.get_clip_near();
        uniform_block_->clip_far = cam.get_clip_far();
        uniform_block_->view_id = view_id;
    }
    uniform_block_.end_manipulation();
}

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
void CameraUniformBlock::update(
    RenderContext const& context, Frustum const& cam, Frustum const& secondary_cam, math::vec3 const& cyclops_position, std::vector<math::vec4> const& clipping_planes, int view_id, math::vec2ui const& screen_resolution)
{
    if(noise_texture_ == math::vec2ui(0))
    {
        noise_texture_ = TextureDatabase::instance()->lookup("gua_noise_texture")->get_handle(context);
    }

    // left camera
    auto camera_position(cam.get_camera_position());
    auto projection(cam.get_projection());
    auto view(cam.get_view());
    auto projection_inv(scm::math::inverse(projection));
    auto view_projection = projection * view;


    // right camera
    auto secondary_camera_position(secondary_cam.get_camera_position());
    auto secondary_projection(secondary_cam.get_projection());
    auto secondary_view(secondary_cam.get_view());
    auto secondary_projection_inv(scm::math::inverse(secondary_projection));
    auto secondary_view_projection = secondary_projection * secondary_view;

    uniform_block_.begin_manipulation(context.render_context);
    {
        // left camera
        uniform_block_->view = view;
        uniform_block_->projection = projection;
        uniform_block_->view_projection = view_projection;
        uniform_block_->projection_inverse = projection_inv;
        uniform_block_->view_projection_inverse = scm::math::inverse(view_projection);
        uniform_block_->position = math::vec4(camera_position, 1.0);
        
        // right camera
        uniform_block_->secondary_view = secondary_view;
        uniform_block_->secondary_projection = secondary_projection;
        uniform_block_->secondary_projection_inverse = secondary_projection_inv;
        uniform_block_->secondary_view_projection = secondary_view_projection;
        uniform_block_->secondary_view_projection_inverse = scm::math::inverse(secondary_view_projection);
        uniform_block_->secondary_position = math::vec4(secondary_camera_position, 1.0);

        uniform_block_->resolution = screen_resolution;
        uniform_block_->noise_texture = noise_texture_;
<<<<<<< HEAD
        for(unsigned int clipping_plane_idx = 0; clipping_plane_idx < 64 && clipping_plane_idx < clipping_planes.size(); ++clipping_plane_idx)
=======

        for(unsigned i(0); i < 64 && i < clipping_planes.size(); ++i)
>>>>>>> upstream_project_rt_oc/feature/multi_view_rendering
        {
            uniform_block_->clipping_planes[clipping_plane_idx] = math::vec4f(clipping_planes[clipping_plane_idx]);
        }

        uniform_block_->clipping_plane_count = clipping_planes.size();
        uniform_block_->cyclops_position = math::vec4(cyclops_position, 1.0);
        uniform_block_->clip_near = cam.get_clip_near();
        uniform_block_->clip_far = cam.get_clip_far();
        uniform_block_->view_id = view_id;
    }
    uniform_block_.end_manipulation();
}
#endif


void CameraUniformBlock::updateHMD(RenderContext const &context, Frustum const &cam, math::mat4 const& camera_parents_transform, math::vec3 const &cyclops_position, std::vector<math::vec4> const &clipping_planes, int view_id,
                                math::vec2ui const &screen_resolution)
{


    const unsigned base_num = CameraMode::LEFT == context.mode ? 0 : 2;
    math::mat4 mat4eyePos = context.render_window->get_latest_matrices(base_num);
    math::mat4 mat4Projection = context.render_window->get_latest_matrices(base_num + 1);
    math::mat4 mat4view = scm::math::inverse(context.render_window->get_latest_matrices(4));
    math::mat4 inverse_navigation = scm::math::inverse(camera_parents_transform);


    if(noise_texture_ == math::vec2ui(0))
    {
        noise_texture_ = TextureDatabase::instance()->lookup("gua_noise_texture")->get_handle(context);
    }


    auto projection(/*cam.get_projection()*/mat4Projection);
    auto view(/*cam.get_view() */ mat4eyePos * mat4view * inverse_navigation);
    cam.set_view(view);
    //auto view(mat4eyePos * scm::math::inverse(cam.get_view()));
    auto projection_inv(scm::math::inverse(projection));
    auto view_projection = projection * view;
    //auto camera_position(cam.get_camera_position());
    auto camera_position = scm::math::inverse(view) * gua::math::vec4(0.0, 0.0, 0.0, 1.0);

    uniform_block_.begin_manipulation(context.render_context);
    {
        uniform_block_->view = view;
        uniform_block_->projection = projection;
        uniform_block_->view_projection = view_projection;
        uniform_block_->projection_inverse = projection_inv;
        uniform_block_->view_projection_inverse = scm::math::inverse(view_projection);
        //uniform_block_->position = gua::math::vec4(camera_position, 1.0f);
        uniform_block_->position = camera_position;
        uniform_block_->resolution = screen_resolution;
        uniform_block_->noise_texture = noise_texture_;
        for(unsigned int clipping_plane_idx = 0; clipping_plane_idx < 64 && clipping_plane_idx < clipping_planes.size(); ++clipping_plane_idx)
        {
            uniform_block_->clipping_planes[clipping_plane_idx] = math::vec4f(clipping_planes[clipping_plane_idx]);
        }
        uniform_block_->clipping_plane_count = clipping_planes.size();
        uniform_block_->cyclops_position = math::vec4(cyclops_position, 1.0);
        uniform_block_->clip_near = cam.get_clip_near();
        uniform_block_->clip_far = cam.get_clip_far();
        uniform_block_->view_id = view_id;
    }
    uniform_block_.end_manipulation();
}

} // namespace gua
