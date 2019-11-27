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

    uniform_block_.begin_manipulation(context.render_context);
    {
        uniform_block_->view = view;
        uniform_block_->projection = projection;
        uniform_block_->projection_inverse = projection_inv;
        uniform_block_->projection_view_inverse = scm::math::inverse(projection * view);
        uniform_block_->position = math::vec4(camera_position, 1.0);
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
    //auto camera_position(cam.get_camera_position());
    auto camera_position = scm::math::inverse(view) * gua::math::vec4(0.0, 0.0, 0.0, 1.0);

    uniform_block_.begin_manipulation(context.render_context);
    {
        uniform_block_->view = view;
        uniform_block_->projection = projection;
        uniform_block_->projection_inverse = projection_inv;
        uniform_block_->projection_view_inverse = scm::math::inverse(projection * view);
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
