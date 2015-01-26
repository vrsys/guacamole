#include <gua/renderer/CameraUniformBlock.hpp>

namespace gua {

CameraUniformBlock::CameraUniformBlock(scm::gl::render_device_ptr const& device)
{
  uniform_block_ = scm::gl::make_uniform_block<CameraBlock>(device);
}

CameraUniformBlock::~CameraUniformBlock()
{
  uniform_block_.reset();
}

void CameraUniformBlock::update(const scm::gl::render_context_ptr& context,
                           Frustum const& cam, int view_id) {

  auto camera_position(cam.get_camera_position());
  auto projection(cam.get_projection());
  auto view(cam.get_view());
  auto projection_inv(scm::math::inverse(projection));

  uniform_block_.begin_manipulation(context); {
      uniform_block_->view = view;
      uniform_block_->projection = projection;
      uniform_block_->projection_inverse = projection_inv;
      uniform_block_->projection_view_inverse = scm::math::inverse(projection * view);
      uniform_block_->position = math::vec4(camera_position, 1.0);
      uniform_block_->clip_near = cam.get_clip_near();
      uniform_block_->clip_far = cam.get_clip_far();
      uniform_block_->view_id = view_id;
  } uniform_block_.end_manipulation();
}

} // namespace gua {
