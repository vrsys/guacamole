// class header
#include <gua/renderer/BoneTransformUniformBlock.hpp>

namespace gua {

BoneTransformUniformBlock::BoneTransformUniformBlock(scm::gl::render_device_ptr const& device) {
  uniform_block_ = scm::gl::make_uniform_block<BoneTransformBlock>(device);
}

BoneTransformUniformBlock::~BoneTransformUniformBlock() {
  uniform_block_.reset();
}

void BoneTransformUniformBlock::update(const scm::gl::render_context_ptr& context,
                           std::vector<math::mat4f> const& new_transforms) {


  uniform_block_.begin_manipulation(context); {

  for (unsigned i = 0; i < 100 && i < new_transforms.size(); ++i) {
    uniform_block_->transforms[i] = new_transforms[i];
  }

  } uniform_block_.end_manipulation();
}

} // namespace gua {
