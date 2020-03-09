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
#include <gua/renderer/ScreenGrabPass.hpp>

namespace gua
{
ScreenGrabPassDescription::ScreenGrabPassDescription() : PipelinePassDescription(), output_prefix_(""), grab_next_(false)
{
    vertex_shader_ = "";
    geometry_shader_ = "";
    fragment_shader_ = "";
    private_.name_ = "ScreenGrabPass";

    private_.needs_color_buffer_as_input_ = true;
    private_.writes_only_color_buffer_ = true;
    private_.enable_for_shadows_ = false;
    private_.rendermode_ = RenderMode::Custom;

    private_.depth_stencil_state_desc_ = boost::make_optional(scm::gl::depth_stencil_state_desc(true, true, scm::gl::COMPARISON_LESS, true, 1, 0, scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)));

    private_.rasterizer_state_desc_ =
        boost::make_optional(scm::gl::rasterizer_state_desc(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0f, false, true, scm::gl::point_raster_state(true)));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> ScreenGrabPassDescription::make_copy() const { return std::make_shared<ScreenGrabPassDescription>(*this); }

PipelinePass ScreenGrabPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
    private_.process_ = [&](PipelinePass&, PipelinePassDescription const&, Pipeline& pipe) {
        RenderContext const& ctx(pipe.get_context());

#ifdef GUACAMOLE_ENABLE_TURBOJPEG
        if(grab_next_)
        {
            grab_next_ = false;
            scm::gl::texture_2d_ptr color_buffer = pipe.get_gbuffer()->get_color_buffer();
            scm::math::vec2ui dims = color_buffer->dimensions();

            auto format = color_buffer->format();

            if(format != scm::gl::data_format::FORMAT_RGB_32F)
            {
                std::cerr << "Invalid use of ScreenGrabPass with non-standard color buffer" << std::endl;
                return;
            }

            std::vector<float> host_color_buffer(dims.x * dims.y * 3);

            ctx.render_context->retrieve_texture_data(color_buffer, 0, &host_color_buffer[0]);
            ctx.render_context->sync();

            ScreenGrabJPEGSaver::instance()->save(output_prefix_, dims, host_color_buffer);
        }
#endif
    };

    PipelinePass pass{*this, ctx, substitution_map};
    return pass;
}
void ScreenGrabPassDescription::set_output_prefix(const std::string& output_prefix)
{
    output_prefix_ = output_prefix;
    touch();
}
void ScreenGrabPassDescription::set_grab_next(bool grab_next)
{
    grab_next_ = grab_next;
    touch();
}



#ifdef GUACAMOLE_ENABLE_TURBOJPEG
#include <stdio.h>
#include <jpeglib.h>
#include <setjmp.h>

ScreenGrabJPEGSaver::ScreenGrabJPEGSaver() : save_lock_(), should_quit_(false), should_save_(false), save_cv_(), output_prefix_(""), dims_(), rgb_32f_(), rgb_8_()     {
    worker_ = std::thread([&]() -> void {
        while(!should_quit_.load())
        {
            std::unique_lock<std::mutex> lk(save_lock_, std::defer_lock);
            if(should_save_.load())
            {
                save_();
            }
            should_save_.store(false);

            while(!save_cv_.wait_for(lk, std::chrono::milliseconds(16), [&]() -> bool { return should_save_.load(); }))
            {
                if(should_quit_.load())
                {
                    break;
                }
            }
        }
    });
}

ScreenGrabJPEGSaver::~ScreenGrabJPEGSaver() {
    should_save_.store(false);
    should_quit_.store(true);
    save_cv_.notify_one();
    worker_.join();
}

void ScreenGrabJPEGSaver::save(std::string const& output_prefix, scm::math::vec2ui const& dims, std::vector<float>& rgb_32f) {
    output_prefix_ = output_prefix;
    dims_ = dims;
    rgb_32f_.swap(rgb_32f);

    size_t length = dims_.x * dims_.y * 3;

    if(rgb_8_.size() != length)
    {
        rgb_8_.resize(length);
    }

    should_save_.store(true);
    save_cv_.notify_one();
}

bool ScreenGrabJPEGSaver::write_JPEG_file(const char* filename, int quality) const {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    FILE* outfile;           /* target file */
    JSAMPROW row_pointer[1]; /* pointer to JSAMPLE row[s] */
    int row_stride;          /* physical row width in image buffer */
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    if((outfile = fopen(filename, "wb")) == NULL)
    {
        fprintf(stderr, "can't open %s\n", filename);
        return false;
    }
    jpeg_stdio_dest(&cinfo, outfile);

    cinfo.image_width = dims_.x; /* image width and height, in pixels */
    cinfo.image_height = dims_.y;
    cinfo.input_components = 3;     /* # of color components per pixel */
    cinfo.in_color_space = JCS_RGB; /* colorspace of input image */

    jpeg_set_defaults(&cinfo);

    jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);

    jpeg_start_compress(&cinfo, TRUE);

    row_stride = dims_.x * 3; /* JSAMPLEs per row in image_buffer */

    while(cinfo.next_scanline < cinfo.image_height)
    {
        row_pointer[0] = const_cast <unsigned char*> (&rgb_8_[cinfo.next_scanline * row_stride]);
        (void)jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);
    fclose(outfile);

    jpeg_destroy_compress(&cinfo);

    return true;
}

void ScreenGrabJPEGSaver::save_() {

    #define F2B(f) ((f) >= 1.0 ? (unsigned char)255 : (unsigned char)((f)*256.0))
    
    unsigned int length = dims_.x * dims_.y;

    for(unsigned int i = 0; i < length; ++i)
    {
        unsigned offset = i * 3;
        unsigned flipped_offset = ((dims_.y - i / dims_.x) * dims_.x + i % dims_.x) * 3;

        rgb_8_[offset] = F2B(rgb_32f_[flipped_offset]);
        rgb_8_[offset + 1] = F2B(rgb_32f_[flipped_offset + 1]);
        rgb_8_[offset + 2] = F2B(rgb_32f_[flipped_offset + 2]);
    }

    std::size_t millis = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    std::string timestamp{std::to_string(millis)};

    if(!write_JPEG_file((output_prefix_ + timestamp + ".jpg").c_str(), 100))
    {
        std::cerr << "Could not write JPEG" << std::endl;
    }
}
#else
ScreenGrabJPEGSaver::ScreenGrabJPEGSaver() : save_lock_(), should_quit_(true), should_save_(false), save_cv_(), output_prefix_(""), dims_(), rgb_32f_(), rgb_8_()     {
    Logger::LOG_WARNING << "Guacamole is not compiled with turbojpeg enabled." << std::endl;
    Logger::LOG_WARNING << "Will not create JPEG screengrabs." << std::endl;
}

ScreenGrabJPEGSaver::~ScreenGrabJPEGSaver() {
}

#endif //GUACAMOLE_ENABLE_TURBOJPEG

} // namespace gua