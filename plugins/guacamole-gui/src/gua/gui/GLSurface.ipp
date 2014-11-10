////////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of Swift2D.                                              //
//                                                                            //
// Copyright: (c) 2011-2014 Simon Schneegans & Felix Lauer                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

class GLSurface : public Awesomium::Surface {

 ///////////////////////////////////////////////////////////////////////////////
 // ----------------------------------------------------------- public interface
 public:

  // ----------------------------------------------------- contruction interface
  GLSurface(int width, int height)
    : tex_(nullptr)
    , buffer_(width * height * 4)
    , width_(width)
    , height_(height)
    , needs_update_(false) {}

  ~GLSurface() {
    delete tex_;
  }

  // ------------------------------------------------------------ public methods

  //////////////////////////////////////////////////////////////////////////////

  void init(RenderContext const& ctx) {
    std::unique_lock<std::mutex> lock(mutex_);
    tex_ = new oglplus::Texture();
    oglplus::Texture::Active(0);
    ctx.gl.Bound(oglplus::Texture::Target::_2D, *tex_)
      .Image2D(0, oglplus::PixelDataInternalFormat::RGBA, width_, height_,
        0, oglplus::PixelDataFormat::BGRA,
        oglplus::PixelDataType::UnsignedByte, &buffer_.front()
      )
      .MaxLevel(0)
      .MinFilter(oglplus::TextureMinFilter::Linear)
      .MagFilter(oglplus::TextureMagFilter::Linear)
      .WrapS(oglplus::TextureWrap::ClampToEdge)
      .WrapT(oglplus::TextureWrap::ClampToEdge);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool bind(RenderContext const& ctx, unsigned location) {

    if (!tex_) {
      if (ctx.upload_budget > 0) {
        --ctx.upload_budget;
         init(ctx);
      } else {
        ++ctx.upload_remaining;
        return false;
      }
    }

    tex_->Active(location);
    ctx.gl.Bind(ose::_2D(), *tex_);

    if (needs_update_) {
      std::unique_lock<std::mutex> lock(mutex_);
      needs_update_ = false;

      tex_->SubImage2D(
        oglplus::Texture::Target::_2D, 0, 0, 0, width_, height_,
        oglplus::PixelDataFormat::BGRA, oglplus::PixelDataType::UnsignedByte,
        &buffer_.front()
      );
    }

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  void Paint(unsigned char* src_buffer, int src_row_span,
             Awesomium::Rect const& src_rect,
             Awesomium::Rect const& dest_rect) {

    std::unique_lock<std::mutex> lock(mutex_);

    for (int row = 0; row < dest_rect.height; row++) {
      memcpy(&buffer_.front() + (row + dest_rect.y) * width_*4 + (dest_rect.x * 4),
             src_buffer + (row + src_rect.y) * src_row_span + (src_rect.x * 4),
             dest_rect.width * 4);
    }

    needs_update_ = true;
  }

  //////////////////////////////////////////////////////////////////////////////

  void Scroll(int dx, int dy, Awesomium::Rect const& clip_rect) {

    if (abs(dx) >= clip_rect.width || abs(dy) >= clip_rect.height) {
      return;
    }

    std::unique_lock<std::mutex> lock(mutex_);

    if (dx < 0 && dy == 0) {
      // Area shifted left by dx
      unsigned char* tempBuffer = new unsigned char[(clip_rect.width + dx) * 4];

      for (int i = 0; i < clip_rect.height; i++) {
        memcpy(tempBuffer, &buffer_.front() + (i + clip_rect.y) * width_*4 +
               (clip_rect.x - dx) * 4, (clip_rect.width + dx) * 4);
        memcpy(&buffer_.front() + (i + clip_rect.y) * width_*4 + (clip_rect.x) * 4,
               tempBuffer, (clip_rect.width + dx) * 4);
      }

      delete[] tempBuffer;

    } else if (dx > 0 && dy == 0) {
      // Area shifted right by dx
      unsigned char* tempBuffer = new unsigned char[(clip_rect.width - dx) * 4];

      for (int i = 0; i < clip_rect.height; i++) {
        memcpy(tempBuffer, &buffer_.front() + (i + clip_rect.y) * width_*4 +
               (clip_rect.x) * 4, (clip_rect.width - dx) * 4);
        memcpy(&buffer_.front() + (i + clip_rect.y) * width_*4 + (clip_rect.x + dx) * 4,
               tempBuffer, (clip_rect.width - dx) * 4);
      }

      delete[] tempBuffer;

    } else if (dy < 0 && dx == 0) {
      // Area shifted down by dy
      for (int i = 0; i < clip_rect.height + dy ; i++)
        memcpy(&buffer_.front() + (i + clip_rect.y) * width_*4 + (clip_rect.x * 4),
               &buffer_.front() + (i + clip_rect.y - dy) * width_*4 + (clip_rect.x * 4),
               clip_rect.width * 4);
    } else if (dy > 0 && dx == 0) {
      // Area shifted up by dy
      for (int i = clip_rect.height - 1; i >= dy; i--)
        memcpy(&buffer_.front() + (i + clip_rect.y) * width_*4 + (clip_rect.x * 4),
               &buffer_.front() + (i + clip_rect.y - dy) * width_*4 + (clip_rect.x * 4),
               clip_rect.width * 4);
    }

    needs_update_ = true;
  }

 ///////////////////////////////////////////////////////////////////////////////
 // ---------------------------------------------------------- private interface
 private:
  oglplus::Texture* tex_;

  std::vector<unsigned char>  buffer_;

  int        width_;
  int        height_;
  std::mutex mutex_;
  bool       needs_update_;
};
