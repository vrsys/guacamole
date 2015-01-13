#ifndef MVT_DXTCOMPRESSOR_H
#define MVT_DXTCOMPRESSOR_H


#include <ctime>
#include <gua/video3d/video3d_geometry/fastdxt/libdxt.h>



//#ifndef DXTCOMPRESSOR_NUMTHREADS
//#define DXTCOMPRESSOR_NUMTHREADS 2
//#endif

class BackgroundDetector;

namespace mvt{

  class DXTCompressor{

  public:
    DXTCompressor();
    ~DXTCompressor();

    unsigned init(unsigned width, unsigned height, unsigned type = FORMAT_DXT1);
    unsigned getStorageSize();
    unsigned char* compress(unsigned char* buff, bool resetbg = false);
    unsigned getType();
  private:

    void docompress(unsigned tid, unsigned char* buff, bool resetbg = false);


    unsigned _fc;
    //sensor::Timer _timer;
    unsigned _width;
    unsigned _height;
    unsigned _height_sub;
    unsigned _type;
    unsigned _storage;
    unsigned _compressed_buff_sub_size;
    byte* _rgba_buff;
//    byte* _rgba_buff_sub[DXTCOMPRESSOR_NUMTHREADS];
    byte* _compressed_buff;
//    byte* _compressed_buff_sub[DXTCOMPRESSOR_NUMTHREADS];

//    BackgroundDetector* _bgd[DXTCOMPRESSOR_NUMTHREADS];

  };



}



#endif // #ifndef  MVT_DXTCOMPRESSOR_H
