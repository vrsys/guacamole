#ifndef KINECTCALIBRATIONFILE_HPP
#define KINECTCALIBRATIONFILE_HPP

#include <scm/core.h>
#include <scm/gl_core.h>

#include <string>

/* cameraView & KinectCalibrationFile
/opt/svn/multiViewTools/libKinect/src
kinectPlayer
/opt/demo/kinecting/simple
*/

namespace video3d
{
struct xyz
{
    float x;
    float y;
    float z;
};

struct uv
{
    float u;
    float v;
};
} // namespace video3d

class KinectCalibrationFile
{
  public:
    KinectCalibrationFile(const std::string& filePath);
    ~KinectCalibrationFile();

    bool parse(); // parse .yml file

    void printInfo() const;

    void updateMatrices();

    std::string get_stream_filename() const;

    /////////////////GETTER/SETTER//////////////////
    scm::math::mat4f const& getImageDToEyeD() const;
    scm::math::mat4f const& getEyeDToImageD() const;
    scm::math::mat4f const& getEyeDToWorld() const;
    scm::math::mat4f const& getEyeDToEyeRGB() const;
    scm::math::mat4f const& getEyeRGBToImageRGB() const;
    scm::math::mat4f const& getEyeRGBToWorld() const;

    scm::math::vec2f const& getColorFocalLength() const;
    scm::math::vec2f const& getColorPrincipalPoint() const;
    scm::math::vec3f const& getColorRadialDistortion() const;
    scm::math::vec2f const& getColorTangentialDistortion() const;

    scm::math::vec2f const& getDepthFocalLength() const;
    scm::math::vec2f const& getDepthPrincipalPoint() const;
    scm::math::vec3f const& getDepthRadialDistortion() const;
    scm::math::vec2f const& getDepthTangentialDistortion() const;

    scm::math::mat4f const& getRelativeRotation() const;
    scm::math::vec3f const& getRelativeTranslation() const;

    scm::math::mat4f const& getWorldRotation() const;
    void setWorldRotation(scm::math::mat4f& r);

    scm::math::vec3f const& getWorldTranslation() const;
    void setWorldTranslation(scm::math::vec3f& t);

    float* getIntrinsicRGB9();
    float* getDistortionRGB5();

    float* getIntrinsicD9();
    float* getDistortionD5();

    bool isCompressedRGB();
    bool isCompressedDepth();

    unsigned getWidthC();
    unsigned getWidth();
    unsigned getHeightC();
    unsigned getHeight();

    scm::math::vec2f const& getTexSizeInvD() const;

    /////////////////////////////////////////////////

  private: // Helper functions
    void advanceToNextToken(const std::string& searchToken, std::ifstream& infile);

    float kommaStringToFloat(const std::string& token);
    float getNextTokenAsFloat(std::ifstream& infile);
    float getNextFloat(std::ifstream& infile);

  public:
    double depth_baseline;  // 7.5e-02;
    double depth_offset;    // 1090;
    double depth_meanfocal; // 580;
                            /*
                            scm::math::mat4f _local_t;
                            scm::math::mat4f _local_r;
                            */

  public:
    static bool s_compress;

  private:
    static int s_compress_rgb;

  private:
    //////////CameraView.h/////////////
    scm::math::mat4f _image_d_to_eye_d;
    scm::math::mat4f _eye_d_to_image_d;
    scm::math::mat4f _eye_d_to_world;
    scm::math::mat4f _eye_d_to_eye_rgb;
    scm::math::mat4f _eye_rgb_to_image_rgb;
    scm::math::mat4f _eye_rgb_to_world;
    ///////////////////////////////////

    scm::math::vec2f _colorFocalLength;
    scm::math::vec2f _colorPrincipalPoint;
    scm::math::vec3f _colorRadialDistortion;
    scm::math::vec2f _colorTangentialDistortion;

    scm::math::vec2f _depthFocalLength;
    scm::math::vec2f _depthPrincipalPoint;
    scm::math::vec3f _depthRadialDistortion;
    scm::math::vec2f _depthTangentialDistortion;

    scm::math::mat4f _relativeRotation;
    scm::math::vec3f _relativeTranslation;

    scm::math::mat4f _worldRotation;
    scm::math::vec3f _worldTranslation; // gloost::Vector3
    scm::math::mat4f _worldRotation2;
    scm::math::vec3f _worldTranslation2;
    scm::math::mat4f _worldRotation3;
    scm::math::vec3f _worldTranslation3;

    float _intrinsic_rgb[9];
    float _distortion_rgb[5];

    float _intrinsic_d[9];
    float _distortion_d[5];

    float _near;
    float _far;
    unsigned _width;
    unsigned _height;
    unsigned _widthc;
    unsigned _heightc;

    unsigned _iscompressedrgb;
    bool _iscompresseddepth;

    std::string _filePath;
    scm::math::vec2f _texSizeInvD;

  public:
    video3d::xyz* cv_xyz;
    video3d::uv* cv_uv;
    unsigned cv_width;
    unsigned cv_height;
    unsigned cv_depth;
    float cv_min_d;
    float cv_max_d;
};

#endif
