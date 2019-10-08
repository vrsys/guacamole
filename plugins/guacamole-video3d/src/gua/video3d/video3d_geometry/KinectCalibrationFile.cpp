#include <iostream>
#include <fstream>
#include <iomanip>
#include <gua/video3d/video3d_geometry/KinectCalibrationFile.hpp>

#include <boost/filesystem.hpp>

/*static*/ bool KinectCalibrationFile::s_compress = false;
/*static*/ int KinectCalibrationFile::s_compress_rgb = -1;

KinectCalibrationFile::KinectCalibrationFile(const std::string& filePath)
    : _image_d_to_eye_d(), _eye_d_to_image_d(), _eye_d_to_world(), _eye_d_to_eye_rgb(), _eye_rgb_to_image_rgb(), _eye_rgb_to_world(),

      _colorFocalLength(), _colorPrincipalPoint(), _colorRadialDistortion(), _colorTangentialDistortion(),

      _depthFocalLength(), _depthPrincipalPoint(), _depthRadialDistortion(), _depthTangentialDistortion(),

      _relativeRotation(), _relativeTranslation(),

      _worldRotation(), _worldTranslation(), // gloost::Vector3
      _worldRotation2(), _worldTranslation2(), _worldRotation3(), _worldTranslation3(),

      _intrinsic_rgb(), _distortion_rgb(), _intrinsic_d(), _distortion_d(),

      _near(0.3), _far(7.0), _width(0), _height(0), _widthc(0), _heightc(0),

      _iscompressedrgb(0), _iscompresseddepth(false),
      /*
          _local_t(),
          _local_r(),
      */
      _filePath(filePath), _texSizeInvD(), cv_xyz(0), cv_uv(0), cv_width(0), cv_height(0), cv_depth(0), cv_min_d(0), cv_max_d(0)
{
    if(s_compress_rgb == -1)
    { // i am the first to look if file exists rgbd_calib/compress.rgb
        std::string filename("rgbd_calib/compress.rgb");
        std::ifstream infile;
        infile.open(filename.c_str());
        if(infile)
        {
            int test_val;
            infile >> test_val;
            infile.close();
            if(test_val == 0 || test_val == 1 || test_val == 5)
            {
                s_compress_rgb = test_val;
                std::cerr << "KinectCalibrationFile setting compress_rgb to " << s_compress_rgb << std::endl;
            }
        }
        else
        {
            s_compress_rgb = -2;
        }
    }
}

KinectCalibrationFile::~KinectCalibrationFile() {}

///////////////// PUBLIC /////////////////////////////

bool KinectCalibrationFile::parse()
{
    _iscompresseddepth = false;

    std::ifstream infile;
    infile.open(_filePath.c_str());

    std::string token;

    //////////////// .yml ///////////////////////////////
    while(infile >> token)
    {
        //    std::cerr << std::endl << token;

        //////////// color ///////////
        if(token == "rgb_intrinsics:")
        {
            //      std::cerr << std::endl << "rgb_intrinsics:";
            advanceToNextToken("[", infile);

            _colorFocalLength[0] = getNextTokenAsFloat(infile);

            infile >> token;

            _colorPrincipalPoint[0] = getNextTokenAsFloat(infile);

            infile >> token;

            _colorFocalLength[1] = getNextTokenAsFloat(infile);
            _colorPrincipalPoint[1] = getNextTokenAsFloat(infile);

            _intrinsic_rgb[0] = _colorFocalLength[0];
            _intrinsic_rgb[1] = 0.0f;
            _intrinsic_rgb[2] = _colorPrincipalPoint[0];
            _intrinsic_rgb[3] = 0.0f;
            _intrinsic_rgb[4] = _colorFocalLength[1];
            _intrinsic_rgb[5] = _colorPrincipalPoint[1];
            _intrinsic_rgb[6] = 0.0f;
            _intrinsic_rgb[7] = 0.0f;
            _intrinsic_rgb[8] = 1.0f;
        }

        else if(token == "rgb_distortion:")
        {
            //      std::cerr << std::endl << "rgb_distortion:";
            advanceToNextToken("[", infile);

            _colorRadialDistortion[0] = getNextTokenAsFloat(infile);
            _colorRadialDistortion[1] = getNextTokenAsFloat(infile);

            _colorTangentialDistortion[0] = getNextTokenAsFloat(infile);
            _colorTangentialDistortion[1] = getNextTokenAsFloat(infile);

            _colorRadialDistortion[3] = getNextFloat(infile);

            _distortion_rgb[0] = _colorRadialDistortion[0];
            _distortion_rgb[1] = _colorRadialDistortion[1];
            _distortion_rgb[2] = _colorTangentialDistortion[0];
            _distortion_rgb[3] = _colorTangentialDistortion[1];
            _distortion_rgb[4] = _colorRadialDistortion[2];
        }

        ////////////// depth ///////////////
        else if(token == "depth_intrinsics:")
        {
            //      std::cerr << std::endl << "depth_intrinsics:";
            advanceToNextToken("[", infile);

            _depthFocalLength[0] = getNextTokenAsFloat(infile);

            infile >> token;

            _depthPrincipalPoint[0] = getNextTokenAsFloat(infile);

            infile >> token;

            _depthFocalLength[1] = getNextTokenAsFloat(infile);
            _depthPrincipalPoint[1] = getNextTokenAsFloat(infile);

            _intrinsic_d[0] = _depthFocalLength[0];
            _intrinsic_d[1] = 0.0f;
            _intrinsic_d[2] = _depthPrincipalPoint[0];
            _intrinsic_d[3] = 0.0f;
            _intrinsic_d[4] = _depthFocalLength[1];
            _intrinsic_d[5] = _depthPrincipalPoint[1];
            _intrinsic_d[6] = 0.0f;
            _intrinsic_d[7] = 0.0f;
            _intrinsic_d[8] = 1.0f;
        }

        else if(token == "depth_distortion:")
        {
            //      std::cerr << std::endl << "depth_distortion:";
            advanceToNextToken("[", infile);

            _depthRadialDistortion[0] = getNextTokenAsFloat(infile);
            _depthRadialDistortion[1] = getNextTokenAsFloat(infile);

            _depthTangentialDistortion[0] = getNextTokenAsFloat(infile);
            _depthTangentialDistortion[1] = getNextTokenAsFloat(infile);

            _depthRadialDistortion[2] = getNextFloat(infile);

            _distortion_d[0] = _depthRadialDistortion[0];
            _distortion_d[1] = _depthRadialDistortion[1];
            _distortion_d[2] = _depthTangentialDistortion[0];
            _distortion_d[3] = _depthTangentialDistortion[1];
            _distortion_d[4] = _depthRadialDistortion[2];
        }

        // relative transform
        else if(token == "R:")
        {
            //      std::cerr << std::endl << "R:";
            advanceToNextToken("[", infile);

            scm::math::set_identity(_relativeRotation);

            _relativeRotation[0] = getNextTokenAsFloat(infile);
            _relativeRotation[1] = getNextTokenAsFloat(infile);
            _relativeRotation[2] = getNextTokenAsFloat(infile);

            _relativeRotation[4] = getNextTokenAsFloat(infile);
            _relativeRotation[5] = getNextTokenAsFloat(infile);
            _relativeRotation[6] = getNextTokenAsFloat(infile);

            _relativeRotation[8] = getNextTokenAsFloat(infile);
            _relativeRotation[9] = getNextTokenAsFloat(infile);
            _relativeRotation[10] = getNextFloat(infile);

            _relativeRotation[12] = 0.0;
            _relativeRotation[13] = 0.0;
            _relativeRotation[14] = 0.0;
        }

        else if(token == "T:")
        {
            //      std::cerr << std::endl << "T:";
            advanceToNextToken("[", infile);

            _relativeTranslation[0] = getNextTokenAsFloat(infile);
            _relativeTranslation[1] = getNextTokenAsFloat(infile);
            _relativeTranslation[2] = getNextFloat(infile);
        }

        else if(token == "rgb_size:")
        {
            //      std::cerr << std::endl << "rgb_size:";
            advanceToNextToken("[", infile);

            _widthc = getNextTokenAsFloat(infile);
            _heightc = getNextFloat(infile);
        }
        else if(token == "depth_size:")
        {
            //      std::cerr << std::endl << "depth_size:";
            advanceToNextToken("[", infile);

            _width = getNextTokenAsFloat(infile);
            _height = getNextFloat(infile);

            _texSizeInvD[0] = 1.0f / _width;
            _texSizeInvD[1] = 1.0f / _height;
        }

        /*else if (token == "baumer_serial:")
        {
        //      std::cerr << std::endl << "baumer_serial:";
        advanceToNextToken("[", infile);
        infile >> _baumer_serial;

        }*/

        else if(token == "depth_base_and_offset:")
        {
            advanceToNextToken("[", infile);
            depth_baseline = getNextTokenAsFloat(infile);
            depth_offset = getNextFloat(infile);
        }

        else if(token == "depth_base_and_offset_meanfocal:")
        {
            advanceToNextToken("[", infile);
            depth_baseline = getNextTokenAsFloat(infile);
            depth_offset = getNextTokenAsFloat(infile);
            depth_meanfocal = getNextFloat(infile);
        }

        else if(token == "near_far:")
        {
            // std::cerr << std::endl << "near_far:";
            advanceToNextToken("[", infile);
            _near = getNextTokenAsFloat(infile);
            _far = getNextFloat(infile);
        }

        else if(token == "compress_rgb:")
        {
            // std::cerr << std::endl << "compress_rgb:";
            advanceToNextToken("[", infile);
            _iscompressedrgb = (unsigned)getNextTokenAsFloat(infile);
            float dummy = getNextFloat(infile);

            // maybe override value
            if(s_compress_rgb == 0 || s_compress_rgb == 1 || s_compress_rgb == 5)
            {
                _iscompressedrgb = (unsigned)s_compress_rgb;
            }
        }

        else if(token == "compress_depth:")
        {
            // std::cerr << std::endl << "compress_depth:";
            advanceToNextToken("[", infile);
            _iscompresseddepth = (bool)((unsigned)getNextTokenAsFloat(infile));
            float dummy = getNextFloat(infile);
        }
    }

    infile.close();

    ///////////////// .ext //////////////////////////
    {
        std::string e_filepath(_filePath.c_str());
        e_filepath.replace(e_filepath.end() - 3, e_filepath.end(), "ext");
        infile.open(e_filepath.c_str());
        if(infile)
        {
            float token;
            infile >> token;
            _worldTranslation[0] = token;
            infile >> token;
            _worldTranslation[1] = token;
            infile >> token;
            _worldTranslation[2] = token;

            scm::math::set_identity(_worldRotation);

            infile >> token;
            _worldRotation[0] = token;
            infile >> token;
            _worldRotation[1] = token;
            infile >> token;
            _worldRotation[2] = token;

            infile >> token;
            _worldRotation[4] = token;
            infile >> token;
            _worldRotation[5] = token;
            infile >> token;
            _worldRotation[6] = token;

            infile >> token;
            _worldRotation[8] = token;
            infile >> token;
            _worldRotation[9] = token;
            infile >> token;
            _worldRotation[10] = token;

            _worldRotation[12] = 0.0;
            _worldRotation[13] = 0.0;
            _worldRotation[14] = 0.0;

            infile.close();
        }
        else
        {
            scm::math::set_identity(_worldRotation);
            _worldTranslation[0] = 0.0;
            _worldTranslation[1] = 0.0;
            _worldTranslation[2] = 0.0;
        }
    }

    ///////////////// .ext2 /////////////////////
    {
        std::string e_filepath(_filePath.c_str());
        e_filepath.replace(e_filepath.end() - 3, e_filepath.end(), "ext2");
        infile.open(e_filepath.c_str());
        if(infile)
        {
            float token;
            infile >> token;
            _worldTranslation2[0] = token;
            infile >> token;
            _worldTranslation2[1] = token;
            infile >> token;
            _worldTranslation2[2] = token;

            scm::math::set_identity(_worldRotation2);

            infile >> token;
            _worldRotation2[0] = token;
            infile >> token;
            _worldRotation2[1] = token;
            infile >> token;
            _worldRotation2[2] = token;

            infile >> token;
            _worldRotation2[4] = token;
            infile >> token;
            _worldRotation2[5] = token;
            infile >> token;
            _worldRotation2[6] = token;

            infile >> token;
            _worldRotation2[8] = token;
            infile >> token;
            _worldRotation2[9] = token;
            infile >> token;
            _worldRotation2[10] = token;

            _worldRotation2[12] = 0.0;
            _worldRotation2[13] = 0.0;
            _worldRotation2[14] = 0.0;

            infile.close();
        }
        else
        {
            scm::math::set_identity(_worldRotation2);
            _worldTranslation2[0] = 0.0;
            _worldTranslation2[1] = 0.0;
            _worldTranslation2[2] = 0.0;
        }
    }

    { // .ext3
        std::string e_filepath(_filePath.c_str());
        e_filepath.replace(e_filepath.end() - 3, e_filepath.end(), "ext3");
        infile.open(e_filepath.c_str());
        if(infile)
        {
            float token;
            infile >> token;
            _worldTranslation3[0] = token;
            infile >> token;
            _worldTranslation3[1] = token;
            infile >> token;
            _worldTranslation3[2] = token;

            scm::math::set_identity(_worldRotation3);

            infile >> token;
            _worldRotation3[0] = token;
            infile >> token;
            _worldRotation3[1] = token;
            infile >> token;
            _worldRotation3[2] = token;

            infile >> token;
            _worldRotation3[4] = token;
            infile >> token;
            _worldRotation3[5] = token;
            infile >> token;
            _worldRotation3[6] = token;

            infile >> token;
            _worldRotation3[8] = token;
            infile >> token;
            _worldRotation3[9] = token;
            infile >> token;
            _worldRotation3[10] = token;

            _worldRotation3[12] = 0.0;
            _worldRotation3[13] = 0.0;
            _worldRotation3[14] = 0.0;

            infile.close();
        }
        else
        {
            scm::math::set_identity(_worldRotation3);
            _worldTranslation3[0] = 0.0;
            _worldTranslation3[1] = 0.0;
            _worldTranslation3[2] = 0.0;
        }
    }

    { // load cv_xyz
        if(cv_xyz)
        {
            delete[] cv_xyz;
        }
        std::string fpath(_filePath.c_str());
        fpath.replace(fpath.end() - 3, fpath.end(), "cv_xyz");
        // std::cerr << "loading " << fpath << std::endl;

        std::fstream fstr(fpath.c_str(), std::ios::in | std::ios::binary);
        if(fstr.good())
        {
            fstr.read((char*)&cv_width, sizeof(unsigned));
            fstr.read((char*)&cv_height, sizeof(unsigned));
            fstr.read((char*)&cv_depth, sizeof(unsigned));
            fstr.read((char*)&cv_min_d, sizeof(float));
            fstr.read((char*)&cv_max_d, sizeof(float));
            cv_xyz = new video3d::xyz[cv_width * cv_height * cv_depth];
            fstr.read((char*)cv_xyz, sizeof(video3d::xyz) * cv_width * cv_height * cv_depth);
            fstr.close();
        }
        else
        {
            std::cerr << "KinectCalibrationFile::parse(): Could not open " << fpath << std::endl;
        }
    }

    { // load cv_uv;
        if(cv_uv)
        {
            delete[] cv_uv;
        }
        std::string fpath(_filePath.c_str());
        fpath.replace(fpath.end() - 3, fpath.end(), "cv_uv");
        // std::cerr << "loading " << fpath << std::endl;

        std::fstream fstr(fpath.c_str(), std::ios::in | std::ios::binary);
        if(fstr.good())
        {
            fstr.read((char*)&cv_width, sizeof(unsigned));
            fstr.read((char*)&cv_height, sizeof(unsigned));
            fstr.read((char*)&cv_depth, sizeof(unsigned));
            fstr.read((char*)&cv_min_d, sizeof(float));
            fstr.read((char*)&cv_max_d, sizeof(float));
            cv_uv = new video3d::uv[cv_width * cv_height * cv_depth];
            fstr.read((char*)cv_uv, sizeof(video3d::uv) * cv_width * cv_height * cv_depth);
            fstr.close();
        }
        else
        {
            std::cerr << "KinectCalibrationFile::parse(): Could not open " << fpath << std::endl;
        }
    }

    return true;
}

//////////////////////////////////////////////////////////////////////////////////////

void KinectCalibrationFile::printInfo() const
{
    std::cerr << std::endl << "### File: #############################################";
    std::cerr << std::endl << "  path:          " << _filePath;
    std::cerr << std::endl << "### Color camera: #####################################";
    std::cerr << std::endl << "  colorFocalLength:          " << _colorFocalLength;
    std::cerr << std::endl << "  colorPrincipalPoint:       " << _colorPrincipalPoint;
    std::cerr << std::endl << "  colorRadialDistortion:     " << _colorRadialDistortion;
    std::cerr << std::endl << "  colorTangentialDistortion: " << _colorTangentialDistortion;

    std::cerr << std::endl << "  intrinsic: ";
    for(unsigned i = 0; i < 9; ++i)
        std::cerr << _intrinsic_rgb[i] << " ,";
    std::cerr << std::endl;

    std::cerr << std::endl << "  distortion: ";
    for(unsigned i = 0; i < 5; ++i)
        std::cerr << _distortion_rgb[i] << " ,";
    std::cerr << std::endl;

    std::cerr << std::endl;

    std::cerr << std::endl << "### Depth camera: #####################################";
    std::cerr << std::endl << "  depthFocalLength:          " << _depthFocalLength;
    std::cerr << std::endl << "  depthPrincipalPoint:       " << _depthPrincipalPoint;
    std::cerr << std::endl << "  depthRadialDistortion:     " << _depthRadialDistortion;
    std::cerr << std::endl << "  depthTangentialDistortion: " << _depthTangentialDistortion;
    std::cerr << std::endl;

    std::cerr << std::endl << "  intrinsic: ";
    for(unsigned i = 0; i < 9; ++i)
        std::cerr << _intrinsic_d[i] << " ,";
    std::cerr << std::endl;

    std::cerr << std::endl << "  distortion: ";
    for(unsigned i = 0; i < 5; ++i)
        std::cerr << _distortion_d[i] << " ,";
    std::cerr << std::endl;

    std::cerr << std::endl << "### Relative transformation: #########################";
    std::cerr << std::endl << "  R: ";
    std::cerr << std::endl << _relativeRotation;
    std::cerr << std::endl;
    std::cerr << std::endl << "  T: ";
    std::cerr << std::endl << _relativeTranslation;
    std::cerr << std::endl;
    std::cerr << std::endl;

    std::cerr << std::endl << "### World transformation: #########################";
    std::cerr << std::endl << "  R: ";
    std::cerr << std::endl << _worldRotation;
    std::cerr << std::endl;
    std::cerr << std::endl << "  T: ";
    std::cerr << std::endl << _worldTranslation;
    std::cerr << std::endl;
    std::cerr << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////

void KinectCalibrationFile::updateMatrices()
{
    scm::math::mat4f relativeTransformMat(this->getRelativeRotation());

    scm::math::vec3f relativeTranslate(this->getRelativeTranslation());

    float fx_rgb = this->getColorFocalLength()[0];
    float fy_rgb = this->getColorFocalLength()[1];
    float cx_rgb = this->getColorPrincipalPoint()[0];
    float cy_rgb = this->getColorPrincipalPoint()[1];

    float fx_d = this->getDepthFocalLength()[0];
    float fy_d = this->getDepthFocalLength()[1];
    float cx_d = this->getDepthPrincipalPoint()[0];
    float cy_d = this->getDepthPrincipalPoint()[1];

    relativeTransformMat = scm::math::transpose(relativeTransformMat);
    relativeTransformMat = scm::math::inverse(relativeTransformMat);
    relativeTranslate = -1.0 * relativeTranslate;

    scm::math::mat4f image_d_scale_pre;
    scm::math::set_identity(image_d_scale_pre);
    scm::math::scale(image_d_scale_pre, static_cast<float>(_width), static_cast<float>(_height), 1.0f);

    scm::math::mat4f image_d_trans;
    scm::math::set_identity(image_d_trans);
    scm::math::translate(image_d_trans, -cx_d, -cy_d, 0.0f);

    scm::math::mat4f image_d_scale_post;
    scm::math::set_identity(image_d_scale_post);
    scm::math::scale(image_d_scale_post, 1.0f / fx_d, 1.0f / fy_d, 1.0f);

    _image_d_to_eye_d = image_d_scale_post * image_d_trans * image_d_scale_pre;

    scm::math::mat4f trans;
    scm::math::set_identity(trans);
    scm::math::translate(trans, _worldTranslation[0], _worldTranslation[1], _worldTranslation[2]);
    // scm::math::mat4f rot(_worldRotation);
    _eye_d_to_world = _worldRotation /*rot*/ * trans;

    scm::math::mat4f trans2;
    scm::math::set_identity(trans2);
    scm::math::translate(trans2, _worldTranslation2[0], _worldTranslation2[1], _worldTranslation2[2]);
    scm::math::mat4f rot2(_worldRotation2);

    scm::math::mat4f trans3;
    scm::math::set_identity(trans3);
    scm::math::translate(trans3, _worldTranslation3[0], _worldTranslation3[1], _worldTranslation3[2]);
    scm::math::mat4f rot3(_worldRotation3);

    _eye_d_to_world = (trans2 * rot2) * _eye_d_to_world;
    _eye_d_to_world = (rot3 * trans3) * _eye_d_to_world;

    scm::math::mat4f w;

    //_eye_d_to_world = _local_r * _local_t  * _eye_d_to_world;

    _eye_d_to_eye_rgb = relativeTransformMat;
    scm::math::translate(_eye_d_to_eye_rgb, relativeTranslate[0], relativeTranslate[1], relativeTranslate[2]);

    /*
        vec4 POS_rgb = eye_d_to_eye_rgb * POS_d;

        vec2 pos_rgb;
        pos_rgb.x = (POS_rgb.x/POS_rgb.z) * fx_rgb + cx_rgb;
        pos_rgb.y = (POS_rgb.y/POS_rgb.z) * fy_rgb + cy_rgb;


        tex_coord = pos_rgb * texSizeInv;
    */

    scm::math::mat4f image_rgb_scale_pre;
    scm::math::set_identity(image_rgb_scale_pre);
    scm::math::scale(image_rgb_scale_pre, fx_rgb, fy_rgb, 1.0f);

    scm::math::mat4f image_rgb_trans;
    scm::math::set_identity(image_rgb_trans);
    scm::math::translate(image_rgb_trans, cx_rgb, cy_rgb, 0.0f);

    scm::math::mat4f image_rgb_scale_post;
    scm::math::set_identity(image_rgb_scale_post);
    scm::math::scale(image_rgb_scale_post, 1.0f / _widthc, 1.0f / _heightc, 1.0f);

    _eye_rgb_to_image_rgb = image_rgb_scale_post * image_rgb_trans * image_rgb_scale_pre;

    _eye_d_to_image_d = _image_d_to_eye_d;
    _eye_d_to_image_d = scm::math::inverse(_eye_d_to_image_d);

    scm::math::mat4f eye_rgb_to_eye_d(_eye_d_to_eye_rgb);
    eye_rgb_to_eye_d = scm::math::inverse(eye_rgb_to_eye_d);
    _eye_rgb_to_world = _eye_d_to_world * eye_rgb_to_eye_d;
}

//////////////////////////////////////////////////////////////////////////////////////

std::string KinectCalibrationFile::get_stream_filename() const
{
    std::string e_filepath(_filePath.c_str());
    e_filepath.replace(e_filepath.end() - 3, e_filepath.end(), "stream");
    return e_filepath;
}

//////////////////////GET/SET//////////////////////////
scm::math::mat4f const& KinectCalibrationFile::getImageDToEyeD() const { return _image_d_to_eye_d; }

scm::math::mat4f const& KinectCalibrationFile::getEyeDToImageD() const { return _eye_d_to_image_d; }

scm::math::mat4f const& KinectCalibrationFile::getEyeDToWorld() const { return _eye_d_to_world; }

scm::math::mat4f const& KinectCalibrationFile::getEyeDToEyeRGB() const { return _eye_d_to_eye_rgb; }

scm::math::mat4f const& KinectCalibrationFile::getEyeRGBToImageRGB() const { return _eye_rgb_to_image_rgb; }

scm::math::mat4f const& KinectCalibrationFile::getEyeRGBToWorld() const { return _eye_rgb_to_world; }

scm::math::vec2f const& KinectCalibrationFile::getColorFocalLength() const { return _colorFocalLength; }

scm::math::vec2f const& KinectCalibrationFile::getColorPrincipalPoint() const { return _colorPrincipalPoint; }

scm::math::vec3f const& KinectCalibrationFile::getColorRadialDistortion() const { return _colorRadialDistortion; }

scm::math::vec2f const& KinectCalibrationFile::getColorTangentialDistortion() const { return _colorTangentialDistortion; }

scm::math::vec2f const& KinectCalibrationFile::getDepthFocalLength() const { return _depthFocalLength; }

scm::math::vec2f const& KinectCalibrationFile::getDepthPrincipalPoint() const { return _depthPrincipalPoint; }

scm::math::vec3f const& KinectCalibrationFile::getDepthRadialDistortion() const { return _depthRadialDistortion; }

scm::math::vec2f const& KinectCalibrationFile::getDepthTangentialDistortion() const { return _depthTangentialDistortion; }

scm::math::mat4f const& KinectCalibrationFile::getRelativeRotation() const { return _relativeRotation; }

scm::math::vec3f const& KinectCalibrationFile::getRelativeTranslation() const { return _relativeTranslation; }

scm::math::mat4f const& KinectCalibrationFile::getWorldRotation() const { return _worldRotation; }

void KinectCalibrationFile::setWorldRotation(scm::math::mat4f& r) { _worldRotation = r; }

scm::math::vec3f const& KinectCalibrationFile::getWorldTranslation() const { return _worldTranslation; }

void KinectCalibrationFile::setWorldTranslation(scm::math::vec3f& t) { _worldTranslation = t; }

float* KinectCalibrationFile::getIntrinsicRGB9() { return _intrinsic_rgb; }

float* KinectCalibrationFile::getDistortionRGB5() { return _distortion_rgb; }

float* KinectCalibrationFile::getIntrinsicD9() { return _intrinsic_d; }

float* KinectCalibrationFile::getDistortionD5() { return _distortion_d; }

bool KinectCalibrationFile::isCompressedRGB() { return _iscompressedrgb; }

bool KinectCalibrationFile::isCompressedDepth() { return _iscompresseddepth; }

unsigned KinectCalibrationFile::getWidthC() { return _widthc; }

unsigned KinectCalibrationFile::getWidth() { return _width; }

unsigned KinectCalibrationFile::getHeightC() { return _heightc; }

unsigned KinectCalibrationFile::getHeight() { return _height; }

scm::math::vec2f const& KinectCalibrationFile::getTexSizeInvD() const { return _texSizeInvD; }

/////////////////////PRIVATE///////////////////////////

void KinectCalibrationFile::advanceToNextToken(const std::string& searchToken, std::ifstream& infile)
{
    std::string token;
    while(infile >> token)
    {
        if(token == searchToken)
        {
            return;
        }
    }
}

float KinectCalibrationFile::kommaStringToFloat(const std::string& token) { return atof(std::string(token.substr(0, token.length() - 1)).c_str()); }

float KinectCalibrationFile::getNextTokenAsFloat(std::ifstream& infile)
{
    std::string token;
    infile >> token;
    // std::cerr << token << std::endl;
    return kommaStringToFloat(token);
}

float KinectCalibrationFile::getNextFloat(std::ifstream& infile)
{
    std::string token;
    infile >> token;
    // std::cerr << token << std::endl;
    return atof(token.c_str());
}
