Guacamole
=========
[![Gitter](https://badges.gitter.im/Join Chat.svg)](https://gitter.im/vrsys/guacamole?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

[![Build Status](https://secure.travis-ci.org/vrsys/guacamole.png)](http://travis-ci.org/vrsys/guacamole)

Guacamole is an extendable and efficient rendering system for visualizing 
different data types. It features a lightweight scene graph and a modern
deferred shading system. Many common post processing effects are already
integrated, e.g. screen space ambient occlusion, HDR, FXAA, volumetric light
effects, bloom and fog.

Please cite our work when using our software in your own research or publication.

https://www.uni-weimar.de/de/medien/professuren/vr/software/

Installation
------------

You need a NVIDIA GPU that supports OpenGL 4.2 and CUDA.

### Windows

We recommend building guacamole with Visual Studio 2013. We created a zip file
with all necessary dependencies.
Get `externals_vc120RC.zip` from https://github.com/vrsys/guacamole-externals.
Create a new directory `externals/` under the project root.
Unpack `externals_vc120RC.zip` into the `externals/` directory.
Then configure with `cmake` and build the library with Visual Studio.

### Linux
The following guide serves as a step-by-step tutorial on how to build the dependencies and guacamole itself on Linux. It was tested on a fresh installation of Ubuntu 16.04, and the latest commit at the point of writing was `1097eff`.

_Bullet_
  * clone https://github.com/bulletphysics/bullet3
  * configure with CMake and enable flag `INSTALL_EXTRA_LIBS`
  * `make install`
  * the Bullet include directory will be `/usr/local/include`
 
_Boost_
  * `apt-get install libboost-all-dev`
  * the Boost include directory will be `/usr/include/boost`
  * the Boost library directory will be `/usr/lib/x86_64-linux-gnu`
 
 _JSONCpp_
  * `apt-get install libjsoncpp-dev`
  * the JSONCpp library will be `/usr/lib/x86_64-linux-gnu/libjsoncpp.so`
  
_GLFW_
  * `apt-get install libglfw3-dev`
  * the GLFW include directory will be `/usr/include`
  * the GLFW library will be `/usr/lib/x86_64-linux-gnu/libglfw.so`

_ZMQ_
  * `apt-get install libzmq3-dev`
  * the ZMQ include directory will be `/usr/include`
  * the ZMQ library will be `/usr/lib/x86_64-linux-gnu/libzmq.so`
 
_FBX_
  * get FBX SDK from [Autodesk](http://usa.autodesk.com/adsk/servlet/pc/item?siteID=123112&id=10775847)
  * unpack and execute install script in desired installation directory
  * when FBX is located in `FBX_PATH`, the FBX include directory will be `FBX_PATH/include`
  * when FBX is located in `FBX_PATH`, the FBX library directories will be `FBX_PATH/lib/gcc4/release` and `FBX_PATH/lib/gcc4/debug`
  
_Schism_
  * `apt-get install libfreetype6-dev libfreeimageplus-dev opencl-headers`
  * clone https://github.com/chrislu/schism
  * adapt find script `build/cmake/custom_scripts/schism_boost.cmake` such that `SCM_BOOST_INCLUDE_SEARCH_DIRS` is `/usr/include` and `SCM_BOOST_LIBRARY_SEARCH_DIRS` is `/usr/lib/x86_64-linux-gnu`
  * configure with CMake
  * `make install`
  * when Schism is located in `SCHISM_PATH`, the Schism inlcude directories will be `SCHISM_PATH/scm_cl_core/src;SCHISM_PATH/scm_core/src;SCHISM_PATH/scm_gl_core/src;SCHISM_PATH/scm_gl_util/src;SCHISM_PATH/scm_input/src`
  * when Schism is located in `SCHISM_PATH`, the Schism library directory will be `SCHISM_PATH/lib/linux_x86`
  
_Lamure_
  * `apt-get install freeglut3-dev libcgal-dev libglm-dev libxmu-dev`
  * clone https://github.com/vrsys/lamure
  * adapt find script `cmake/modules/find_schism.cmake` such that `SCHISM_INCLUDE_SEARCH_DIRS` and `SCHISM_LIBRARY_SEARCH_DIRS` contain the paths mentioned above
  * configure with CMake
  * `make install`
  * when Lamure is located in `LAMURE_PATH`, the Lamure include directory will be `LAMURE_PATH/install/include`
  * when Lamure is located in `LAMURE_PATH`, the Lamure library directory will be `LAMURE_PATH/install/lib`
  
_guacamole_
  * `apt-get install libassimp-dev`
  * clone https://github.com/vrsys/guacamole (this repository)
  * configure with CMake and set paths to dependencies as mentioned above if not found automatically
  * CMake flags that were set to ON during writing this tutorial: `GUACAMOLE_ENABLE_PHYSICS`, `GUACAMOLE_EXAMPLES`, `GUACAMOLE_FBX`, `GUACAMOLE_GLFW3`, `GUACAMOLE_RUNTIME_PROGRAM_COMP`, `PLUGIN_guacamole-lod`, `PLUGIN_guacamole-skelanim`, `PLUGIN_guacamole-tv_3`, `PLUGIN_guacamole-video3d`, `PLUGIN_guacamole-volume`
  * `make install`

If everything worked well, you can start by running an example in the `examples/` directory, e.g. the `input` example.


Get involved!
-------------

Please report bugs via the
[github issue tracker](https://github.com/vrsys/guacamole/issues).

Authors
-------

* Simon Schneegans
* Felix Lauer
* Andreas Bernstein
* André Schollmeyer
* Andrey Babanin
