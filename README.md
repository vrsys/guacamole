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

You need schism, boost, CUDA, bullet, assimp, libjson, and cmake.

 * [schism](https://github.com/chrislu/schism)
 * [assimp](https://github.com/assimp/assimp)

Configure with cmake and build with make.

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
