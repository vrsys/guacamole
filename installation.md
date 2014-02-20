---
layout: page
title : Installation
header : Installation
group: navigation
---
{% include JB/setup %}


You need a NVIDIA GPU that supports OpenGL 4.2 and CUDA.

---

### Windows

We recommend building guacamole with Visual Studio 2013. We created a zip file
with all necessary dependencies.
Get `externals_vc120RC.zip` from https://github.com/vrsys/guacamole-externals.
Create a new directory `externals/` under the project root.
Unpack `externals_vc120RC.zip` into the `externals/` directory.
Then configure with `cmake` and build the library with Visual Studio.

---

### Linux

You need schism, boost, CUDA, bullet, assimp, libjson, and cmake.

 * [schism](https://github.com/chrislu/schism)
 * [assimp](https://github.com/assimp/assimp)

Configure with cmake and build with make.
