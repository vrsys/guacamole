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

#ifndef GUA_FBXFWD_HPP
#define GUA_FBXFWD_HPP

#define FBX_NAMESPACE fbxsdk

namespace FBX_NAMESPACE
{
class FbxMesh;
class FbxNode;
class FbxManager;
class FbxAnimStack;
class FbxScene;
class FbxTakeInfo;
class FbxSurfaceMaterial;
class FbxAMatrix;
class FbxQuaternion;

template <class T>
class FbxLayerElementTemplate;
} // namespace FBX_NAMESPACE

using FBX_NAMESPACE::FbxAMatrix;
using FBX_NAMESPACE::FbxAnimStack;
using FBX_NAMESPACE::FbxLayerElementTemplate;
using FBX_NAMESPACE::FbxManager;
using FBX_NAMESPACE::FbxMesh;
using FBX_NAMESPACE::FbxNode;
using FBX_NAMESPACE::FbxQuaternion;
using FBX_NAMESPACE::FbxScene;
using FBX_NAMESPACE::FbxSurfaceMaterial;
using FBX_NAMESPACE::FbxTakeInfo;

#endif // GUA_FBXFWD_HPP
