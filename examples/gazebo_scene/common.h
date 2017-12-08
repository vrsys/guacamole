#ifndef GUACAMOLE_COMMON_H
#define GUACAMOLE_COMMON_H

#include <list>
#include <memory>
#include <string>
#include <unordered_set>

#include "GLFW/glfw3.h"

#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>

#include <gua/guacamole.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>

#include <gua/renderer/Material.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/utils/Mesh.hpp>

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/node/TriMeshNode.hpp>
#include <gua/renderer/MaterialLoader.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/TextFile.hpp>
#include <gua/utils/ToGua.hpp>
#include <gua/utils/string_utils.hpp>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <functional>

#include <boost/lexical_cast.hpp>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/rendering/Road2d.hh"
#include "gazebo/rendering/Projector.hh"
#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/LaserVisual.hh"
#include "gazebo/rendering/SonarVisual.hh"
#include "gazebo/rendering/WrenchVisual.hh"
#include "gazebo/rendering/CameraVisual.hh"
#include "gazebo/rendering/LogicalCameraVisual.hh"
#include "gazebo/rendering/JointVisual.hh"
#include "gazebo/rendering/COMVisual.hh"
#include "gazebo/rendering/InertiaVisual.hh"
#include "gazebo/rendering/LinkFrameVisual.hh"
#include "gazebo/rendering/ContactVisual.hh"

#include "gazebo/rendering/ContactDebugVisual.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Light.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/WideAngleCamera.hh"
#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/GpuLaser.hh"
#include "gazebo/rendering/Grid.hh"
#include "gazebo/rendering/OriginVisual.hh"
#include "gazebo/rendering/RFIDVisual.hh"
#include "gazebo/rendering/RFIDTagVisual.hh"
#include "gazebo/rendering/VideoVisual.hh"
#include "gazebo/rendering/TransmitterVisual.hh"
#include "gazebo/rendering/SelectionObj.hh"

#include "gazebo/rendering/RTShaderSystem.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/rendering/Scene.hh"

#include <list>
#include <map>
#include <string>
#include <vector>
#include <mutex>

#include <boost/unordered/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <sdf/sdf.hh>

#include "gazebo/common/Events.hh"
#include "gazebo/gazebo_config.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

#include <gua/renderer/SSAAPass.hpp>

#endif // GUACAMOLE_COMMON_H
