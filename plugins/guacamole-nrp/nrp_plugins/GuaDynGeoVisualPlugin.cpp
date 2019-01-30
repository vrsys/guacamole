#include "GuaDynGeoVisualPlugin.hpp"

namespace gazebo
{
GZ_REGISTER_VISUAL_PLUGIN(GuaDynGeoVisualPlugin)

#define GUA_DEBUG 1

GuaDynGeoVisualPlugin::GuaDynGeoVisualPlugin() : _entity_name(""), _buffer_rcv(SGTP::MAX_MESSAGE_SIZE), _buffer_index(10000000), _is_need_swap(false), _is_recv_running(true), _mutex_swap()
{
#if GUA_DEBUG == 1
    gzerr << "DynGeo: constructor" << std::endl;
    std::cerr << "DynGeo: constructor" << std::endl;
#endif
}

GuaDynGeoVisualPlugin::~GuaDynGeoVisualPlugin()
{
    _is_recv_running.store(false);
    _thread_recv.join();
}

void GuaDynGeoVisualPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
#if GUA_DEBUG == 1
    gzerr << "DynGeo: load before" << std::endl;
    std::cerr << "DynGeo: load before" << std::endl;
#endif

    if(!visual || !sdf)
    {
        gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
        return;
    }

    _visual = visual;
    _update_connection = event::Events::ConnectPreRender(std::bind(&GuaDynGeoVisualPlugin::Update, this));

#if GUA_DEBUG == 1
    gzerr << "DynGeo: load after" << std::endl;
    std::cerr << "DynGeo: load after" << std::endl;
#endif

    std::iota(_buffer_index.begin(), _buffer_index.end(), 0);

    for(size_t i = 0; i < _buffer_index.size() - 2; i += 3)
    {
        int32_t swapSpace = _buffer_index[i + 2];
        _buffer_index[i + 2] = _buffer_index[i + 1];
        _buffer_index[i + 1] = swapSpace;
    }

    _thread_recv = std::thread([&]() { _ReadLoop(); });
}
void GuaDynGeoVisualPlugin::_ReadLoop()
{
#if GUA_DEBUG == 1
    gzerr << "DynGeo: _ReadLoop" << std::endl;
    std::cerr << "DynGeo: _ReadLoop" << std::endl;
#endif

    zmq::context_t ctx(1);
    zmq::socket_t socket(ctx, ZMQ_SUB);

    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
#if ZMQ_VERSION_MAJOR < 3
    int64_t hwm = 1;
    socket.setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
#else
    int hwm = 1;
    socket.setsockopt(ZMQ_RCVHWM, &hwm, sizeof(hwm));
#endif

    std::string endpoint("tcp://141.54.147.29:7050");
    socket.connect(endpoint.c_str());

    while(_is_recv_running.load())
    {
        zmq::message_t zmqm;
        socket.recv(&zmqm);

#if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: socket.recv" << std::endl;
        std::cerr << std::endl << "DynGeo: socket.recv" << std::endl;
#endif

        while(true)
        {
            std::lock_guard<std::mutex> lock(_mutex_swap);

            if(!_is_need_swap.load() || !_is_recv_running.load())
            {
                break;
            }
        }

#if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: memcpy" << std::endl;
        std::cerr << std::endl << "DynGeo: memcpy" << std::endl;
#endif

        SGTP::header_data_t header;
        memcpy(&header, (unsigned char *)zmqm.data(), SGTP::HEADER_BYTE_SIZE);

        // TODO: textures

        _num_geometry_bytes = header.geometry_payload_size;
        memcpy(&_bb_min, &header.global_bb_min, sizeof(float) * 3);
        memcpy(&_bb_max, &header.global_bb_max, sizeof(float) * 3);
        memcpy(&_buffer_rcv[0], (unsigned char *)zmqm.data() + SGTP::HEADER_BYTE_SIZE, header.geometry_payload_size);

        {
            std::lock_guard<std::mutex> lock(_mutex_swap);
            _is_need_swap.store(true);
        }
    }
}
void GuaDynGeoVisualPlugin::AddTriangleSoup()
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: AddTriangleSoup" << std::endl;
    std::cerr << std::endl << "DynGeo: AddTriangleSoup" << std::endl;
#endif

    if(!_visual)
    {
        return;
    }

    _scene_node = _visual->GetSceneNode();
    _scene_manager = _scene_node->getCreator();

    if(!_scene_node || !_scene_manager)
    {
        return;
    }

    _scene_node->setVisible(false, false);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: scene manager acquired" << std::endl;
    std::cerr << std::endl << "DynGeo: scene manager acquired" << std::endl;
#endif

    size_t num_vertices = _num_geometry_bytes / (sizeof(float) * 5);
    size_t faces = num_vertices / 3;

    float z = 0;
    for(size_t i = 0; i < num_vertices; i++)
    {
        size_t z_offset = i * 5 * sizeof(float) + 2 * sizeof(float);

        memcpy(&z, &_buffer_rcv[z_offset], sizeof(float));

        z = -z;

        memcpy(&_buffer_rcv[z_offset], &z, sizeof(float));
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;
    std::cerr << std::endl << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;
#endif

    std::string meshname = std::to_string(rand());

    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(meshname, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    // "-" for Z axis and min <-> max flip is NOT A MISTAKE!
    mesh->_setBounds(Ogre::AxisAlignedBox({_bb_min[0], _bb_min[1], -_bb_max[2]}, {_bb_max[0], _bb_max[1], -_bb_min[2]}));
    mesh->_setBoundingSphereRadius(1.73f);

    mesh->sharedVertexData = new Ogre::VertexData();
    mesh->sharedVertexData->vertexCount = num_vertices;

    Ogre::VertexDeclaration *decl = mesh->sharedVertexData->vertexDeclaration;
    Ogre::VertexBufferBinding *bind = mesh->sharedVertexData->vertexBufferBinding;

    size_t offset = 0;
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

    Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(offset, num_vertices, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    vbuf->writeData(0, vbuf->getSizeInBytes(), &_buffer_rcv[0], true);
    bind->setBinding(0, vbuf);

    Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_32BIT, num_vertices, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    ibuf->writeData(0, ibuf->getSizeInBytes(), &_buffer_index[0], true);

#if GUA_DEBUG == 1
    float vx[3];
    float tx[2];

    memcpy(&vx[0], &_buffer_rcv[100 * 5 * sizeof(float)], 3 * sizeof(float));
    memcpy(&tx[0], &_buffer_rcv[100 * 5 * sizeof(float) + 3 * sizeof(float)], 2 * sizeof(float));

    gzerr << std::endl << "DynGeo: vx 500 " << vx[0] << " " << vx[1] << " " << vx[2] << std::endl;
    std::cerr << std::endl << "DynGeo: vx 500 " << vx[0] << " " << vx[1] << " " << vx[2] << std::endl;

    gzerr << std::endl << "DynGeo: tx 500 " << tx[0] << " " << tx[1] << std::endl;
    std::cerr << std::endl << "DynGeo: tx 500 " << tx[0] << " " << tx[1] << std::endl;
#endif

    Ogre::SubMesh *sub = mesh->createSubMesh();
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = num_vertices;
    sub->indexData->indexStart = 0;

    mesh->load();

    while(!mesh->isLoaded())
    {
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: mesh loaded" << std::endl;
    std::cerr << std::endl << "DynGeo: mesh loaded" << std::endl;
#endif

    _entity_name = std::to_string(rand());

    _entity = _scene_manager->createEntity(_entity_name, meshname, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    _entity->setMaterialName("Examples/OgreLogo");
    _avatar_node = _scene_node->createChildSceneNode(std::to_string(rand()));
    _avatar_node->attachObject(_entity);

    _avatar_node->setVisible(true, true);

    while(!_entity->isAttached())
    {
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: triangles added" << std::endl;
    std::cerr << std::endl << "DynGeo: triangles added" << std::endl;
#endif

#if GUA_DEBUG == 1
    float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    // test access to scene
    const Ogre::ColourValue ambient(r, g, b, 1.f);
    _scene_manager->setAmbientLight(ambient);
#endif

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: test colors written" << std::endl;
    std::cerr << std::endl << "DynGeo: test colors written" << std::endl;
#endif
}
void GuaDynGeoVisualPlugin::RemoveTriangleSoup()
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: RemoveTriangleSoup" << std::endl;
    std::cerr << std::endl << "DynGeo: RemoveTriangleSoup" << std::endl;
#endif

    if(!_avatar_node || !_scene_node || !_scene_manager || !_entity)
    {
        return;
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: null check passed" << std::endl;
    std::cerr << std::endl << "DynGeo: null check passed" << std::endl;
#endif

    _entity->detachFromParent();

    while(_entity->isAttached())
    {
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: entity detached" << std::endl;
    std::cerr << std::endl << "DynGeo: entity detached" << std::endl;
#endif

    _avatar_node->removeAllChildren();
    _scene_node->removeAllChildren();

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: destroying entity" << std::endl;
    std::cerr << std::endl << "DynGeo: destroying entity" << std::endl;
#endif

    _scene_manager->destroyEntity(_entity);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: entity destroyed" << std::endl;
    std::cerr << std::endl << "DynGeo: entity destroyed" << std::endl;
#endif
}
void GuaDynGeoVisualPlugin::Update()
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: pre-render update before" << std::endl;
    std::cerr << std::endl << "DynGeo: pre-render update before" << std::endl;
#endif

    {
        std::lock_guard<std::mutex> lock(_mutex_swap);
        if(_is_need_swap.load())
        {
            RemoveTriangleSoup();
            AddTriangleSoup();
            _is_need_swap.store(false);
        }
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: pre-render update after" << std::endl;
    std::cerr << std::endl << "DynGeo: pre-render update after" << std::endl;
#endif
}
}
