#include "GuaDynGeoVisualPlugin.hpp"

namespace gazebo
{
GZ_REGISTER_VISUAL_PLUGIN(GuaDynGeoVisualPlugin)

#define GUA_DEBUG 1
#define TEX_DEBUG 1

GuaDynGeoVisualPlugin::GuaDynGeoVisualPlugin()
    : _entity_name(""), _mesh_name(""), _material_name(""), _texture_name(""), _buffer_rcv(SGTP::MAX_MESSAGE_SIZE), _buffer_rcv_texture(SGTP::MAX_MESSAGE_SIZE), _buffer_index(10000000),
      _is_need_swap(false), _is_recv_running(true), _mutex_swap()
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: constructor" << std::endl;
    std::cerr << std::endl << "DynGeo: constructor" << std::endl;
#endif
}

GuaDynGeoVisualPlugin::~GuaDynGeoVisualPlugin()
{
    _is_recv_running.store(false);
    _thread_recv.join();

    Ogre::MaterialManager::getSingleton().remove(_material_name);
    Ogre::TextureManager::getSingleton().remove(_texture_name);
}

void GuaDynGeoVisualPlugin::Load(rendering::VisualPtr visual, sdf::ElementPtr sdf)
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: load before" << std::endl;
    std::cerr << std::endl << "DynGeo: load before" << std::endl;
#endif

    if(!visual || !sdf)
    {
        gzerr << std::endl << "No visual or SDF element specified. Plugin won't load." << std::endl;
        return;
    }

    _visual = visual;

    _scene_node = _visual->GetSceneNode();
    _scene_manager = _scene_node->getCreator();

    _update_connection = event::Events::ConnectPreRender(std::bind(&GuaDynGeoVisualPlugin::Update, this));

    std::iota(_buffer_index.begin(), _buffer_index.end(), 0);

    for(size_t i = 0; i < _buffer_index.size() - 2; i += 3)
    {
        int32_t swapSpace = _buffer_index[i + 2];
        _buffer_index[i + 2] = _buffer_index[i + 1];
        _buffer_index[i + 1] = swapSpace;
    }

    _texture_name = std::to_string(rand());

    _texture_width = 4096;
    // unsigned int texture_height = (unsigned int)pow(2, ceil(log(SGTP::TEXTURE_DIMENSION_Y) / log(2)));

    Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().createManual(_texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, _texture_width,
                                                                                 _texture_width, 0, Ogre::PF_BYTE_BGR, Ogre::TU_DYNAMIC_WRITE_ONLY);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: texture created" << std::endl;
    std::cerr << std::endl << "DynGeo: texture created" << std::endl;
#endif

#if TEX_DEBUG == 1

    Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture->getBuffer();

    pixel_buffer->lock(Ogre::Image::Box(0, 0, 1280, 720), Ogre::HardwareBuffer::HBL_WRITE_ONLY);
    const Ogre::PixelBox &pixel_box_1 = pixel_buffer->getCurrentLock();
    // memset(pixel_box_1.getTopLeftFrontPixelPtr(), 0x20, Ogre::PixelUtil::getMemorySize(1280, 720, 1, pixel_buffer->getFormat()));
    uint32_t *data = static_cast<uint32_t *>(pixel_box_1.data);
    for(int y = 0; y < pixel_box_1.getHeight(); ++y)
    {
        for(int x = 0; x < pixel_box_1.getWidth(); ++x)
        {
            data[pixel_box_1.rowPitch * y + x] = 0x20202020;
        }
    }
    pixel_buffer->unlock();

    pixel_buffer->lock(Ogre::Image::Box(0, 720, 1280, 1440), Ogre::HardwareBuffer::HBL_WRITE_ONLY);
    const Ogre::PixelBox &pixel_box_2 = pixel_buffer->getCurrentLock();
    // memset(pixel_box_2.getTopLeftFrontPixelPtr(), 0x80, Ogre::PixelUtil::getMemorySize(1280, 720, 1, pixel_buffer->getFormat()));
    data = static_cast<uint32_t *>(pixel_box_2.data);
    for(int y = 0; y < pixel_box_2.getHeight(); ++y)
    {
        for(int x = 0; x < pixel_box_2.getWidth(); ++x)
        {
            data[pixel_box_2.rowPitch * y + x] = 0x80808080;
        }
    }
    pixel_buffer->unlock();

    pixel_buffer->lock(Ogre::Image::Box(1280, 0, 2560, 720), Ogre::HardwareBuffer::HBL_WRITE_ONLY);
    const Ogre::PixelBox &pixel_box_3 = pixel_buffer->getCurrentLock();
    // memset(pixel_box_3.getTopLeftFrontPixelPtr(), 0xC0, Ogre::PixelUtil::getMemorySize(1280, 720, 1, pixel_buffer->getFormat()));
    data = static_cast<uint32_t *>(pixel_box_3.data);
    for(int y = 0; y < pixel_box_3.getHeight(); ++y)
    {
        for(int x = 0; x < pixel_box_3.getWidth(); ++x)
        {
            data[pixel_box_3.rowPitch * y + x] = 0xC0C0C0C0;
        }
    }
    pixel_buffer->unlock();

    pixel_buffer->lock(Ogre::Image::Box(1280, 720, 2560, 1440), Ogre::HardwareBuffer::HBL_WRITE_ONLY);
    const Ogre::PixelBox &pixel_box_4 = pixel_buffer->getCurrentLock();
    // memset(pixel_box_4.getTopLeftFrontPixelPtr(), 0xFF, Ogre::PixelUtil::getMemorySize(1280, 720, 1, pixel_buffer->getFormat()));
    data = static_cast<uint32_t *>(pixel_box_4.data);
    for(int y = 0; y < pixel_box_4.getHeight(); ++y)
    {
        for(int x = 0; x < pixel_box_4.getWidth(); ++x)
        {
            data[pixel_box_4.rowPitch * y + x] = 0xFFFFFFFF;
        }
    }
    pixel_buffer->unlock();

#endif

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: PB updated" << std::endl;
    std::cerr << std::endl << "DynGeo: PB updated" << std::endl;
#endif

    _material_name = std::to_string(rand());

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(_material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);

    material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    material->getTechnique(0)->getPass(0)->createTextureUnitState(_texture_name);
    material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CullingMode::CULL_NONE);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: material set" << std::endl;
    std::cerr << std::endl << "DynGeo: material set" << std::endl;
#endif

    _thread_recv = std::thread([&]() { _ReadLoop(); });

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: load after" << std::endl;
    std::cerr << std::endl << "DynGeo: load after" << std::endl;
#endif
}
void GuaDynGeoVisualPlugin::_ReadLoop()
{
#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: _ReadLoop" << std::endl;
    std::cerr << std::endl << "DynGeo: _ReadLoop" << std::endl;
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

    /// CONFLATE
    int conflate = 1;
    socket.setsockopt(ZMQ_CONFLATE, &conflate, sizeof(conflate));

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
        memcpy(&_texture_bounding_boxes, (unsigned char *)header.tex_bounding_box, sizeof(header.tex_bounding_box));

        _num_geometry_bytes = header.geometry_payload_size;
        memcpy(&_bb_min, &header.global_bb_min, sizeof(float) * 3);
        memcpy(&_bb_max, &header.global_bb_max, sizeof(float) * 3);
        memcpy(&_buffer_rcv[0], (unsigned char *)zmqm.data() + SGTP::HEADER_BYTE_SIZE, header.geometry_payload_size);
        memcpy(&_buffer_rcv_texture[0], (unsigned char *)zmqm.data() + SGTP::HEADER_BYTE_SIZE + header.geometry_payload_size, header.texture_payload_size);

        /*#if GUA_DEBUG == 1
                gzerr << "DynGeo: geometry bytes " << header.geometry_payload_size << std::endl;
                std::cerr << "DynGeo: geometry bytes " << header.geometry_payload_size << std::endl;
        #endif

        #if GUA_DEBUG == 1
                gzerr << "DynGeo: texture payload " << header.texture_payload_size << std::endl;
                std::cerr << "DynGeo: texture payload " << header.texture_payload_size << std::endl;
        #endif*/

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

    if(!_visual || !_scene_node || !_scene_manager)
    {
        return;
    }

    _scene_node->setVisible(false, false);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: scene manager acquired" << std::endl;
    std::cerr << std::endl << "DynGeo: scene manager acquired" << std::endl;
#endif

    size_t texture_offset = 0;

    for(unsigned int i = 0; i < SGTP::_MAX_NUM_SENSORS; i++)
    {
        auto texture_bounding_box = _texture_bounding_boxes[i];

        if(texture_bounding_box.min.u == texture_bounding_box.max.u)
        {
            continue;
        }

/*#if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: min x" << texture_bounding_box.min.u << " y " << texture_bounding_box.min.v << std::endl;
        std::cerr << std::endl << "DynGeo: min x" << texture_bounding_box.min.u << " y " << texture_bounding_box.min.v << std::endl;
#endif

#if GUA_DEBUG == 1
        gzerr << std::endl << "DynGeo: max x" << texture_bounding_box.max.u << " y " << texture_bounding_box.max.v << std::endl;
        std::cerr << std::endl << "DynGeo: max x" << texture_bounding_box.max.u << " y " << texture_bounding_box.max.v << std::endl;
#endif*/

        Ogre::HardwarePixelBufferSharedPtr pixel_buffer = Ogre::TextureManager::getSingleton().getByName(_texture_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME)->getBuffer();

        pixel_buffer->lock(Ogre::Image::Box(texture_bounding_box.min.u, texture_bounding_box.min.v, texture_bounding_box.max.u, texture_bounding_box.max.v), Ogre::HardwareBuffer::HBL_WRITE_ONLY);
        const Ogre::PixelBox &pixel_box = pixel_buffer->getCurrentLock();

        uint32_t *data = static_cast<uint32_t *>(pixel_box.data);
        unsigned char px[4];
        px[0] = 0xFF;
        for(int y = 0; y < pixel_box.getHeight(); ++y)
        {
            for(int x = 0; x < pixel_box.getWidth(); ++x)
            {
                memcpy(&px[1], &_buffer_rcv_texture[texture_offset + pixel_box.getWidth() * y + x], 3);
                data[pixel_box.rowPitch * y + x] = *(uint32_t *)&px[0];

                texture_offset += 3;
            }
        }

        pixel_buffer->unlock();
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: texture updated" << std::endl;
    std::cerr << std::endl << "DynGeo: texture updated" << std::endl;
#endif

#if TEX_DEBUG == 1

    size_t num_vertices = 6;

    std::vector<unsigned char> buffer_quad(num_vertices * sizeof(float) * 5);

    for(unsigned int i = 0; i < num_vertices; i++)
    {
        float vx[5];
        vx[0] = 0.f;

        switch(i)
        {
        case 0:
            vx[1] = 0.f;
            vx[2] = 0.f;

            vx[3] = 0.f;
            vx[4] = 0.f;
            break;
        case 1:
            vx[1] = 0.f;
            vx[2] = 1.f;

            vx[3] = 0.f;
            vx[4] = 1.f;
            break;
        case 2:
            vx[1] = 1.f;
            vx[2] = 0.f;

            vx[3] = 1.f;
            vx[4] = 0.f;
            break;
        case 3:
            vx[1] = 1.f;
            vx[2] = 0.f;

            vx[3] = 1.f;
            vx[4] = 0.f;
            break;
        case 4:
            vx[1] = 1.f;
            vx[2] = 1.f;

            vx[3] = 1.f;
            vx[4] = 1.f;
            break;
        case 5:
            vx[1] = 0.f;
            vx[2] = 1.f;

            vx[3] = 0.f;
            vx[4] = 1.f;
            break;
        }

        memcpy(&buffer_quad[i * sizeof(float) * 5], &vx[0], sizeof(float) * 5);
    }

    std::vector<int32_t> buffer_quad_index(num_vertices);

    std::iota(buffer_quad_index.begin(), buffer_quad_index.end(), 0);

    for(size_t i = 0; i < buffer_quad_index.size() - 2; i += 3)
    {
        int32_t swapSpace = buffer_quad_index[i + 2];
        buffer_quad_index[i + 2] = buffer_quad_index[i + 1];
        buffer_quad_index[i + 1] = swapSpace;
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;
    std::cerr << std::endl << "DynGeo: vertices in buffer " << std::to_string(num_vertices) << std::endl;
#endif

    _mesh_name = std::to_string(rand());

    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(_mesh_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    // "-" for Z axis and min <-> max flip is NOT A MISTAKE!
    mesh->_setBounds(Ogre::AxisAlignedBox::BOX_INFINITE);
    mesh->_setBoundingSphereRadius(10.f);

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
    vbuf->writeData(0, vbuf->getSizeInBytes(), &buffer_quad[0], true);
    bind->setBinding(0, vbuf);

    Ogre::HardwareIndexBufferSharedPtr ibuf =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_32BIT, num_vertices, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    ibuf->writeData(0, ibuf->getSizeInBytes(), &buffer_quad_index[0], true);

    /*#if GUA_DEBUG == 1
        float vx[3];
        float tx[2];

        memcpy(&vx[0], &_buffer_rcv[100 * 5 * sizeof(float)], 3 * sizeof(float));
        memcpy(&tx[0], &_buffer_rcv[100 * 5 * sizeof(float) + 3 * sizeof(float)], 2 * sizeof(float));

        gzerr << std::endl << "DynGeo: vx 500 " << vx[0] << " " << vx[1] << " " << vx[2] << std::endl;
        std::cerr << std::endl << "DynGeo: vx 500 " << vx[0] << " " << vx[1] << " " << vx[2] << std::endl;

        gzerr << std::endl << "DynGeo: tx 500 " << tx[0] << " " << tx[1] << std::endl;
        std::cerr << std::endl << "DynGeo: tx 500 " << tx[0] << " " << tx[1] << std::endl;
    #endif*/

    Ogre::SubMesh *sub = mesh->createSubMesh();
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = num_vertices;
    sub->indexData->indexStart = 0;

    mesh->load();

    while(!mesh->isLoaded())
    {
    }

#else

    size_t num_vertices = _num_geometry_bytes / (sizeof(float) * 5);

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

    _mesh_name = std::to_string(rand());

    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(_mesh_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

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

    Ogre::HardwareIndexBufferSharedPtr ibuf =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_32BIT, num_vertices, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    ibuf->writeData(0, ibuf->getSizeInBytes(), &_buffer_index[0], true);

    /*#if GUA_DEBUG == 1
        float vx[3];
        float tx[2];

        memcpy(&vx[0], &_buffer_rcv[100 * 5 * sizeof(float)], 3 * sizeof(float));
        memcpy(&tx[0], &_buffer_rcv[100 * 5 * sizeof(float) + 3 * sizeof(float)], 2 * sizeof(float));

        gzerr << std::endl << "DynGeo: vx 500 " << vx[0] << " " << vx[1] << " " << vx[2] << std::endl;
        std::cerr << std::endl << "DynGeo: vx 500 " << vx[0] << " " << vx[1] << " " << vx[2] << std::endl;

        gzerr << std::endl << "DynGeo: tx 500 " << tx[0] << " " << tx[1] << std::endl;
        std::cerr << std::endl << "DynGeo: tx 500 " << tx[0] << " " << tx[1] << std::endl;
    #endif*/

    Ogre::SubMesh *sub = mesh->createSubMesh();
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = num_vertices;
    sub->indexData->indexStart = 0;

    mesh->load();

    while(!mesh->isLoaded())
    {
    }

#endif

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: mesh loaded" << std::endl;
    std::cerr << std::endl << "DynGeo: mesh loaded" << std::endl;
#endif

    _entity_name = std::to_string(rand());

    _entity = _scene_manager->createEntity(_entity_name, _mesh_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    _entity->setMaterialName(_material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
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

/*#if GUA_DEBUG == 1
    float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

    // test access to scene
    const Ogre::ColourValue ambient(r, g, b, 1.f);
    _scene_manager->setAmbientLight(ambient);
#endif*/

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

    if(!_avatar_node->isInSceneGraph())
    {
        return;
    }

    _avatar_node->removeAllChildren();
    while(_avatar_node->numChildren() != 0)
    {
    }

    _scene_node->removeAllChildren();

    while(_scene_node->numChildren() != 0)
    {
    }

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: destroying entity" << std::endl;
    std::cerr << std::endl << "DynGeo: destroying entity" << std::endl;
#endif

    _scene_manager->destroyEntity(_entity);

#if GUA_DEBUG == 1
    gzerr << std::endl << "DynGeo: entity destroyed" << std::endl;
    std::cerr << std::endl << "DynGeo: entity destroyed" << std::endl;
#endif

    if(_mesh_name.empty())
    {
        return;
    }

    Ogre::MeshManager::getSingleton().remove(_mesh_name);
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
