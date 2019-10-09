#ifndef GUACAMOLE_OCTREENODE_HPP
#define GUACAMOLE_OCTREENODE_HPP

// guacamole headers
#include <gua/renderer/Lod.hpp>
#include <gua/scenegraph/PickResult.hpp>
// external headers
#include <set>
#include <unordered_set>
#include <memory>
#include <vector>


namespace gua {


    class GUA_LOD_DLL OctreeNode {
            public:

            OctreeNode();
            OctreeNode(uint64_t _idx, uint32_t _child_mask, uint32_t _child_idx,
                       const scm::math::vec3f& _min, const scm::math::vec3f& _max,
                       const std::set<uint32_t>& _fotos);

            public:

            void test_wrapping() const;

            uint64_t get_idx() const;
            uint32_t get_child_mask() const;
            uint32_t get_child_idx() const;

            const scm::math::vec3f& get_min() const;
            const scm::math::vec3f& get_max() const;

            const std::set<uint32_t>& get_fotos() const;
            uint32_t get_fotos_size() const;
            uint32_t get_foto_by_id(uint32_t id) const;


            private: // methods

            private: // member
            uint64_t idx_;
            uint32_t child_mask_;   //char r_, g_, b_, child_mask_
            uint32_t child_idx_;    //idx of first child
            scm::math::vec3f min_;
            scm::math::vec3f max_;
            std::set<uint32_t> fotos_;
    };

}


#endif //GUACAMOLE_OCTREENODE_HPP
