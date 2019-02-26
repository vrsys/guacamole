#include <unittest++/UnitTest++.h>
#include <iostream>
#include <array>
#include <gua/math/BoundingBox.hpp>
#include <gua/math/traits.hpp>

namespace
{
struct vec3
{
    vec3() : xyz({{0.0f, 0.0f, 0.0f}}) {}
    vec3(float x) : xyz({{x, x, x}}) {}
    vec3(float x, float y, float z) : xyz({{x, y, z}}) {}
    float& operator[](int i) { return xyz[i]; }
    float operator[](int i) const { return xyz[i]; }
    bool operator==(vec3 const& rhs) const
    {
        bool acc(true);
        for(int i(0); i < 3; ++i)
        {
            acc = acc && xyz[i] == rhs.xyz[i];
        }
        return acc;
    }
    friend std::ostream& operator<<(std::ostream& os, vec3 const& v)
    {
        os << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
        return os;
    }
    std::array<float, 3> xyz;
};

vec3 operator*(float s, vec3 const& v) { return vec3(s * v[0], s * v[1], s * v[2]); }

vec3 operator+(vec3 const& u, vec3 const& v) { return vec3(u[0] + v[0], u[1] + v[1], u[2] + v[2]); }
} // namespace

namespace gua
{
namespace traits
{
template <>
struct dimension<vec3>
{
    static const unsigned int value = 3;
};
template <>
struct scalar<vec3>
{
    typedef float type;
};

} // namespace traits
} // namespace gua

TEST(default_ctor_creates_empty_bbox)
{
    gua::math::BoundingBox<vec3> bbox;
    CHECK(bbox.isEmpty());
}

TEST(min_and_max_should_be_equal_for_ctor_with_one_parameter)
{
    gua::math::BoundingBox<vec3> bbox(vec3(1.0, 1.0, 1.0));
    CHECK_EQUAL(bbox.min, bbox.max);
}

TEST(ctor)
{
    gua::math::BoundingBox<vec3> bbox(vec3(0.0, 0.0, 0.0), vec3(1.0, 1.0, 1.0));
    CHECK_EQUAL(vec3(0.0, 0.0, 0.0), bbox.min);
    CHECK_EQUAL(vec3(1.0, 1.0, 1.0), bbox.max);
}

TEST(it_should_check_if_a_point_is_contained)
{
    gua::math::BoundingBox<vec3> bbox(vec3(-1.0, -1.0, -1.0), vec3(1.0, 1.0, 1.0));
    vec3 p(0, 0, 0.5);
    CHECK(bbox.contains(p));
}

TEST(it_should_be_possible_to_intersect_two_bounding_boxes)
{
    gua::math::BoundingBox<vec3> a(vec3(-1.0, -1.0, -1.0), vec3(1.0, 1.0, 1.0));
    gua::math::BoundingBox<vec3> b(vec3(0.0, 0.0, 0.0), vec3(2.0, 2.0, 2.0));
    CHECK(a.intersects(b));
}

TEST(it_should_be_possible_to_test_for_intersection)
{
    gua::math::BoundingBox<vec3> a(vec3(-1.0, -1.0, -1.0), vec3(0.0, 0.0, 0.0));
    gua::math::BoundingBox<vec3> b(vec3(0.1, 0.1, 0.1), vec3(2.0, 2.0, 2.0));
    CHECK(!a.intersects(b));
}

TEST(it_should_be_possible_to_construct_from_list)
{
    std::vector<vec3> ps;
    for(unsigned int i = 0; i < 10; ++i)
    {
        ps.push_back(vec3(float(i) / 10.0f, float(i) / 10.0f, float(i) / 10.0f));
    }
    ps.push_back(vec3(1.0, 1.0, 1.0));
    gua::math::BoundingBox<vec3> bbox(ps);
    CHECK_EQUAL(vec3(0.0, 0.0, 0.0), bbox.min);
    CHECK_EQUAL(vec3(1.0, 1.0, 1.0), bbox.max);
}

TEST(it_should_be_possible_to_combine_two_bboxes)
{
    gua::math::BoundingBox<vec3> a(vec3(0.0, -2.0, 0.0), vec3(1.0, 1.0, -1.0));
    gua::math::BoundingBox<vec3> b(vec3(-2.0, 0.0, -2.0), vec3(-1.0, -1.0, 1.0));

    gua::math::BoundingBox<vec3> c = combine(a, b);
    CHECK_EQUAL(vec3(-2.0, -2.0, -2.0), c.min);
    CHECK_EQUAL(vec3(1.0, 1.0, 1.0), c.max);
}

TEST(it_should_be_possible_to_compute_the_intersection)
{
    gua::math::BoundingBox<vec3> a(vec3(-2.0, -2.0, -2.0), vec3(0.5, 0.5, 0.5));
    gua::math::BoundingBox<vec3> b(vec3(-1.0, -1.0, -1.0), vec3(1.0, 1.0, 1.0));

    gua::math::BoundingBox<vec3> c = intersection(a, b);
    CHECK_EQUAL(vec3(-1.0, -1.0, -1.0), c.min);
    CHECK_EQUAL(vec3(0.5, 0.5, 0.5), c.max);
}
