#ifndef GUA_INSTANCE_COLLECTION_HPP
#define GUA_INSTANCE_COLLECTION_HPP

#include <memory>
#include <unordered_map>

namespace gua
{
class InstanceCollection
{
  public:
    template <typename T>
    std::shared_ptr<T> get()
    {
        auto value = std::static_pointer_cast<T, void>(collection_[typeid(T)].lock());
        if(!value)
        {
            value = std::make_shared<T>();
            collection_[typeid(T)] = std::static_pointer_cast<void, T>(value);
        }
        return value;
    }

    template <typename T>
    bool has()
    {
        return !collection_[typeid(T)].expired();
    }

  private:
    // Adapted from: http://en.cppreference.com/w/cpp/types/type_info/hash_code
    using TypeInfoRef = std::reference_wrapper<const std::type_info>;

    struct Hasher
    {
        std::size_t operator()(TypeInfoRef code) const { return code.get().hash_code(); }
    };

    struct EqualTo
    {
        bool operator()(TypeInfoRef lhs, TypeInfoRef rhs) const { return lhs.get() == rhs.get(); }
    };

    std::unordered_map<TypeInfoRef, std::weak_ptr<void>, Hasher, EqualTo> collection_;
};

} // namespace gua

#endif // GUA_INSTANCE_COLLECTION_HPP
