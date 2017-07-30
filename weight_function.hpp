#ifndef NET_CODERODDE_PATHFINDING_WEIGHT_FUNCTION_HPP
#define NET_CODERODDE_PATHFINDING_WEIGHT_FUNCTION_HPP

namespace net {
namespace coderodde {
namespace pathfinding {
    
    template<typename Node, typename WeightType>
    class weight_function {
        
    public:
        virtual WeightType operator()(const Node& a, const Node& b) const = 0;
    };
    
} // End of namespace net::coderodde::pathfinding.
} // End of namespace net::coderodde.
} // End of namespace net.

#endif // NET_CODERODDE_PATHFINDING_WEIGHT_FUNCTION_HPP
