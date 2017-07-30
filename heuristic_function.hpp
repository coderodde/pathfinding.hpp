#ifndef NET_CODERODDE_PATHFINDING_HEURISTIC_FUNCTION_HPP
#define NET_CODERODDE_PATHFINDING_HEURISTIC_FUNCTION_HPP

namespace net {
namespace coderodde {
namespace pathfinding {
    
    template<typename Node, typename DistanceType>
    class heuristic_function {
        
    public:
        virtual DistanceType operator()(const Node& target) const = 0;
    };
    
} // End of namespace net::coderodde::pathfinding.
} // End of namespace net::coderodde.
} // End of namespace net.

#endif // End of NET_CODERODDE_PATHFINDING_HEURISTIC_FUNCTION_HPP.
