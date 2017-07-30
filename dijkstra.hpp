#ifndef NET_CODERODDE_PATHFINDING_DIJKSTRA_HPP
#define NET_CODERODDE_PATHFINDING_DIJKSTRA_HPP

#include "a_star.hpp"
#include "heuristic_function.hpp"

namespace net {
namespace coderodde {
namespace pathfinding {
    
    template<typename Node, typename DistanceType>
    class zero_heuristic :
    public virtual heuristic_function<Node, DistanceType> {
        
    public:
        DistanceType operator()(const Node& target) const {
            DistanceType zero{};
            return zero;
        }
    };
    
    template<typename Node, typename Weight>
    weighted_path<Node, Weight> search(Node& source,
                                       Node& target,
                                       weight_function<Node, Weight>& w) {
        zero_heuristic<Node, Weight> h;
        return search(source, target, w, h);
    }
    
} // End of namespace net::coderodde::pathfinding.
} // End of namespace net::coderodde.
} // End of namespace net.

#endif // NET_CODERODDE_PATHFINDING_DIJKSTRA_HPP
