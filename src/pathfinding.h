#include <unordered_map>
#include <vector>
#include <queue>
#include <array>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>

struct GridLocation {
    long x, y;
};

namespace std {
    template <> struct hash<GridLocation> {
        typedef GridLocation argument_type;
        typedef std::size_t result_type;
        std::size_t operator()(const GridLocation& id) const noexcept {
            return std::hash<int>()(id.x ^ (id.y << 4));
        }
    };
}

bool operator == (GridLocation a, GridLocation b) {
  return a.x == b.x && a.y == b.y;
}

bool operator != (GridLocation a, GridLocation b) {
  return !(a == b);
}

bool operator < (GridLocation a, GridLocation b) {
  return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

std::basic_iostream<char>::basic_ostream& operator<<(std::basic_iostream<char>::basic_ostream& out, const GridLocation& loc) {
  out << '(' << loc.x << ',' << loc.y << ')';
  return out;
}

#define DEFAULT_COST 10
struct GridWithWeights {
    static std::array<GridLocation, 4> DIRS;
    
    int width, height;

    GridWithWeights(int w, int h): width(w), height(h) {}

    bool in_bounds(GridLocation id) const {
        return 0 <= id.x && id.x < width
        && 0 <= id.y && id.y < height;
    }

    std::vector<GridLocation> neighbors(GridLocation id) const {
        std::vector<GridLocation> result;

        for(GridLocation dir: DIRS) {
            GridLocation next{id.x + dir.x, id.y + dir.y};
            if(in_bounds(next)) {
                result.push_back(next);
            }
        }

        return result;
    }

    double cost(GridLocation from, GridLocation to) const {
        return DEFAULT_COST;
    }
};

std::array<GridLocation, 4> GridWithWeights::DIRS = {
    GridLocation{1, 0}, GridLocation{-1, 0},
    GridLocation{0, -1}, GridLocation{0, 1}
};

template<typename T, typename priority_t>
struct PriorityQueue {
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>,
            std::greater<PQElement> > elements;
    inline bool empty() const {
        return elements.empty();
    }

    inline void put(T item, priority_t priority) {
        elements.emplace(priority, item);
    }

    T get() {
        T bestItem = elements.top().second;
        elements.pop();
        return bestItem;
    }
};

GridWithWeights make_diagram() {
  GridWithWeights grid(10, 10);
  return grid;
}

template<typename Location, typename Graph>
void dijkstraSearch
(
    Graph graph,
    Location start,
    Location goal,
    std::unordered_map<Location, Location>& cameFrom,
    std::unordered_map<Location, double>& costSoFar)
{
    PriorityQueue<Location, double> frontier;
    frontier.put(start, 0);

    cameFrom[start] = start;
    costSoFar[start] = 0;

    while(!frontier.empty()) {
        Location current = frontier.get();

        if(current == goal) break;

        for(Location next: graph.neighbors(current)) {
            double newCost = costSoFar[current] + graph.cost(current, next);
            if(costSoFar.find(next) == costSoFar.end()
            || newCost < costSoFar[next]) {
                costSoFar[next] = newCost;
                cameFrom[next] = current;
                frontier.put(next, newCost);
            }
        }
    }
}

template<typename Location>
std::vector<Location> reconstructPath(
    Location start,
    Location goal,
    std::unordered_map<Location, Location> cameFrom){
    std::vector<Location> path;
    Location current = goal;
    while(current != start) {
        path.push_back(current);
        current = cameFrom[current];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

template<typename Location>
std::vector<Location> searchOnDefaultGraph(Location start, Location goal) {
    GridWithWeights defaultGraph = make_diagram();
    std::unordered_map<GridLocation, GridLocation> cameFrom;
    std::unordered_map<GridLocation, double> costSoFar;
    dijkstraSearch(defaultGraph, start, goal, cameFrom, costSoFar);
    std::vector<GridLocation> path = reconstructPath(start, goal, cameFrom);
    return path;
}