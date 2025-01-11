#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <limits>
#include <algorithm>

struct Point{
    double x, y, z;

    Point(double x, double y, double z) : x(x), y(y), z(z) {}

    double distance(const Point &other) const {
        return std::sqrt(std::pow(x - other.x, 2) + 
               std::pow(y - other.y, 2) + 
               std::pow(z - other.z, 2));
    }
};

struct Node {
    Point position;
    Node* parent;
    std::vector<Node*> children;

    Node(Point position, Node* parent) : position(position), parent(parent) {}
};

//cuboid obstacles
struct Cuboid {
    
    Point min, max;
    
    Cuboid(Point min, Point max) : min(min), max(max) {}
    
    //check if a point is inside the cuboid
    bool point_intersect(const Point &point) const {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    //check if a line intersects the cuboid
    bool line_intersect(const Point &start, const Point &end) const{
        double tx1 = (min.x - start.x) / (end.x - start.x);
        double tx2 = (max.x - start.x) / (end.x - start.x);
        double ty1 = (min.y - start.y) / (end.y - start.y);
        double ty2 = (max.y - start.y) / (end.y - start.y);
        double tz1 = (min.z - start.z) / (end.z - start.z);
        double tz2 = (max.z - start.z) / (end.z - start.z);

        double tmin = std::max(std::max(std::min(tx1, tx2), std::min(ty1, ty2)),
                             std::min(tz1, tz2));
        double tmax = std::min(std::min(std::max(tx1, tx2), std::max(ty1, ty2)),
                             std::max(tz1, tz2));

        return tmax >= tmin && tmax >= 0 && tmin <= 1;
    }
};

class RRT {
public:
    //Workspace bounds
    Point min_bound, max_bound;

    //Obstacles
    std::vector<Cuboid> obstacles {};

    //Tree
    Node* root {};
    std::vector<Node*> nodes {};
    //this is the maximum distance between a new node and the nearest node in the tree
    double step_size;

    //Random number generator
    std::mt19937 rng {};
    // std::random_device rd {}; 
    std::uniform_real_distribution<double> dist_x {};
    std::uniform_real_distribution<double> dist_y {};
    std::uniform_real_distribution<double> dist_z {};

    //Constructor
    RRT(const Point &start, const Point &min_bound, const Point &max_bound, double step_size = 1.0)
    : min_bound(min_bound), max_bound(max_bound), rng(std::random_device()()), step_size(step_size) {
        
        //initialize the tree with the start node
        std::cout << "RRT constructor" << std::endl;
        root = new Node(start, nullptr); //nullptr means no parent
        nodes.push_back(root); 
        dist_x = std::uniform_real_distribution<double>(min_bound.x, max_bound.x); 
        dist_y = std::uniform_real_distribution<double>(min_bound.y, max_bound.y);
        dist_z = std::uniform_real_distribution<double>(min_bound.z, max_bound.z);
    } 

    //Destructor
    ~RRT() {
        for(Node* node : nodes) {
            delete node;
        }
    }

    //setup the obstacles
    void add_obstacle(const Cuboid &obstacle) {
        obstacles.push_back(obstacle);
        std::cout << "Obstacle added!" << std::endl;
    }

    //Randomly sample a point in the workspace
    Point random_point() {
        return Point(dist_x(rng), dist_y(rng), dist_z(rng));
    }

    //Find the nearest node in the tree to a given point
    Node* nearest_node(const Point &point) {
        Node* nearest = nullptr;
        double min_distance = std::numeric_limits<double>::infinity();

        for (Node* node : nodes) {
            double distance = node->position.distance(point); 
            //position.distance(point) means the distance between the position of the node and the given point
            if (distance < min_distance) {
                nearest = node;
                min_distance = distance;
            }
        }
        return nearest;
    }

    //Check is path is collision free
    bool is_clear_path(const Point &start, const Point &end){

        for (const Cuboid &obstacle : obstacles) {
            if (obstacle.line_intersect(start, end)) {
                return false;
            }
        }
        return true;
    }

    //Extend the tree towards a point
    Node* extend_tree(Node* nearest, const Point &to){
        double distance = nearest->position.distance(to);
        if (distance < step_size) {
            Node* new_node = new Node(to, nearest);
            nearest->children.push_back(new_node);
            nodes.push_back(new_node);
            return new_node;
        } else {
            double dx = to.x - nearest->position.x;
            double dy = to.y - nearest->position.y;
            double dz = to.z - nearest->position.z;

            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            dx = dx / dist * step_size;
            dy = dy / dist * step_size;
            dz = dz / dist * step_size;

            Point new_position(nearest->position.x + dx, nearest->position.y + dy, nearest->position.z + dz);
            Node* new_node = new Node(new_position, nearest);
            nearest->children.push_back(new_node);
            nodes.push_back(new_node);
            return new_node;
        }
        return nullptr;
    }

    //Find a path from the start to the goal
    std::vector<Point> find_rrt_path(const Point &goal, int max_it = 10000) {

        for (int i = 0; i < max_it; i++){
            
            // std::cout << "Iteration: " << i << std::endl;
            //Randomly sample a point in the workspace (Vanilla RRT)
            Point random = random_point();

            //Find the nearest node in the tree to the random point
            Node* nearest = nearest_node(random);

            //Extend the tree towards the random point
            Node* new_node = extend_tree(nearest, random);

            //Check if the new node is close to the goal
            if (new_node && new_node->position.distance(goal) < step_size) {
                std::cout << "Goal found!" << std::endl;
                std::vector<Point> path;
                Node* current = new_node;
                while (current != nullptr) {
                    path.push_back(current->position);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
        }
        return std::vector<Point>();
    }
};

// int main() {

//     Point start(0.0, 0.0, 0.0);
//     Point goal(10.0, 10.0, 10.0);
//     Point min_bound(-10.0, -10.0, -10.0);
//     Point max_bound(11.0, 11.0, 11.0);
//     std::cout << "Start: " << start.x << " " << start.y << " " << start.z << std::endl;
//     std::cout << "About to create RRT " << std::endl;

//     RRT rrt(start, min_bound, max_bound);
//     std::cout << "RRT created!" << std::endl;

//     Cuboid obstacle1(Point(-5.0, -5.0, -5.0), Point(5.0, 5.0, 5.0));
//     // rrt.add_obstacle(obstacle1);

//     std::vector<Point> path = rrt.find_rrt_path(goal);

//     for (const Point &point : path) {
//         std::cout << point.x << " " << point.y << " " << point.z << std::endl;
//     }

//     return 0;
// }