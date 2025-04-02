#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

// Each node represents a point on the grid. 
// The cost is the distance travelled so far from the start
// The heuristic is a function define later that calcualtes the eclidean distance to the goal
// Parent points to the previous node
// the total cost is how far it has travelled and how far it is from the goal. I.E, low score is good
struct Node {
    int x, y;  
    float cost, heuristic;
    Node* parent; 

    Node(int x, int y, float cost, float heuristic, Node* parent = nullptr)
        : x(x), y(y), cost(cost), heuristic(heuristic), parent(parent) {}

    float totalCost() const { return cost + heuristic; }
};


// This function is what is used to figure out what is the highest priority to search the next node. 
// I.E if this node is a low cost, search from there first
struct CompareNode {
    bool operator()(const Node* a, const Node* b) {
        return a->totalCost() > b->totalCost();
    }
};

// Eclidiean distace from point to point
float heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}


// Impliment A* algorithim. Take an initial start X and Y positon, a goal X and Y Position and a map
std::vector<Node*> aStarSearch(int startX, int startY, int goalX, int goalY, std::vector<std::vector<int>>& grid) {
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> openList;
    std::unordered_map<int, Node*> visited;

    // Create an initial node at the start position with a cost of zero as it hasn't moved yet
    Node* start = new Node(startX, startY, 0, heuristic(startX, startY, goalX, goalY));
    openList.push(start);

    // Define the movements allowed, I.E. up down left and right.
    // This can be modified if more dirrections are required, I.E diagonal - if the resolution gets big enough, maybe this can be variable?
    std::vector<std::pair<int, int>> directions = {{0,1}, {1,0}, {0,-1}, {-1,0}};

    // Start the A* loop, this will continue to run while there is nodes in the priority queue, as the initial node was pushed back above
    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();

        // check if we have reached the goal
        if (current->x == goalX && current->y == goalY) {
            
            // Reconstruct path
            std::vector<Node*> path;
            while (current) {
                path.push_back(current);
                current = current->parent;
            }
            return path;
        }
        for (auto [dx, dy] : directions) {
            int nx = current->x + dx;
            int ny = current->y + dy;

            if (nx >= 0 && ny >= 0 && nx < static_cast<int>(grid.size()) && ny < static_cast<int>(grid[0].size()) && grid[nx][ny] == 0){
                float newCost = current->cost + 1;
                Node* neighbor = new Node(nx, ny, newCost, heuristic(nx, ny, goalX, goalY), current);
                openList.push(neighbor);
            }
        }
    }
    return {};
}

// Main function to test A*
int main() {
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 1},
        {0, 1, 1, 0, 1},
        {0, 0, 0, 0, 0},
        {1, 1, 0, 1, 1},
        {0, 0, 0, 0, 0}
    };

    auto path = aStarSearch(0, 0, 4, 4, grid);

    for (auto node : path) {
        std::cout << "(" << node->x << ", " << node->y << ") <- ";
    }
    std::cout << "Start\n";

    return 0;
}

    
