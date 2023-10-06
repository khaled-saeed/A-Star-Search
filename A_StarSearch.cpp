// A_StarSearch.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <algorithm>
#include <vector>
#include <queue>
#include <set>
#include <unordered_set>
#include <cmath>

struct point
{
    int16_t X;
    int16_t Y; 

};
typedef enum {
    UP,
    DOWN,
    RIGHT,
    LEFT
}Action_t;

constexpr point availableActions[4] = { {0,-1}, // UP
                                       {0,1},   // DOWN
                                       {1,0},   // RIGHT 
                                       {-1,0} };// LEFT      

class node
{
public:
    uint16_t x, y; 
    uint8_t value;
    float heuristics;
    uint16_t cost;
    node* parent;
};

struct NodeComparator {
    bool operator()(const node* n1, const node* n2) const {
		bool result = (n1->cost + n1->heuristics) > (n2->cost + n2->heuristics);
        return result;
    }
};
class grid
{
public:
    grid(uint16_t rows, uint16_t cols):rows(rows),cols(cols)
    {
        gridVec = std::vector<std::vector<node>>(rows, std::vector<node>(cols));
    }
    ~grid() {

    }
    uint16_t noOfObsticals;
    uint16_t rows, cols; 
    node* StartingPoint;
    node* Goal;
    std::vector<std::vector<node>> gridVec;


    void vPrintGrid()
    {
        for (auto i = 0; i < gridVec.size(); i++)
        {
            for (auto j = 0; j < gridVec[i].size(); j++)
            {
                std::cout << " " << gridVec[i][j].value << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    void vPrintHeuristics()
    {
        for (auto i = 0; i < gridVec.size(); i++)
        {
            for (auto j = 0; j < gridVec[i].size(); j++)
            {
                std::cout << " " << gridVec[i][j].heuristics << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    void vGridInit(void)
    {
        uint8_t startingArray[][10] = {
        {'1' , '1' , '1' , '1' , '1' , '1' , '1' , '1' , '1' , '1'},
        {'1' , '1' , '1' , '0' , '0' , '0' , '0' , '0' , '0' , '1'},
        {'1' , '1' , '1' , '0' , '1' , '1' , '1' , '1' , '0' , '1'},
        {'1' , '1' , '1' , '0' , '1' , '1' , '1' , '1' , '0' , '1'},
        {'1' , '1' , '1' , '0' , '0' , '0' , '0' , '1' , '0' , '1'},
        {'1' , '1' , '1' , '1' , '1' , '1' , '0' , '1' , '0' , '1'},
        {'1' , '1' , '1' , '1' , '1' , '1' , '0' , '1' , '0' , '1'},
        {'1' , '0' , '0' , '0' , '0' , '0' , '0' , '1' , '0' , '1'},
        {'1' , '0' , '1' , '1' , '1' , '1' , '1' , '1' , '0' , '1'},
        {'1' , '1' , '1' , '1' , '1' , '1' , '1' , '1' , '1' , '1'},
        };
        for (auto i = 0; i < gridVec.size(); i++)
        {
            for (auto j = 0; j < gridVec[i].size(); j++)
            {
                gridVec[i][j].value = startingArray[i][j];
                gridVec[i][j].x = j; 
                gridVec[i][j].y = i; 
            }
        }

        std::cout << "Enter Starting Point (x,y):";
        uint16_t x, y; 
        std::cin >> x >> y;
        StartingPoint = &gridVec[y][x];
        StartingPoint->cost = 0; 
        gridVec[StartingPoint->y][StartingPoint->x].value = 'S';
        std::cout << "Enter Goal Point (x,y):";
        std::cin >>x >>y;
        Goal = &gridVec[y][x];
        gridVec[Goal->y][Goal->x].value = 'G';
        std::cout << "Enter No Obstacles: ";
        std::cin >> noOfObsticals;
        for (auto i = 0; i < noOfObsticals; i++)
        {
            std::cout << "Enter Obstacle Coordinates (x,y): ";
            uint16_t x, y;
            std::cin >> x >> y;
            if (!(x < gridVec[0].size() && y < gridVec.size())) {
                std::cout << "you entered invalid coordinates\n";
                break;
            }
            else
            {
                gridVec[y][x].value = '1';
            }
        }
    }

    std::vector<node*> getNeighbors(node* currNode)
    {
        std::vector<node*> Neighbors; 
        for (auto i = 0; i < 4; i++)
        {
            int16_t x = currNode->x + availableActions[i].X;
            int16_t y = currNode->y + availableActions[i].Y; 
            if (x >= 0 && x < cols && y >= 0 && y < rows && gridVec[y][x].value != '1')
            {
                Neighbors.push_back(&gridVec[y][x]);
            }
        }
        return Neighbors;

    }
    void vCalculateHeuristic(void)
    {
        for (auto i = 0; i < gridVec.size(); i++)
        {
            for (auto j = 0; j < gridVec[i].size(); j++)
            {
                gridVec[i][j].heuristics = sqrt(pow((Goal->x - j), 2) + pow((Goal->y - i), 2));
                
            }
        }
    }
private:

};


class A_Star {
public:
    grid* mapGrid; 
    std::priority_queue<node*,std::vector<node*> ,NodeComparator> nextPosition;
    std::unordered_set<node*> visited;
    std::vector<node*>myPath; 
    A_Star(grid* grid)
    {
        this->mapGrid = grid;         
    }
    void aStarInit()
    {
       
        visited.clear();
        nextPosition.push(mapGrid->StartingPoint);

    }

    node* find_min() {
        return nextPosition.top();             // the set is ordered by f value
    }

    bool find_path() {
        while (!nextPosition.empty()) {          // while there are nodes to explore
            node* current = find_min();     // get the node with the lowest cost
			visited.insert(current);
            nextPosition.pop();
            if (current == mapGrid->Goal) { // if it is the goal, we are done
                while (current != mapGrid->StartingPoint)
                {
                    myPath.push_back(current); 
                    current = current->parent; 
                }
                std::reverse(myPath.begin(), myPath.end());
                return true;
            }
            else {

                for (auto nieghbour : mapGrid->getNeighbors(current)) {
                    if (visited.find(nieghbour) == visited.end())
                    {
					    nieghbour->parent = current;
					    nieghbour->cost = current->cost + 1; // update cost
                        nextPosition.push(nieghbour); 

                    }
                }
            }
        }
        return false;  
    }
    void PathVisualisation()
    {
        for (uint16_t i = 0 ; i < myPath.size() ; i++)
        {
            if (myPath[i] != mapGrid->Goal)
            {
                if (myPath[i + 1]->x - myPath[i]->x == 0 && myPath[i + 1]->y - myPath[i]->y == 1)
                {
                    myPath[i]->value = 'v';
                }
                if (myPath[i + 1]->x - myPath[i]->x == 0 && myPath[i + 1]->y - myPath[i]->y == -1)
                {
                    myPath[i]->value = '^';
                }
                if (myPath[i + 1]->x - myPath[i]->x == 1 && myPath[i + 1]->y - myPath[i]->y == 0)
                {
                    myPath[i]->value = '>';
                }
                if (myPath[i + 1]->x - myPath[i]->x == -1 && myPath[i + 1]->y - myPath[i]->y == 0)
                {
                    myPath[i]->value = '<';
                }
            }
            else
            {
                mapGrid->vPrintGrid(); 
            }

        }
    }
};


int main()
{
	//generate a code to use the A_Star class to find the shortest path in the grid


    std::queue<node*> myPath; 
    uint16_t r, c; 
    std::cout << "Enter the Grid Dimensions\nNo Rows: ";
    std::cin >> r; 
    std::cout << "NO Columns:";
    std::cin >> c; 
    grid myGrid(r, c); 
    myGrid.vGridInit(); 
    myGrid.vPrintGrid(); 
	myGrid.vCalculateHeuristic();
    //myGrid.vPrintHeuristics(); 
    myPath.push(myGrid.StartingPoint);
    std::cout << "Start A* Algorithm ? yes/no: ";
    std::string stStartCmd;
    std::cin >> stStartCmd; 
    if (stStartCmd == "yes")
    {
        A_Star myAStar(&myGrid);
        myAStar.aStarInit();
        bool pathFound = myAStar.find_path();
        if (pathFound)
        {
            myAStar.PathVisualisation(); 
        }
        else
        {
			std::cout << "No path found" << std::endl;
        }
    }
    else if (stStartCmd == "no")
    {
        std::cout << "Start A* Algorithm ? yes/no: ";
        std::cin >> stStartCmd;
    }
    else
    {
        std::cout << "Incorrect Entry \n"; 
    }
    system("pause");

}

