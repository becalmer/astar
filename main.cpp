#include "astar.h"

int main(int argc, const char* argv[])
{
    astar_t astar("data.txt");

    astar.init();

    astar.pathFinding();

    astar.printPath();
    return 0;
}
