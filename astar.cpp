#include "astar.h"
#include <iostream>
#include <cstdio>
#include <cmath>
#include <string>
#include <fstream>
#include <string.h>
#include <stdlib.h>

using namespace std;

const int MaxDistance = 9999;

astar_t::astar_t(const char* mapFileName) :m_steps(0)
{
    m_mapFileName = (char *)malloc((strlen(mapFileName) + 1) * sizeof(char));
    strcpy(m_mapFileName,mapFileName);
}

astar_t::~astar_t()
{
    free(m_mapFileName);
    
    // 千万不能有这句代码，因为startNode已加入OPEN表，会在释放OPEN表以及
    // 的时候释放，否则会造成重复释放，出现bug
    // delete startNode;
    delete endNode;
    
    // 释放迷宫布局数组：注意多维数组空间释放
    for (int i = 0;i < m_rows;++i)
    {
        delete[] m_maze[i];
    }
    delete[] m_maze;
    
    for (int i = 0;i < m_rows;++i)
    {
        delete[] m_path[i];
    }
    delete[] m_path;
    
    // 释放OPEN表以及CLOSED表内存空间
    vector<Node*>::iterator iter;
    for (iter = OPENTable.begin();iter != OPENTable.end();++iter)
    {
        delete (*iter);
    }
    
    OPENTable.clear();
    
    vector<Node*>::iterator iter2;
    for (iter2 = CLOSEDTable.begin();iter2 != CLOSEDTable.end();++iter2)
    {
        delete (*iter2);
    }

    CLOSEDTable.clear();
}

void astar_t::init()
{
    dx[0] = dx[4] = dx[5] = -1;
    dx[1] = dx[3] = 0;
    dx[2] = dx[6] = dx[7] = 1;
    dy[3] = dy[4] = dy[7] = -1;
    dy[0] = dy[2] = 0;
    dy[1] = dy[5] = dy[6] = 1;
    
    readMap();
    
    // 分配空间
    m_path = new int *[m_rows];
    
    for (int i = 0;i < m_rows;++i)
    {
        m_path[i] = new int[m_cols];
    }
    
    startNode = new Node;
    
    startNode->x = m_startX;
    startNode->y = m_startY;
    startNode->g = 0;
    startNode->h = judge(startNode->x,startNode->y);
    startNode->f = startNode->g + startNode->h;
    startNode->father = NULL;

    endNode = new Node;

    endNode->x = m_endX;
    endNode->y = m_endY;
    endNode->father = NULL;
}

bool astar_t::pathFinding()
{
    // 判断起始点和目标点是否是同一点
    if (m_startX == m_endX && m_startY == m_endY)
    {
        cout <<"WARNING: The start point is the same as the destination!" <<endl;
        return true;
    }

    // 起始点装入OPEN表
    OPENTable.push_back(startNode);

    // 对vector中元素进行排序：将最后一个元素加入原本已序的heap内
    push_heap(OPENTable.begin(), OPENTable.end(), HeapCompare_f());

    Node* tempNode = new Node;

    // 开始遍历
    for( ;; )
    {
        // 判断OPEN表是否为空
        if(OPENTable.empty())
        {
            cout << "ERROR: unable to find the destination!" <<endl;
            return false;
        }

        // 注意：OPEN表为空会导致未定义行为
        tempNode = OPENTable.front();
        ++ m_steps;

        // 将第一个元素移到最后，并将剩余区间重新排序，组成新的heap
        pop_heap(OPENTable.begin(), OPENTable.end(), HeapCompare_f());

        // 删除最后一个元素
        OPENTable.pop_back();

        // 判断是否已经搜寻到目标节点
        if(tempNode->x == m_endX && tempNode->y == m_endY)
        {
            cout << "OK: success to find the destination" << endl;
            endNode->g = tempNode->g;
            endNode->h = tempNode->h;
            endNode->f = tempNode->f;
            endNode->father = tempNode->father;

            generatePath();

            return true;
        }

        // 针对每个子节点
        for(int i = 0; i < 8; ++ i)
        {
            int nextX = tempNode->x + dx[i];
            int nextY = tempNode->y + dy[i];

            if(isIllegle(nextX, nextY))
            {
                // 注意：障碍物角落不能直接通过
                if(1 == *(*(m_maze + tempNode->x) + nextY) || 1 == *(*(m_maze + nextX) + tempNode->y))
                {
                    continue;
                }

                // 计算此节点的g值
                int newGVal;

                // 位于对角线上
                if(!dx[i] && !dy[i])
                {
                    newGVal = tempNode->g + 14;
                }
                else
                {
                    newGVal = tempNode->g + 10;
                }

                // 搜索OPEN表，判断此点是否在OPEN表中
                vector<Node*>::iterator OPENTableResult;

                for(OPENTableResult = OPENTable.begin(); OPENTableResult != OPENTable.end(); ++ OPENTableResult)
                {
                    if((*OPENTableResult)->x == nextX && (*OPENTableResult)->y == nextY)
                    {
                        break;
                    }
                }

                // 此子节点已经存在于OPEN表中
                if(OPENTableResult != OPENTable.end())
                {
                    // OPEN表中节点的g值已经是最优的，则跳过此节点
                    if((*OPENTableResult)->g <= newGVal)
                    {
                        continue;
                    }
                }

                // 搜索CLOSED表，判断此节点是否已经存在于其中
                vector<Node*>::iterator CLOSEDTableResult;

                for(CLOSEDTableResult = CLOSEDTable.begin(); CLOSEDTableResult != CLOSEDTable.end(); ++ CLOSEDTableResult)
                {
                    if((*CLOSEDTableResult)->x == nextX && (*CLOSEDTableResult)->y == nextY)
                    {
                        break;
                    }
                }

                // 此节点已经存在于CLOSED表中
                if(CLOSEDTableResult != CLOSEDTable.end())
                {
                    // CLOSED表中的节点已经是最优的，则跳过
                    if((*CLOSEDTableResult)->g <= newGVal)
                    {
                        continue;
                    }
                }

                // 此节点是迄今为止的最优节点
                Node* bestNode = new Node;
                bestNode->x = nextX;
                bestNode->y = nextY;
                bestNode->father = tempNode;
                bestNode->g = newGVal;
                bestNode->h = judge(nextX, nextY);
                bestNode->f = bestNode->g + bestNode->h;

                // 如果已经存在于CLOSED表中，将其移除
                if(CLOSEDTableResult != CLOSEDTable.end())
                {
                    delete (*CLOSEDTableResult);
                    CLOSEDTable.erase(CLOSEDTableResult);

                    // 重新建堆，实现排序。注意不能用sort_heap，因为如果容器为空的话会出现
                    make_heap(OPENTable.begin(), OPENTable.end(), HeapCompare_f());
                }

                // 将最优节点放入OPEN表
                OPENTable.push_back(bestNode);

                // 重新排序
                push_heap(OPENTable.begin(), OPENTable.end(), HeapCompare_f());
            }
        }

        CLOSEDTable.push_back(tempNode);
    }

    return false;
}

void astar_t::generatePath()
{
    Node* nodeChild = endNode;
    Node* nodeParent = endNode->father;

    do
    {
        // 标记为路径上的点
        *(*(m_path + nodeChild->x) + nodeChild->y) = NODE_PATH;
        nodeChild = nodeParent;
        nodeParent = nodeParent->father;
    }while(nodeChild != startNode);

    // 标记为路径上的点
    *(*(m_path + startNode->x) + startNode->y) = NODE_PATH;
}

void astar_t::printPath()
{
    cout << "The path is " << endl;

    for(int i = 0; i < m_rows; ++ i)
    {
        for(int j = 0; j < m_cols; ++ j)
        {
            if(NODE_PATH == *(*(m_path + i) + j))
            {
                cout << "*" << " ";
            }
            else
            {
                cout << *(*(m_maze + i) + j) << " ";
            }
        }
        cout << endl;
    }

    cout << "Search Step: " << m_steps << endl;
}

bool astar_t::readMap()
{
    ifstream mapFileStream(m_mapFileName);
    if(!mapFileStream)
    {
        cerr <<"ERROR: unable to open map file" <<endl;
        return false;
    }

    mapFileStream >> m_rows >> m_cols;

    // 多维数组空间分配
    m_maze = new int *[m_rows];
    for (int i = 0;i < m_rows;++i)
    {
        m_maze[i] = new int[m_cols];
    }

    mapFileStream >> m_startX >> m_startY;
    mapFileStream >> m_endX >> m_endY;

    for (int i = 0;i < m_rows;++i)
    {
        for (int j = 0;j < m_cols;++j)
        {
            mapFileStream >> *(*(m_maze + i) + j);
        }
    }

    return true;
}

int astar_t::judge(int x, int y)
{
    return (10 * (abs(m_endX - x) + abs(m_endY - y)));
}

bool astar_t::isIllegle(int x, int y)
{
    if (x >= 0 && x < m_rows && y >= 0 && y < m_cols && *(*(m_maze + x) + y) == 0)
    {
        return true;
    }
    return false;
}
