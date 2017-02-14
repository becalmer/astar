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
    
    // ǧ�����������룬��ΪstartNode�Ѽ���OPEN�������ͷ�OPEN���Լ�
    // ��ʱ���ͷţ����������ظ��ͷţ�����bug
    // delete startNode;
    delete endNode;
    
    // �ͷ��Թ��������飺ע���ά����ռ��ͷ�
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
    
    // �ͷ�OPEN���Լ�CLOSED���ڴ�ռ�
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
    
    // ����ռ�
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
    // �ж���ʼ���Ŀ����Ƿ���ͬһ��
    if (m_startX == m_endX && m_startY == m_endY)
    {
        cout <<"WARNING: The start point is the same as the destination!" <<endl;
        return true;
    }

    // ��ʼ��װ��OPEN��
    OPENTable.push_back(startNode);

    // ��vector��Ԫ�ؽ������򣺽����һ��Ԫ�ؼ���ԭ�������heap��
    push_heap(OPENTable.begin(), OPENTable.end(), HeapCompare_f());

    Node* tempNode = new Node;

    // ��ʼ����
    for( ;; )
    {
        // �ж�OPEN���Ƿ�Ϊ��
        if(OPENTable.empty())
        {
            cout << "ERROR: unable to find the destination!" <<endl;
            return false;
        }

        // ע�⣺OPEN��Ϊ�ջᵼ��δ������Ϊ
        tempNode = OPENTable.front();
        ++ m_steps;

        // ����һ��Ԫ���Ƶ���󣬲���ʣ������������������µ�heap
        pop_heap(OPENTable.begin(), OPENTable.end(), HeapCompare_f());

        // ɾ�����һ��Ԫ��
        OPENTable.pop_back();

        // �ж��Ƿ��Ѿ���Ѱ��Ŀ��ڵ�
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

        // ���ÿ���ӽڵ�
        for(int i = 0; i < 8; ++ i)
        {
            int nextX = tempNode->x + dx[i];
            int nextY = tempNode->y + dy[i];

            if(isIllegle(nextX, nextY))
            {
                // ע�⣺�ϰ�����䲻��ֱ��ͨ��
                if(1 == *(*(m_maze + tempNode->x) + nextY) || 1 == *(*(m_maze + nextX) + tempNode->y))
                {
                    continue;
                }

                // ����˽ڵ��gֵ
                int newGVal;

                // λ�ڶԽ�����
                if(!dx[i] && !dy[i])
                {
                    newGVal = tempNode->g + 14;
                }
                else
                {
                    newGVal = tempNode->g + 10;
                }

                // ����OPEN���жϴ˵��Ƿ���OPEN����
                vector<Node*>::iterator OPENTableResult;

                for(OPENTableResult = OPENTable.begin(); OPENTableResult != OPENTable.end(); ++ OPENTableResult)
                {
                    if((*OPENTableResult)->x == nextX && (*OPENTableResult)->y == nextY)
                    {
                        break;
                    }
                }

                // ���ӽڵ��Ѿ�������OPEN����
                if(OPENTableResult != OPENTable.end())
                {
                    // OPEN���нڵ��gֵ�Ѿ������ŵģ��������˽ڵ�
                    if((*OPENTableResult)->g <= newGVal)
                    {
                        continue;
                    }
                }

                // ����CLOSED���жϴ˽ڵ��Ƿ��Ѿ�����������
                vector<Node*>::iterator CLOSEDTableResult;

                for(CLOSEDTableResult = CLOSEDTable.begin(); CLOSEDTableResult != CLOSEDTable.end(); ++ CLOSEDTableResult)
                {
                    if((*CLOSEDTableResult)->x == nextX && (*CLOSEDTableResult)->y == nextY)
                    {
                        break;
                    }
                }

                // �˽ڵ��Ѿ�������CLOSED����
                if(CLOSEDTableResult != CLOSEDTable.end())
                {
                    // CLOSED���еĽڵ��Ѿ������ŵģ�������
                    if((*CLOSEDTableResult)->g <= newGVal)
                    {
                        continue;
                    }
                }

                // �˽ڵ�������Ϊֹ�����Žڵ�
                Node* bestNode = new Node;
                bestNode->x = nextX;
                bestNode->y = nextY;
                bestNode->father = tempNode;
                bestNode->g = newGVal;
                bestNode->h = judge(nextX, nextY);
                bestNode->f = bestNode->g + bestNode->h;

                // ����Ѿ�������CLOSED���У������Ƴ�
                if(CLOSEDTableResult != CLOSEDTable.end())
                {
                    delete (*CLOSEDTableResult);
                    CLOSEDTable.erase(CLOSEDTableResult);

                    // ���½��ѣ�ʵ������ע�ⲻ����sort_heap����Ϊ�������Ϊ�յĻ������
                    make_heap(OPENTable.begin(), OPENTable.end(), HeapCompare_f());
                }

                // �����Žڵ����OPEN��
                OPENTable.push_back(bestNode);

                // ��������
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
        // ���Ϊ·���ϵĵ�
        *(*(m_path + nodeChild->x) + nodeChild->y) = NODE_PATH;
        nodeChild = nodeParent;
        nodeParent = nodeParent->father;
    }while(nodeChild != startNode);

    // ���Ϊ·���ϵĵ�
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

    // ��ά����ռ����
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
