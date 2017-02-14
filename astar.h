#ifndef astar_t_H
#define astar_t_H

// 为了使用优先级队列priority_queue
#include <queue>
#include <stack>
#include <vector>

// 迷宫地图中节点类型标记
enum
{
    // 可以通过的节点
    NODE_EMPTY,
    // 障碍物，不可通过
    NODE_OBSTACLE,
    // 路径上的点
    NODE_PATH
};

// 记录路径上的点的坐标
typedef struct tagpathNode
{
    int x;
    int y;
}PathNode;

// 节点数据结构定义
typedef struct tagNode
{
    // 当前点在迷宫中的位置坐标
    int x;
    int y;
    // 起始点到当前点实际代价
    int g;
    // 当前节点到目标节点最佳路径的估计代价
    int h;
    // 估计函数：f = g + h。
    int f;
    // 指向其父节点的指针
    struct tagNode *father;
}Node;

// 定义STL优先队列的排序方式
class HeapCompare_f
{
public:
    bool operator()(Node* x,Node* y) const
    {
        // 依据估价函数进行排序：升序排列
        return x->f > y->f;
    }
};

// 迷宫寻路：A*算法
class astar_t
{
private:
    // 存储地图信息的文件名
    char *m_mapFileName;
    // 迷宫的高度和宽度
    int  m_rows;
    int  m_cols;
    // 迷宫布局
    int  **m_maze;
    // 起始点坐标
    int  m_startX;
    int  m_startY;
    // 目标点坐标
    int  m_endX;
    int  m_endY;
    // 8个子节点移动方向：上、下、左、右、左上、右上、右下、左下
    int  dx[8];
    int  dy[8];
    // 起始节点和目标节点
    Node *startNode;
    Node *endNode;
    // 记录路径信息
    int  **m_path;
    // 搜索所花费的总步数
    int  m_steps;
    
    // OPEN表：采用C++ STL中vector实现优先级队列功能
    // 注意：存储的是Node*指针
    std::vector<Node*> OPENTable;
    
    // CLOSED表：存储的也是Node*指针
    std::vector<Node*> CLOSEDTable;

public:
    // 构造函数
    astar_t(const char* mapFileName);
    // 析构函数
    ~astar_t();
    
    // 初始化
    void init();
    
    // 读取地图信息
    bool readMap();
    
    // 寻路主函数
    bool pathFinding();
    
    // 产生路径信息
    void generatePath();
    
    // 打印路径信息
    void printPath();
    
    // 估计当前点到目标点的距离：曼哈顿距离
    int judge(int x, int y);
    
    // 判断某一节点是否合法
    bool isIllegle(int x, int y);

};

#endif
