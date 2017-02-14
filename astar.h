#ifndef astar_t_H
#define astar_t_H

// Ϊ��ʹ�����ȼ�����priority_queue
#include <queue>
#include <stack>
#include <vector>

// �Թ���ͼ�нڵ����ͱ��
enum
{
    // ����ͨ���Ľڵ�
    NODE_EMPTY,
    // �ϰ������ͨ��
    NODE_OBSTACLE,
    // ·���ϵĵ�
    NODE_PATH
};

// ��¼·���ϵĵ������
typedef struct tagpathNode
{
    int x;
    int y;
}PathNode;

// �ڵ����ݽṹ����
typedef struct tagNode
{
    // ��ǰ�����Թ��е�λ������
    int x;
    int y;
    // ��ʼ�㵽��ǰ��ʵ�ʴ���
    int g;
    // ��ǰ�ڵ㵽Ŀ��ڵ����·���Ĺ��ƴ���
    int h;
    // ���ƺ�����f = g + h��
    int f;
    // ָ���丸�ڵ��ָ��
    struct tagNode *father;
}Node;

// ����STL���ȶ��е�����ʽ
class HeapCompare_f
{
public:
    bool operator()(Node* x,Node* y) const
    {
        // ���ݹ��ۺ�������������������
        return x->f > y->f;
    }
};

// �Թ�Ѱ·��A*�㷨
class astar_t
{
private:
    // �洢��ͼ��Ϣ���ļ���
    char *m_mapFileName;
    // �Թ��ĸ߶ȺͿ��
    int  m_rows;
    int  m_cols;
    // �Թ�����
    int  **m_maze;
    // ��ʼ������
    int  m_startX;
    int  m_startY;
    // Ŀ�������
    int  m_endX;
    int  m_endY;
    // 8���ӽڵ��ƶ������ϡ��¡����ҡ����ϡ����ϡ����¡�����
    int  dx[8];
    int  dy[8];
    // ��ʼ�ڵ��Ŀ��ڵ�
    Node *startNode;
    Node *endNode;
    // ��¼·����Ϣ
    int  **m_path;
    // ���������ѵ��ܲ���
    int  m_steps;
    
    // OPEN������C++ STL��vectorʵ�����ȼ����й���
    // ע�⣺�洢����Node*ָ��
    std::vector<Node*> OPENTable;
    
    // CLOSED���洢��Ҳ��Node*ָ��
    std::vector<Node*> CLOSEDTable;

public:
    // ���캯��
    astar_t(const char* mapFileName);
    // ��������
    ~astar_t();
    
    // ��ʼ��
    void init();
    
    // ��ȡ��ͼ��Ϣ
    bool readMap();
    
    // Ѱ·������
    bool pathFinding();
    
    // ����·����Ϣ
    void generatePath();
    
    // ��ӡ·����Ϣ
    void printPath();
    
    // ���Ƶ�ǰ�㵽Ŀ���ľ��룺�����پ���
    int judge(int x, int y);
    
    // �ж�ĳһ�ڵ��Ƿ�Ϸ�
    bool isIllegle(int x, int y);

};

#endif
