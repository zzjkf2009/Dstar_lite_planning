/**
 * @Author: Zejiang Zeng <zzj>
 * @Date:   2018-03-27T10:19:23-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: Dlite.h
 * @Last modified by:  zzj
 * @Last modified time: 2018-03-27T10:19:56-04:00
 * @ This work is originally inspired by  palmieri at: https://github.com/palmieri/srl_dstar_lite
 * Modifications are made.
 */
#ifndef DSTARLITE_H
#define DSTARLITE_H

#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <ext/hash_map>


using namespace std;
using namespace __gnu_cxx;

/**
 * [Node  a grid with its specific parameters, (x,y) in 2D space,g and rhs, keys
 * see more details in [S. Koenig, 2002]]
 * @param x   [x position]
 * @param y   [y position]
 * @param g   [cost of the path from the start node]
 * @param rhs [min of the predecessor's g(s') + c(s,s')]
 * @param k    [key of the node]
 */
class Node {
public:
int x;
int y;
pair<double,double> k;

// Define operator overloading to comparing the Nodes
bool operator == (const Node &s2) const {
        return ((x == s2.x) && (y == s2.y));
}

bool operator != (const Node &s2) const {
        return ((x != s2.x) || (y != s2.y));
}
// Since keys k1, k2 are double, so we will condiser it are the same with a torlence 0.0001
bool operator > (const Node &s2) const {
        if (k.first-0.00001 > s2.k.first) return true;
        else if (k.first < s2.k.first-0.00001) return false;
        return k.second > s2.k.second;
}

bool operator <= (const Node &s2) const {
        if (k.first < s2.k.first) return true;
        else if (k.first > s2.k.first) return false;
        return k.second < s2.k.second + 0.00001;
}


bool operator < (const Node &s2) const {
        if (k.first + 0.000001 < s2.k.first) return true;
        else if (k.first - 0.000001 > s2.k.first) return false;
        return k.second < s2.k.second;
}

};

struct ipoint2 {
        int x,y;
};

struct NodeInfo {

        double g;
        double rhs;
        double cost;

};

class Node_hash {
public:
size_t operator()(const Node &s) const {
        return s.x + 34245*s.y;
}
};


typedef priority_queue<Node, vector<Node>, greater<Node> > ds_pq;
typedef hash_map<Node,NodeInfo, Node_hash, equal_to<Node> > ds_ch;
typedef hash_map<Node, float, Node_hash, equal_to<Node> > ds_oh;

class Dstar {

public:

Dstar(int startX, int startY, int goalX, int goalY);
Dstar();
void   init(int sX, int sY, int gX, int gY);
void   updateCell(int x, int y, double val);
void   updateStart(int x, int y);
void   updateGoal(int x, int y);
bool   replan();
void   draw();
void   drawCell(Node s,float z);

list<Node> getPath();

private:

list<Node> path;

double D; // the unit distance between two nodes
double k_m; // the accumulate key value for every time edge change
Node s_start, s_goal, s_last;
int maxSteps;

priority_queue<Node, vector<Node>, greater<Node> > openList;
hash_map<Node,NodeInfo, Node_hash, equal_to<Node> > cellHash;
hash_map<Node, float, Node_hash, equal_to<Node> >  openHash;

bool   AreSame(double x, double y);
void   makeNewCell(Node u);
double getG(Node u);
double getRHS(Node u);
void   setG(Node u, double g);
double setRHS(Node u, double rhs);
double eightCondist(Node a, Node b);
int    computeShortestPath();
void   updateVertex(Node u);
void   insert(Node u);
void   remove(Node u);
double trueDist(Node a, Node b);
double heuristic(Node a, Node b);
Node  calculateKey(Node u);
void   getSucc(Node u, list<Node> &s);
void   getPred(Node u, list<Node> &s);
double cost(Node a, Node b);
bool   occupied(Node u);
bool   isValid(Node u);
float  keyHashCode(Node u);
};

#endif
