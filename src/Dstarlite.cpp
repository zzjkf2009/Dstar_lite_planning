/**
 * @Author: Zejiang Zeng <zzj>
 * @Date:   2018-03-27T09:48:02-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: dstar.cpp
 * @Last modified by:   zzj
 * @Last modified time: 2018-03-27T10:33:43-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */

#include "Dstar_lite_planning/Dstarlite.h"
#include <stdio.h>
#include <cmath>

/**
 * [class constructor]
 */
Dstar::Dstar(int startX, int startY, int goalX, int goalY) {
        init(startX,startY,goalX,goalY);
        maxSteps = 80000; // node expansions before we give up
        D       = 1; // cost of an unseen cell
}
/**
 * [class constructor]
 */
Dstar::Dstar() {

        maxSteps = 80000; // node expansions before we give up
        D       = 1; // cost of an unseen cell

}


/**
 * [Dstar::keyHashCode Returns the key hash code for the Node u, this is used
 * to compare, Node that have been updated, based on the x and y value]
 * @param  u [Node]
 * @return   [float]
 */
float Dstar::keyHashCode(Node u) {

        return (float)(u.k.first + 1193*u.k.second);

}

/**
 * [Dstar::isValid Returns true if Node u is on the open list by checking if
 * it is in the hash table]
 * @param  u [Node]
 * @return   [boolean]
 */
bool Dstar::isValid(Node u) {

        ds_oh::iterator cur = openHash.find(u);
        if (cur == openHash.end()) return false;
        if (!AreSame(keyHashCode(u), cur->second)) return false;
        return true;

}

/* void Dstar::getPath()
 * --------------------------
 * Returns the path created by replan()
 */
list<Node> Dstar::getPath() {
        return path;
}

/* bool Dstar::occupied(Node u)
 * --------------------------
 * returns true if the cell is occupied (non-traversable), false
 * otherwise. non-traversable are marked with a cost < 0.
 */
bool Dstar::occupied(Node u) {

        ds_ch::iterator cur = cellHash.find(u);
        if (cur == cellHash.end()) return false;
        return (cur->second.cost < 0);
}

/* void Dstar::init(int sX, int sY, int gX, int gY)
 * --------------------------
 * Init dstar with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void Dstar::init(int sX, int sY, int gX, int gY) {

        cellHash.clear();
        path.clear();
        openHash.clear();
        while(!openList.empty()) openList.pop();

        k_m = 0;

        s_start.x = sX;
        s_start.y = sY;
        s_goal.x  = gX;
        s_goal.y  = gY;

        NodeInfo tmp;
        tmp.g = tmp.rhs =  0;
        tmp.cost = D;

        cellHash[s_goal] = tmp;

        tmp.g = tmp.rhs = heuristic(s_start,s_goal);
        tmp.cost = D;
        cellHash[s_start] = tmp;
        s_start = calculateKey(s_start);
        openList.push(s_start);
        s_last = s_start;

}


/* void Dstar::makeNewCell(Node u)
 * --------------------------
 * Checks if a cell is in the hash table, if not it adds it in.
 */
void Dstar::makeNewCell(Node u) {

        if (cellHash.find(u) != cellHash.end()) return;

        NodeInfo tmp;
        tmp.g       = tmp.rhs = heuristic(u,s_goal);
        tmp.cost    = D;
        cellHash[u] = tmp;

}

/* double Dstar::getG(Node u)
 * --------------------------
 * Returns the G value for Node u.
 */
double Dstar::getG(Node u) {

        if (cellHash.find(u) == cellHash.end())
                return heuristic(u,s_goal);
        return cellHash[u].g;

}

/* double Dstar::getRHS(Node u)
 * --------------------------
 * Returns the rhs value for Node u.
 */
double Dstar::getRHS(Node u) {

        if (u == s_goal) return 0;

        if (cellHash.find(u) == cellHash.end())
                return heuristic(u,s_goal);
        return cellHash[u].rhs;

}

/* void Dstar::setG(Node u, double g)
 * --------------------------
 * Sets the G value for Node u
 */
void Dstar::setG(Node u, double g) {

        makeNewCell(u);
        cellHash[u].g = g;
}

/* void Dstar::setRHS(Node u, double rhs)
 * --------------------------
 * Sets the rhs value for Node u
 */
double Dstar::setRHS(Node u, double rhs) {

        makeNewCell(u);
        cellHash[u].rhs = rhs;

}

/* double Dstar::eightCondist(Node a, Node b)
 * --------------------------
 * Returns the 8-way distance between Node a and Node b.
 */
double Dstar::eightCondist(Node a, Node b) {
        double temp;
        double min = abs(a.x - b.x);
        double max = abs(a.y - b.y);
        if (min > max) {
                double temp = min;
                min = max;
                max = temp;
        }
        return ((M_SQRT2-1.0)*min + max);
}

/* int Dstar::computeShortestPath()
 * --------------------------
 * As per [S. Koenig, 2002] except for 2 main modifications:
 * 1. We stop planning after a number of steps, 'maxsteps' we do this
 *    because this algorithm can plan forever if the start is
 *    surrounded by obstacles.
 * 2. We lazily remove Nodes from the open list so we never have to
 *    iterate through it.
 */
int Dstar::computeShortestPath() {

        list<Node> s;
        list<Node>::iterator i;

        if (openList.empty()) {
                printf("openlist is empyt\n");
                return -1;
        }

        int k=0;
        while ((!openList.empty()) ||
               (openList.top() < (s_start = calculateKey(s_start))) ||
               (getRHS(s_start) != getG(s_start))) {

                if (k++ > maxSteps) {
                        fprintf(stderr, "At maxsteps\n");
                        return -1;
                }


                Node u;

                bool test = (getRHS(s_start) != getG(s_start));

                // lazy remove
                while(1) {
                        if (openList.empty()) return 1;
                        u = openList.top();
                        openList.pop();

                        if (!isValid(u)) continue;
                        if (!(u < s_start) && (!test)) return 2;
                        break;
                }

                ds_oh::iterator cur = openHash.find(u);
                openHash.erase(cur);

                Node k_old = u;

                if (k_old < calculateKey(u)) { // u is out of date
                        insert(u);
                } else if (getG(u) > getRHS(u)) { // needs update (got better)
                        setG(u,getRHS(u));
                        getPred(u,s);
                        for (i=s.begin(); i != s.end(); i++) {
                                updateVertex(*i);
                        }
                } else { // g <= rhs, Node has got worse
                        setG(u,INFINITY);
                        getPred(u,s);
                        for (i=s.begin(); i != s.end(); i++) {
                                updateVertex(*i);
                        }
                        updateVertex(u);
                }
        }
        return 0;
}

/**
 * [Dstar::AreSame determine two numbers are approximately same]
 * @param  x [double]
 * @param  y [double]
 * @return   [bool]
 */
bool Dstar::AreSame(double x, double y) {

        if (std::isinf(x) && std::isinf(y)) return true;
        return (fabs(x-y) < 0.00001);

}

/* void Dstar::updateVertex(Node u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateVertex(Node u) {

        list<Node> s;
        list<Node>::iterator i;

        if (u != s_goal) {
                getSucc(u,s);
                double tmp = INFINITY;
                double tmp2;

                for (i=s.begin(); i != s.end(); i++) {
                        tmp2 = getG(*i) + cost(u,*i);
                        if (tmp2 < tmp) tmp = tmp2;
                }
                if (!AreSame(getRHS(u),tmp)) setRHS(u,tmp);
        }

        if (!AreSame(getG(u),getRHS(u))) insert(u);

}

/* void Dstar::insert(Node u)
 * --------------------------
 * Inserts Node u into openList and openHash.
 */
void Dstar::insert(Node u) {

        ds_oh::iterator cur;
        float csum;

        u    = calculateKey(u);
        cur  = openHash.find(u);
        csum = keyHashCode(u);
        // return if cell is already in list. TODO: this should be
        // uncommented except it introduces a bug, I suspect that there is a
        // bug somewhere else and having duplicates in the openList queue
        // hides the problem...
        //if ((cur != openHash.end()) && (AreSame(csum,cur->second))) return;

        openHash[u] = csum;
        openList.push(u);
}

/* void Dstar::remove(Node u)
 * --------------------------
 * Removes Node u from openHash. The Node is removed from the
 * openList lazilily (in replan) to save computation.
 */
void Dstar::remove(Node u) {

        ds_oh::iterator cur = openHash.find(u);
        if (cur == openHash.end()) return;
        openHash.erase(cur);
}


/* double Dstar::trueDist(Node a, Node b)
 * --------------------------
 * Euclidean cost between Node a and Node b.
 */
double Dstar::trueDist(Node a, Node b) {

        float x = a.x-b.x;
        float y = a.y-b.y;
        return sqrt(x*x + y*y);

}

/* double Dstar::heuristic(Node a, Node b)
 * --------------------------
 * Pretty self explanitory, the heristic we use is the 8-way distance
 * scaled by a constant D (should be set to <= min cost).
 */
double Dstar::heuristic(Node a, Node b) {
        return eightCondist(a,b)*D;
}

/* Node Dstar::calculateKey(Node u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
Node Dstar::calculateKey(Node u) {

        double val = fmin(getRHS(u),getG(u));

        u.k.first  = val + heuristic(u,s_start) + k_m;
        u.k.second = val;

        return u;

}

/* double Dstar::cost(Node a, Node b)
 * --------------------------
 * Returns the cost of moving from Node a to Node b. This could be
 * either the cost of moving off Node a or onto Node b, we went with
 * the former. This is also the 8-way cost.
 */
double Dstar::cost(Node a, Node b) {

        int xd = abs(a.x-b.x);
        int yd = abs(a.y-b.y);
        double scale = 1;

        if (xd+yd>1) scale = M_SQRT2;

        if (cellHash.count(a) == 0) return scale*D;
        return scale*cellHash[a].cost;

}
/* void Dstar::updateCell(int x, int y, double val)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void Dstar::updateCell(int x, int y, double val) {

        Node u;

        u.x = x;
        u.y = y;

        if ((u == s_start) || (u == s_goal)) return;

        makeNewCell(u);
        cellHash[u].cost = val;

        updateVertex(u);
}

/* void Dstar::getSucc(Node u,list<Node> &s)
 * --------------------------
 * Returns a list of successor Nodes for Node u, since this is an
 * 8-way graph this list contains all of a cells neighbours. Unless
 * the cell is occupied in which case it has no successors.
 */
void Dstar::getSucc(Node u,list<Node> &s) {

        s.clear();
        u.k.first  = -1;
        u.k.second = -1;

        if (occupied(u)) return;

        u.x += 1;
        s.push_front(u);
        u.y += 1;
        s.push_front(u);
        u.x -= 1;
        s.push_front(u);
        u.x -= 1;
        s.push_front(u);
        u.y -= 1;
        s.push_front(u);
        u.y -= 1;
        s.push_front(u);
        u.x += 1;
        s.push_front(u);
        u.x += 1;
        s.push_front(u);

}

/* void Dstar::getPred(Node u,list<Node> &s)
 * --------------------------
 * Returns a list of all the predecessor Nodes for Node u. Since
 * this is for an 8-way connected graph the list contails all the
 * neighbours for Node u. Occupied neighbours are not added to the
 * list.
 */
void Dstar::getPred(Node u,list<Node> &s) {

        s.clear();
        u.k.first  = -1;
        u.k.second = -1;

        Node ua, ub;
        // u.x += 1;
        // if (!occupied(u)) s.push_front(u);
        // u.y += 1; // TODO
        // if (!occupied(u)) s.push_front(u);
        // u.x -= 1;
        // if (!occupied(u)) s.push_front(u);
        // u.x -= 1; // TODO
        // if (!occupied(u)) s.push_front(u);
        // u.y -= 1;
        // if (!occupied(u)) s.push_front(u);
        // u.y -= 1; // TODO
        // if (!occupied(u)) s.push_front(u);
        // u.x += 1;
        // if (!occupied(u)) s.push_front(u);
        // u.x += 1; // TODO
        // if (!occupied(u)) s.push_front(u);

        u.x += 1;
        if (!occupied(u))
                s.push_front(u);
        u.y += 1; // Case 1

        ua.x = u.x-1;
        ua.y = u.y;
        ub.x = u.x;
        ub.y = u.y-1;
        if (!occupied(u) && !occupied(ua) && !occupied(ub) )
                s.push_front(u);


        u.x -= 1;
        if (!occupied(u)) s.push_front(u);

        u.x -= 1; // Case 2
        ua.x = u.x+1;
        ua.y = u.y;
        ub.x = u.x;
        ub.y = u.y-1;
        if (!occupied(u) && !occupied(ua) && !occupied(ub))
                s.push_front(u);

        u.y -= 1;
        if (!occupied(u)) s.push_front(u);

        u.y -= 1; // Case 3
        ua.x = u.x;
        ua.y = u.y+1;
        ub.x = u.x+1;
        ub.y = u.y;
        if (!occupied(u) && !occupied(ua) && !occupied(ub))
                s.push_front(u);

        u.x += 1;
        if (!occupied(u)) s.push_front(u);

        u.x += 1; // Case 4
        ua.x = u.x;
        ua.y = u.y+1;
        ub.x = u.x-1;
        ub.y = u.y;
        if (!occupied(u) && !occupied(ua) && !occupied(ub))
                s.push_front(u);

}

/* void Dstar::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void Dstar::updateStart(int x, int y) {

        s_start.x = x;
        s_start.y = y;

        k_m += heuristic(s_last,s_start);

        s_start = calculateKey(s_start);
        s_last  = s_start;

}

/* void Dstar::updateGoal(int x, int y)
 * --------------------------
 * This is somewhat of a hack, to change the position of the goal we
 * first save all of the non-empty on the map, clear the map, move the
 * goal, and re-add all of non-empty cells. Since most of these cells
 * are not between the start and goal this does not seem to hurt
 * performance too much. Also it free's up a good deal of memory we
 * likely no longer use.
 */
void Dstar::updateGoal(int x, int y) {

        list< pair<ipoint2, double> > toAdd;
        pair<ipoint2, double> tp;

        ds_ch::iterator i;
        list< pair<ipoint2, double> >::iterator kk;

        for(i=cellHash.begin(); i!=cellHash.end(); i++) {
                if (!AreSame(i->second.cost, D)) {
                        tp.first.x = i->first.x;
                        tp.first.y = i->first.y;
                        tp.second = i->second.cost;
                        toAdd.push_back(tp);
                }
        }

        cellHash.clear();
        openHash.clear();

        while(!openList.empty())
                openList.pop();

        k_m = 0;

        s_goal.x  = x;
        s_goal.y  = y;

        NodeInfo tmp;
        tmp.g = tmp.rhs =  0;
        tmp.cost = D;

        cellHash[s_goal] = tmp;

        tmp.g = tmp.rhs = heuristic(s_start,s_goal);
        tmp.cost = D;
        cellHash[s_start] = tmp;
        s_start = calculateKey(s_start);

        s_last = s_start;

        for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
                updateCell(kk->first.x, kk->first.y, kk->second);
        }


}

/* bool Dstar::replan()
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 *  the metric euclidean(Node, goal) + euclidean(Node,start).
 */
bool Dstar::replan() {

        path.clear();

        int res = computeShortestPath();
        //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
        if (res < 0) {
                fprintf(stderr, "NO PATH TO GOAL\n");
                return false;
        }
        list<Node> n;
        list<Node>::iterator i;

        Node cur = s_start;

        if (std::isinf(getG(s_start))) {
                fprintf(stderr, "Start node is not valid, No path\n");
                return false;
        }

        while(cur != s_goal) {

                path.push_back(cur);
                getSucc(cur, n);

                if (n.empty()) {
                        fprintf(stderr, "Path is empty\n");
                        return false;
                }

                double cmin = INFINITY;
                double tmin;
                Node smin;

                for (i=n.begin(); i!=n.end(); i++) {

                        //if (occupied(*i)) continue;
                        double val  = cost(cur,*i);
                        double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i);   // (Euclidean) cost to goal + cost to pred
                        val += getG(*i);

                        if (AreSame(val,cmin)) {
                                if (tmin > val2) {
                                        tmin = val2;
                                        cmin = val;
                                        smin = *i;
                                }
                        } else if (val < cmin) {
                                tmin = val2;
                                cmin = val;
                                smin = *i;
                        }
                }
                n.clear();
                cur = smin;
        }
        path.push_back(s_goal);
        return true;
}
