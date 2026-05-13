#include <bits/stdc++.h>
#include <unistd.h>
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;
#define PIR 0.002//0.0005 //0.001 //1.0

const int Gw = 8;
const int Gl = 8;
const int Gh = 4;

const double sigmastar_const = 1.15;
const int task_multiplyer = 10000;
int glbmark = 1;
int mnt = 0;

const int BEAM_WIDTH = 1;

double edges_on_tsv = 0;
double avg_node_layer = 0;

ofstream testTraffic("./mapping/test_traffic.txt", std::ios::out | std::ios::trunc);
ofstream reportValues("./mapping/onruntime_report_values.txt", std::ios::out | std::ios::trunc);

struct Application
{
    int id;
    vector<int> tasks;
    vector<vector<int>> edges;
    vector<double> commVolume;
    int NOL = -1;
    int MD = -1;
    double avgCommValue;
    int xmin = INT_MAX, xmax = INT_MIN, ymin = INT_MAX, ymax = INT_MIN;
    int startX, startY, startZ;
    int placed = 0;
    int run_time = 0;
    void computeAvg()
    {
        if (!commVolume.empty())
        {
            avgCommValue = accumulate(commVolume.begin(), commVolume.end(), 0.0) / commVolume.size();
        }
        else
        {
            avgCommValue = 0.0; // Handle empty vector case
        }
    }
    void computeRuntime()
    {
        if (run_time == 0)
            for (auto x : tasks)
                run_time += x;
    }
};

struct Core
{
    int isFree = -1;
    int x, y, z;
    int timeofdeath = 0;
    int num_time_task = 0;
    string task_no = "";
};

Core NoC[Gw][Gl][Gh];
int Pc[Gw][Gl][Gh];

int removed_app_death_time = 0;
int total_sim_time = 0;
std::vector<int> freecores((int)Gh, (int)(Gw * Gl));
using AppIntPair = pair<vector<Application *>, vector<int>>;
vector<Application> tapps;
unordered_map<int, int> task_to_core;
map<int, vector<vector<int>>> task_timestamp;

double ERT(int a, double v, int nol, int md)
{
    double c0 = 1.0, c1 = 2.0, c2 = 2.0, c3 = -1.0, c4 = -1.0;
    double ans = (double)c0 + ((double)c1 * a) + ((double)c2 * v) + ((double)c3 * nol) + ((double)c4 * md);
    return ans;
}

double lowerbound_ert(Application app)
{
    int max_task_per_layer = Gw * Gl;
    int nol = (int)(ceil((double)(app.tasks.size() / (double)max_task_per_layer)));
    // Note: Removed +1 to be consistent with ceil logic,
    // but ensured nol is at least 1 and within bounds.
    if (nol == 0)
        nol = 1;

    if (nol > Gh)
    {
        return -1.0;
    }
    int md = 0;
    double ert = ERT(app.tasks.size(), app.avgCommValue, nol, md);
    return ert;
}

double computeSigmaStar(const std::vector<Application> &apps)
{
    double sigmaStar = 0;
    for (const auto app : apps)
    {
        double ert = lowerbound_ert(app);
        if (ert == -1.0)
            continue;
        sigmaStar += ert;
    }
    return sigmaStar;
}

int cnv_task_buf(string task_no)
{
    int buf = 0;
    stringstream ss(task_no);
    string buff_t;
    int divd = 10000;
    while (getline(ss, buff_t, '.'))
        buf += stoi(buff_t) * divd, divd /= 10000;
    return buf;
}

void remove_prev_mapped_tod(std::pair<int, std::vector<int>> prefix_to_remove)
{
    const std::vector<int> &prefix_vec = prefix_to_remove.second;

    if (prefix_vec.empty())
        return;
    auto &vec_to_modify = task_timestamp[prefix_to_remove.first];

    for (auto it = vec_to_modify.begin(); it != vec_to_modify.end(); ++it)
    {
        const std::vector<int> &x = *it;

        bool vec_is_long_enough = (x.size() >= prefix_vec.size());
        bool prefix_matches = false;
        if (vec_is_long_enough)
        {
            prefix_matches = std::equal(prefix_vec.begin(), prefix_vec.end(), x.begin());
        }
        if (prefix_matches)
        {
            int jump = 0;
            if (x.size() > 2)
                jump = x[2];
            std::vector<int> new_vec = {prefix_vec[0], prefix_vec[1], jump, removed_app_death_time};
            *it = new_vec;
            break;
        }
    }
}

void update_mvtasks(Core *node1, Core *node2)
{
    int jump = removed_app_death_time;
    int buf = cnv_task_buf(node1->task_no);
    remove_prev_mapped_tod({buf, {node1->num_time_task, node1->x + (node1->y * Gw) + (node1->z * Gw * Gl)}});
    node2->num_time_task = node1->num_time_task + 1;
    task_timestamp[buf].push_back({node2->num_time_task, node2->x + (node2->y * Gw) + (node2->z * Gw * Gl), jump, node1->timeofdeath});

    node2->timeofdeath = node1->timeofdeath;
    node1->timeofdeath = removed_app_death_time;
}

// --- OPTIMIZED FIND CORE REGION SHAPE (BEAM SEARCH) ---

struct SearchNode
{
    int app_idx;
    double accumulated_cost;
    double estimated_total_cost;
    std::vector<std::pair<int, int>> decisions;
    std::vector<int> current_free_cores;
};

AppIntPair reconstructResult(const std::vector<Application *> &original_apps, const SearchNode &bestNode)
{
    AppIntPair result;
    result.second = bestNode.current_free_cores;
    for (size_t i = 0; i < original_apps.size(); i++)
    {
        Application *newApp = new Application(*original_apps[i]);
        if (i < bestNode.decisions.size())
        {
            newApp->MD = bestNode.decisions[i].first;
            newApp->NOL = bestNode.decisions[i].second;
        }
        result.first.push_back(newApp);
    }
    return result;
}

AppIntPair findCoreRegionShape(std::vector<Application *> &apps)
{
    // 1. Pre-calculate Lower Bounds
    std::vector<double> lower_bounds(apps.size());
    double initial_lb_sum = 0;

    for (int i = 0; i < apps.size(); i++)
    {
        apps[i]->computeAvg();
        lower_bounds[i] = lowerbound_ert(*apps[i]);
        if (lower_bounds[i] != -1.0)
            initial_lb_sum += lower_bounds[i];
    }

    // 2. Initialize Queue
    std::queue<SearchNode> WQ;
    SearchNode root;
    root.app_idx = 0;
    root.accumulated_cost = 0;
    root.estimated_total_cost = initial_lb_sum;
    root.current_free_cores = freecores;
    WQ.push(root);

    double sigmaStar = initial_lb_sum * sigmastar_const;
    SearchNode bestNode = root;
    bool solutionFound = false;

    // 3. Main Search Loop
    while (!WQ.empty())
    {
        SearchNode current = WQ.front();
        WQ.pop();

        if (current.estimated_total_cost > sigmaStar)
            continue;

        if (current.app_idx >= apps.size())
        {
            if (current.accumulated_cost < sigmaStar)
            {
                sigmaStar = current.accumulated_cost;
                bestNode = current;
                solutionFound = true;
            }
            continue;
        }

        int idx = current.app_idx;
        Application *currApp = apps[idx];
        int tasks_count = currApp->tasks.size();

        int nolmin = (tasks_count + (Gw * Gl) - 1) / (Gw * Gl);
        if (nolmin == 0)
            nolmin = 1;
        int nolmax = std::min((int)Gh, tasks_count);
        int mdmax = (int)Gh - nolmin;

        struct Candidate
        {
            int md, nol;
            double cost;
            std::vector<int> updated_cores;
        };
        std::vector<Candidate> candidates;

        for (int nol = nolmin; nol <= nolmax; nol++)
        {
            int current_md_max = (int)Gh - nol;
            for (int md = 0; md <= current_md_max; md++)
            {

                int avgcores = (tasks_count + nol - 1) / nol;
                bool feasible = true;

                if (md + nol > Gh)
                    continue;

                for (int k = md; k < md + nol; k++)
                {
                    if (current.current_free_cores[k] < avgcores)
                    {
                        feasible = false;
                        break;
                    }
                }
                if (!feasible)
                    continue;

                Candidate cand;
                cand.md = md;
                cand.nol = nol;
                cand.updated_cores = current.current_free_cores;

                int temp_tasks = tasks_count;
                for (int k = md; k < md + nol; k++)
                {
                    int consume = std::min(avgcores, temp_tasks);
                    cand.updated_cores[k] -= consume;
                    temp_tasks -= consume;
                }

                double this_ert = ERT(tasks_count, currApp->avgCommValue, nol, md);
                double future_lb = current.estimated_total_cost - current.accumulated_cost - lower_bounds[idx];
                cand.cost = current.accumulated_cost + this_ert + future_lb;

                if (cand.cost <= sigmaStar)
                {
                    candidates.push_back(cand);
                }
            }
        }

        std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b)
                  { return a.cost < b.cost; });

        for (size_t k = 0; k < candidates.size() && k < BEAM_WIDTH; k++)
        {
            SearchNode nextNode;
            nextNode.app_idx = current.app_idx + 1;
            nextNode.accumulated_cost = current.accumulated_cost + ERT(tasks_count, currApp->avgCommValue, candidates[k].nol, candidates[k].md);
            nextNode.estimated_total_cost = candidates[k].cost;
            nextNode.current_free_cores = candidates[k].updated_cores;
            nextNode.decisions = current.decisions;
            nextNode.decisions.push_back({candidates[k].md, candidates[k].nol});
            WQ.push(nextNode);
        }
    }

    if (!solutionFound && bestNode.decisions.empty())
    {
        // Warning: No feasible mapping found
    }

    return reconstructResult(apps, bestNode);
}

// --- PLACEMENT AND MIGRATION ---

bool findCoreRegionLocation(Application &app, int cornerIndex)
{
    int width = ceil(sqrt(app.tasks.size() / (double)app.NOL));
    int length = ceil(sqrt(app.tasks.size() / (double)app.NOL));

    int startX, startY, startZ, dirX, dirY;
    if (cornerIndex % 4 == 0)
    {
        startX = 0;
        startY = 0;
        startZ = app.MD;
        dirX = 1;
        dirY = 1;
    }
    else if (cornerIndex % 4 == 1)
    {
        startX = Gw - 1;
        startY = 0;
        startZ = app.MD;
        dirX = -1;
        dirY = 1;
    }
    else if (cornerIndex % 4 == 2)
    {
        startX = Gw - 1;
        startY = Gl - 1;
        startZ = app.MD;
        dirX = -1;
        dirY = -1;
    }
    else
    {
        startX = 0;
        startY = Gl - 1;
        startZ = app.MD;
        dirX = 1;
        dirY = -1;
    }

    int available = 0;
    bool inmark = false;
    for (int k = startZ; k <= Gh - app.NOL; k++)
    {
        for (int y = startY; (dirY > 0 ? y < Gl : y >= 0); y += dirY)
        {
            for (int x = startX; (dirX > 0 ? x < Gw : x >= 0); x += dirX)
            {
                available = 0;
                for (int z = k; z < k + app.NOL; z++)
                {
                    for (int dy = 0; dy < length; ++dy)
                    {
                        for (int dx = 0; dx < width; ++dx)
                        {
                            int nx = x + dx * dirX;
                            int ny = y + dy * dirY;
                            if (nx >= 0 && nx < Gw && ny >= 0 && ny < Gl && NoC[nx][ny][z].isFree == -1)
                            {
                                available++;
                            }
                        }
                    }
                }
                if (available >= app.tasks.size())
                {
                    inmark = true;
                    int assigned = 0;
                    for (int z = k; z < k + app.NOL && assigned < app.tasks.size(); z++)
                    {
                        for (int dy = 0; dy < length && assigned < app.tasks.size(); dy++)
                        {
                            for (int dx = 0; dx < width && assigned < app.tasks.size(); dx++)
                            {
                                int nx = x + dx * dirX;
                                int ny = y + dy * dirY;
                                if (nx >= 0 && nx < Gw && ny >= 0 && ny < Gl && NoC[nx][ny][z].isFree == -1)
                                {
                                    NoC[nx][ny][z].isFree = app.id;
                                    NoC[nx][ny][z].task_no = to_string(app.id) + "." + to_string(assigned);
                                    NoC[nx][ny][z].num_time_task = 0;
                                    int jump = total_sim_time;
                                    NoC[nx][ny][z].timeofdeath = total_sim_time + app.run_time;
                                    int buf = cnv_task_buf(NoC[nx][ny][z].task_no);
                                    task_timestamp[buf].push_back({0, z * (Gw * Gl) + ny * (Gw) + nx, jump, NoC[nx][ny][z].timeofdeath});
                                    avg_node_layer += z;

                                    app.xmax = max(app.xmax, nx);
                                    app.xmin = min(app.xmin, nx);
                                    app.ymax = max(app.ymax, ny);
                                    app.ymin = min(app.ymin, ny);
                                    assigned++;
                                }
                            }
                        }
                    }
                    app.startX = startX;
                    app.startY = startY;
                    app.startZ = k;
                    break;
                }
            }
            if (inmark)
                break;
        }
        if (inmark)
            break;
    }
    return inmark;
}

vector<Core> lineFreeCoreCount(Core current, char direction)
{
    vector<Core> freeCores;
    int dx = 0, dy = 0;
    if (direction == 'X')
        dx = 1;
    else if (direction == 'Y')
        dy = 1;
    if (current.isFree == -1)
        freeCores.push_back(current);
    for (int i = 1; i < max(Gw, Gl); ++i)
    {
        int nx = current.x + i * dx;
        int ny = current.y + i * dy;
        if (nx >= 0 && nx < Gw && ny >= 0 && ny < Gl && NoC[nx][ny][current.z].isFree == -1)
            freeCores.push_back(NoC[nx][ny][current.z]);
        else
            break;
    }
    for (int i = 1; i < max(Gw, Gl); ++i)
    {
        int nx = current.x - i * dx;
        int ny = current.y - i * dy;
        if (nx >= 0 && nx < Gw && ny >= 0 && ny < Gl && NoC[nx][ny][current.z].isFree == -1)
            freeCores.push_back(NoC[nx][ny][current.z]);
        else
            break;
    }
    return freeCores;
}

int calculateCenterFreeCores()
{
    int centerFreeCores = 0;
    for (int z = 0; z < Gh; ++z)
    {
        Core centerCore = NoC[Gw / 2][Gl / 2][z];
        vector<Core> xFreeCores = lineFreeCoreCount(centerCore, 'X');
        for (auto &core : xFreeCores)
        {
            vector<Core> yFreeCores = lineFreeCoreCount(core, 'Y');
            centerFreeCores += yFreeCores.size();
        }
    }
    return centerFreeCores;
}

pair<pair<int, int>, pair<int, int>> findVirtualMigrationPaths(Application &app, int dx, int dy)
{
    pair<int, int> pathXY, pathYX;
    int xpos, ypos;
    if (dx == -1)
    {
        xpos = app.xmin;
    }
    else if (dx == 1)
    {
        xpos = app.xmax;
    }
    if (dy == -1)
    {
        ypos = app.ymin;
    }
    else if (dy == 1)
    {
        ypos = app.ymax;
    }
    int mark = 0;
    int tempxXY = 0, tempyXY = 0;
    while (1)
    {
        xpos += dx;
        for (int z = app.startZ; z < app.startZ + app.NOL; z++)
        {
            int inner = 0;
            for (int y = app.ymin; y <= app.ymax; y++)
            {
                if (NoC[xpos][y][z].isFree != -1 || y < 0 || y >= Gl || xpos >= Gw || xpos <= -1)
                {
                    mark = 1;
                    inner = 1;
                    break;
                }
            }
            if (inner == 1)
                break;
        }
        if (mark == 1 || xpos >= Gw || xpos <= -1)
            break;
        tempxXY++;
    }
    mark = 0;
    while (1)
    {
        ypos += dy;
        for (int z = app.startZ; z < app.startZ + app.NOL; z++)
        {
            int inner = 0;
            for (int x = xpos - dx; x <= xpos - dx + (app.xmax - app.xmin); x++)
            {
                if (NoC[x][ypos][z].isFree != -1 || x < 0 || x >= Gw || ypos >= Gl || ypos <= -1)
                {
                    mark = 1;
                    inner = 1;
                    break;
                }
            }
            if (inner == 1)
                break;
        }
        if (mark == 1 || ypos >= Gl || ypos <= -1)
            break;
        tempyXY++;
    }
    pathXY.first = tempxXY;
    pathXY.second = tempyXY;

    xpos, ypos;
    if (dx == -1)
    {
        xpos = app.xmin;
    }
    else if (dx == 1)
    {
        xpos = app.xmax;
    }
    if (dy == -1)
    {
        ypos = app.ymin;
    }
    else if (dy == 1)
    {
        ypos = app.ymax;
    }

    int tempxYX = 0, tempyYX = 0;
    mark = 0;
    while (1)
    {
        ypos += dy;
        for (int z = app.startZ; z < app.startZ + app.NOL; z++)
        {
            int inner = 0;
            for (int x = app.xmin; x <= xpos - dx + (app.xmax - app.xmin); x++)
            {
                if (NoC[x][ypos][z].isFree != -1 || x < 0 || x >= Gw || ypos >= Gl || ypos <= -1)
                {
                    mark = 1;
                    inner = 1;
                    break;
                }
            }
            if (inner == 1)
                break;
        }
        if (mark == 1 || ypos >= Gl || ypos <= -1)
            break;
        tempyYX++;
    }
    mark = 0;
    while (1)
    {
        xpos += dx;
        for (int z = app.startZ; z < app.startZ + app.NOL; z++)
        {
            int inner = 0;
            for (int y = ypos - dy; y <= ypos - dy + (app.ymax - app.ymin); y++)
            {
                if (NoC[xpos][y][z].isFree != -1 || y < 0 || y >= Gl || xpos >= Gw || xpos <= -1)
                {
                    mark = 1;
                    inner = 1;
                    break;
                }
            }
            if (inner == 1)
                break;
        }
        if (mark == 1 || xpos >= Gw || xpos <= -1)
            break;
        tempxYX++;
    }
    pathYX.first = tempxYX;
    pathYX.second = tempyYX;

    return {pathXY, pathYX};
}

void migrateApplication(Application &app, pair<int, int> path, pair<int, int> dir)
{
    int startZ = app.startZ;
    for (int z = startZ; z < startZ + app.NOL; z++)
    {
        for (int y = app.ymax; y >= app.ymin; y--)
        {
            for (int x = app.xmax; x >= app.xmin; x--)
            {
                int newx = x + (dir.first * path.first), newy = y + (dir.second * path.second);
                if ((newx != x || newy != y) && NoC[x][y][z].isFree == app.id)
                {
                    NoC[x][y][z].isFree = -1;
                    NoC[newx][newy][z].isFree = app.id;
                    update_mvtasks(&NoC[x][y][z], &NoC[newx][newy][z]);
                }
            }
        }
    }
}

bool compare(const Application *a, const Application *b)
{
    return a->tasks.size() > b->tasks.size();
}

void defragmentation(vector<Application *> &apps)
{
    double F = 1.0 - (double)calculateCenterFreeCores() / (Gw * Gl * Gh);
    const double FTH = 0.01;

    if (F > FTH)
    {
        // sort(apps.begin(), apps.end(), compare);
        for (auto &app : apps)
        {
            if (app->placed != 0)
            {
                int x = 0, y = 0;
                int RE = Gw - (app->xmax + 1);
                int RW = (app->xmin + 1);
                int RN = Gl - (app->ymax + 1);
                int RS = (app->ymin + 1);
                if (RN >= RS && RE >= RW)
                {
                    x = -1, y = -1;
                }
                else if (RN < RS && RE >= RW)
                {
                    x = -1, y = 1;
                }
                else if (RN >= RS && RE < RW)
                {
                    x = 1, y = -1;
                }
                else if (RN < RS && RE < RW)
                {
                    x = 1, y = 1;
                }

                pair<pair<int, int>, pair<int, int>> paths = findVirtualMigrationPaths(*app, x, y);
                pair<int, int> path;
                if (paths.first.first + paths.first.second > paths.second.first + paths.second.second)
                    path = paths.first;
                else
                    path = paths.second;

                migrateApplication(*app, path, {x, y});
            }
        }
    }
}

int compute_layer_freecores()
{
    int buf = 0;
    for (int i = 0; i < freecores.size(); i++)
        buf += freecores[i];
    return buf;
}

// --- XML PARSING ---

int extractValue(const string &str)
{
    size_t start = str.find('(');
    size_t end = str.find(')', start);
    if (start != string::npos && end != string::npos && end > start)
        return stoi(str.substr(start + 1, end - start - 1));
    return 0;
}
int extractNodeIndex(const string &title)
{
    size_t underscore = title.find('_');
    if (underscore != string::npos && underscore + 1 < title.size())
        return stoi(title.substr(underscore + 1));
    return -1;
}
int extractGraphId(const string &title)
{
    if (title.size() < 2)
        return 0;
    size_t underscore = title.find('_');
    if (underscore != string::npos)
        return stoi(title.substr(1, underscore - 1));
    return 0;
}
int graphsUpdating(int argc, char *argv[])
{
    if (argc < 2)
    {
        cerr << "Usage: " << argv[0] << " input.xml" << endl;
        return 1;
    }

    const char *filename = argv[1];
    XMLDocument doc;
    if (doc.LoadFile(filename) != XML_SUCCESS)
    {
        cerr << "Error loading XML: " << filename << endl;
        return 1;
    }

    XMLElement *graphElem = doc.FirstChildElement("graph");
    if (!graphElem)
        return 1;

    map<int, vector<pair<int, int>>> graphNodes;
    XMLElement *nodesElem = graphElem->FirstChildElement("nodes");
    if (nodesElem)
    {
        for (XMLElement *nodeElem = nodesElem->FirstChildElement("node"); nodeElem; nodeElem = nodeElem->NextSiblingElement("node"))
        {
            const char *title = nodeElem->Attribute("title");
            const char *label = nodeElem->Attribute("label");
            if (title && label)
            {
                int gid = extractGraphId(title);
                int idx = extractNodeIndex(title);
                int value = extractValue(label);
                graphNodes[gid].push_back({idx, value});
            }
        }
    }

    map<int, vector<tuple<int, int, int>>> graphEdges;
    XMLElement *edgesElem = graphElem->FirstChildElement("edges");
    if (edgesElem)
    {
        for (XMLElement *edgeElem = edgesElem->FirstChildElement("edge"); edgeElem; edgeElem = edgeElem->NextSiblingElement("edge"))
        {
            const char *src = edgeElem->Attribute("sourcename");
            const char *tgt = edgeElem->Attribute("targetname");
            const char *label = edgeElem->Attribute("label");
            if (src && tgt && label)
            {
                int gid = extractGraphId(src);
                int s = extractNodeIndex(src);
                int t = extractNodeIndex(tgt);
                int weight = extractValue(label);
                graphEdges[gid].push_back(make_tuple(s, t, weight));
            }
        }
    }

    for (auto &kv : graphNodes)
    {
        int gid = kv.first;
        vector<pair<int, int>> &nodesVec = kv.second;
        sort(nodesVec.begin(), nodesVec.end(), [](const pair<int, int> &a, const pair<int, int> &b)
             { return a.first < b.first; });

        map<int, int> reindex;
        vector<int> tasks;
        for (size_t i = 0; i < nodesVec.size(); i++)
        {
            reindex[nodesVec[i].first] = i;
            tasks.push_back(nodesVec[i].second);
        }

        vector<vector<int>> edgesVec;
        vector<double> commVol;
        if (graphEdges.find(gid) != graphEdges.end())
        {
            for (auto &t : graphEdges[gid])
            {
                int origS, origT, weight;
                tie(origS, origT, weight) = t;
                if (reindex.find(origS) != reindex.end() && reindex.find(origT) != reindex.end())
                {
                    edgesVec.push_back({reindex[origS], reindex[origT]});
                    commVol.push_back(weight);
                }
            }
        }

        Application app;
        app.id = gid + 1;
        app.tasks = tasks;
        app.edges = edgesVec;
        app.commVolume = commVol;
        tapps.push_back(app);
    }
    return 1;
}

// --- MAIN ---

int main(int argc, char *argv[])
{
    graphsUpdating(argc, argv);
    for (int i = 0; i < Gh; i++)
        for (int j = 0; j < Gw; j++)
            for (int k = 0; k < Gl; k++)
            {
                Pc[j][k][i] = 1;
                NoC[j][k][i].x = j;
                NoC[j][k][i].y = k;
                NoC[j][k][i].z = i;
            }

    vector<int> app_runtimes(1);
    for (auto &app : tapps)
    {
        app.computeAvg();
        app.computeRuntime();
        app_runtimes.push_back(app.run_time);
        cout << app.run_time << " " << app.id << "th runtime" << endl;
    }

    map<int, Application *> application_map;
    for (int i = 0; i < tapps.size(); i++)
        application_map[tapps[i].id] = &tapps[i];

    vector<Application *> active_apps;

    int count = 0, prev_count = count;
    bool application_removed_marker = false;
    int prev_removed_app_min_max_id = -1, prev_removed_app_min_max_RT = INT_MIN;
    auto start = chrono::high_resolution_clock::now();

    while (count < tapps.size())
    {
        vector<Application *> apps;
        int avaliablecores = compute_layer_freecores();

        int sz = 0;
        for (int i = 0; i < tapps.size(); i++)
        {
            if (!tapps[i].placed && sz + tapps[i].tasks.size() < avaliablecores)
            {
                sz += tapps[i].tasks.size();
                apps.push_back(&tapps[i]);
            }
        }
        cout << apps.size() << " apps selected" << endl;

        // 1. OPTIMIZATION: FIND SHAPE
        AppIntPair regionShape = findCoreRegionShape(apps);

        // 2. APPLY RESULTS (COPY BACK TO ORIGINALS & CLEAN UP)
        // We do this to avoid using the copies in the main loop, preventing memory leaks
        // and keeping all state in the global 'tapps' vector.
        for (size_t i = 0; i < regionShape.first.size(); i++)
        {
            if (i < apps.size())
            {
                apps[i]->MD = regionShape.first[i]->MD;
                apps[i]->NOL = regionShape.first[i]->NOL;
            }
            delete regionShape.first[i]; // Delete temp copy
        }
        freecores = regionShape.second;

        cout << "Dimensions" << endl;
        for (auto *app : apps)
            cout << app->id << " " << app->MD << " " << app->NOL << endl;

        // 3. PLACEMENT
        for (int i = 0; i < Gh; i++)
        {
            int cornerIndex = 0;
            for (auto *app : apps)
            {
                if (app->MD == i && !app->placed)
                {
                    bool possible = true;
                    for (int x = 1; x <= 4; x++)
                    {
                        possible = findCoreRegionLocation(*app, cornerIndex % 4);
                        if (possible)
                            break;
                        cornerIndex++;
                    }
                    if (possible)
                    {
                        app->placed = 1;
                    }
                    else
                    {
                        app->placed = 0;
                    }
                    cornerIndex++;
                }
            }
        }

        // 4. UPDATE ACTIVE LIST
        for (auto *app : apps)
        {
            if (app->placed)
            {
                // Ensure we don't add duplicates if logic repeats for same app
                bool exists = false;
                for (auto *exist : active_apps)
                    if (exist->id == app->id)
                        exists = true;

                if (!exists)
                {
                    application_map[app->id]->placed = 1;
                    active_apps.push_back(app);
                    count++;
                }
            }
        }

        // 5. SIMULATION / TIME ADVANCE
        if (count > prev_count)
        {
            prev_count = count;
            application_removed_marker = false;
        }
        else
        {
            int minRT_applicaiton_id = -1, minRT = INT_MAX;
            for (int z = 0; z < Gh; ++z)
                for (int y = 0; y < Gl; ++y)
                    for (int x = 0; x < Gw; ++x)
                        if (NoC[x][y][z].isFree != -1)
                        {
                            if (minRT > NoC[x][y][z].timeofdeath)
                            {
                                minRT_applicaiton_id = NoC[x][y][z].isFree;
                                minRT = NoC[x][y][z].timeofdeath; // Correct: Use timeofdeath, not runtime duration
                                // minRT = app_rminRT = NoC[x][y][z].timeofdeath; // Correct: Use timeofdeath, not runtime durationuntimes[minRT_applicaiton_id]; // Check this index access (1-based vs 0-based)
                            }
                        }

            if (minRT_applicaiton_id == -1)
                break; // Gridlock or done

            if (!application_removed_marker)
            {
                prev_removed_app_min_max_id = minRT_applicaiton_id;
                prev_removed_app_min_max_RT = minRT;
                // total_sim_time += minRT;
                total_sim_time = minRT;
            }
            else
            {
                // total_sim_time -= prev_removed_app_min_max_RT;
                // total_sim_time += max(prev_removed_app_min_max_RT, minRT);
                if (minRT > total_sim_time)
                    total_sim_time = minRT;
                if (prev_removed_app_min_max_RT < minRT)
                {
                    prev_removed_app_min_max_RT = minRT;
                    prev_removed_app_min_max_id = minRT_applicaiton_id;
                }
            }
            removed_app_death_time = total_sim_time;

            for (int z = 0; z < Gh; ++z)
            {
                int going_to_be_free = 0;
                for (int y = 0; y < Gl; ++y)
                {
                    for (int x = 0; x < Gw; ++x)
                    {
                        if (NoC[x][y][z].isFree == minRT_applicaiton_id)
                            NoC[x][y][z].isFree = -1;
                        if (NoC[x][y][z].isFree == -1)
                            going_to_be_free++;
                    }
                }
                freecores[z] = going_to_be_free;
            }

            for (int i = 0; i < active_apps.size(); i++)
            {
                if (active_apps[i]->id == minRT_applicaiton_id)
                {
                    active_apps.erase(active_apps.begin() + i);
                    break;
                }
            }
            application_removed_marker = true;
        }

        // 6. DEFRAGMENTATION
        defragmentation(active_apps);

        mnt++;
        // if (mnt == 50)
        //     break;
    }

    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> diff = end - start;

    // --- REPORTING ---
    for (auto &[buf, vector_list] : task_timestamp)
    {
        for (auto x : vector_list)
        {
            cout << buf << " ";
            for (auto y : x)
                cout << y << " ";
            cout << endl;
        }
    }
    for (int i = 0; i < tapps.size(); i++)
    {
        int buf = (tapps[i].id) * 10000;

        // FIX: Do not calculate a global 'noof_changes' based on just the base task.
        // Different tasks might have slightly different history lengths if something went wrong
        // or during specific migration scenarios.

        for (int j = 0; j < tapps[i].edges.size(); j++)
        {
            int first_task = buf + tapps[i].edges[j][0];
            int second_task = buf + tapps[i].edges[j][1];

            // Safety Check: Ensure both tasks exist in the map before accessing
            if (task_timestamp.find(first_task) == task_timestamp.end() ||
                task_timestamp.find(second_task) == task_timestamp.end())
            {
                continue;
            }

            // Access the specific history vectors for these two tasks
            const auto &vec_first = task_timestamp[first_task];
            const auto &vec_second = task_timestamp[second_task];

            // FIX: Iterate using the actual size of the FIRST task's vector
            for (size_t k = 0; k < vec_first.size(); k++)
            {
                // FIX: Iterate using the actual size of the SECOND task's vector
                for (size_t l = 0; l < vec_second.size(); l++)
                {
                    // Check if the sequence ID of the second task matches the loop index 'k'
                    // (Matches the logic: task_timestamp[second_task][l][0] == k)
                    if (vec_second[l][0] == k)
                    {
                        // Use safe vector access (vec_first[k] and vec_second[l])
                        if (abs(vec_first[k][1] - vec_second[l][1]) == Gw * Gl)
                            edges_on_tsv++;

                        if (vec_first[k][2] != vec_first[k][3])
                        {
                            testTraffic << vec_first[k][1] << " "
                                        << vec_second[l][1] << " "
                                        << PIR * tapps[i].commVolume[j] << " "
                                        << PIR * tapps[i].commVolume[j] << " "
                                        << vec_first[k][2] << " "
                                        << vec_first[k][3] << endl;
                        }
                        break;
                    }
                }
            }
        }
    }
    double total_nodes = 0, total_edges = 0;
    for (auto app : tapps)
    {
        total_nodes += app.tasks.size();
        total_edges += app.edges.size();
    }
    avg_node_layer /= total_nodes;
    edges_on_tsv /= total_edges;

    reportValues << "Avg node layer: " << avg_node_layer << endl;
    reportValues << "Avg Edges on tsv: " << edges_on_tsv << endl;
    reportValues << "\nRunning time: " << diff.count() << " seconds" << endl;

    return 0;
}