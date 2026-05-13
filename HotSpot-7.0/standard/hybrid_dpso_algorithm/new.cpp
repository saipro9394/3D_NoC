#include <bits/stdc++.h>
#include <unistd.h>
#include "tinyxml2.h"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <tuple>
#include <algorithm>
#include <iomanip>

#include "thermal_simulator.h"

using namespace std;
using namespace tinyxml2;

/*
                                            IMPORVMENT HAVE TO BE DONE
                                    1. Runtime of the application formula have to change
                                    2. While you are doing FillGaps() are you checking the ware_of_constant
                                        (the fillGaps() I am talking about is which the function running before task mapping )
*/

#define SATURATION_THRESHOLD 1000
#define PIR 0.002//0.0005 //0.001 //1.0
int TEMP_THRESHOLD = 6000;
const int Gw = 8;
const int Gl = 8;
const int Gh = 4;
const int NUNITS = 30; // These units are represent the no of components of everynode which is used in the thermal simulation
int f = 0, s = 0;
int glbmark = 1;
int SEED = 42;

float ambient_temp = 25.0; // Ambient temperature in °C
float time_interval = 1.0; // Time interval for temperature
clock_t start, finish;

struct Application
{
    int id;
    int runtime;
    vector<int> tasks;
    vector<vector<int>> edges;
    vector<double> communicationVolume;
};
struct Core
{
    int isFree = 1;
    string task_no = "";
    int num_time_task = 0;
    int isnotBlocked = 1;
    int wareoff_const = 0;
    int x = -1, y = -1, z = -1;

    float temp;               // Current temperature of the core
    float thermal_capacity;   // Heat capacity (J/°C)
    float thermal_resistance; // Thermal resistance (°C/W)
    float power_dynamic;

    int time_of_death;
    void updateOcc(int freeness, string num, int blockedNess)
    {
        isFree = freeness;
        task_no = num;
        isnotBlocked = blockedNess;
    }
};
Core example;
Core mesh[Gw][Gl][Gh];
int strx = 0;
int stry = 0;
int strz = 0;
int removed_app_id;
int removed_app_death_time;
int total_sim_time = 0;
vector<vector<int>> emptySingleCores;
vector<pair<Core *, Core *>> brickVector;
vector<pair<int, int>> appsLoc;
map<int, vector<vector<int>>> task_timestamp;
int extra = 0;
vector<Application> apps;
vector<Application> ind_apps;
vector<bool> mapped;
set<pair<int, int>> mapped_apps_rt;

double edges_on_tsv = 0;
double avg_node_layer = 0;
double thermal_overhead_time = 0;

ofstream testTraffic("./mapping/test_traffic.txt", std::ios::out | std::ios::trunc);
ofstream reportValues("./mapping/hybrid_dpso_report_values.txt", std::ios::out | std::ios::trunc);
ofstream mapFile("./mapping/mapFile.txt");

/* IMPLEMENTATION OF: "Hybrid Optimization Algorithm Based on Double Particle Swarm in 3D NoC Mapping"
   PAPER REFERENCE: Micromachines 2023, 14, 628

   INTEGRATION GUIDE:
   1. Keep your existing struct definitions (Core, Application) and globals (mesh, Gw, Gl, Gh).
   2. Replace the 'pair_algorithm' and its helper functions with the code below.
   3. This implementation ignores 'brickVector' logic in favor of the paper's 'Region Division'.
*/

#include <random>
#include <cmath>
#include <limits>

// --- Constants from Paper ---
#define W_MAX 0.9
#define W_MIN 0.4
#define C1 2.0      // Cognitive coefficient
#define C2 2.0      // Social coefficient
#define ITER_MAX 25//50 // Max iterations for PSO
#define POP_SIZE 4//20 // Particle population size
#define DIVERSITY_THRESHOLD 0.1

// --- Helper Structs for PSO ---
struct Coordinate
{
    int x, y, z;
    int id_flat; // Flattened ID for quick indexing
};

struct Particle
{
    vector<int> task_to_node_map; // Index: Task ID, Value: Index in VG (Virtual Group)
    double fitness;
    vector<int> pbest_map;
    double pbest_fitness;
    // Velocity in discrete PSO is often represented as a swap sequence,
    // but here we simulate it via crossover/mutation probabilities derived from inertia.
};
double random_double();
int random_int(int min, int max);
int get_distance(Coordinate a, Coordinate b);
double get_thermal_resistance_proxy(Coordinate a, Coordinate b);
double calculate_fitness(const Application &app, const vector<int> &mapping, const vector<Coordinate> &VG);
vector<Coordinate> region_division(int required_nodes);
void apply_swaps(vector<int> &map, const vector<int> &target_map, const vector<vector<int>> &forbidden_table, double prob);
void mutate(vector<int> &map);
vector<int> random_mapping(int task_count, int vg_size);
double calculate_diversity(const vector<Particle> &pop);
int cnv_task_buf(string task_no);
bool is_forbidden(const vector<int> &map, const vector<vector<int>> &forbidden_table);
bool compareParticles(const Particle &a, const Particle &b);
void neighborhood_search(Particle &p, const Application &app, const vector<Coordinate> &VG);
vector<int> run_dpso(const Application &app, const vector<Coordinate> &VG);

// --- Global helper for random numbers ---
double random_double()
{
    return (double)rand() / RAND_MAX;
}

int random_int(int min, int max)
{
    return min + (rand() % (max - min + 1));
}

// --- COST FUNCTIONS (Equations 2-10 from Paper) ---

// Manhattan Distance in 3D
int get_distance(Coordinate a, Coordinate b)
{
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
}

// Thermal Resistance approximation (Eq 11)
// Paper implies R is proportional to distance and inversely to power,
// but often simplified to distance for "neighbor" calculations in heuristics.
// We use a simplified model: closer = lower thermal resistance impact.
double get_thermal_resistance_proxy(Coordinate a, Coordinate b)
{
    int dist = get_distance(a, b);
    if (dist == 0)
        return 0;
    return 1.0 / (double)dist;
}

// Calculate Fitness (Power + Delay + Temp Penalty)
// CONSTANTS for Energy Model (Normalized to 1.0 if specific Joules not provided)
#define E_ROUTER_BIT 1.0 // E'_Sbit
#define E_LINK_BIT 1.0   // E'_Lbit
#define MU_TSV 0.075     // µ factor (7.5% of horizontal link energy) [cite: 130]

// --- STRICT FITNESS FUNCTION (Includes Eqs 6, 9, 11, 12, 13) ---
double calculate_fitness(const Application &app, const vector<int> &mapping, const vector<Coordinate> &VG)
{
    double total_energy_comm = 0; // Eq 6
    double total_delay = 0;       // Eq 9
    double max_temp_rise = 0;     // Eq 13 (Peak Temp)

    int vg_size = VG.size();

    // 1. Calculate Communication Energy & Delay
    for (size_t e = 0; e < app.edges.size(); e++)
    {
        int t1 = app.edges[e][0];
        int t2 = app.edges[e][1];
        double comm_vol = app.communicationVolume[e];

        Coordinate n1 = VG[mapping[t1]];
        Coordinate n2 = VG[mapping[t2]];

        // Distances
        int N_x = abs(n1.x - n2.x);
        int N_y = abs(n1.y - n2.y);
        int N_z = abs(n1.z - n2.z);

        // Eq 4 & 5: Energy (Vertical TSV factor = 0.075)
        double E_Sbit = (N_x + N_y + N_z + 1) * 1.0;       // Router energy (no discount)
        double E_Lbit = (N_x + N_y + (N_z * 0.075)) * 1.0; // Link energy (TSV discount)
        total_energy_comm += comm_vol * (E_Sbit + E_Lbit);

        // Eq 9: Delay (Vertical delay is negligible)
        total_delay += comm_vol * (N_x + N_y);
    }

    // 2. Calculate Temperature (Eq 11 & 12)
    // We must check every node in the mapping to see how hot it gets.
    for (int t_i = 0; t_i < mapping.size(); t_i++)
    {
        double temp_i = 0;
        Coordinate node_i = VG[mapping[t_i]];

        // Sum heat from all other nodes 'j' acting on 'i'
        for (int t_j = 0; t_j < mapping.size(); t_j++)
        {
            if (t_i == t_j)
                continue; // Skip self

            Coordinate node_j = VG[mapping[t_j]];

            // Power of node j (Ej)
            // Using the task weight from your 'app.tasks' as the power proxy
            double E_j = (double)app.tasks[t_j];

            // Thermal Resistance Ri,j (Eq 11)
            // Modeled as Inverse Distance
            double dist = get_distance(node_i, node_j);
            double R_ij = (dist > 0) ? (1.0 / dist) : 0.0;

            // Eq 12: Ti += Ri,j * Ej
            temp_i += R_ij * E_j;
        }

        // Eq 13: Find Peak Temperature
        if (temp_i > max_temp_rise)
        {
            max_temp_rise = temp_i;
        }
    }

    // Final Fitness: Weighted Sum
    // We sum them to get a single minimization objective.
    // (You can tune these weights if one factor dominates too much,
    // but typically raw sum is the starting point in papers).
    return total_energy_comm + total_delay + max_temp_rise;
}
// --- ALGORITHM 1: Regional Division ---
// Selects a subset of nodes (VG) from the mesh that are best suited for this app.
vector<Coordinate> region_division(int required_nodes)
{
    vector<Coordinate> VG;
    vector<pair<float, Coordinate>> temp_list;

    // 1. Sort all available nodes by current Temperature (Line 1-3)
    for (int i = 0; i < Gh; i++)
    {
        for (int j = 0; j < Gl; j++)
        {
            for (int k = 0; k < Gw; k++)
            {
                if (mesh[k][j][i].isFree && mesh[k][j][i].isnotBlocked)
                {
                    // Flattened ID for reference
                    int flat = (i * Gw * Gl) + (j * Gw) + k;
                    temp_list.push_back({mesh[k][j][i].temp, {k, j, i, flat}});
                }
            }
        }
    }

    if (temp_list.empty())
        return VG; // No space

    // Sort ascending by temperature
    sort(temp_list.begin(), temp_list.end(), [](const auto &a, const auto &b)
         { return a.first < b.first; });

    // 2. Pick node with min Temp (Line 4-5)
    Coordinate center = temp_list[0].second;
    VG.push_back(center);

    // Helper to check if node is already in VG
    auto is_in_VG = [&](Coordinate c)
    {
        for (auto &v : VG)
            if (v.id_flat == c.id_flat)
                return true;
        return false;
    };

    // 3. Grow region until size == required_nodes (Line 6-31)
    while (VG.size() < required_nodes)
    {
        // Current expansion center
        Coordinate curr = VG.back();
        Coordinate best_candidate = {-1, -1, -1, -1};
        double best_score = 1e9; // Minimize distance/thermal cost

        // Search mechanism from paper:
        // Priority 1: Vertical neighbor (TSV)
        // Priority 2: Horizontal neighbor with min Manhattan distance to all nodes in VG

        // Let's iterate all free nodes to find the best candidate based on the paper's logic
        for (const auto &item : temp_list)
        {
            Coordinate cand = item.second;
            if (is_in_VG(cand))
                continue;

            // Check if it is a TSV neighbor of the current center (Line 7)
            bool is_tsv = (cand.x == curr.x && cand.y == curr.y && abs(cand.z - curr.z) == 1);

            double score = 0;
            if (is_tsv)
            {
                score = 0; // Highest priority (lowest score)
            }
            else
            {
                // Calculate Dist X and Dist Y to entire VG (Line 11-12)
                double dist_sum = 0;
                for (const auto &v : VG)
                {
                    dist_sum += get_distance(cand, v);
                }
                score = 100 + dist_sum; // Lower priority than TSV

                // Thermal resistance check (Line 13-15) implied in score if distances are equal
                // We add a small thermal bias
                score += (item.first * 0.1); // Add slight penalty for higher temp
            }

            if (score < best_score)
            {
                best_score = score;
                best_candidate = cand;
            }
        }

        if (best_candidate.id_flat != -1)
        {
            VG.push_back(best_candidate);
        }
        else
        {
            break; // Should not happen if enough nodes exist
        }
    }
    return VG;
}

// --- ALGORITHM 2: DPSO Operations ---

// Swap sequence operator (Paper definition of particle difference)
void apply_swaps(vector<int> &map, const vector<int> &target_map, const vector<vector<int>> &forbidden_table, double prob)
{
    vector<int> original_map = map; // Backup
    bool changed = false;

    for (size_t i = 0; i < map.size(); i++)
    {
        if (map[i] != target_map[i] && random_double() < prob)
        {
            int wanted = target_map[i];
            int swap_idx = -1;
            for (size_t j = 0; j < map.size(); j++)
            {
                if (map[j] == wanted)
                {
                    swap_idx = j;
                    break;
                }
            }
            if (swap_idx != -1)
            {
                swap(map[i], map[swap_idx]);
                changed = true;
            }
        }
    }

    // STRICT TABOO CHECK:
    // If the new position is forbidden, revert to original position.
    if (changed && is_forbidden(map, forbidden_table))
    {
        map = original_map;
    }
}
// Mutation (Gene Cross-Mutation)
void mutate(vector<int> &map)
{
    int n = map.size();
    if (n < 2)
        return;
    int i = random_int(0, n - 1);
    int j = random_int(0, n - 1);
    swap(map[i], map[j]);
}

// Generate valid random mapping
vector<int> random_mapping(int task_count, int vg_size)
{
    vector<int> map(vg_size);
    // Fill 0..vg_size-1
    for (int i = 0; i < vg_size; i++)
        map[i] = i;
    // Shuffle
    mt19937 gen(SEED);
    shuffle(map.begin(), map.end(), gen);
    // Resize to task_count (assuming task_count <= vg_size)
    map.resize(task_count);
    return map;
}

// Calculate diversity (simple Hamming distance average)
double calculate_diversity(const vector<Particle> &pop)
{
    if (pop.empty())
        return 0.0;
    double diff_sum = 0;
    int n = pop[0].task_to_node_map.size();
    int count = 0;
    for (size_t i = 0; i < pop.size(); i++)
    {
        for (size_t j = i + 1; j < pop.size(); j++)
        {
            for (int k = 0; k < n; k++)
            {
                if (pop[i].task_to_node_map[k] != pop[j].task_to_node_map[k])
                    diff_sum++;
            }
            count++;
        }
    }
    if (count == 0)
        return 0;
    return diff_sum / (double)(count * n);
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

// --- MAIN DPSO ALGORITHM (Replaces mapping logic) ---
// --- HELPER: Sort Comparator ---
// Check if a mapping exists in the forbidden table
bool is_forbidden(const vector<int> &map, const vector<vector<int>> &forbidden_table)
{
    for (const auto &f_map : forbidden_table)
    {
        if (map == f_map)
            return true;
    }
    return false;
}

bool compareParticles(const Particle &a, const Particle &b)
{
    return a.fitness < b.fitness; // Ascending (lower fitness is better)
}

// --- HELPER: Neighborhood Search (Line 21) ---
// Tries to improve a particle by swapping random tasks (Local Search)
void neighborhood_search(Particle &p, const Application &app, const vector<Coordinate> &VG)
{
    // "Deep" Search: Try multiple times to find a better neighbor
    int attempts = 0;
    int max_attempts = 10; // Prevent infinite loops

    while (attempts < max_attempts)
    {
        Particle neighbor = p;

        // Perform swap
        int n = neighbor.task_to_node_map.size();
        int i = random_int(0, n - 1);
        int j = random_int(0, n - 1);
        swap(neighbor.task_to_node_map[i], neighbor.task_to_node_map[j]);

        neighbor.fitness = calculate_fitness(app, neighbor.task_to_node_map, VG);

        if (neighbor.fitness < p.fitness)
        {
            p = neighbor; // Move to better spot
            attempts = 0; // Reset attempts to keep climbing
        }
        else
        {
            attempts++;
        }
    }
}
// --- MAIN ALGORITHM: DPSO (Strict Algorithm 2 Structure) ---
vector<int> run_dpso(const Application &app, const vector<Coordinate> &VG)
{
    int task_count = app.tasks.size();
    int vg_size = VG.size();

    // 1. Initialization (Lines 1-3)
    vector<Particle> pop1(POP_SIZE);
    vector<Particle> pop2(POP_SIZE);
    vector<vector<int>> forbidden_table;

    // Best particle tracked globally
    Particle gbest;
    gbest.fitness = numeric_limits<double>::max();

    // Init Pop1 & Pop2
    for (int i = 0; i < POP_SIZE; i++)
    {
        pop1[i].task_to_node_map = random_mapping(task_count, vg_size);
        pop1[i].fitness = calculate_fitness(app, pop1[i].task_to_node_map, VG);
        pop1[i].pbest_map = pop1[i].task_to_node_map;
        pop1[i].pbest_fitness = pop1[i].fitness;

        pop2[i].task_to_node_map = random_mapping(task_count, vg_size);
        pop2[i].fitness = calculate_fitness(app, pop2[i].task_to_node_map, VG);
        pop2[i].pbest_map = pop2[i].task_to_node_map;
        pop2[i].pbest_fitness = pop2[i].fitness;

        // Initial gbest check
        if (pop1[i].fitness < gbest.fitness)
            gbest = pop1[i];
        if (pop2[i].fitness < gbest.fitness)
            gbest = pop2[i];
    }

    double w = W_MAX;
    double prev_gbest_fitness = gbest.fitness;

    // --- MAIN LOOP (Line 4) ---
    for (int iter = 0; iter < ITER_MAX; iter++)
    {

        // --- PRE-CALCULATIONS for Eq 15 & 16 ---
        // Calculate 'd' (feedback factor)
        double improvement = (prev_gbest_fitness - gbest.fitness);
        double d = (prev_gbest_fitness > 0) ? (improvement / prev_gbest_fitness) : 0.0;
        if (d > 0.5)
            d = 0.5;
        if (d < 0.0)
            d = 0.0;
        prev_gbest_fitness = gbest.fitness;

        // Calculate Delta Omega (Eq 15)
        double term1 = (W_MAX - W_MIN) / (double)ITER_MAX;
        double term2 = (double)(ITER_MAX - iter) / (double)ITER_MAX;
        double term3 = exp(((double)iter / (double)ITER_MAX) - d);
        double delta_w = term1 * term2 * term3;

        // Update Inertia (Line 8)
        w = w - delta_w;
        if (w < W_MIN)
            w = W_MIN;
        double prob_move = (1.0 - w);

        // Find current bests of each population for Mutual Disturbance (Eq 17)
        // We need the "other population's optimal" (q_best)
        Particle pop1_best = pop1[0];
        for (auto &p : pop1)
            if (p.fitness < pop1_best.fitness)
                pop1_best = p;

        Particle pop2_best = pop2[0];
        for (auto &p : pop2)
            if (p.fitness < pop2_best.fitness)
                pop2_best = p;

        // --- STEP 9-11: UPDATE POSITIONS (Eq 17) ---
        // "Updates particle position according to Equation (17)"
        // "pop1[iter] <- pop1[iter-1]"
        // "pop2[iter] <- pop2[iter-1]"

        // Update Pop1
        for (int i = 0; i < POP_SIZE; i++)
        {
            // Cognitive (pbest) + Social (gbest)
            apply_swaps(pop1[i].task_to_node_map, pop1[i].pbest_map, forbidden_table, prob_move);
            apply_swaps(pop1[i].task_to_node_map, gbest.task_to_node_map, forbidden_table, prob_move);
            // Mutual Disturbance: Pop1 learns from Pop2's best [cite: 337]
            apply_swaps(pop1[i].task_to_node_map, pop2_best.task_to_node_map, forbidden_table, prob_move);

            // Evaluate & Update Local Best
            pop1[i].fitness = calculate_fitness(app, pop1[i].task_to_node_map, VG);
            if (pop1[i].fitness < pop1[i].pbest_fitness)
            {
                pop1[i].pbest_fitness = pop1[i].fitness;
                pop1[i].pbest_map = pop1[i].task_to_node_map;
            }
            // Update Global Best
            if (pop1[i].fitness < gbest.fitness)
                gbest = pop1[i];
        }

        // Update Pop2
        for (int i = 0; i < POP_SIZE; i++)
        {
            // Cognitive + Social
            apply_swaps(pop2[i].task_to_node_map, pop2[i].pbest_map, forbidden_table, prob_move);
            apply_swaps(pop2[i].task_to_node_map, gbest.task_to_node_map, forbidden_table, prob_move);
            // Mutual Disturbance: Pop2 learns from Pop1's best
            apply_swaps(pop2[i].task_to_node_map, pop1_best.task_to_node_map, forbidden_table, prob_move);

            // Evaluate & Update Local Best
            pop2[i].fitness = calculate_fitness(app, pop2[i].task_to_node_map, VG);
            if (pop2[i].fitness < pop2[i].pbest_fitness)
            {
                pop2[i].pbest_fitness = pop2[i].fitness;
                pop2[i].pbest_map = pop2[i].task_to_node_map;
            }
            if (pop2[i].fitness < gbest.fitness)
                gbest = pop2[i];
        }

        // --- STEP 12-17: MUTATION PROBABILITY (Pop2 Only) ---
        // "Updates mutation probability... For pop in pop2 do..."
        double mutation_prob = ((double)(ITER_MAX - iter) / ITER_MAX) * d; // Eq 16

        for (int i = 0; i < POP_SIZE; i++)
        {
            // "If Equation (16) then crossover..." (Line 14-15)
            if (random_double() < mutation_prob)
            {
                mutate(pop2[i].task_to_node_map);

                // Re-evaluate after mutation
                pop2[i].fitness = calculate_fitness(app, pop2[i].task_to_node_map, VG);
                if (pop2[i].fitness < pop2[i].pbest_fitness)
                {
                    pop2[i].pbest_fitness = pop2[i].fitness;
                    pop2[i].pbest_map = pop2[i].task_to_node_map;
                }
                if (pop2[i].fitness < gbest.fitness)
                    gbest = pop2[i];
            }
        }

        // --- STEP 18: SORT POP1 ---
        // "sorts pop1[iter] according to fitness1[iter]"
        sort(pop1.begin(), pop1.end(), compareParticles);

        // --- STEP 19-22: DIVERSITY CHECK & FORBIDDEN LIST ---
        // "If particles diversity < threshold then"
        if (calculate_diversity(pop1) < DIVERSITY_THRESHOLD)
        {

            // Line 20: "ForbiddenList <- pop1[iter][0]"
            forbidden_table.push_back(pop1[0].task_to_node_map);

            // Line 21: "pop2[iter][0] searches neighborhood"
            // Note: Paper strictly says pop2 searches here, even if pop1 is the one checked for diversity.
            // We optimize pop2[0] (the best of pop2, since we assume pop2 is also implicitly sorted or we just pick index 0)
            // To be safe, let's find the best of pop2 first since we didn't sort pop2
            int best_p2_idx = 0;
            for (int i = 1; i < POP_SIZE; i++)
            {
                if (pop2[i].fitness < pop2[best_p2_idx].fitness)
                    best_p2_idx = i;
            }
            neighborhood_search(pop2[best_p2_idx], app, VG);

            // Check if this search found a new global best
            if (pop2[best_p2_idx].fitness < gbest.fitness)
            {
                gbest = pop2[best_p2_idx];
            }

            // Line 22: "pbest <- pop1[iter][1]"
            // This implies we reset the swarm's focus to the *second best* particle
            // because the first one is now forbidden/stagnated.
            // In a practical PSO implementation, we can update the global best (gbest) to this second best
            // to steer particles away from the forbidden one.
            gbest = pop1[1];
        }
    }

    return gbest.task_to_node_map;
}

string shape = to_string(Gw) + "x" + to_string(Gl) + "x" + to_string(Gh);
void thermal_initiation()
{
    const vector<string> args = {
        "./a.out", "-c", "../NOC/" + shape + "/hotspot.config", "-init_file", "../NOC/" + shape + "/avg.init", "-p",
        "../NOC/" + shape + "/new_core3D.ptrace", "-grid_layer_file", "../NOC/" + shape + "/NoC_layer.lcf", "-model_type",
        "grid", "-detailed_3D", "on", "-o", "../NOC/" + shape + "/avg.ttrace",
        "-grid_transient_file", "../NOC/" + shape + "/avg.grid.ttrace", "-grid_map_mode", "avg"};
    std::vector<char *> argv;
    for (const auto &arg : args)
    {
        // We use const_cast because argv expects `char*`, not `const char*`.
        // This is safe if program_logic doesn't modify the arguments.
        argv.push_back(const_cast<char *>(arg.c_str()));
    }
    argv.push_back(nullptr); // Null-terminate the argv array
    initiation(argv.size() - 1, argv.data());
}
void free_mesh()
{
    // pair<int, int> min_element = *mapped_apps_rt.begin();
    // mapped_apps_rt.erase(mapped_apps_rt.begin());
    pair<int, int> min_element = {INT_MAX, 0};
    for (int i = 0; i < Gh; i++)
    {
        for (int j = 0; j < Gl; j++)
        {
            for (int k = 0; k < Gw; k++)
            {
                if (mesh[k][j][i].task_no != "")
                {
                    if (min_element.first > mesh[k][j][i].time_of_death)
                    {
                        std::string int_part = mesh[k][j][i].task_no.substr(0, mesh[k][j][i].task_no.find('.'));
                        int app_id = stoi(int_part) - 1;
                        // min_element.first = mesh[k][j][i].time_of_death
                        min_element.first = mesh[k][j][i].time_of_death;
                        min_element.second = app_id;
                    }
                }
            }
        }
    }
    // cout << "app " << min_element.second << " " << min_element.first << endl;
    if (min_element.first != INT_MAX)
        total_sim_time = min_element.first;
    for (int i = 0; i < Gh; i++)
    {
        for (int j = 0; j < Gl; j++)
        {
            for (int k = 0; k < Gw; k++)
            {
                if (mesh[k][j][i].task_no != "")
                {
                    std::string int_part = mesh[k][j][i].task_no.substr(0, mesh[k][j][i].task_no.find('.'));
                    int app_id = stoi(int_part) - 1;
                    if (min_element.second == app_id)
                    {
                        removed_app_id = app_id;
                        removed_app_death_time = total_sim_time;
                        mesh[k][j][i].isFree = 1;
                        mesh[k][j][i].task_no = "";
                    }
                }
            }
        }
    }
}

void thermal_simulation(int *act)
{
    simulation(act);
}
void thermal_termination()
{
    stop();
}
void get_active(int *act)
{
    for (int i = 0; i < Gh; i++)
    {
        for (int j = 0; j < Gl; j++)
        {
            for (int k = 0; k < Gw; k++)
            {
                int mf = 1;
                if (mesh[k][j][i].isFree)
                    mf = 0;
                for (int id = 0; id < NUNITS; id++)
                    act[(i * Gw * Gl) + (j * Gw) + k + id] = mf;
            }
        }
    }
}
void Thermal_block()
{
    int *act;
    act = (int *)malloc(n * sizeof(int));
    if (act == NULL)
    {
        std::cerr << "unable to allocate memory for 'act' array\n";
    }
    get_active(act);

    auto t_sim_start = chrono::high_resolution_clock::now();

    thermal_simulation(act);

    auto t_sim_end = chrono::high_resolution_clock::now();
    chrono::duration<double> t_sim_diff = t_sim_end - t_sim_start;
    thermal_overhead_time += t_sim_diff.count();

    free(act);

    for (int i = 0; i < Gh; i++)
    {
        for (int j = 0; j < Gl; j++)
        {
            for (int k = 0; k < Gw; k++)
            {
                mesh[k][j][i].temp = get_max_grid_temperature(i * 2, k, j);
                if (mesh[k][j][i].temp >= TEMP_THRESHOLD)
                    mesh[k][j][i].isnotBlocked = 0;
                else
                    mesh[k][j][i].isnotBlocked = 1;
            }
        }
    }
}
double calc_runtime(Application app)
{
    double time = 0;
    for (int j = 0; j < app.tasks.size(); j++)
    {
        time += app.tasks[j];
    }
    return time;
}
// --- REPLACEMENT FOR pair_algorithm ---
void pair_algorithm()
{
    int cnt = 0;
    cout << apps.size() << " apps\n";

    // Iterate through all applications
    while (cnt < apps.size())
    {
        // 0. Update Thermal State (User's existing function)
        free_mesh(); // Clear dead tasks
        Thermal_block();
        for (int i = 0; i < apps.size(); i++)
        {
            if (!mapped[i])
            {
                // 1. ALGORITHM 1: Region Division
                // We need as many nodes as there are tasks in the app
                int required_nodes = apps[i].tasks.size();
                vector<Coordinate> VG = region_division(required_nodes);

                if (VG.size() < required_nodes)
                {
                    // cout << "Error: Not enough free nodes for App " << apps[i].id << endl;
                    continue; // Skip or try later
                }

                // 2. ALGORITHM 2: DPSO Mapping
                vector<int> final_mapping = run_dpso(apps[i], VG);

                // 3. Apply Mapping to Mesh (User's Core structure)
                // final_mapping[t] = index in VG
                for (int t = 0; t < final_mapping.size(); t++)
                {
                    int vg_idx = final_mapping[t];
                    Coordinate c = VG[vg_idx];

                    // Update the mesh Core
                    Core *core = &mesh[c.x][c.y][c.z];

                    core->task_no = to_string(apps[i].id) + "." + to_string(t);
                    core->isFree = 0;
                    core->wareoff_const += apps[i].tasks[t];

                    // Set lifetime (Dynamic Mapping)
                    // Using total_sim_time + runtime as death time
                    core->time_of_death = total_sim_time + apps[i].runtime;

                    // Update user's timestamp tracking
                    int buf = cnv_task_buf(core->task_no);
                    int flat_idx = c.x + (c.y * Gw) + (c.z * Gw * Gl);
                    task_timestamp[buf].push_back({0, flat_idx, total_sim_time, core->time_of_death});
                    avg_node_layer += c.z;
                }

                mapped[i] = true;
                cnt++;
                cout << "App " << apps[i].id << " Mapped Successfully." << endl;
            }
        }

        // Final Print of Mesh State (User's style)
        for (int i = 0; i < Gh; i++)
        {
            cout << "layer " << i + 1 << endl;
            for (int j = 0; j < Gl; j++)
            {
                for (int k = 0; k < Gw; k++)
                {
                    if (mesh[k][j][i].task_no == "")
                        cout << " #   ";
                    else
                        cout << mesh[k][j][i].task_no << "  ";
                }
                cout << endl;
            }
            cout << endl;
        }
    }
}
void initiate()
{
    for (int i = 0; i < Gh; i++)
    {
        for (int j = 0; j < Gl; j++)
        {
            for (int k = 0; k < Gw; k++)
            {
                mesh[k][j][i].x = k, mesh[k][j][i].y = j, mesh[k][j][i].z = i;
            }
        }
    }
}

// XML PARSING UTILS
int extractValue(const string &str)
{
    size_t start = str.find('(');
    size_t end = str.find(')', start);
    return (start != string::npos && end != string::npos && end > start) ? stoi(str.substr(start + 1, end - start - 1)) : 0;
}
int extractNodeIndex(const string &title)
{
    size_t underscore = title.find('_');
    return (underscore != string::npos) ? stoi(title.substr(underscore + 1)) : -1;
}
int extractGraphId(const string &title)
{
    size_t underscore = title.find('_');
    return (title.size() >= 2 && underscore != string::npos) ? stoi(title.substr(1, underscore - 1)) : 0;
}
int graphsUpdating(int argc, char *argv[])
{
    if (argc < 2)
    {
        cerr << "Usage error" << endl;
        return 1;
    }
    XMLDocument doc;
    if (doc.LoadFile(argv[1]) != XML_SUCCESS)
        return 1;

    XMLElement *graphElem = doc.FirstChildElement("graph");
    if (!graphElem)
        return 1;

    map<int, vector<pair<int, int>>> graphNodes;
    XMLElement *nodesElem = graphElem->FirstChildElement("nodes");

    if (nodesElem)
    {
        for (XMLElement *nodeElem = nodesElem->FirstChildElement("node"); nodeElem; nodeElem = nodeElem->NextSiblingElement("node"))
        {
            const char *t = nodeElem->Attribute("title");
            const char *l = nodeElem->Attribute("label");
            if (t && l)
                graphNodes[extractGraphId(t)].push_back({extractNodeIndex(t), extractValue(l)});
        }
    }

    map<int, vector<tuple<int, int, int>>> graphEdges;
    XMLElement *edgesElem = graphElem->FirstChildElement("edges");
    if (edgesElem)
    {
        for (XMLElement *e = edgesElem->FirstChildElement("edge"); e; e = e->NextSiblingElement("edge"))
        {
            const char *s = e->Attribute("sourcename");
            const char *t = e->Attribute("targetname");
            const char *l = e->Attribute("label");
            if (s && t && l)
                graphEdges[extractGraphId(s)].push_back(make_tuple(extractNodeIndex(s), extractNodeIndex(t), extractValue(l)));
        }
    }

    // int a = 0; // Removed ind_apps logic var
    for (auto &kv : graphNodes)
    {
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
        if (graphEdges.count(kv.first))
        {
            for (auto &t : graphEdges[kv.first])
            {
                int s, tr, w;
                tie(s, tr, w) = t;
                if (reindex.count(s) && reindex.count(tr))
                {
                    edgesVec.push_back({reindex[s], reindex[tr]});
                    commVol.push_back(w); // Keeping original logic
                }
            }
        }

        Application app;
        app.id = kv.first + 1;
        app.tasks = tasks;
        app.edges = edgesVec;
        app.communicationVolume = commVol;
        app.runtime = calc_runtime(app);
        apps.push_back(app);
    }
    mapped.resize(apps.size(), false);
    return 1;
}

int main(int argc, char *argv[])
{
    // int tot_nodes = Gw * Gl * Gh;
    // if (tot_nodes <= 64)
    //     TEMP_THRESHOLD = 1500;
    // else if (tot_nodes <= 256)
    //     TEMP_THRESHOLD = 2000;
    // else if (tot_nodes <= 512)
    //     TEMP_THRESHOLD = 4500;
    // else
    //     TEMP_THRESHOLD = 5500;
    // 1. Parsing & Initialization
    // Parses the XML to create 'apps'. Keep strict.
    graphsUpdating(argc, argv);

    // Sets up x,y,z coordinates in the mesh objects. Essential.
    initiate();

    // NOTE: 'loadNodes()' and 'extra' calculation are REMOVED.
    // The new DPSO algorithm maps directly to mesh coordinates,
    // so we do not need to pre-allocate 'bricks' or calculate 'extra' offsets.

    // 2. Thermal Setup
    thermal_initiation();

    auto start = chrono::high_resolution_clock::now();

    // 3. The New Algorithm (DPSO)
    // This replaces the old logic. It uses the globals 'apps' and 'mesh' directly.
    pair_algorithm();

    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> diff = end - start;
    double algorithm_time_only = diff.count() - thermal_overhead_time;

    // 4. Logging & Verification
    // Print the raw timestamp map for debugging
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

    // Generate Traffic File for Post-Processing
    for (int i = 0; i < apps.size(); i++)
    {
        // CORRECTION: Standardized the offset.
        // Original code had "(apps[i].id - 2)". This is fragile.
        // Standard mapping "AppID.TaskID" corresponds to "AppID * 10000 + TaskID".
        int buf = apps[i].id * 10000;

        int noof_changes = task_timestamp[buf].size(); // Uses buf if tasks are 0-indexed, or buf+task_id if map is specific

        // Iterate edges to log communication traffic
        for (int j = 0; j < apps[i].edges.size(); j++)
        {
            // Calculate key for the map (AppID*10000 + TaskID)
            int first_task = buf + apps[i].edges[j][0];
            int second_task = buf + apps[i].edges[j][1];

            // Note: This nested loop assumes your 'task_timestamp' map uses
            // the exact key 'first_task'. If your vector_list logic changes, this needs update.
            // Currently, it matches the structure of 'task_timestamp' maintained in 'pair_algorithm'.

            // We need to find the specific valid entries in the vector_list.
            // Using a simple check to ensure keys exist to prevent segfaults:
            if (task_timestamp.count(first_task) && task_timestamp.count(second_task))
            {
                auto &list1 = task_timestamp[first_task];
                auto &list2 = task_timestamp[second_task];

                // Using the logic from your original file:
                // It matches the k-th interval of task1 with the l-th interval of task2
                for (int k = 0; k < list1.size(); k++)
                {
                    for (int l = 0; l < list2.size(); l++)
                    {
                        // Your specific condition: checking if timestamps overlap or align?
                        // Preserving original logic: checking index alignment [0]
                        if (list2[l][0] == k)
                        {
                            if (abs(list1[k][1] - list2[l][1]) == Gw * Gl)
                                edges_on_tsv++;
                            testTraffic << list1[k][1] << " "
                                        << list2[l][1] << " "
                                        << PIR * apps[i].communicationVolume[j] << " "
                                        << PIR * apps[i].communicationVolume[j] << " "
                                        << list1[k][2] << " "
                                        << list1[k][3] << endl;
                            break;
                        }
                    }
                }
            }
        }
    }
    reportValues << avg_node_layer << " " << edges_on_tsv << endl;
    double total_nodes = 0, total_edges = 0;
    for (auto app : apps)
    {
        total_nodes += app.tasks.size();
        total_edges += app.edges.size();
    }
    avg_node_layer /= total_nodes;
    edges_on_tsv /= total_edges;

    reportValues << "Avg node layer: " << avg_node_layer << endl;
    reportValues << "Avg Edges on tsv: " << edges_on_tsv << endl;
    reportValues << "\nRunning time: " << algorithm_time_only << " seconds" << endl;
    thermal_termination();

    return 0;
}