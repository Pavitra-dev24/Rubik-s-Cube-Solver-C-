#include <bits/stdc++.h>
using namespace std;

struct Vec3 { int x,y,z; };
struct Vec3d { double x,y,z; };

double PI = acos(-1.0);

Vec3d crossd(const Vec3d &a, const Vec3d &b) {
    Vec3d r; r.x = a.y*b.z - a.z*b.y; r.y = a.z*b.x - a.x*b.z; r.z = a.x*b.y - a.y*b.x; return r;
}
double dotd(const Vec3d &a, const Vec3d &b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
Vec3 rotate_about_axis(const Vec3 &v_in, const Vec3 &axis_in, double angle_rad) {
    Vec3d v{(double)v_in.x,(double)v_in.y,(double)v_in.z};
    Vec3d k{(double)axis_in.x,(double)axis_in.y,(double)axis_in.z};
    double klen = sqrt(k.x*k.x + k.y*k.y + k.z*k.z);
    if (klen == 0) return v_in;
    k.x /= klen; k.y /= klen; k.z /= klen;
    double c = cos(angle_rad);
    double s = sin(angle_rad);
    Vec3d term1{v.x*c, v.y*c, v.z*c};
    Vec3d kxd = crossd(k, v);
    Vec3d term2{kxd.x * s, kxd.y * s, kxd.z * s};
    double kdotv = dotd(k, v);
    Vec3d term3{ k.x * kdotv * (1-c), k.y * kdotv * (1-c), k.z * kdotv * (1-c) };
    Vec3d res{ term1.x + term2.x + term3.x, term1.y + term2.y + term3.y, term1.z + term2.z + term3.z };
    auto roundi = [](double x)->int {
        if (x > 0.5) return 1;
        if (x < -0.5) return -1;
        return 0;
    };
    return Vec3{ roundi(res.x), roundi(res.y), roundi(res.z) };
}

void state_to_key_chars(int s[54], char key[55]) {
    for (int i = 0; i < 54; ++i) key[i] = char('0' + s[i]);
    key[54] = '\0';
}

void apply_permutation_int(const int src[54], int dest[54], int perm[54]) {
    for (int i = 0; i < 54; ++i) dest[i] = src[perm[i]];
}

int heuristic_mismatch_int(const int s[54], const int goal[54]) {
    int mis = 0;
    for (int i = 0; i < 54; ++i) if (s[i] != goal[i]) ++mis;
    return mis;
}

int start_state[54];
int goal_state[54];
Vec3 positions[54];
int perms_arr[12][54];
char moveNames[12][4];

int iddfs_limit_default = 12;
long long nodes_expanded_iddfs = 0;
int iddfs_solution_moves[1000];
int iddfs_solution_len = 0;

bool iddfs_dfs_rec(int curr[54], int depth, int limit, int prev_move, int path[], int path_len, int goal_st[54]) {
    ++nodes_expanded_iddfs;
    bool equal = true;
    for (int i = 0; i < 54; ++i) if (curr[i] != goal_st[i]) { equal = false; break; }
    if (equal) {
        iddfs_solution_len = path_len;
        for (int i = 0; i < path_len; ++i) iddfs_solution_moves[i] = path[i];
        return true;
    }
    if (depth == limit) return false;
    for (int mi = 0; mi < 12; ++mi) {
        if (prev_move != -1 && ((mi ^ 1) == prev_move)) continue;
        int nexts[54];
        apply_permutation_int(curr, nexts, perms_arr[mi]);
        path[path_len] = mi;
        if (iddfs_dfs_rec(nexts, depth + 1, limit, mi, path, path_len + 1, goal_st)) return true;
    }
    return false;
}

bool run_iddfs_only(int start_st[54], int goal_st[54], int max_limit) {
    nodes_expanded_iddfs = 0;
    iddfs_solution_len = 0;
    int path[1000];
    for (int limit = 0; limit <= max_limit; ++limit) {
        if (iddfs_dfs_rec(start_st, 0, limit, -1, path, 0, goal_st)) return true;
    }
    return false;
}

struct Node {
    int state[54];
    int g;
    int h;
    int f;
    int parent;
    int move;
};

int POOL_CAP = 500000;
Node *pool_nodes;
int pool_size;

int HASH_SIZE = 200003;
char (*ht_key)[55];
int *ht_index;

int heap_cap = 600000;
int *heap_idx;
int *heap_f;
int *heap_h;
int heap_size = 0;

int pool_create_node_and_insert_ht(const char key_in[55], int state_in[54]) {
    if (pool_size >= POOL_CAP) return -1;
    int nid = pool_size++;
    for (int i = 0; i < 54; ++i) pool_nodes[nid].state[i] = state_in[i];
    pool_nodes[nid].g = 0; pool_nodes[nid].h = 0; pool_nodes[nid].f = 0; pool_nodes[nid].parent = -1; pool_nodes[nid].move = -1;
    int h = 0;
    for (int i = 0; i < 54; ++i) { h = (h * 7 + (key_in[i] - '0' + 1)) % HASH_SIZE; }
    int idx = h;
    while (true) {
        if (ht_index[idx] == -1) {
            strcpy(ht_key[idx], key_in);
            ht_index[idx] = nid;
            break;
        } else if (strcmp(ht_key[idx], key_in) == 0) {
            ht_index[idx] = nid;
            break;
        } else {
            idx = idx + 1;
            if (idx == HASH_SIZE) idx = 0;
        }
    }
    return nid;
}

int ht_find(const char key_in[55]) {
    int h = 0;
    for (int i = 0; i < 54; ++i) { h = (h * 7 + (key_in[i] - '0' + 1)) % HASH_SIZE; }
    int idx = h;
    while (true) {
        if (ht_index[idx] == -1) return -1;
        if (strcmp(ht_key[idx], key_in) == 0) return ht_index[idx];
        idx = idx + 1;
        if (idx == HASH_SIZE) idx = 0;
    }
    return -1;
}

void heap_swap(int a, int b) {
    int ta = heap_idx[a], tb = heap_idx[b];
    int fa = heap_f[a], fb = heap_f[b];
    int ha = heap_h[a], hb = heap_h[b];
    heap_idx[a] = tb; heap_f[a] = fb; heap_h[a] = hb;
    heap_idx[b] = ta; heap_f[b] = fa; heap_h[b] = ha;
}

void heap_push(int node_index) {
    int fval = pool_nodes[node_index].f;
    int hval = pool_nodes[node_index].h;
    int i = heap_size++;
    heap_idx[i] = node_index; heap_f[i] = fval; heap_h[i] = hval;
    int cur = i;
    while (cur > 0) {
        int p = (cur - 1) / 2;
        if (heap_f[p] > heap_f[cur] || (heap_f[p] == heap_f[cur] && heap_h[p] > heap_h[cur])) {
            heap_swap(p, cur);
            cur = p;
        } else break;
    }
}

int heap_pop_valid_or_minus1() {
    while (heap_size > 0) {
        int top_idx = heap_idx[0];
        int top_f = heap_f[0];
        int top_h = heap_h[0];
        int pool_f = pool_nodes[top_idx].f;
        int pool_h = pool_nodes[top_idx].h;
        heap_idx[0] = heap_idx[--heap_size];
        heap_f[0] = heap_f[heap_size];
        heap_h[0] = heap_h[heap_size];
        int cur = 0;
        while (true) {
            int l = cur * 2 + 1;
            int r = cur * 2 + 2;
            int best = cur;
            if (l < heap_size && (heap_f[l] < heap_f[best] || (heap_f[l] == heap_f[best] && heap_h[l] < heap_h[best]))) best = l;
            if (r < heap_size && (heap_f[r] < heap_f[best] || (heap_f[r] == heap_f[best] && heap_h[r] < heap_h[best]))) best = r;
            if (best != cur) { heap_swap(cur, best); cur = best; } else break;
        }
        if (pool_f == top_f && pool_h == top_h) return top_idx;
    }
    return -1;
}

long long nodes_expanded_astar = 0;
int astar_solution_moves[10000];
int astar_solution_len = 0;

bool run_astar_only(int start_st[54], int goal_st[54]) {
    pool_nodes = (Node*)malloc(sizeof(Node) * POOL_CAP);
    ht_key = (char(*)[55])malloc(sizeof(char[55]) * HASH_SIZE);
    ht_index = (int*)malloc(sizeof(int) * HASH_SIZE);
    heap_idx = (int*)malloc(sizeof(int) * heap_cap);
    heap_f = (int*)malloc(sizeof(int) * heap_cap);
    heap_h = (int*)malloc(sizeof(int) * heap_cap);
    for (int i = 0; i < HASH_SIZE; ++i) { ht_index[i] = -1; ht_key[i][0] = '\0'; }
    pool_size = 0;
    heap_size = 0;
    nodes_expanded_astar = 0;
    char key_start[55], key_goal[55];
    state_to_key_chars(start_st, key_start);
    state_to_key_chars(goal_st, key_goal);
    int nid = pool_create_node_and_insert_ht(key_start, start_st);
    if (nid == -1) return false;
    pool_nodes[nid].g = 0;
    pool_nodes[nid].h = heuristic_mismatch_int(start_st, goal_st);
    pool_nodes[nid].f = pool_nodes[nid].g + pool_nodes[nid].h;
    pool_nodes[nid].parent = -1;
    pool_nodes[nid].move = -1;
    heap_push(nid);
    int max_iters = 5000000;
    int iters = 0;
    while (true) {
        int cur_idx = heap_pop_valid_or_minus1();
        if (cur_idx == -1) break;
        ++nodes_expanded_astar;
        bool is_goal = true;
        for (int i = 0; i < 54; ++i) if (pool_nodes[cur_idx].state[i] != goal_st[i]) { is_goal = false; break; }
        if (is_goal) {
            vector<int> rev;
            int x = cur_idx;
            while (x != -1 && pool_nodes[x].move != -1) {
                rev.push_back(pool_nodes[x].move);
                x = pool_nodes[x].parent;
            }
            astar_solution_len = rev.size();
            for (int i = 0; i < astar_solution_len; ++i) astar_solution_moves[i] = rev[astar_solution_len - 1 - i];
            free(pool_nodes); free(ht_key); free(ht_index); free(heap_idx); free(heap_f); free(heap_h);
            return true;
        }
        if (++iters > max_iters) break;
        for (int mi = 0; mi < 12; ++mi) {
            int succ[54];
            apply_permutation_int(pool_nodes[cur_idx].state, succ, perms_arr[mi]);
            char skey[55]; state_to_key_chars(succ, skey);
            int found = ht_find(skey);
            int gnew = pool_nodes[cur_idx].g + 1;
            if (found == -1) {
                int newnid = pool_create_node_and_insert_ht(skey, succ);
                if (newnid == -1) continue;
                pool_nodes[newnid].g = gnew;
                pool_nodes[newnid].h = heuristic_mismatch_int(succ, goal_st);
                pool_nodes[newnid].f = pool_nodes[newnid].g + pool_nodes[newnid].h;
                pool_nodes[newnid].parent = cur_idx;
                pool_nodes[newnid].move = mi;
                heap_push(newnid);
            } else {
                if (gnew < pool_nodes[found].g) {
                    pool_nodes[found].g = gnew;
                    pool_nodes[found].h = heuristic_mismatch_int(succ, goal_st);
                    pool_nodes[found].f = pool_nodes[found].g + pool_nodes[found].h;
                    pool_nodes[found].parent = cur_idx;
                    pool_nodes[found].move = mi;
                    heap_push(found);
                }
            }
        }
    }
    free(pool_nodes); free(ht_key); free(ht_index); free(heap_idx); free(heap_f); free(heap_h);
    return false;
}

int find_position_index(const Vec3 &v) {
    for (int i = 0; i < 54; ++i) {
        if (positions[i].x == v.x && positions[i].y == v.y && positions[i].z == v.z) return i;
    }
    return -1;
}

int main(int argc, char** argv) {
    string filename = "";
    bool only_iddfs = false;
    bool only_astar = false;
    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        if (a == "--id") only_iddfs = true;
        else if (a == "--astar") only_astar = true;
        else filename = a;
    }
    if (only_iddfs && only_astar) { printf("Cannot specify both --id and --astar\n"); return 1; }
    if (filename.size() > 0) {
        ifstream ifs(filename.c_str());
        if (!ifs) { printf("Cannot open file\n"); return 1; }
        for (int i = 0; i < 54; ++i) ifs >> start_state[i];
        for (int i = 0; i < 54; ++i) ifs >> goal_state[i];
        ifs.close();
    } else {
        for (int i = 0; i < 54; ++i) if (!(cin >> start_state[i])) { printf("Expected 54 numbers for initial state\n"); return 1; }
        for (int i = 0; i < 54; ++i) if (!(cin >> goal_state[i])) { printf("Expected 54 numbers for goal state\n"); return 1; }
    }

    Vec3 normals[6] = { {0,0,1}, {0,0,-1}, {0,1,0}, {0,-1,0}, {-1,0,0}, {1,0,0} };
    Vec3 rights[6] = { {1,0,0}, {-1,0,0}, {1,0,0}, {1,0,0}, {0,0,-1}, {0,0,1} };
    Vec3 ups[6] = { {0,1,0}, {0,1,0}, {0,0,-1}, {0,0,1}, {0,1,0}, {0,1,0} };
    int idx = 0;
    int order_v[3] = {1,0,-1};
    int order_u[3] = {-1,0,1};
    for (int f = 0; f < 6; ++f) {
        Vec3 center = normals[f];
        Vec3 r = rights[f];
        Vec3 u = ups[f];
        for (int iv = 0; iv < 3; ++iv) {
            for (int iu = 0; iu < 3; ++iu) {
                int vy = order_v[iv];
                int ux = order_u[iu];
                Vec3 p;
                p.x = center.x + ux * r.x + vy * u.x;
                p.y = center.y + ux * r.y + vy * u.y;
                p.z = center.z + ux * r.z + vy * u.z;
                positions[idx++] = p;
            }
        }
    }

    for (int f = 0; f < 6; ++f) {
        for (int i = 0; i < 54; ++i) perms_arr[f*2 + 0][i] = i;
        for (int i = 0; i < 54; ++i) perms_arr[f*2 + 1][i] = i;
        Vec3 axis = normals[f];
        int layer_idxs[54];
        int layer_count = 0;
        for (int i = 0; i < 54; ++i) {
            int dot = positions[i].x * axis.x + positions[i].y * axis.y + positions[i].z * axis.z;
            if (dot == 1) { layer_idxs[layer_count++] = i; }
        }
        double ang_cw = -PI/2.0;
        double ang_ccw = PI/2.0;
        for (int j = 0; j < layer_count; ++j) {
            int idx_old = layer_idxs[j];
            Vec3 newp_cw = rotate_about_axis(positions[idx_old], axis, ang_cw);
            Vec3 newp_ccw = rotate_about_axis(positions[idx_old], axis, ang_ccw);
            int idx_cw = find_position_index(newp_cw);
            int idx_ccw = find_position_index(newp_ccw);
            if (idx_cw == -1 || idx_ccw == -1) { printf("Rotation mapping error\n"); return 1; }
            perms_arr[f*2 + 0][idx_cw] = idx_old;
            perms_arr[f*2 + 1][idx_ccw] = idx_old;
        }
    }

    const char *faceNames[6] = {"F","B","U","D","L","R"};
    for (int f = 0; f < 6; ++f) {
        strcpy(moveNames[f*2 + 0], faceNames[f]);
        strcpy(moveNames[f*2 + 1], (string(faceNames[f]) + "'").c_str());
    }

    if (!only_astar) {
        bool ok = run_iddfs_only(start_state, goal_state, iddfs_limit_default);
        if (ok) {
            printf("IDDFS: solution found\nMoves (%d): ", iddfs_solution_len);
            for (int i = 0; i < iddfs_solution_len; ++i) {
                if (i) printf(" ");
                printf("%s", moveNames[iddfs_solution_moves[i]]);
            }
            printf("\n");
        } else {
            printf("IDDFS: no solution found up to depth %d\n", iddfs_limit_default);
        }
        printf("IDDFS: nodes expanded = %lld\n\n", nodes_expanded_iddfs);
    }

    if (!only_iddfs) {
        bool ok2 = run_astar_only(start_state, goal_state);
        if (ok2) {
            printf("A*: solution found\nMoves (%d): ", astar_solution_len);
            for (int i = 0; i < astar_solution_len; ++i) {
                if (i) printf(" ");
                printf("%s", moveNames[astar_solution_moves[i]]);
            }
            printf("\n");
        } else {
            printf("A*: no solution found (or aborted)\n");
        }
        printf("A*: nodes expanded = %lld\n\n", nodes_expanded_astar);
    }

    return 0;
}
