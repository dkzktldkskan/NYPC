#pragma GCC optimize("O3")
#pragma GCC optimize("unroll-loops")
#pragma GCC target("avx2,bmi,bmi2,lzcnt,popcnt")

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <random>
#include <algorithm>
#include <string>

using namespace std;

// 전역 변수 설정
int C_cols, T_limit, M_scale;
vector<long long> A, B;
int R_rows;

// 아리스 전용 초고속 난수 생성기 (xorshift32)
uint32_t x_rand = 123456789;
inline uint32_t fast_rand() {
    x_rand ^= x_rand << 13;
    x_rand ^= x_rand >> 17;
    x_rand ^= x_rand << 5;
    return x_rand;
}
// 0.0 ~ 1.0 사이의 난수 생성
inline double fast_rand_double() {
    return (fast_rand() & 0xFFFFFF) / 16777216.0;
}

// 17가지 스킬 트리
const int CMD_D = 0, CMD_L = 1, CMD_R = 2;
const int CMD_DL = 3, CMD_LD = 4, CMD_DR = 5, CMD_RD = 6;
const int CMD_DLR = 7, CMD_DRL = 8, CMD_LDR = 9;
const int CMD_H2D = 10, CMD_H3D = 11, CMD_H4D = 12;
const int CMD_H2L = 13, CMD_H3L = 14;
const int CMD_H2R = 15, CMD_H3R = 16;
const int MAX_CMDS = 17;

string get_cmd_string(int cmd) {
    static const string cmds[] = {
        "D", "L", "R", "DL", "LD", "DR", "RD", "DLR", "DRL", "LDR",
        "2D", "3D", "4D", "2L", "3L", "2R", "3R"
    };
    return cmds[cmd];
}

// 스킬별 초당 처리 한계치
inline int get_capacity(int cmd) {
    if (cmd <= 2) return 1;
    if (cmd <= 6) return 2;
    if (cmd <= 9) return 3;
    return 1;
}

// 낙사 방지 안전장치
inline bool is_valid_cmd(int cmd, int r, int c, int R, int C) {
    if(c == 0 && (cmd==1 || cmd==3 || cmd==4 || cmd==7 || cmd==8 || cmd==9)) return false;
    if(c == C-1 && (cmd==2 || cmd==5 || cmd==6 || cmd==7 || cmd==8 || cmd==9)) return false;
    if(cmd == 13 && c - 2 < 0) return false;
    if(cmd == 14 && c - 3 < 0) return false;
    if(cmd == 15 && c + 2 >= C) return false;
    if(cmd == 16 && c + 3 >= C) return false;
    if(cmd == 10 && r + 2 > R) return false;
    if(cmd == 11 && r + 3 > R) return false;
    if(cmd == 12 && r + 4 > R) return false;
    return true;
}

long long calc_current[20];
double rate_current[20];
long long hamster_buffer_vol[50][20]; 
double hamster_buffer_rate[50][20]; 
long long global_final_seeds[20];

// 위상 정렬 기반 시뮬레이터
double calculate_score(const vector<vector<int>>& grid) {
    for (int c = 0; c < C_cols; ++c) {
        calc_current[c] = A[c];
        rate_current[c] = (A[c] > 0) ? 1.0 : 0.0;
    }
    
    for (int r = 0; r <= R_rows + 5; ++r) {
        fill(hamster_buffer_vol[r], hamster_buffer_vol[r] + C_cols, 0);
        fill(hamster_buffer_rate[r], hamster_buffer_rate[r] + C_cols, 0.0);
    }
    
    double penalty = 0.0;
    double max_time_spent = 0.0;

    for (int r = 0; r < R_rows; ++r) {
        long long cur_vol[15];
        double cur_rate[15];
        for (int c = 0; c < C_cols; ++c) {
            cur_vol[c] = calc_current[c] + hamster_buffer_vol[r][c];
            cur_rate[c] = rate_current[c] + hamster_buffer_rate[r][c];
        }

        int indeg[15] = {0};
        for (int c = 0; c < C_cols; ++c) {
            int cmd = grid[r][c];
            if(cmd==1 || cmd==3 || cmd==4 || cmd==7 || cmd==8 || cmd==9) indeg[c-1]++;
            if(cmd==2 || cmd==5 || cmd==6 || cmd==7 || cmd==8 || cmd==9) indeg[c+1]++;
            if(cmd==13) indeg[c-2]++;
            if(cmd==14) indeg[c-3]++;
            if(cmd==15) indeg[c+2]++;
            if(cmd==16) indeg[c+3]++;
        }

        int topo[15], head = 0, tail = 0;
        for (int c = 0; c < C_cols; ++c) if (indeg[c] == 0) topo[tail++] = c;

        while (head < tail) {
            int c = topo[head++];
            int cmd = grid[r][c];
            if(cmd==1 || cmd==3 || cmd==4 || cmd==7 || cmd==8 || cmd==9) if(--indeg[c-1] == 0) topo[tail++] = c-1;
            if(cmd==2 || cmd==5 || cmd==6 || cmd==7 || cmd==8 || cmd==9) if(--indeg[c+1] == 0) topo[tail++] = c+1;
            if(cmd==13) if(--indeg[c-2] == 0) topo[tail++] = c-2;
            if(cmd==14) if(--indeg[c-3] == 0) topo[tail++] = c-3;
            if(cmd==15) if(--indeg[c+2] == 0) topo[tail++] = c+2;
            if(cmd==16) if(--indeg[c+3] == 0) topo[tail++] = c+3;
        }

        long long next_vol[15] = {0};
        double next_rate[15] = {0.0};

        if (tail < C_cols) { // 🌟 무한 핑퐁 사이클 감지 시 즉시 커트!
            penalty += 1e12; 
            for (int c = 0; c < C_cols; ++c) {
                calc_current[c] = cur_vol[c]; // 씨앗 무한 복사 버그 방지
                rate_current[c] = 0;
            }
            continue;
        }

        for (int i = 0; i < C_cols; ++i) {
            int c = topo[i];
            long long V = cur_vol[c];
            double R_rt = cur_rate[c];
            int cmd = grid[r][c];
            int cap = get_capacity(cmd);

            if (R_rt > cap + 1e-6) penalty += (R_rt - cap) * 1e8;
            
            if (V > 0) {
                double est_time = (double)V / cap;
                if (est_time > max_time_spent) max_time_spent = est_time;
            }

            if (V == 0 && R_rt == 0) continue;

            long long vD=0, vL=0, vR=0;
            double rD=0, rL=0, rR=0;

            switch(cmd) {
                case 0: vD=V; rD=R_rt; break;
                case 1: vL=V; rL=R_rt; break;
                case 2: vR=V; rR=R_rt; break;
                case 3: vD=(V+1)/2; vL=V/2; rD=R_rt/2.0; rL=R_rt/2.0; break;
                case 4: vL=(V+1)/2; vD=V/2; rL=R_rt/2.0; rD=R_rt/2.0; break;
                case 5: vD=(V+1)/2; vR=V/2; rD=R_rt/2.0; rR=R_rt/2.0; break;
                case 6: vR=(V+1)/2; vD=V/2; rR=R_rt/2.0; rD=R_rt/2.0; break;
                case 7: vD=(V+2)/3; vL=(V+1)/3; vR=V/3; rD=R_rt/3.0; rL=R_rt/3.0; rR=R_rt/3.0; break;
                case 8: vD=(V+2)/3; vR=(V+1)/3; vL=V/3; rD=R_rt/3.0; rR=R_rt/3.0; rL=R_rt/3.0; break;
                case 9: vL=(V+2)/3; vD=(V+1)/3; vR=V/3; rL=R_rt/3.0; rD=R_rt/3.0; rR=R_rt/3.0; break;
                case 10: hamster_buffer_vol[r+2][c]+=V; hamster_buffer_rate[r+2][c]+=R_rt; break;
                case 11: hamster_buffer_vol[r+3][c]+=V; hamster_buffer_rate[r+3][c]+=R_rt; break;
                case 12: hamster_buffer_vol[r+4][c]+=V; hamster_buffer_rate[r+4][c]+=R_rt; break;
                case 13: cur_vol[c-2]+=V; cur_rate[c-2]+=R_rt; break;
                case 14: cur_vol[c-3]+=V; cur_rate[c-3]+=R_rt; break;
                case 15: cur_vol[c+2]+=V; cur_rate[c+2]+=R_rt; break;
                case 16: cur_vol[c+3]+=V; cur_rate[c+3]+=R_rt; break;
            }

            if(vD > 0) { next_vol[c] += vD; next_rate[c] += rD; }
            if(vL > 0) { cur_vol[c-1] += vL; cur_rate[c-1] += rL; }
            if(vR > 0) { cur_vol[c+1] += vR; cur_rate[c+1] += rR; }
        }

        for (int c = 0; c < C_cols; ++c) {
            calc_current[c] = next_vol[c];
            rate_current[c] = next_rate[c];
        }
    }

    double error_sum = 0.0;
    for (int c = 0; c < C_cols; ++c) {
        long long final_val = calc_current[c] + hamster_buffer_vol[R_rows][c];
        error_sum += abs(final_val - B[c]);
        global_final_seeds[c] = final_val; 
    }
    
    for (int r = R_rows + 1; r <= R_rows + 5; ++r) {
        for (int c = 0; c < C_cols; ++c) {
            penalty += hamster_buffer_vol[r][c] * 1e9;
        }
    }

    double delay_penalty = max(0.0, max_time_spent - M_scale) * 1.5;

    return error_sum + penalty + delay_penalty + (R_rows * 2.0); 
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    auto global_start_time = chrono::high_resolution_clock::now();
    double TOTAL_TIME_LIMIT = 1.85; // 🌟 2초 리미트 전 완벽한 안전 착지!

    if (!(cin >> C_cols >> T_limit >> M_scale)) return 0;

    A.resize(C_cols);
    for (int i = 0; i < C_cols; ++i) cin >> A[i];

    B.resize(C_cols);
    for (int i = 0; i < C_cols; ++i) cin >> B[i];

    int num_restarts = 5; 
    double time_per_restart = TOTAL_TIME_LIMIT / num_restarts;
    
    vector<vector<int>> absolute_best_grid;
    double absolute_best_score = 1e18; 
    int absolute_best_R = C_cols + 20;

    for (int restart = 0; restart < num_restarts; ++restart) {
        R_rows = min(C_cols + 20, C_cols + 4 + (restart % 5));

        vector<vector<int>> current_grid(R_rows, vector<int>(C_cols, CMD_D));
        double current_score = calculate_score(current_grid);

        vector<vector<int>> best_grid = current_grid;
        double best_score = current_score;

        double initial_temp = 1e7; 
        double final_temp = 0.1;
        double temp = initial_temp;
        double progress = 0.0;
        int iter_count = 0;
        
        int stuck_count = 0;
        double reheat_multiplier = 1.0; 

        while (true) {
            // 🌟 시간 초과(TLE) 방지: 1024번마다 확인하여 딜레이 없이 탈출!
            if ((iter_count++ & 1023) == 0) { 
                auto current_time = chrono::high_resolution_clock::now();
                chrono::duration<double> elapsed = current_time - global_start_time;
                
                if (elapsed.count() > TOTAL_TIME_LIMIT) goto END_ALL_RESTARTS;
                
                double current_restart_elapsed = elapsed.count() - (restart * time_per_restart);
                if (current_restart_elapsed > time_per_restart) break;

                progress = current_restart_elapsed / time_per_restart;
                temp = initial_temp * pow(final_temp / initial_temp, progress) * reheat_multiplier;
                
                if (reheat_multiplier > 1.0) reheat_multiplier *= 0.95; 
                else reheat_multiplier = 1.0;
            }

            int r = fast_rand() % R_rows;
            int c = fast_rand() % C_cols;
            
            if (fast_rand_double() < 0.6) {
                int c1 = fast_rand() % C_cols;
                int c2 = fast_rand() % C_cols;
                c = (abs(global_final_seeds[c1] - B[c1]) > abs(global_final_seeds[c2] - B[c2])) ? c1 : c2;
            }

            int old_cmd = current_grid[r][c];
            int new_cmd = fast_rand() % MAX_CMDS;
            
            while (!is_valid_cmd(new_cmd, r, c, R_rows, C_cols) || old_cmd == new_cmd) {
                new_cmd = fast_rand() % MAX_CMDS; 
            }
            
            current_grid[r][c] = new_cmd;
            double new_score = calculate_score(current_grid);

            bool accept = false;
            double diff = current_score - new_score;
            
            // 🌟 아리스의 엑스포넨셜 가드! (Denormalized Float TLE 방지)
            if (diff >= 0) {
                accept = true;
            } else {
                double exponent = diff / temp;
                if (exponent < -20.0) { 
                    // 너무 나쁜 길이라 가챠를 돌릴 가치도 없으면 CPU 과부하 전에 커트!
                    accept = false;
                } else if (fast_rand_double() < exp(exponent)) {
                    accept = true;
                }
            }

            if (accept) {
                current_score = new_score;
                if (current_score < best_score) {
                    best_score = current_score;
                    best_grid = current_grid;
                    stuck_count = 0; 
                } else {
                    stuck_count++; 
                }
            } else {
                stuck_count++; 
                current_grid[r][c] = old_cmd; 
            }

            if (stuck_count > 4000) {
                reheat_multiplier = min(10.0, reheat_multiplier * 3.0);
                temp = min(initial_temp, temp * 3.0);
                if (current_score > best_score * 1.5) {
                    current_grid = best_grid;
                    current_score = calculate_score(current_grid); 
                }
                stuck_count = 0;
            }
        }
        
        if (best_score < absolute_best_score) {
            absolute_best_score = best_score;
            absolute_best_grid = best_grid;
            absolute_best_R = R_rows; 
        }
    }

END_ALL_RESTARTS:

    cout << absolute_best_R << "\n";
    for (int r = 0; r < absolute_best_R; ++r) {
        for (int c = 0; c < C_cols; ++c) {
            cout << get_cmd_string(absolute_best_grid[r][c]) << (c == C_cols - 1 ? "" : " ");
        }
        cout << "\n";
    }

    return 0;
}