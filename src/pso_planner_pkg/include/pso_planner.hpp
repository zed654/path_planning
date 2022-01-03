// #ifndef pso_planner_hpp
// #define pso_planner_hpp

// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <unistd.h>
// #include <memory.h>
// //C언어
// //#include <stdio.h>
// //#include <stdlib.h>
// //#include <time.h>

// //C++
// //#include <iostream>
// #include <cstdlib>
// #include <ctime>

// #include <memory>
// // FIXME: Set to Visualize
// // FIXME: Set to Visualize
// // FIXME: Set to Visualize
// // FIXME: Set to Visualize
// // FIXME: Set to Visualize
// std::vector<double> adj_nodes_x;
// std::vector<double> adj_nodes_y;

// #define __PSO_DEBUG__

// struct Particles
// {
//     float position;
//     float velocity;

//     // V(k+1) = w*V(k) + c1*rand*(Xp-Xi) + c2*rand*(Xg-Xi)
//     // X(k+1) = V(k+1) + X(k)
//     Particles() : position(0), velocity(0)
//     {
// #ifdef __PSO_DEBUG__
//         std::cout << "파티클 생성자얌" << std::endl;
// #endif
//     }
// };
// struct Swarm
// {
//     std::vector<Particles> particles;
//     float fitness;

//     Swarm(const int &particles_size) : fitness(0)
//     {
// #ifdef __PSO_DEBUG__
//         std::cout << "particles_size : " << particles_size << std::endl;
// #endif
//         const Particles particles_tmp;
//         particles.assign(particles_size, particles_tmp);
//     }
//     Swarm() : fitness(0)
//     {
// #ifdef __PSO_DEBUG__
//         std::cout << "Me!" << std::endl;
// #endif
//     }
// };

// class PSOPlanner
// {
// public:
//     float m_w, m_c1, m_c2;
//     int m_swarm_size;
//     std::vector<int> m_personal_best_array_index;  // m_swarm[index][      ]
//     std::pair<int, int> m_global_best_array_index; // m_swarm[first][second]
//     bool m_init_flag;
//     double m_fitness_w1;
//     double m_fitness_w2;
//     int m_repeat_count;
//     float m_time_sample;

//     std::vector<std::vector<std::pair<double, double>>> m_pos_xy;

//     std::pair<double, double> m_pso_target_xy;
//     std::pair<double, double> m_closed_obsts_xy;
//     std::pair<double, double> m_init_xy;
//     /////////////
//     /////////////
//     /////////////
//     int m_n_particles;

//     std::vector<std::vector<Swarm>> m_swarm;
//     int m_particle_size;

//     std::vector<std::pair<double, double>> m_pso_result_path_points;
//     std::vector<std::vector<std::pair<double, double>>> m_pso_total_result_path_points;

//     std::vector<std::pair<double, double>> get_pso_result_path_points() { return this->m_pso_result_path_points; }
//     std::vector<std::vector<std::pair<double, double>>> get_pso_total_result_path_points() { return this->m_pso_total_result_path_points; }
//     std::pair<double, double> get_closed_obsts_xy() { return this->m_closed_obsts_xy; }

//     float get_random_value(int st_val, int end_val) // Get Rand Value ranged st_val_ <= x < end_val_
//     {
//         if (this->m_init_flag == true)
//         {
//             srand((unsigned)time(NULL)); //seed값으로 현재시간 부여
//             this->m_init_flag = false;
//         }
//         int range = end_val - st_val;
//         double rand_val = rand() / (RAND_MAX + 1.0);
//         rand_val = rand_val * range;
//         rand_val += st_val;
//         return rand_val;
//     }
//     PSOPlanner(const std::pair<double, double> &init_xy, const std::pair<double, double> &target_xy, const std::pair<double, double> &obsts_xy)
//     {
//         /*
//             Swarm 중에서 Personal Best 뽑음
//             m_swarm[Swarm 개수][증식 개수].particles[특정 particle]
//         */
//         this->m_personal_best_array_index;
//         this->m_global_best_array_index;
//         this->m_swarm;

//         /* Init Hyper-parameter */
//         // this->m_w = 1;            //0.25;
//         // this->m_c1 = 2;           //2;
//         // this->m_c2 = 2;           //2;
//         // this->m_init_flag = true; // Rand 함수 때문
//         // this->m_fitness_w1 = 10;
//         // this->m_fitness_w2 = 1; //1;
//         this->m_w = 1;            //0.25;
//         this->m_c1 = 2;           //2;
//         this->m_c2 = 2;           //2;
//         this->m_init_flag = true; // Rand 함수 때문
//         this->m_fitness_w1 = 100; //20000;
//         this->m_fitness_w2 = 1;   //1000;
//         this->m_time_sample = 0;

//         /* Particle 개수 초기화 (ex. x, y) */
//         this->m_particle_size = 2; // x, y

//         /* Swarm 개수 초기화 */
//         this->m_swarm_size = 5;

//         /* 반복 횟수 초기화 */
//         this->m_repeat_count = 10;

//         this->m_pso_target_xy = target_xy;
//         this->m_closed_obsts_xy = obsts_xy;
//         this->m_init_xy = init_xy;

//         std::vector<std::pair<double, double>> pos_xy_tmp;
//         pos_xy_tmp.assign(this->m_swarm_size, this->m_init_xy);
//         this->m_pos_xy.assign(this->m_swarm_size, pos_xy_tmp);

//         run();
//     }

//     void run()
//     {
//         /* m_swarm 초기화 */
//         init_first_particles();

//         for (int i = 0; i < this->m_repeat_count; i++)
//         {
//             /* swarm 의 가장 끝 단의 Particles 의 fitness 를 최적화. */
//             update_new_swarm_fitness_value();

//             /* Updated 된 Fitness 를 기반으로 Personal Best 에 해당하는 Index 갱신 */
//             update_personal_best_swarm();

//             /* Updated 된 Fitness 를 기반으로 Global Best 에 해당되는 Index 갱신 */
//             update_global_best_swarm();

//             /* 새로운 Swarm 을 생성 (Particles 의 position, velocity 계산 완료) */
//             update_new_swarm();

// #ifdef __PSO_DEBUG__
//             std::cout << std::endl;
//             std::cout << std::endl;
//             std::cout << std::endl;
//             std::cout << std::endl;
//             std::cout << std::endl;
//             std::cout << std::endl;
//             std::cout << std::endl;
// #endif
//         }

//         /* Update Final Result Path Points */
//         for (int i = 0; i < this->m_pos_xy.size(); i++)
//         {
//             // std::cout << "Result Pos XY : ";
//             std::vector<std::pair<double, double>> swarm_particles_points_tmp;
//             for (int j = 0; j < this->m_pos_xy[i].size(); j++)
//             {
//                 swarm_particles_points_tmp.push_back(std::make_pair(this->m_pos_xy[i][j].first, this->m_pos_xy[i][j].second));
//                 // std::cout << m_pos_xy[i][j].first << ", " << m_pos_xy[i][j].second << "\t\t";
//             }
//             // std::cout << std::endl;
//             this->m_pso_total_result_path_points.push_back(swarm_particles_points_tmp);
//         }

//         /* Update Final Result Path Points */
//         // for (int i = 0; i < this->m_swarm[this->m_global_best_array_index.first].size(); i++)
//         // {
//         //     double x_tmp = this->m_swarm[this->m_global_best_array_index.first][i].particles[0].position;
//         //     double y_tmp = this->m_swarm[this->m_global_best_array_index.first][i].particles[1].position;
//         //     this->m_pso_result_path_points.push_back(std::make_pair(x_tmp, y_tmp));
//         // }

//         // /* Update Final Result Path Points */
//         // for (int i = 0; i < this->m_swarm.size(); i++)
//         // {
//         //     std::vector<std::pair<double, double>> swarm_particles_points_tmp;
//         //     for (int j = 0; j < this->m_swarm[i].size(); j++)
//         //     {
//         //         swarm_particles_points_tmp.push_back(std::make_pair(this->m_swarm[i][j].particles[0].position, this->m_swarm[i][j].particles[1].position));
//         //     }
//         //     this->m_pso_total_result_path_points.push_back(swarm_particles_points_tmp);
//         // }
//     }

//     void init_first_particles()
//     {
//         /* Swarm 개수 초기화 */
//         static Swarm init_swarm(m_particle_size);    // Swarm 생성 및 Particles 개수 선언
//         std::vector<Swarm> propagation_swarm_tmp;    // Swarm 이중 벡터화
//         propagation_swarm_tmp.push_back(init_swarm); // Swarm 이중 벡터화
//         for (int i = 0; i < this->m_swarm_size; i++)
//             this->m_swarm.push_back(propagation_swarm_tmp);
// #ifdef __PSO_DEBUG__
//         std::cout << "Swarm 개수 초기화. Swarm 사이즈 : " << this->m_swarm.size() << "\t\t 증식 개수 : " << this->m_swarm[0].size() << std::endl;
// #endif
//         /* Personal best 배열 index 초기화 */
//         this->m_personal_best_array_index.assign(this->m_swarm_size, 0);
// #ifdef __PSO_DEBUG__
//         std::cout << "Personal Best 배열 인덱스 초기화. 사이즈(Swarm 사이즈) : " << m_personal_best_array_index.size() << std::endl;
// #endif
//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             for (int j = 0; j < this->m_swarm[i].size(); j++)
//             {
//                 for (int k = 0; k < this->m_swarm[i][j].particles.size(); k++)
//                 {
//                     double rand_value = get_random_value(-1, 1);
//                     this->m_swarm[i][j].particles[k].position = rand_value;
//                 }
//             }
// #ifdef __PSO_DEBUG__
//             std::cout << std::endl;
// #endif
//         }

// #ifdef __PSO_DEBUG__
//         std::cout << std::endl;
//         std::cout << std::endl;
//         std::cout << std::endl;
//         std::cout << std::endl;
//         std::cout << std::endl;
//         std::cout << std::endl;
//         std::cout << std::endl;
// #endif
//     }

//     void update_new_swarm_fitness_value()
//     {
//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             /* TODO: Particle 에 따라 fitness 를 정의하시오 */
//             // int swarm_depth_end_index = this->m_swarm[i].size() - 1;
//             // for (int k = 0; k < m_swarm[i][swarm_depth_end_index].particles.size(); k++)
//             // {
//             //     m_swarm[i][swarm_depth_end_index].particles[k].fitness = 1; // FIXME: 계산하시오. (ㅡ.ㅡ)
//             // }

//             /* 본 예제는 fitness 함수를 작게 만드는 것이 목적 */
//             int swarm_depth_end_index = this->m_swarm[i].size() - 1;
//             double x_tmp = this->m_pos_xy[i][swarm_depth_end_index].first;
//             double y_tmp = this->m_pos_xy[i][swarm_depth_end_index].second;
//             this->m_swarm[i][swarm_depth_end_index].fitness = this->m_fitness_w2 * std::hypot(m_pso_target_xy.first - x_tmp, m_pso_target_xy.second - y_tmp);

//             // if (m_swarm[i][swarm_depth_end_index].particles.size() == 2)
//             // {
//             //     /* Origin */
//             //     // float x_tmp = this->m_swarm[i][swarm_depth_end_index].particles[0].position;
//             //     // float y_tmp = this->m_swarm[i][swarm_depth_end_index].particles[1].position;
//             //     // this->m_swarm[i][swarm_depth_end_index].fitness = this->m_fitness_w1 * 1. / std::hypot(m_closed_obsts_xy.first - x_tmp, m_closed_obsts_xy.second - y_tmp) + this->m_fitness_w2 * std::hypot(m_pso_target_xy.first - x_tmp, m_pso_target_xy.second - y_tmp);
//             //     // this->m_swarm[i][swarm_depth_end_index].fitness = this->m_fitness_w2 * std::hypot(m_pso_target_xy.first - x_tmp, m_pso_target_xy.second - y_tmp);

//             //     double x_tmp = this->m_pos_xy[i][swarm_depth_end_index].first;
//             //     double y_tmp = this->m_pos_xy[i][swarm_depth_end_index].second;

//             //     // this->m_swarm[i][swarm_depth_end_index].fitness = this->m_fitness_w1 * 1. / std::hypot(m_closed_obsts_xy.first - x_tmp, m_closed_obsts_xy.second - y_tmp) + this->m_fitness_w2 * std::hypot(m_pso_target_xy.first - x_tmp, m_pso_target_xy.second - y_tmp);
//             //     this->m_swarm[i][swarm_depth_end_index].fitness = this->m_fitness_w2 * std::hypot(m_pso_target_xy.first - x_tmp, m_pso_target_xy.second - y_tmp);
//             // }
//             // else
//             // {
//             //     std::cout << "너 무언가 잘못되었다 ^^" << std::endl;
//             // }
//         }

// #ifdef __PSO_DEBUG__
//         std::cout << "Fitness Value - " << std::endl;
//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             for (int j = 0; j < this->m_swarm[i].size(); j++)
//             {
//                 std::cout << this->m_swarm[i][j].fitness << "\t\t";
//             }
//             std::cout << std::endl;
//         }
//         std::cout << "-------------------" << std::endl;
// #endif
//     }

//     void update_personal_best_swarm()
//     {
//         this->m_personal_best_array_index; // std::vector<int>
//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             double tmp = 1e+15;
//             for (int j = 0; j < this->m_swarm[i].size(); j++)
//             {
//                 if (tmp > this->m_swarm[i][j].fitness)
//                 {
//                     this->m_personal_best_array_index[i] = j;
//                     tmp = this->m_swarm[i][j].fitness;
//                 }
//             }
//         }

// #ifdef __PSO_DEBUG__
//         std::cout << "Personal Best Array Index - " << std::endl;
//         for (int i = 0; i < m_personal_best_array_index.size(); i++)
//         {
//             std::cout << "\t\t" << m_personal_best_array_index[i] << std::endl;
//         }
//         std::cout << "-------------------" << std::endl;
// #endif
//     }

//     void update_global_best_swarm()
//     {
//         double tmp = 1e+15;
//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             for (int j = 0; j < this->m_swarm[i].size(); j++)
//             {
//                 if (tmp > this->m_swarm[i][j].fitness)
//                 {
//                     this->m_global_best_array_index = std::make_pair(i, j);
//                     tmp = this->m_swarm[i][j].fitness;
//                 }
//             }
//         }

// #ifdef __PSO_DEBUG__
//         std::cout << "Global Best Index : [Swarm][Depth] [" << this->m_global_best_array_index.first << "][" << this->m_global_best_array_index.second << "]" << std::endl;
//         std::cout << "-------------------" << std::endl;
// #endif
//     }

//     void update_new_swarm()
//     {
//         /* Updaet New Swarm */
//         this->m_time_sample = this->m_time_sample + 5; // Unit : sec

//         // std::cout << "Particle New w : ";
//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             int swarm_depth_end_index = this->m_swarm[i].size() - 1;
//             Swarm new_swarm_particles(this->m_particle_size);
//             std::vector<double> particle_weight;
//             // Swarm new_swarm_particles(this->m_swarm[i][swarm_depth_end_index].particles.size());
//             for (int k = 0; k < this->m_swarm[i][swarm_depth_end_index].particles.size(); k++)
//             {
//                 Particles personal_best_swarm_particle = this->m_swarm[i][this->m_personal_best_array_index[i]].particles[k];
//                 Particles global_best_swarm_particle = this->m_swarm[this->m_global_best_array_index.first][this->m_global_best_array_index.second].particles[k];

//                 Particles *current_particle = &(this->m_swarm[i][swarm_depth_end_index].particles[k]);
//                 double rand_1 = get_random_value(0, 1);
//                 double rand_2 = get_random_value(0, 1);
//                 // double new_velocity = current_particle->velocity * this->m_w + this->m_c1 * rand_1 * (current_particle->position - personal_best_swarm_particle.position) + this->m_c2 * rand_2 * (current_particle->position - global_best_swarm_particle.position);
//                 double new_velocity = current_particle->velocity * this->m_w + this->m_c1 * rand_1 * (personal_best_swarm_particle.position - current_particle->position) + this->m_c2 * rand_2 * (global_best_swarm_particle.position - current_particle->position);
//                 if (new_velocity > 5)
//                     new_velocity = 5;
//                 else if (new_velocity < -5)
//                     new_velocity = -5;
//                 double new_position = current_particle->position + new_velocity;

//                 // std::cout << "rand : " << rand_1 << ", " << rand_2 << std::endl;
//                 // std::cout << "w, c1, c2 : " << this->m_w << ", " << this->m_c1 << ", " << this->m_c2 << std::endl;
//                 // std::cout << "cur particle pos, personal pos, global pos : " << current_particle->position << ", " << personal_best_swarm_particle.position << ", " << global_best_swarm_particle.position << std::endl;
//                 // std::cout << "real cur particle pos : " << this->m_swarm[i][swarm_depth_end_index].particles[k].position << std::endl;
//                 // std::cout << "new vel/pos : " << new_velocity << ", " << new_position << std::endl;
//                 new_swarm_particles.particles[k].velocity = new_velocity;
//                 new_swarm_particles.particles[k].position = new_position;
//                 // std::cout << new_position << ", ";
//                 particle_weight.push_back(new_position);
//             }
//             // std::cout << std::endl;
//             this->m_swarm[i].push_back(new_swarm_particles);

//             /* Update New Position */
//             double x_tmp = this->m_init_xy.first + this->m_time_sample;
//             double y_tmp = this->m_init_xy.second;
//             for (int l = 0; l < particle_weight.size(); l = l + 2)
//             {
//                 y_tmp += std::cos(0.05 * this->m_time_sample) * particle_weight[l] + std::sin(0.05 * this->m_time_sample) * particle_weight[l + 1];
//             }
//             std::pair<double, double> pos_xy_tmp = std::make_pair(x_tmp, y_tmp);
//             // std::cout << "New Pos : " << x_tmp << ", " << y_tmp << std::endl;
//             this->m_pos_xy[i].push_back(pos_xy_tmp);
//         }
//         // std::cout << std::endl;

//         /* Updaet New Swarm */
//         // for (int i = 0; i < this->m_swarm.size(); i++)
//         // {
//         //     int swarm_depth_end_index = this->m_swarm[i].size() - 1;
//         //     Swarm new_swarm_particles(this->m_particle_size);
//         //     // Swarm new_swarm_particles(this->m_swarm[i][swarm_depth_end_index].particles.size());
//         //     for (int k = 0; k < this->m_swarm[i][swarm_depth_end_index].particles.size(); k++)
//         //     {
//         //         Particles personal_best_swarm_particle = this->m_swarm[i][this->m_personal_best_array_index[i]].particles[k];
//         //         Particles global_best_swarm_particle = this->m_swarm[this->m_global_best_array_index.first][this->m_global_best_array_index.second].particles[k];

//         //         Particles *current_particle = &(this->m_swarm[i][swarm_depth_end_index].particles[k]);
//         //         double rand_1 = get_random_value(0, 1);
//         //         double rand_2 = get_random_value(0, 1);
//         //         // double new_velocity = current_particle->velocity * this->m_w + this->m_c1 * rand_1 * (current_particle->position - personal_best_swarm_particle.position) + this->m_c2 * rand_2 * (current_particle->position - global_best_swarm_particle.position);
//         //         double new_velocity = current_particle->velocity * this->m_w + this->m_c1 * rand_1 * (personal_best_swarm_particle.position - current_particle->position) + this->m_c2 * rand_2 * (global_best_swarm_particle.position - current_particle->position);
//         //         if (new_velocity > 5)
//         //             new_velocity = 5;
//         //         else if (new_velocity < -5)
//         //             new_velocity = -5;
//         //         double new_position = current_particle->position + new_velocity;

//         //         // std::cout << "rand : " << rand_1 << ", " << rand_2 << std::endl;
//         //         // std::cout << "w, c1, c2 : " << this->m_w << ", " << this->m_c1 << ", " << this->m_c2 << std::endl;
//         //         // std::cout << "cur particle pos, personal pos, global pos : " << current_particle->position << ", " << personal_best_swarm_particle.position << ", " << global_best_swarm_particle.position << std::endl;
//         //         // std::cout << "real cur particle pos : " << this->m_swarm[i][swarm_depth_end_index].particles[k].position << std::endl;
//         //         // std::cout << "new vel/pos : " << new_velocity << ", " << new_position << std::endl;
//         //         new_swarm_particles.particles[k].velocity = new_velocity;
//         //         new_swarm_particles.particles[k].position = new_position;
//         //     }
//         //     this->m_swarm[i].push_back(new_swarm_particles);
//         // }

//         // std::cout << this->m_swarm.size() << "\t\t" << this->m_swarm[0].size() << "\t\t" << this->m_swarm[0][0].fitness << std::endl;

// #ifdef __PSO_DEBUG__
//         std::cout << "Updated New Swarm Particle Position - " << std::endl;
//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             for (int j = 0; j < this->m_swarm[i].size(); j++)
//             {
//                 // for (int k = 0; k < this->m_swarm[i][j].size(); k++)
//                 // {
//                 std::cout << "(" << this->m_swarm[i][j].particles[0].position << ", " << this->m_swarm[i][j].particles[1].position << ")\t\t";
//                 // }
//             }
//             std::cout << std::endl;
//         }
//         std::cout << "-------------------" << std::endl;
// #endif
//     }

//     ~PSOPlanner()
//     {
//     }

// protected:
// private:
// };

// #endif /* pso_planner_hpp */

///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
///////////////////////////////////////
#ifndef pso_planner_hpp
#define pso_planner_hpp

#include <iostream>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <memory.h>
//C언어
//#include <stdio.h>
//#include <stdlib.h>
//#include <time.h>

//C++
//#include <iostream>
#include <cstdlib>
#include <ctime>

#include <memory>
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
std::vector<double> adj_nodes_x;
std::vector<double> adj_nodes_y;

// #define __PSO_DEBUG__

struct Particles
{
    float position;
    float velocity;

    // V(k+1) = w*V(k) + c1*rand*(Xp-Xi) + c2*rand*(Xg-Xi)
    // X(k+1) = V(k+1) + X(k)
    Particles() : position(0), velocity(0)
    {
#ifdef __PSO_DEBUG__
        std::cout << "파티클 생성자얌" << std::endl;
#endif
    }
};
struct Swarm
{
    std::vector<Particles> particles;
    float fitness;

    Swarm(const int &particles_size) : fitness(0)
    {
#ifdef __PSO_DEBUG__
        std::cout << "particles_size : " << particles_size << std::endl;
#endif
        const Particles particles_tmp;
        particles.assign(particles_size, particles_tmp);
    }
    Swarm() : fitness(0)
    {
#ifdef __PSO_DEBUG__
        std::cout << "Me!" << std::endl;
#endif
    }
};

class PSOPlanner
{
public:
    float m_w, m_c1, m_c2;
    int m_swarm_size;
    std::vector<int> m_personal_best_array_index;  // m_swarm[index][      ]
    std::pair<int, int> m_global_best_array_index; // m_swarm[first][second]
    bool m_init_flag;
    double m_fitness_w1;
    double m_fitness_w2;
    int m_repeat_count;

    std::pair<double, double> m_pso_target_xy;
    std::pair<double, double> m_closed_obsts_xy;
    std::pair<double, double> m_init_xy;
    float m_step_size;
    /////////////
    /////////////
    /////////////
    int m_n_particles;

    std::vector<std::vector<Swarm>> m_swarm;
    int m_particle_size;

    std::vector<std::pair<double, double>> m_pso_result_path_points;
    std::vector<std::vector<std::pair<double, double>>> m_pso_total_result_path_points;

    std::vector<std::pair<double, double>> get_pso_result_path_points() { return this->m_pso_result_path_points; }
    std::vector<std::vector<std::pair<double, double>>> get_pso_total_result_path_points() { return this->m_pso_total_result_path_points; }
    std::pair<double, double> get_closed_obsts_xy() { return this->m_closed_obsts_xy; }

    float get_random_value(int st_val, int end_val) // Get Rand Value ranged st_val_ <= x < end_val_
    {
        if (this->m_init_flag == true)
        {
            srand((unsigned)time(NULL)); //seed값으로 현재시간 부여
            this->m_init_flag = false;
        }
        int range = end_val - st_val;
        double rand_val = rand() / (RAND_MAX + 1.0);
        rand_val = rand_val * range;
        rand_val += st_val;
        return rand_val;
    }
    PSOPlanner(const std::pair<double, double> &init_xy, const std::pair<double, double> &target_xy, const std::pair<double, double> &obsts_xy)
    {
        /*
            Swarm 중에서 Personal Best 뽑음
            m_swarm[Swarm 개수][증식 개수].particles[특정 particle]
        */
        this->m_personal_best_array_index;
        this->m_global_best_array_index;
        this->m_swarm;

        /* Init Hyper-parameter */
        // this->m_w = 1;            //0.25;
        // this->m_c1 = 2;           //2;
        // this->m_c2 = 2;           //2;
        // this->m_init_flag = true; // Rand 함수 때문
        // this->m_fitness_w1 = 10;
        // this->m_fitness_w2 = 1; //1;
        this->m_w = 0.25;           //0.25;
        this->m_c1 = 1;             //2;
        this->m_c2 = 1;             //2;
        this->m_init_flag = true;   // Rand 함수 때문
        this->m_fitness_w1 = 10000; //20000;
        this->m_fitness_w2 = 1;     //1000;
        this->m_step_size = 50.0;

        /* Particle 개수 초기화 (ex. x, y) */
        this->m_particle_size = 2; // x, y

        /* Swarm 개수 초기화 */
        this->m_swarm_size = 30;

        /* 반복 횟수 초기화 */
        this->m_repeat_count = 30;

        this->m_pso_target_xy = target_xy;
        this->m_closed_obsts_xy = obsts_xy;
        this->m_init_xy = init_xy;

        run();
    }

    void run()
    {
        /* m_swarm 초기화 */
        init_first_particles();

        for (int i = 0; i < this->m_repeat_count; i++)
        {
            /* swarm 의 가장 끝 단의 Particles 의 fitness 를 최적화. */
            update_new_swarm_fitness_value();

            /* Updated 된 Fitness 를 기반으로 Personal Best 에 해당하는 Index 갱신 */
            update_personal_best_swarm();

            /* Updated 된 Fitness 를 기반으로 Global Best 에 해당되는 Index 갱신 */
            update_global_best_swarm();

            /* 새로운 Swarm 을 생성 (Particles 의 position, velocity 계산 완료) */
            update_new_swarm();

#ifdef __PSO_DEBUG__
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << std::endl;
            std::cout << std::endl;
#endif
        }

        /* Update Final Result Path Points */
        for (int i = 0; i < this->m_swarm[this->m_global_best_array_index.first].size(); i++)
        {
            double x_tmp = this->m_swarm[this->m_global_best_array_index.first][i].particles[0].position;
            double y_tmp = this->m_swarm[this->m_global_best_array_index.first][i].particles[1].position;
            this->m_pso_result_path_points.push_back(std::make_pair(x_tmp, y_tmp));
        }

        /* Update Final Result Path Points */
        for (int i = 0; i < this->m_swarm.size(); i++)
        {
            std::vector<std::pair<double, double>> swarm_particles_points_tmp;
            for (int j = 0; j < this->m_swarm[i].size(); j++)
            {
                swarm_particles_points_tmp.push_back(std::make_pair(this->m_swarm[i][j].particles[0].position, this->m_swarm[i][j].particles[1].position));
            }
            this->m_pso_total_result_path_points.push_back(swarm_particles_points_tmp);
        }
    }

    void init_first_particles()
    {
        /* Swarm 개수 초기화 */
        static Swarm init_swarm(m_particle_size);    // Swarm 생성 및 Particles 개수 선언
        std::vector<Swarm> propagation_swarm_tmp;    // Swarm 이중 벡터화
        propagation_swarm_tmp.push_back(init_swarm); // Swarm 이중 벡터화
        for (int i = 0; i < this->m_swarm_size; i++)
            this->m_swarm.push_back(propagation_swarm_tmp);
#ifdef __PSO_DEBUG__
        std::cout << "Swarm 개수 초기화. Swarm 사이즈 : " << this->m_swarm.size() << "\t\t 증식 개수 : " << this->m_swarm[0].size() << std::endl;
#endif
        /* Personal best 배열 index 초기화 */
        this->m_personal_best_array_index.assign(this->m_swarm_size, 0);
#ifdef __PSO_DEBUG__
        std::cout << "Personal Best 배열 인덱스 초기화. 사이즈(Swarm 사이즈) : " << m_personal_best_array_index.size() << std::endl;
#endif
        for (int i = 0; i < this->m_swarm.size(); i++)
        {
            for (int j = 0; j < this->m_swarm[i].size(); j++)
            {
                // FIXME: 초기위치 바꿨는대, 적용 안됨
                double rand_value_1 = get_random_value(-1, 1);
                double rand_value_2 = get_random_value(-1, 1);
                float rand_value_size = std::sqrt(std::pow(rand_value_1, 2) + std::pow(rand_value_2, 2));
                rand_value_1 = rand_value_1 * 1 / rand_value_size;
                rand_value_2 = rand_value_2 * 1 / rand_value_size;
                this->m_swarm[i][j].particles[0].position = rand_value_1 + this->m_init_xy.first;
                this->m_swarm[i][j].particles[1].position = rand_value_2 + this->m_init_xy.second;
                // std::cout << "init rand value : " << rand_value_1 << ", " << rand_value_2 << std::endl;
                // std::cout << "this->m_init_xy : " << this->m_init_xy.first << "\t\t" << this->m_init_xy.second << std::endl;
                //                 for (int k = 0; k < this->m_swarm[i][j].particles.size(); k++)
                //                 {
                //                     double rand_value = get_random_value(-1, 1);
                //                     this->m_swarm[i][j].particles[k].position = rand_value;
                // #ifdef __PSO_DEBUG__
                //                     std::cout << rand_value << "\t\t";
                // #endif
                //                 }
            }
#ifdef __PSO_DEBUG__
            std::cout << std::endl;
#endif
        }

#ifdef __PSO_DEBUG__
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
#endif
    }

    void update_new_swarm_fitness_value()
    {
        for (int i = 0; i < this->m_swarm.size(); i++)
        {
            /* TODO: Particle 에 따라 fitness 를 정의하시오 */
            // int swarm_depth_end_index = this->m_swarm[i].size() - 1;
            // for (int k = 0; k < m_swarm[i][swarm_depth_end_index].particles.size(); k++)
            // {
            //     m_swarm[i][swarm_depth_end_index].particles[k].fitness = 1; // FIXME: 계산하시오. (ㅡ.ㅡ)
            // }

            // int swarm_depth_end_index = this->m_swarm[i].size() - 1;
            // if (m_swarm[i][swarm_depth_end_index].particles.size() == 2)
            // {
            //     float x_tmp = this->m_swarm[i][swarm_depth_end_index].particles[0].position;
            //     float y_tmp = this->m_swarm[i][swarm_depth_end_index].particles[1].position;
            //     this->m_swarm[i][swarm_depth_end_index].particles[0].fitness = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.first - x_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.first - x_tmp);
            //     this->m_swarm[i][swarm_depth_end_index].particles[1].fitness = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.second - y_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);
            // }
            // else
            // {
            //     std::cout << "너 무언가 잘못되었다 ^^" << std::endl;
            // }

            /* 본 예제는 fitness 함수를 작게 만드는 것이 목적 */
            int swarm_depth_end_index = this->m_swarm[i].size() - 1;
            if (m_swarm[i][swarm_depth_end_index].particles.size() == 2)
            {
                float x_tmp = this->m_swarm[i][swarm_depth_end_index].particles[0].position;
                float y_tmp = this->m_swarm[i][swarm_depth_end_index].particles[1].position;
                this->m_swarm[i][swarm_depth_end_index].fitness = this->m_fitness_w1 * 1. / std::hypot(m_closed_obsts_xy.first - x_tmp, m_closed_obsts_xy.second - y_tmp) + this->m_fitness_w2 * std::hypot(m_pso_target_xy.first - x_tmp, m_pso_target_xy.second - y_tmp);
                // this->m_swarm[i][swarm_depth_end_index].fitness = this->m_fitness_w2 * std::hypot(m_pso_target_xy.first - x_tmp, m_pso_target_xy.second - y_tmp);
            }
            else
            {
                std::cout << "너 무언가 잘못되었다 ^^" << std::endl;
            }
        }

#ifdef __PSO_DEBUG__
        std::cout << "Fitness Value - " << std::endl;
        for (int i = 0; i < this->m_swarm.size(); i++)
        {
            for (int j = 0; j < this->m_swarm[i].size(); j++)
            {
                std::cout << this->m_swarm[i][j].fitness << "\t\t";
            }
            std::cout << std::endl;
        }
        std::cout << "-------------------" << std::endl;
#endif
    }

    void update_personal_best_swarm()
    {
        this->m_personal_best_array_index; // std::vector<int>
        for (int i = 0; i < this->m_swarm.size(); i++)
        {
            double tmp = 1e+15;
            for (int j = 0; j < this->m_swarm[i].size(); j++)
            {
                if (tmp > this->m_swarm[i][j].fitness)
                {
                    this->m_personal_best_array_index[i] = j;
                    tmp = this->m_swarm[i][j].fitness;
                }
            }
        }

#ifdef __PSO_DEBUG__
        std::cout << "Personal Best Array Index - " << std::endl;
        for (int i = 0; i < m_personal_best_array_index.size(); i++)
        {
            std::cout << "\t\t" << m_personal_best_array_index[i] << std::endl;
        }
        std::cout << "-------------------" << std::endl;
#endif
    }

    void update_global_best_swarm()
    {
        double tmp = 1e+15;
        for (int i = 0; i < this->m_swarm.size(); i++)
        {
            for (int j = 0; j < this->m_swarm[i].size(); j++)
            {
                if (tmp > this->m_swarm[i][j].fitness)
                {
                    this->m_global_best_array_index = std::make_pair(i, j);
                    tmp = this->m_swarm[i][j].fitness;
                }
            }
        }

#ifdef __PSO_DEBUG__
        std::cout << "Global Best Index : [Swarm][Depth] [" << this->m_global_best_array_index.first << "][" << this->m_global_best_array_index.second << "]" << std::endl;
        std::cout << "-------------------" << std::endl;
#endif
    }

    void update_new_swarm()
    {
        // /* Updaet New Swarm */
        // for (int i = 0; i < this->m_swarm.size(); i++)
        // {
        //     int swarm_depth_end_index = this->m_swarm[i].size() - 1;
        //     Swarm new_swarm_particles(this->m_particle_size);
        //     // Swarm new_swarm_particles(this->m_swarm[i][swarm_depth_end_index].particles.size());
        //     for (int k = 0; k < this->m_swarm[i][swarm_depth_end_index].particles.size(); k++)
        //     {
        //         Particles personal_best_swarm_particle = this->m_swarm[i][this->m_personal_best_array_index[i]].particles[k];
        //         Particles global_best_swarm_particle = this->m_swarm[this->m_global_best_array_index.first][this->m_global_best_array_index.second].particles[k];

        //         Particles *current_particle = &(this->m_swarm[i][swarm_depth_end_index].particles[k]);
        //         double new_velocity = current_particle[k].velocity * this->m_w + this->m_c1 * get_random_value(0, 1) * (current_particle->position - personal_best_swarm_particle.position) + this->m_c2 * get_random_value(0, 1) * (current_particle->position - global_best_swarm_particle.position);
        //         double new_position = current_particle[k].position + new_velocity;

        //         new_swarm_particles.particles[k].velocity = new_velocity;
        //         new_swarm_particles.particles[k].position = new_position;
        //     }
        //     this->m_swarm[i].push_back(new_swarm_particles);
        // }

        /* Updaet New Swarm */
        for (int i = 0; i < this->m_swarm.size(); i++)
        {
            int swarm_depth_end_index = this->m_swarm[i].size() - 1;
            Swarm new_swarm_particles(this->m_particle_size);

            // std::cout << "Particles : " << std::endl;
            // for (int k = 0; k < this->m_swarm[i][swarm_depth_end_index].particles.size(); k++)
            // {
            //     Particles personal_best_swarm_particle = this->m_swarm[i][this->m_personal_best_array_index[i]].particles[k];
            //     Particles global_best_swarm_particle = this->m_swarm[this->m_global_best_array_index.first][this->m_global_best_array_index.second].particles[k];

            //     Particles *current_particle = &(this->m_swarm[i][swarm_depth_end_index].particles[k]);
            //     double rand_1 = get_random_value(0, 1);
            //     double rand_2 = get_random_value(0, 1);
            //     // double rand_value_size = std::sqrt(std::pow(rand_1, 2) + std::pow(rand_2, 2));
            //     // rand_1 = rand_1 * 1.0 / rand_value_size;
            //     // rand_2 = rand_2 * 1.0 / rand_value_size;
            //     // std::cout << "rand_1, 2 : " << rand_1 << "\t\t" << rand_2 << std::endl;
            //     // double new_velocity = current_particle->velocity * this->m_w + this->m_c1 * rand_1 * (current_particle->position - personal_best_swarm_particle.position) + this->m_c2 * rand_2 * (current_particle->position - global_best_swarm_particle.position);
            //     double new_velocity = current_particle->velocity * this->m_w + this->m_c1 * rand_1 * (personal_best_swarm_particle.position - current_particle->position) + this->m_c2 * rand_2 * (global_best_swarm_particle.position - current_particle->position);
            //     // if (new_velocity > 5)
            //     // {
            //     //     std::cout << "Assert ! The velocity is converged to 5" << std::endl;
            //     //     new_velocity = 5;
            //     // }
            //     // else if (new_velocity < -5)
            //     // {
            //     //     std::cout << "Assert ! The velocity is converged to -5" << std::endl;
            //     //     new_velocity = -5;
            //     // }
            //     double new_position = current_particle->position + new_velocity;
            //     std::cout << "\t\t new_velocity : " << new_velocity << ", new pos : " << new_position << ", " << personal_best_swarm_particle.position - current_particle->position << ", " << global_best_swarm_particle.position - current_particle->position << std::endl;

            //     // std::cout << "rand : " << rand_1 << ", " << rand_2 << std::endl;
            //     // std::cout << "w, c1, c2 : " << this->m_w << ", " << this->m_c1 << ", " << this->m_c2 << std::endl;
            //     // std::cout << "cur particle pos, personal pos, global pos : " << current_particle->position << ", " << personal_best_swarm_particle.position << ", " << global_best_swarm_particle.position << std::endl;
            //     // std::cout << "real cur particle pos : " << this->m_swarm[i][swarm_depth_end_index].particles[k].position << std::endl;
            //     // std::cout << "new vel/pos : " << new_velocity << ", " << new_position << std::endl;
            //     new_swarm_particles.particles[k].velocity = new_velocity;
            //     new_swarm_particles.particles[k].position = new_position;
            // }
            // std::cout << std::endl;

            double norm_particle_velocity = 0;
            std::vector<double> new_particles_velocity;
            new_particles_velocity.assign(this->m_particle_size, 0);
            for (int k = 0; k < this->m_swarm[i][swarm_depth_end_index].particles.size(); k++)
            {
                Particles personal_best_swarm_particle = this->m_swarm[i][this->m_personal_best_array_index[i]].particles[k];
                Particles global_best_swarm_particle = this->m_swarm[this->m_global_best_array_index.first][this->m_global_best_array_index.second].particles[k];

                Particles *current_particle = &(this->m_swarm[i][swarm_depth_end_index].particles[k]);
                double rand_1 = get_random_value(0, 1);
                double rand_2 = get_random_value(0, 1);
                double new_velocity = current_particle->velocity * this->m_w + this->m_c1 * rand_1 * (personal_best_swarm_particle.position - current_particle->position) + this->m_c2 * rand_2 * (global_best_swarm_particle.position - current_particle->position);
                double new_position = current_particle->position + new_velocity;
                new_particles_velocity[k] = new_velocity;
                norm_particle_velocity += std::pow(new_velocity, 2);
            }
            norm_particle_velocity = std::sqrt(norm_particle_velocity);
            if (norm_particle_velocity == 0)
                norm_particle_velocity = 1;

            for (int k = 0; k < this->m_swarm[i][swarm_depth_end_index].particles.size(); k++)
            {
                Particles *current_particle = &(this->m_swarm[i][swarm_depth_end_index].particles[k]);
                new_swarm_particles.particles[k].velocity = new_particles_velocity[k] * this->m_step_size / norm_particle_velocity;
                double new_position = current_particle->position + new_swarm_particles.particles[k].velocity;
                new_swarm_particles.particles[k].position = new_position;

                // std::cout << "New Velocity : " << new_swarm_particles.particles[k].velocity << std::endl;
                // new_swarm_particles.particles[k].position += new_swarm_particles.particles[k].velocity;
            }

            this->m_swarm[i].push_back(new_swarm_particles);
        }

        // std::cout << this->m_swarm.size() << "\t\t" << this->m_swarm[0].size() << "\t\t" << this->m_swarm[0][0].fitness << std::endl;

#ifdef __PSO_DEBUG__
        std::cout << "Updated New Swarm Particle Position - " << std::endl;
        for (int i = 0; i < this->m_swarm.size(); i++)
        {
            for (int j = 0; j < this->m_swarm[i].size(); j++)
            {
                // for (int k = 0; k < this->m_swarm[i][j].size(); k++)
                // {
                std::cout << "(" << this->m_swarm[i][j].particles[0].position << ", " << this->m_swarm[i][j].particles[1].position << ")\t\t";
                // }
            }
            std::cout << std::endl;
        }
        std::cout << "-------------------" << std::endl;
#endif
    }

    ~PSOPlanner()
    {
    }

protected:
private:
};

#endif /* pso_planner_hpp */
