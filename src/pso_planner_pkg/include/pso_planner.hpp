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
        this->m_w = 1;            //0.25;
        this->m_c1 = 1;           //2;
        this->m_c2 = 2;           //2;
        this->m_init_flag = true; // Rand 함수 때문
        this->m_fitness_w1 = 20000;
        this->m_fitness_w2 = 1000; //1;

        /* Particle 개수 초기화 (ex. x, y) */
        this->m_particle_size = 2; // x, y

        /* Swarm 개수 초기화 */
        this->m_swarm_size = 10;

        /* 반복 횟수 초기화 */
        this->m_repeat_count = 100;

        this->m_pso_target_xy = target_xy;
        this->m_closed_obsts_xy = obsts_xy;
        this->m_init_xy = init_xy;

        /* m_swarm 초기화 */
        init_first_node();

        run();
    }

    void run()
    {
        for (int i = 0; i < this->m_repeat_count; i++)
        {
            /* swarm 의 가장 끝 단의 Particles 의 fitness 를 최적화. */
            update_new_node_fitness_value();

            /* Updated 된 Fitness 를 기반으로 Personal Best 에 해당하는 Index 갱신 */
            renew_personal_best_position();

            /* Updated 된 Fitness 를 기반으로 Global Best 에 해당되는 Index 갱신 */
            renew_global_best_position();

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

    void init_first_node()
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
                double rand_value = get_random_value(-1, 1);
                this->m_swarm[i][j].particles[0].position = rand_value + this->m_init_xy.first;
                this->m_swarm[i][j].particles[1].position = rand_value + this->m_init_xy.second;
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

    void update_new_node_fitness_value()
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

    void renew_personal_best_position()
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

    void renew_global_best_position()
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
            // Swarm new_swarm_particles(this->m_swarm[i][swarm_depth_end_index].particles.size());
            for (int k = 0; k < this->m_swarm[i][swarm_depth_end_index].particles.size(); k++)
            {
                Particles personal_best_swarm_particle = this->m_swarm[i][this->m_personal_best_array_index[i]].particles[k];
                Particles global_best_swarm_particle = this->m_swarm[this->m_global_best_array_index.first][this->m_global_best_array_index.second].particles[k];

                Particles *current_particle = &(this->m_swarm[i][swarm_depth_end_index].particles[k]);
                double rand_1 = get_random_value(0, 1);
                double rand_2 = get_random_value(0, 1);
                // double new_velocity = current_particle->velocity * this->m_w + this->m_c1 * rand_1 * (current_particle->position - personal_best_swarm_particle.position) + this->m_c2 * rand_2 * (current_particle->position - global_best_swarm_particle.position);
                double new_velocity = current_particle->velocity * this->m_w + this->m_c1 * rand_1 * (personal_best_swarm_particle.position - current_particle->position) + this->m_c2 * rand_2 * (global_best_swarm_particle.position - current_particle->position);
                if (new_velocity > 5)
                    new_velocity = 5;
                else if (new_velocity < -5)
                    new_velocity = -5;
                double new_position = current_particle->position + new_velocity;

                // std::cout << "rand : " << rand_1 << ", " << rand_2 << std::endl;
                // std::cout << "w, c1, c2 : " << this->m_w << ", " << this->m_c1 << ", " << this->m_c2 << std::endl;
                // std::cout << "cur particle pos, personal pos, global pos : " << current_particle->position << ", " << personal_best_swarm_particle.position << ", " << global_best_swarm_particle.position << std::endl;
                // std::cout << "real cur particle pos : " << this->m_swarm[i][swarm_depth_end_index].particles[k].position << std::endl;
                // std::cout << "new vel/pos : " << new_velocity << ", " << new_position << std::endl;
                new_swarm_particles.particles[k].velocity = new_velocity;
                new_swarm_particles.particles[k].position = new_position;
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

    void origin_func()
    {
        // this->m_swarm.push_back(new_swarm);

        // Particles current_particle;
        // = std::make_shared<particles>();
        // = this->m_swarm[0].particles[0];
        // new_swarm.particles[0].v = this->m_swarm[0].particles[0];
        // Calc Next Swarm

        // swarm[0].particles[1].v.push_back(2);
        // swarm[0].particles[1].x.push_back(3);

        // Swarm swarm_tmp(10);
        // m_swarm.assign(10, swarm_tmp);
        // m_swarm[0].particles[0].c1;

        // m_swarm[0].particles[0];

        std::pair<double, double> init_pos;
        init_pos = std::make_pair(0, 0);
        this->m_n_particles = 20;
        // this->m_repeat_count = 100;

        // m_pso_target_xy = std::make_pair(200, 200);
        // m_closed_obsts_xy = std::make_pair(50, 50);

        std::vector<double> p_x, p_y;
        std::vector<double> min_p_x, min_p_y;
        p_x.assign(this->m_n_particles, 0);
        p_y.assign(this->m_n_particles, 0);
        min_p_x.assign(this->m_n_particles, 0);
        min_p_y.assign(this->m_n_particles, 0);
        double g_x = 0;
        double g_y = 0;

        //        std::vector<double> vel_x, vel_y;
        //        vel_x.push_back(0); vel_y.push_back(0);
        std::vector<std::vector<double>> vel_x(this->m_n_particles, std::vector<double>(1, 0));
        std::vector<std::vector<double>> vel_y(this->m_n_particles, std::vector<double>(1, 0));

        int min_g_x_index = 0;
        int min_g_y_index = 0;
        double min_g_x = 0;
        double min_g_y = 0;

        // Get X Init Pos Swarm
        std::vector<std::vector<double>> x_swarm(this->m_n_particles, std::vector<double>(1, 0));
        for (int i = 0; i < x_swarm.size(); i++)
            for (int j = 0; j < x_swarm[i].size(); j++)
            {
                x_swarm[i][j] = get_random_value(0, 1) + init_pos.first;
            }

        // Get Y Init Pos Swarm
        std::vector<std::vector<double>> y_swarm(this->m_n_particles, std::vector<double>(1, 0));
        for (int i = 0; i < y_swarm.size(); i++)
            for (int j = 0; j < y_swarm[i].size(); j++)
            {
                y_swarm[i][j] = get_random_value(0, 1) + init_pos.second;
            }

        for (int e = 0; e < this->m_repeat_count; e++)
        {
            // Update fitness_x
            std::vector<double> fitness_x, fitness_y;
            fitness_x.assign(this->m_n_particles, 0);
            fitness_y.assign(this->m_n_particles, 0);
            for (int i = 0; i < this->m_n_particles; i++)
            {
                int index_tmp = x_swarm[i].size() - 1;
                double x_tmp = x_swarm[i][index_tmp];
                double y_tmp = y_swarm[i][index_tmp];
                //                fitness_x[i] = this->m_fitness_w2 * std::fabs(m_pso_target_xy.first  - x_tmp);
                //                fitness_y[i] = this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);

                fitness_x[i] = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.first - x_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.first - x_tmp);
                fitness_y[i] = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.second - y_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);
            }

            if (e == 0)
            {
                // Update Perosnal POS_x
                double fitness_x_tmp = 1e+10;
                for (int i = 0; i < this->m_n_particles; i++)
                {
                    // Update personal pos
                    p_x[i] = x_swarm[i][0];
                    min_p_x[i] = fitness_x[i];

                    // Update global pos 1
                    if (fitness_x[i] < fitness_x_tmp)
                    {
                        fitness_x_tmp = fitness_x[i];
                        //                        min_g_x = min_p_x[i];
                        min_g_x_index = i;
                    }
                }

                // Update Personal POS_y
                double fitness_y_tmp = 1e+10;
                for (int i = 0; i < this->m_n_particles; i++)
                {
                    // Update personal pos
                    p_y[i] = y_swarm[i][0];
                    min_p_y[i] = fitness_y[i];

                    // Update global pos 1
                    if (fitness_y[i] < fitness_y_tmp)
                    {
                        fitness_y_tmp = fitness_y[i];
                        //                        min_g_x = min_p_x[i];
                        min_g_y_index = i;
                    }
                }

                // Update global pos_xy
                min_g_x = fitness_x[min_g_x_index];
                min_g_y = fitness_x[min_g_y_index];
                g_x = p_x[min_g_x_index];
                g_y = p_y[min_g_y_index];
            }
            else
            {
                // Update personal POS_x
                for (int i = 0; i < this->m_n_particles; i++)
                {
                    // If new fitness_x is smaller than existing fitness_x, update personal.
                    if (min_p_x[i] > fitness_x[i])
                    {
                        int index_tmp = x_swarm[i].size() - 1;
                        min_p_x[i] = fitness_x[i];
                        p_x[i] = x_swarm[i][index_tmp];
                    }
                }

                // Update global pos 1 among personal POS_x
                double fitness_x_tmp = 1e+10;
                for (int i = 0; i < this->m_n_particles; i++)
                {
                    if (min_p_x[i] < fitness_x_tmp)
                    {
                        fitness_x_tmp = min_p_x[i];
                        //                        min_g_x = min_p_x[i];
                        min_g_x_index = i;
                    }
                }

                // Update personal POS_y
                for (int i = 0; i < this->m_n_particles; i++)
                {
                    // If new fitness_y is smaller than existing fitness_y, update personal.
                    if (min_p_y[i] > fitness_y[i])
                    {
                        int index_tmp = y_swarm[i].size() - 1;
                        min_p_y[i] = fitness_y[i];
                        p_y[i] = y_swarm[i][index_tmp];
                    }
                }

                // Update global pos 1 among personal POS_y
                double fitness_y_tmp = 1e+10;
                for (int i = 0; i < this->m_n_particles; i++)
                {
                    if (min_p_y[i] < fitness_y_tmp)
                    {
                        fitness_y_tmp = min_p_y[i];
                        //                        min_g_y = min_p_y[i];
                        min_g_y_index = i;
                    }
                }

                // Update global pos 2
                min_g_x = min_p_x[min_g_x_index];
                min_g_y = min_p_y[min_g_y_index];
                g_x = p_x[min_g_x_index];
                g_y = p_y[min_g_y_index];
            }

            // Update Pos X
            for (int i = 0; i < this->m_n_particles; i++)
            {
                int index_tmp = x_swarm[i].size() - 1;
                int vel_x_index_tmp = vel_x[i].size() - 1;
                //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
                //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
                double p_err = p_x[i] - x_swarm[i][index_tmp];
                double g_err = g_x - x_swarm[i][index_tmp];
                //                double vel_x_new = this->m_w * vel_x[vel_x.size()-1] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
                double vel_x_new = this->m_w * vel_x[i][vel_x_index_tmp] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
                if (vel_x_new > 10)
                    vel_x_new = 10;
                if (vel_x_new < -10)
                    vel_x_new = -10;

                double pos_new = vel_x_new + x_swarm[i][index_tmp];
                vel_x[i].push_back(vel_x_new);
                x_swarm[i].push_back(pos_new);
            }

            // Update Pos Y
            for (int i = 0; i < this->m_n_particles; i++)
            {
                int index_tmp = y_swarm[i].size() - 1;
                int vel_y_index_tmp = vel_y[i].size() - 1;

                //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
                //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
                double p_err = p_y[i] - y_swarm[i][index_tmp];
                double g_err = g_y - y_swarm[i][index_tmp];

                double param_1 = this->m_c1 * get_random_value(0, 1);
                double param_2 = this->m_c2 * get_random_value(0, 1);
                //                double vel_y_new = this->m_w * vel_y[vel_y.size()-1] + param_1 * p_err + param_2 * g_err;
                double vel_y_new = this->m_w * vel_y[i][vel_y_index_tmp] + param_1 * p_err + param_2 * g_err;
                if (vel_y_new > 10)
                    vel_y_new = 10;
                if (vel_y_new < -10)
                    vel_y_new = -10;
                //                double vel_y_new = this->m_w * vel_y[vel_y.size()-1] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
                double pos_new = vel_y_new + y_swarm[i][index_tmp];
                vel_y[i].push_back(vel_y_new);
                y_swarm[i].push_back(pos_new);
            }

            //            std::cout << "g_x : " << g_x << ", " << g_y << "\t\t min_g_xy : " << min_g_x << ", " << min_g_y << "\t\t" << std::hypot(min_g_x, min_g_y) << std::endl;

            m_pso_result_path_points.push_back(std::make_pair(g_x, g_y));
        }
    }

    // PSOPlanner(int width_, int height_, int init_pos_x_, int init_pos_y_)
    // PSOPlanner()
    // {
    //     /*
    //         Swarm 중에서 Personal Best 뽑음
    //         m_swarm[Swarm 개수][증식 개수].particles[특정 particle]
    //     */
    //     this->m_personal_best_array_index;
    //     this->m_swarm;

    //     // Particle 개수 초기화 (ex. x, y)
    //     this->m_particle_size = 2; // x, y

    //     // Swarm 개수 초기화
    //     this->m_swarm_size = 5;

    //     // Swarm 개수 초기화
    //     // Swarm init_swarm(this->m_particle_size);
    //     // for (int i = 0; i < this->m_swarm_size; i++)
    //     //     this->m_swarm.push_back(init_swarm);
    //     Swarm init_swarm(m_particle_size);           // Swarm 생성 및 Particles 개수 선언
    //     std::vector<Swarm> propagation_swarm_tmp;    // Swarm 이중 벡터화
    //     propagation_swarm_tmp.push_back(init_swarm); // Swarm 이중 벡터화
    //     for (int i = 0; i < this->m_swarm_size; i++)
    //         this->m_swarm.push_back(propagation_swarm_tmp);
    //     std::cout << "Swarm 개수 초기화. Swarm 사이즈 : " << this->m_swarm.size() << "\t\t 증식 개수 : " << this->m_swarm[0].size() << std::endl;

    //     // Personal best 배열 index 초기화
    //     this->m_personal_best_array_index.assign(this->m_swarm_size, 0);
    //     std::cout << "Personal Best 배열 인덱스 초기화. 사이즈(Swarm 사이즈) : " << m_personal_best_array_index.size() << std::endl;

    //     // std::shared_ptr<Particles> current_particle = std::shared_ptr<Particles>(this->m_swarm[0].particles[0]);
    //     Particles *current_particle = &(this->m_swarm[0][0].particles[0]);

    //     Swarm new_swarm(this->m_particle_size);
    //     new_swarm.particles.size();
    //     new_swarm.particles[0].velocity = current_particle->velocity * this->m_w + this->m_c1 * get_random_value(0, 1) + this->m_c2 * get_random_value(0, 1);
    //     // Particles current_particle;
    //     // = std::make_shared<particles>();
    //     // = this->m_swarm[0].particles[0];
    //     // new_swarm.particles[0].v = this->m_swarm[0].particles[0];
    //     // Calc Next Swarm

    //     // swarm[0].particles[1].v.push_back(2);
    //     // swarm[0].particles[1].x.push_back(3);

    //     // Swarm swarm_tmp(10);
    //     // m_swarm.assign(10, swarm_tmp);
    //     // m_swarm[0].particles[0].c1;

    //     // m_swarm[0].particles[0];

    //     std::pair<double, double> init_pos;
    //     init_pos = std::make_pair(0, 0);
    //     this->m_w = 1;  //0.25;
    //     this->m_c1 = 2; //2;
    //     this->m_c2 = 2; //2;
    //     this->m_n_particles = 20;
    //     this->m_repeat_count = 100;
    //     this->m_init_flag = true;
    //     this->m_fitness_w1 = 10;
    //     this->m_fitness_w2 = 1;

    //     m_pso_target_xy = std::make_pair(200, 200);

    //     m_closed_obsts_xy = std::make_pair(50, 50);

    //     std::vector<double> p_x, p_y;
    //     std::vector<double> min_p_x, min_p_y;
    //     p_x.assign(this->m_n_particles, 0);
    //     p_y.assign(this->m_n_particles, 0);
    //     min_p_x.assign(this->m_n_particles, 0);
    //     min_p_y.assign(this->m_n_particles, 0);
    //     double g_x = 0;
    //     double g_y = 0;

    //     //        std::vector<double> vel_x, vel_y;
    //     //        vel_x.push_back(0); vel_y.push_back(0);
    //     std::vector<std::vector<double>> vel_x(this->m_n_particles, std::vector<double>(1, 0));
    //     std::vector<std::vector<double>> vel_y(this->m_n_particles, std::vector<double>(1, 0));

    //     int min_g_x_index = 0;
    //     int min_g_y_index = 0;
    //     double min_g_x = 0;
    //     double min_g_y = 0;

    //     // Get X Init Pos Swarm
    //     std::vector<std::vector<double>> x_swarm(this->m_n_particles, std::vector<double>(1, 0));
    //     for (int i = 0; i < x_swarm.size(); i++)
    //         for (int j = 0; j < x_swarm[i].size(); j++)
    //         {
    //             x_swarm[i][j] = get_random_value(0, 1) + init_pos.first;
    //         }

    //     // Get Y Init Pos Swarm
    //     std::vector<std::vector<double>> y_swarm(this->m_n_particles, std::vector<double>(1, 0));
    //     for (int i = 0; i < y_swarm.size(); i++)
    //         for (int j = 0; j < y_swarm[i].size(); j++)
    //         {
    //             y_swarm[i][j] = get_random_value(0, 1) + init_pos.second;
    //         }

    //     for (int e = 0; e < this->m_repeat_count; e++)
    //     {
    //         // Update fitness_x
    //         std::vector<double> fitness_x, fitness_y;
    //         fitness_x.assign(this->m_n_particles, 0);
    //         fitness_y.assign(this->m_n_particles, 0);
    //         for (int i = 0; i < this->m_n_particles; i++)
    //         {
    //             int index_tmp = x_swarm[i].size() - 1;
    //             double x_tmp = x_swarm[i][index_tmp];
    //             double y_tmp = y_swarm[i][index_tmp];
    //             //                fitness_x[i] = this->m_fitness_w2 * std::fabs(m_pso_target_xy.first  - x_tmp);
    //             //                fitness_y[i] = this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);

    //             fitness_x[i] = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.first - x_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.first - x_tmp);
    //             fitness_y[i] = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.second - y_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);
    //         }

    //         if (e == 0)
    //         {
    //             // Update Perosnal POS_x
    //             double fitness_x_tmp = 1e+10;
    //             for (int i = 0; i < this->m_n_particles; i++)
    //             {
    //                 // Update personal pos
    //                 p_x[i] = x_swarm[i][0];
    //                 min_p_x[i] = fitness_x[i];

    //                 // Update global pos 1
    //                 if (fitness_x[i] < fitness_x_tmp)
    //                 {
    //                     fitness_x_tmp = fitness_x[i];
    //                     //                        min_g_x = min_p_x[i];
    //                     min_g_x_index = i;
    //                 }
    //             }

    //             // Update Personal POS_y
    //             double fitness_y_tmp = 1e+10;
    //             for (int i = 0; i < this->m_n_particles; i++)
    //             {
    //                 // Update personal pos
    //                 p_y[i] = y_swarm[i][0];
    //                 min_p_y[i] = fitness_y[i];

    //                 // Update global pos 1
    //                 if (fitness_y[i] < fitness_y_tmp)
    //                 {
    //                     fitness_y_tmp = fitness_y[i];
    //                     //                        min_g_x = min_p_x[i];
    //                     min_g_y_index = i;
    //                 }
    //             }

    //             // Update global pos_xy
    //             min_g_x = fitness_x[min_g_x_index];
    //             min_g_y = fitness_x[min_g_y_index];
    //             g_x = p_x[min_g_x_index];
    //             g_y = p_y[min_g_y_index];
    //         }
    //         else
    //         {
    //             // Update personal POS_x
    //             for (int i = 0; i < this->m_n_particles; i++)
    //             {
    //                 // If new fitness_x is smaller than existing fitness_x, update personal.
    //                 if (min_p_x[i] > fitness_x[i])
    //                 {
    //                     int index_tmp = x_swarm[i].size() - 1;
    //                     min_p_x[i] = fitness_x[i];
    //                     p_x[i] = x_swarm[i][index_tmp];
    //                 }
    //             }

    //             // Update global pos 1 among personal POS_x
    //             double fitness_x_tmp = 1e+10;
    //             for (int i = 0; i < this->m_n_particles; i++)
    //             {
    //                 if (min_p_x[i] < fitness_x_tmp)
    //                 {
    //                     fitness_x_tmp = min_p_x[i];
    //                     //                        min_g_x = min_p_x[i];
    //                     min_g_x_index = i;
    //                 }
    //             }

    //             // Update personal POS_y
    //             for (int i = 0; i < this->m_n_particles; i++)
    //             {
    //                 // If new fitness_y is smaller than existing fitness_y, update personal.
    //                 if (min_p_y[i] > fitness_y[i])
    //                 {
    //                     int index_tmp = y_swarm[i].size() - 1;
    //                     min_p_y[i] = fitness_y[i];
    //                     p_y[i] = y_swarm[i][index_tmp];
    //                 }
    //             }

    //             // Update global pos 1 among personal POS_y
    //             double fitness_y_tmp = 1e+10;
    //             for (int i = 0; i < this->m_n_particles; i++)
    //             {
    //                 if (min_p_y[i] < fitness_y_tmp)
    //                 {
    //                     fitness_y_tmp = min_p_y[i];
    //                     //                        min_g_y = min_p_y[i];
    //                     min_g_y_index = i;
    //                 }
    //             }

    //             // Update global pos 2
    //             min_g_x = min_p_x[min_g_x_index];
    //             min_g_y = min_p_y[min_g_y_index];
    //             g_x = p_x[min_g_x_index];
    //             g_y = p_y[min_g_y_index];
    //         }

    //         // Update Pos X
    //         for (int i = 0; i < this->m_n_particles; i++)
    //         {
    //             int index_tmp = x_swarm[i].size() - 1;
    //             int vel_x_index_tmp = vel_x[i].size() - 1;
    //             //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
    //             //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
    //             double p_err = p_x[i] - x_swarm[i][index_tmp];
    //             double g_err = g_x - x_swarm[i][index_tmp];
    //             //                double vel_x_new = this->m_w * vel_x[vel_x.size()-1] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
    //             double vel_x_new = this->m_w * vel_x[i][vel_x_index_tmp] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
    //             if (vel_x_new > 10)
    //                 vel_x_new = 10;
    //             if (vel_x_new < -10)
    //                 vel_x_new = -10;

    //             double pos_new = vel_x_new + x_swarm[i][index_tmp];
    //             vel_x[i].push_back(vel_x_new);
    //             x_swarm[i].push_back(pos_new);
    //         }

    //         // Update Pos Y
    //         for (int i = 0; i < this->m_n_particles; i++)
    //         {
    //             int index_tmp = y_swarm[i].size() - 1;
    //             int vel_y_index_tmp = vel_y[i].size() - 1;

    //             //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
    //             //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
    //             double p_err = p_y[i] - y_swarm[i][index_tmp];
    //             double g_err = g_y - y_swarm[i][index_tmp];

    //             double param_1 = this->m_c1 * get_random_value(0, 1);
    //             double param_2 = this->m_c2 * get_random_value(0, 1);
    //             //                double vel_y_new = this->m_w * vel_y[vel_y.size()-1] + param_1 * p_err + param_2 * g_err;
    //             double vel_y_new = this->m_w * vel_y[i][vel_y_index_tmp] + param_1 * p_err + param_2 * g_err;
    //             if (vel_y_new > 10)
    //                 vel_y_new = 10;
    //             if (vel_y_new < -10)
    //                 vel_y_new = -10;
    //             //                double vel_y_new = this->m_w * vel_y[vel_y.size()-1] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
    //             double pos_new = vel_y_new + y_swarm[i][index_tmp];
    //             vel_y[i].push_back(vel_y_new);
    //             y_swarm[i].push_back(pos_new);
    //         }

    //         //            std::cout << "g_x : " << g_x << ", " << g_y << "\t\t min_g_xy : " << min_g_x << ", " << min_g_y << "\t\t" << std::hypot(min_g_x, min_g_y) << std::endl;

    //         m_pso_result_path_points.push_back(std::make_pair(g_x, g_y));
    //     }
    // }
    ~PSOPlanner()
    {
    }

protected:
private:
};

#endif /* pso_planner_hpp */

/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
/////////////////
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

// struct Particles
// {
//     float position;
//     float velocity;
//     float fitness;

//     // V(k+1) = w*V(k) + c1*rand*(Xp-Xi) + c2*rand*(Xg-Xi)
//     // X(k+1) = V(k+1) + X(k)
//     Particles() : position(0), velocity(0), fitness(0) {}
// };
// struct Swarm
// {
//     std::vector<Particles> particles;

//     Swarm(const int &particles_size)
//     {
//         std::cout << "particles_size : " << particles_size << std::endl;
//         const Particles particles_tmp;
//         particles.assign(particles_size, particles_tmp);
//     }
//     Swarm()
//     {
//         std::cout << "Me!" << std::endl;
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

//     /////////////
//     /////////////
//     /////////////
//     int m_n_particles;
//     int m_repeat_count;

//     std::vector<std::vector<Swarm>> m_swarm;
//     int m_particle_size;

//     std::vector<std::pair<double, double>> m_pso_result_path_points;
//     std::pair<double, double> m_pso_target_xy;
//     std::pair<double, double> m_closed_obsts_xy;

//     std::vector<std::pair<double, double>> get_m_pso_result_path_points()
//     {
//         return this->m_pso_result_path_points;
//     }

//     std::pair<double, double> get_closed_obsts_xy()
//     {
//         return this->m_closed_obsts_xy;
//     }
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
//     PSOPlanner()
//     {
//         /*
//             Swarm 중에서 Personal Best 뽑음
//             m_swarm[Swarm 개수][증식 개수].particles[특정 particle]
//         */
//         this->m_personal_best_array_index;
//         this->m_global_best_array_index;
//         this->m_swarm;

//         /* Init Hyper-parameter */
//         this->m_w = 1;            //0.25;
//         this->m_c1 = 2;           //2;
//         this->m_c2 = 2;           //2;
//         this->m_init_flag = true; // Rand 함수 때문
//         this->m_fitness_w1 = 10;
//         this->m_fitness_w2 = 1;

//         /* Particle 개수 초기화 (ex. x, y) */
//         this->m_particle_size = 2; // x, y

//         /* Swarm 개수 초기화 */
//         this->m_swarm_size = 5;

//         /* FIXME: 아래 선언은 지우시오. */
//         this->m_pso_target_xy = std::make_pair(200, 200);
//         this->m_closed_obsts_xy = std::make_pair(50, 50);

//         /* m_swarm 초기화 */
//         init_first_node();

//         run();
//     }

//     void init_first_node()
//     {
//         /* Swarm 개수 초기화 */
//         Swarm init_swarm(m_particle_size);           // Swarm 생성 및 Particles 개수 선언
//         std::vector<Swarm> propagation_swarm_tmp;    // Swarm 이중 벡터화
//         propagation_swarm_tmp.push_back(init_swarm); // Swarm 이중 벡터화
//         for (int i = 0; i < this->m_swarm_size; i++)
//             this->m_swarm.push_back(propagation_swarm_tmp);
//         std::cout << "Swarm 개수 초기화. Swarm 사이즈 : " << this->m_swarm.size() << "\t\t 증식 개수 : " << this->m_swarm[0].size() << std::endl;

//         /* Personal best 배열 index 초기화 */
//         this->m_personal_best_array_index.assign(this->m_swarm_size, 0);
//         std::cout << "Personal Best 배열 인덱스 초기화. 사이즈(Swarm 사이즈) : " << m_personal_best_array_index.size() << std::endl;
//     }

//     void update_new_node_fitness_value()
//     {
//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             /* TODO: Particle 에 따라 fitness 를 정의하시오 */
//             // int swarm_depth_end_index = this->m_swarm[i].size() - 1;
//             // for (int k = 0; k < m_swarm[i][swarm_depth_end_index].particles.size(); k++)
//             // {
//             //     m_swarm[i][swarm_depth_end_index].particles[k].fitness = 1; // FIXME: 계산하시오. (ㅡ.ㅡ)
//             // }

//             int swarm_depth_end_index = this->m_swarm[i].size() - 1;
//             if (m_swarm[i][swarm_depth_end_index].particles.size() == 2)
//             {
//                 float x_tmp = this->m_swarm[i][swarm_depth_end_index].particles[0].position;
//                 float y_tmp = this->m_swarm[i][swarm_depth_end_index].particles[1].position;
//                 this->m_swarm[i][swarm_depth_end_index].particles[0].fitness = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.first - x_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.first - x_tmp);
//                 this->m_swarm[i][swarm_depth_end_index].particles[1].fitness = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.second - y_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);
//             }
//             else
//             {
//                 std::cout << "너 무언가 잘못되었다 ^^" << std::endl;
//             }
//         }
//     }

//     void renew_personal_best_position()
//     {
//         this->m_personal_best_array_index; // std::vector<int>

//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             for (int j = 0; j < this->m_swarm[i].size(); j++)
//             {
//                 for (int k = 0; k < this->m_swarm[i][j].particles.size(); k++)
//                 {
//                     this->m_swarm[i][j].particles[k].fitness;
//                 }
//             }
//         }
//     }

//     void renew_global_best_position()
//     {
//         this->m_global_best_array_index; // std::pair<int, int>
//     }

//     void run()
//     {
//         /* swarm 의 가장 끝 단의 Particles 의 fitness 를 최적화. */
//         update_new_node_fitness_value();

//         /* Updated 된 Fitness 를 기반으로 Personal Best 에 해당하는 Index 갱신 */
//         renew_personal_best_position();

//         /* 새로운 Swarm 을 생성 (Particles 의 position, velocity 계산 완료) */
//         update_new_swarm();
//     }

//     void update_new_swarm()
//     {
//         /* Updaet New Swarm */
//         this->m_swarm[0][0].particles[0].fitness;
//         this->m_swarm[0][0].particles[0].position;
//         this->m_swarm[0][0].particles[0].velocity;

//         std::cout << this->m_swarm.size() << "\t\t" << this->m_swarm[0].size() << "\t\t" << this->m_swarm[0][0].particles[0].fitness << std::endl;

//         // std::vector<Swarm> new_swarm;
//         for (int i = 0; i < this->m_swarm.size(); i++)
//         {
//             int swarm_depth_end_index = this->m_swarm[i].size() - 1;
//             Swarm new_swarm_particles(this->m_particle_size);
//             // Swarm new_swarm_particles(this->m_swarm[i][swarm_depth_end_index].particles.size());
//             for (int k = 0; k < this->m_swarm[i][swarm_depth_end_index].particles.size(); k++)
//             {
//                 Particles *current_particle = &(this->m_swarm[i][swarm_depth_end_index].particles[k]);
//                 double new_velocity = current_particle[k].velocity * this->m_w + this->m_c1 * get_random_value(0, 1) + this->m_c2 * get_random_value(0, 1);
//                 double new_position = current_particle[k].position + new_velocity;

//                 new_swarm_particles.particles[k].velocity = new_velocity;
//                 new_swarm_particles.particles[k].position = new_position;
//             }
//             this->m_swarm[i].push_back(new_swarm_particles);
//         }
//         std::cout << this->m_swarm.size() << "\t\t" << this->m_swarm[0].size() << "\t\t" << this->m_swarm[0][0].particles[0].fitness << std::endl;

//         // this->m_swarm.push_back(new_swarm);

//         // Particles current_particle;
//         // = std::make_shared<particles>();
//         // = this->m_swarm[0].particles[0];
//         // new_swarm.particles[0].v = this->m_swarm[0].particles[0];
//         // Calc Next Swarm

//         // swarm[0].particles[1].v.push_back(2);
//         // swarm[0].particles[1].x.push_back(3);

//         // Swarm swarm_tmp(10);
//         // m_swarm.assign(10, swarm_tmp);
//         // m_swarm[0].particles[0].c1;

//         // m_swarm[0].particles[0];

//         std::pair<double, double> init_pos;
//         init_pos = std::make_pair(0, 0);
//         this->m_n_particles = 20;
//         this->m_repeat_count = 100;

//         // m_pso_target_xy = std::make_pair(200, 200);
//         // m_closed_obsts_xy = std::make_pair(50, 50);

//         std::vector<double> p_x, p_y;
//         std::vector<double> min_p_x, min_p_y;
//         p_x.assign(this->m_n_particles, 0);
//         p_y.assign(this->m_n_particles, 0);
//         min_p_x.assign(this->m_n_particles, 0);
//         min_p_y.assign(this->m_n_particles, 0);
//         double g_x = 0;
//         double g_y = 0;

//         //        std::vector<double> vel_x, vel_y;
//         //        vel_x.push_back(0); vel_y.push_back(0);
//         std::vector<std::vector<double>> vel_x(this->m_n_particles, std::vector<double>(1, 0));
//         std::vector<std::vector<double>> vel_y(this->m_n_particles, std::vector<double>(1, 0));

//         int min_g_x_index = 0;
//         int min_g_y_index = 0;
//         double min_g_x = 0;
//         double min_g_y = 0;

//         // Get X Init Pos Swarm
//         std::vector<std::vector<double>> x_swarm(this->m_n_particles, std::vector<double>(1, 0));
//         for (int i = 0; i < x_swarm.size(); i++)
//             for (int j = 0; j < x_swarm[i].size(); j++)
//             {
//                 x_swarm[i][j] = get_random_value(0, 1) + init_pos.first;
//             }

//         // Get Y Init Pos Swarm
//         std::vector<std::vector<double>> y_swarm(this->m_n_particles, std::vector<double>(1, 0));
//         for (int i = 0; i < y_swarm.size(); i++)
//             for (int j = 0; j < y_swarm[i].size(); j++)
//             {
//                 y_swarm[i][j] = get_random_value(0, 1) + init_pos.second;
//             }

//         for (int e = 0; e < this->m_repeat_count; e++)
//         {
//             // Update fitness_x
//             std::vector<double> fitness_x, fitness_y;
//             fitness_x.assign(this->m_n_particles, 0);
//             fitness_y.assign(this->m_n_particles, 0);
//             for (int i = 0; i < this->m_n_particles; i++)
//             {
//                 int index_tmp = x_swarm[i].size() - 1;
//                 double x_tmp = x_swarm[i][index_tmp];
//                 double y_tmp = y_swarm[i][index_tmp];
//                 //                fitness_x[i] = this->m_fitness_w2 * std::fabs(m_pso_target_xy.first  - x_tmp);
//                 //                fitness_y[i] = this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);

//                 fitness_x[i] = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.first - x_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.first - x_tmp);
//                 fitness_y[i] = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.second - y_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);
//             }

//             if (e == 0)
//             {
//                 // Update Perosnal POS_x
//                 double fitness_x_tmp = 1e+10;
//                 for (int i = 0; i < this->m_n_particles; i++)
//                 {
//                     // Update personal pos
//                     p_x[i] = x_swarm[i][0];
//                     min_p_x[i] = fitness_x[i];

//                     // Update global pos 1
//                     if (fitness_x[i] < fitness_x_tmp)
//                     {
//                         fitness_x_tmp = fitness_x[i];
//                         //                        min_g_x = min_p_x[i];
//                         min_g_x_index = i;
//                     }
//                 }

//                 // Update Personal POS_y
//                 double fitness_y_tmp = 1e+10;
//                 for (int i = 0; i < this->m_n_particles; i++)
//                 {
//                     // Update personal pos
//                     p_y[i] = y_swarm[i][0];
//                     min_p_y[i] = fitness_y[i];

//                     // Update global pos 1
//                     if (fitness_y[i] < fitness_y_tmp)
//                     {
//                         fitness_y_tmp = fitness_y[i];
//                         //                        min_g_x = min_p_x[i];
//                         min_g_y_index = i;
//                     }
//                 }

//                 // Update global pos_xy
//                 min_g_x = fitness_x[min_g_x_index];
//                 min_g_y = fitness_x[min_g_y_index];
//                 g_x = p_x[min_g_x_index];
//                 g_y = p_y[min_g_y_index];
//             }
//             else
//             {
//                 // Update personal POS_x
//                 for (int i = 0; i < this->m_n_particles; i++)
//                 {
//                     // If new fitness_x is smaller than existing fitness_x, update personal.
//                     if (min_p_x[i] > fitness_x[i])
//                     {
//                         int index_tmp = x_swarm[i].size() - 1;
//                         min_p_x[i] = fitness_x[i];
//                         p_x[i] = x_swarm[i][index_tmp];
//                     }
//                 }

//                 // Update global pos 1 among personal POS_x
//                 double fitness_x_tmp = 1e+10;
//                 for (int i = 0; i < this->m_n_particles; i++)
//                 {
//                     if (min_p_x[i] < fitness_x_tmp)
//                     {
//                         fitness_x_tmp = min_p_x[i];
//                         //                        min_g_x = min_p_x[i];
//                         min_g_x_index = i;
//                     }
//                 }

//                 // Update personal POS_y
//                 for (int i = 0; i < this->m_n_particles; i++)
//                 {
//                     // If new fitness_y is smaller than existing fitness_y, update personal.
//                     if (min_p_y[i] > fitness_y[i])
//                     {
//                         int index_tmp = y_swarm[i].size() - 1;
//                         min_p_y[i] = fitness_y[i];
//                         p_y[i] = y_swarm[i][index_tmp];
//                     }
//                 }

//                 // Update global pos 1 among personal POS_y
//                 double fitness_y_tmp = 1e+10;
//                 for (int i = 0; i < this->m_n_particles; i++)
//                 {
//                     if (min_p_y[i] < fitness_y_tmp)
//                     {
//                         fitness_y_tmp = min_p_y[i];
//                         //                        min_g_y = min_p_y[i];
//                         min_g_y_index = i;
//                     }
//                 }

//                 // Update global pos 2
//                 min_g_x = min_p_x[min_g_x_index];
//                 min_g_y = min_p_y[min_g_y_index];
//                 g_x = p_x[min_g_x_index];
//                 g_y = p_y[min_g_y_index];
//             }

//             // Update Pos X
//             for (int i = 0; i < this->m_n_particles; i++)
//             {
//                 int index_tmp = x_swarm[i].size() - 1;
//                 int vel_x_index_tmp = vel_x[i].size() - 1;
//                 //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
//                 //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
//                 double p_err = p_x[i] - x_swarm[i][index_tmp];
//                 double g_err = g_x - x_swarm[i][index_tmp];
//                 //                double vel_x_new = this->m_w * vel_x[vel_x.size()-1] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
//                 double vel_x_new = this->m_w * vel_x[i][vel_x_index_tmp] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
//                 if (vel_x_new > 10)
//                     vel_x_new = 10;
//                 if (vel_x_new < -10)
//                     vel_x_new = -10;

//                 double pos_new = vel_x_new + x_swarm[i][index_tmp];
//                 vel_x[i].push_back(vel_x_new);
//                 x_swarm[i].push_back(pos_new);
//             }

//             // Update Pos Y
//             for (int i = 0; i < this->m_n_particles; i++)
//             {
//                 int index_tmp = y_swarm[i].size() - 1;
//                 int vel_y_index_tmp = vel_y[i].size() - 1;

//                 //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
//                 //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
//                 double p_err = p_y[i] - y_swarm[i][index_tmp];
//                 double g_err = g_y - y_swarm[i][index_tmp];

//                 double param_1 = this->m_c1 * get_random_value(0, 1);
//                 double param_2 = this->m_c2 * get_random_value(0, 1);
//                 //                double vel_y_new = this->m_w * vel_y[vel_y.size()-1] + param_1 * p_err + param_2 * g_err;
//                 double vel_y_new = this->m_w * vel_y[i][vel_y_index_tmp] + param_1 * p_err + param_2 * g_err;
//                 if (vel_y_new > 10)
//                     vel_y_new = 10;
//                 if (vel_y_new < -10)
//                     vel_y_new = -10;
//                 //                double vel_y_new = this->m_w * vel_y[vel_y.size()-1] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
//                 double pos_new = vel_y_new + y_swarm[i][index_tmp];
//                 vel_y[i].push_back(vel_y_new);
//                 y_swarm[i].push_back(pos_new);
//             }

//             //            std::cout << "g_x : " << g_x << ", " << g_y << "\t\t min_g_xy : " << min_g_x << ", " << min_g_y << "\t\t" << std::hypot(min_g_x, min_g_y) << std::endl;

//             m_pso_result_path_points.push_back(std::make_pair(g_x, g_y));
//         }
//     }

//     // PSOPlanner(int width_, int height_, int init_pos_x_, int init_pos_y_)
//     // PSOPlanner()
//     // {
//     //     /*
//     //         Swarm 중에서 Personal Best 뽑음
//     //         m_swarm[Swarm 개수][증식 개수].particles[특정 particle]
//     //     */
//     //     this->m_personal_best_array_index;
//     //     this->m_swarm;

//     //     // Particle 개수 초기화 (ex. x, y)
//     //     this->m_particle_size = 2; // x, y

//     //     // Swarm 개수 초기화
//     //     this->m_swarm_size = 5;

//     //     // Swarm 개수 초기화
//     //     // Swarm init_swarm(this->m_particle_size);
//     //     // for (int i = 0; i < this->m_swarm_size; i++)
//     //     //     this->m_swarm.push_back(init_swarm);
//     //     Swarm init_swarm(m_particle_size);           // Swarm 생성 및 Particles 개수 선언
//     //     std::vector<Swarm> propagation_swarm_tmp;    // Swarm 이중 벡터화
//     //     propagation_swarm_tmp.push_back(init_swarm); // Swarm 이중 벡터화
//     //     for (int i = 0; i < this->m_swarm_size; i++)
//     //         this->m_swarm.push_back(propagation_swarm_tmp);
//     //     std::cout << "Swarm 개수 초기화. Swarm 사이즈 : " << this->m_swarm.size() << "\t\t 증식 개수 : " << this->m_swarm[0].size() << std::endl;

//     //     // Personal best 배열 index 초기화
//     //     this->m_personal_best_array_index.assign(this->m_swarm_size, 0);
//     //     std::cout << "Personal Best 배열 인덱스 초기화. 사이즈(Swarm 사이즈) : " << m_personal_best_array_index.size() << std::endl;

//     //     // std::shared_ptr<Particles> current_particle = std::shared_ptr<Particles>(this->m_swarm[0].particles[0]);
//     //     Particles *current_particle = &(this->m_swarm[0][0].particles[0]);

//     //     Swarm new_swarm(this->m_particle_size);
//     //     new_swarm.particles.size();
//     //     new_swarm.particles[0].velocity = current_particle->velocity * this->m_w + this->m_c1 * get_random_value(0, 1) + this->m_c2 * get_random_value(0, 1);
//     //     // Particles current_particle;
//     //     // = std::make_shared<particles>();
//     //     // = this->m_swarm[0].particles[0];
//     //     // new_swarm.particles[0].v = this->m_swarm[0].particles[0];
//     //     // Calc Next Swarm

//     //     // swarm[0].particles[1].v.push_back(2);
//     //     // swarm[0].particles[1].x.push_back(3);

//     //     // Swarm swarm_tmp(10);
//     //     // m_swarm.assign(10, swarm_tmp);
//     //     // m_swarm[0].particles[0].c1;

//     //     // m_swarm[0].particles[0];

//     //     std::pair<double, double> init_pos;
//     //     init_pos = std::make_pair(0, 0);
//     //     this->m_w = 1;  //0.25;
//     //     this->m_c1 = 2; //2;
//     //     this->m_c2 = 2; //2;
//     //     this->m_n_particles = 20;
//     //     this->m_repeat_count = 100;
//     //     this->m_init_flag = true;
//     //     this->m_fitness_w1 = 10;
//     //     this->m_fitness_w2 = 1;

//     //     m_pso_target_xy = std::make_pair(200, 200);

//     //     m_closed_obsts_xy = std::make_pair(50, 50);

//     //     std::vector<double> p_x, p_y;
//     //     std::vector<double> min_p_x, min_p_y;
//     //     p_x.assign(this->m_n_particles, 0);
//     //     p_y.assign(this->m_n_particles, 0);
//     //     min_p_x.assign(this->m_n_particles, 0);
//     //     min_p_y.assign(this->m_n_particles, 0);
//     //     double g_x = 0;
//     //     double g_y = 0;

//     //     //        std::vector<double> vel_x, vel_y;
//     //     //        vel_x.push_back(0); vel_y.push_back(0);
//     //     std::vector<std::vector<double>> vel_x(this->m_n_particles, std::vector<double>(1, 0));
//     //     std::vector<std::vector<double>> vel_y(this->m_n_particles, std::vector<double>(1, 0));

//     //     int min_g_x_index = 0;
//     //     int min_g_y_index = 0;
//     //     double min_g_x = 0;
//     //     double min_g_y = 0;

//     //     // Get X Init Pos Swarm
//     //     std::vector<std::vector<double>> x_swarm(this->m_n_particles, std::vector<double>(1, 0));
//     //     for (int i = 0; i < x_swarm.size(); i++)
//     //         for (int j = 0; j < x_swarm[i].size(); j++)
//     //         {
//     //             x_swarm[i][j] = get_random_value(0, 1) + init_pos.first;
//     //         }

//     //     // Get Y Init Pos Swarm
//     //     std::vector<std::vector<double>> y_swarm(this->m_n_particles, std::vector<double>(1, 0));
//     //     for (int i = 0; i < y_swarm.size(); i++)
//     //         for (int j = 0; j < y_swarm[i].size(); j++)
//     //         {
//     //             y_swarm[i][j] = get_random_value(0, 1) + init_pos.second;
//     //         }

//     //     for (int e = 0; e < this->m_repeat_count; e++)
//     //     {
//     //         // Update fitness_x
//     //         std::vector<double> fitness_x, fitness_y;
//     //         fitness_x.assign(this->m_n_particles, 0);
//     //         fitness_y.assign(this->m_n_particles, 0);
//     //         for (int i = 0; i < this->m_n_particles; i++)
//     //         {
//     //             int index_tmp = x_swarm[i].size() - 1;
//     //             double x_tmp = x_swarm[i][index_tmp];
//     //             double y_tmp = y_swarm[i][index_tmp];
//     //             //                fitness_x[i] = this->m_fitness_w2 * std::fabs(m_pso_target_xy.first  - x_tmp);
//     //             //                fitness_y[i] = this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);

//     //             fitness_x[i] = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.first - x_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.first - x_tmp);
//     //             fitness_y[i] = this->m_fitness_w1 * 1. / std::fabs(m_closed_obsts_xy.second - y_tmp) + this->m_fitness_w2 * std::fabs(m_pso_target_xy.second - y_tmp);
//     //         }

//     //         if (e == 0)
//     //         {
//     //             // Update Perosnal POS_x
//     //             double fitness_x_tmp = 1e+10;
//     //             for (int i = 0; i < this->m_n_particles; i++)
//     //             {
//     //                 // Update personal pos
//     //                 p_x[i] = x_swarm[i][0];
//     //                 min_p_x[i] = fitness_x[i];

//     //                 // Update global pos 1
//     //                 if (fitness_x[i] < fitness_x_tmp)
//     //                 {
//     //                     fitness_x_tmp = fitness_x[i];
//     //                     //                        min_g_x = min_p_x[i];
//     //                     min_g_x_index = i;
//     //                 }
//     //             }

//     //             // Update Personal POS_y
//     //             double fitness_y_tmp = 1e+10;
//     //             for (int i = 0; i < this->m_n_particles; i++)
//     //             {
//     //                 // Update personal pos
//     //                 p_y[i] = y_swarm[i][0];
//     //                 min_p_y[i] = fitness_y[i];

//     //                 // Update global pos 1
//     //                 if (fitness_y[i] < fitness_y_tmp)
//     //                 {
//     //                     fitness_y_tmp = fitness_y[i];
//     //                     //                        min_g_x = min_p_x[i];
//     //                     min_g_y_index = i;
//     //                 }
//     //             }

//     //             // Update global pos_xy
//     //             min_g_x = fitness_x[min_g_x_index];
//     //             min_g_y = fitness_x[min_g_y_index];
//     //             g_x = p_x[min_g_x_index];
//     //             g_y = p_y[min_g_y_index];
//     //         }
//     //         else
//     //         {
//     //             // Update personal POS_x
//     //             for (int i = 0; i < this->m_n_particles; i++)
//     //             {
//     //                 // If new fitness_x is smaller than existing fitness_x, update personal.
//     //                 if (min_p_x[i] > fitness_x[i])
//     //                 {
//     //                     int index_tmp = x_swarm[i].size() - 1;
//     //                     min_p_x[i] = fitness_x[i];
//     //                     p_x[i] = x_swarm[i][index_tmp];
//     //                 }
//     //             }

//     //             // Update global pos 1 among personal POS_x
//     //             double fitness_x_tmp = 1e+10;
//     //             for (int i = 0; i < this->m_n_particles; i++)
//     //             {
//     //                 if (min_p_x[i] < fitness_x_tmp)
//     //                 {
//     //                     fitness_x_tmp = min_p_x[i];
//     //                     //                        min_g_x = min_p_x[i];
//     //                     min_g_x_index = i;
//     //                 }
//     //             }

//     //             // Update personal POS_y
//     //             for (int i = 0; i < this->m_n_particles; i++)
//     //             {
//     //                 // If new fitness_y is smaller than existing fitness_y, update personal.
//     //                 if (min_p_y[i] > fitness_y[i])
//     //                 {
//     //                     int index_tmp = y_swarm[i].size() - 1;
//     //                     min_p_y[i] = fitness_y[i];
//     //                     p_y[i] = y_swarm[i][index_tmp];
//     //                 }
//     //             }

//     //             // Update global pos 1 among personal POS_y
//     //             double fitness_y_tmp = 1e+10;
//     //             for (int i = 0; i < this->m_n_particles; i++)
//     //             {
//     //                 if (min_p_y[i] < fitness_y_tmp)
//     //                 {
//     //                     fitness_y_tmp = min_p_y[i];
//     //                     //                        min_g_y = min_p_y[i];
//     //                     min_g_y_index = i;
//     //                 }
//     //             }

//     //             // Update global pos 2
//     //             min_g_x = min_p_x[min_g_x_index];
//     //             min_g_y = min_p_y[min_g_y_index];
//     //             g_x = p_x[min_g_x_index];
//     //             g_y = p_y[min_g_y_index];
//     //         }

//     //         // Update Pos X
//     //         for (int i = 0; i < this->m_n_particles; i++)
//     //         {
//     //             int index_tmp = x_swarm[i].size() - 1;
//     //             int vel_x_index_tmp = vel_x[i].size() - 1;
//     //             //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
//     //             //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
//     //             double p_err = p_x[i] - x_swarm[i][index_tmp];
//     //             double g_err = g_x - x_swarm[i][index_tmp];
//     //             //                double vel_x_new = this->m_w * vel_x[vel_x.size()-1] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
//     //             double vel_x_new = this->m_w * vel_x[i][vel_x_index_tmp] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
//     //             if (vel_x_new > 10)
//     //                 vel_x_new = 10;
//     //             if (vel_x_new < -10)
//     //                 vel_x_new = -10;

//     //             double pos_new = vel_x_new + x_swarm[i][index_tmp];
//     //             vel_x[i].push_back(vel_x_new);
//     //             x_swarm[i].push_back(pos_new);
//     //         }

//     //         // Update Pos Y
//     //         for (int i = 0; i < this->m_n_particles; i++)
//     //         {
//     //             int index_tmp = y_swarm[i].size() - 1;
//     //             int vel_y_index_tmp = vel_y[i].size() - 1;

//     //             //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
//     //             //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
//     //             double p_err = p_y[i] - y_swarm[i][index_tmp];
//     //             double g_err = g_y - y_swarm[i][index_tmp];

//     //             double param_1 = this->m_c1 * get_random_value(0, 1);
//     //             double param_2 = this->m_c2 * get_random_value(0, 1);
//     //             //                double vel_y_new = this->m_w * vel_y[vel_y.size()-1] + param_1 * p_err + param_2 * g_err;
//     //             double vel_y_new = this->m_w * vel_y[i][vel_y_index_tmp] + param_1 * p_err + param_2 * g_err;
//     //             if (vel_y_new > 10)
//     //                 vel_y_new = 10;
//     //             if (vel_y_new < -10)
//     //                 vel_y_new = -10;
//     //             //                double vel_y_new = this->m_w * vel_y[vel_y.size()-1] + this->m_c1 * get_random_value(0, 1) * p_err + this->m_c2 * get_random_value(0, 1) * g_err;
//     //             double pos_new = vel_y_new + y_swarm[i][index_tmp];
//     //             vel_y[i].push_back(vel_y_new);
//     //             y_swarm[i].push_back(pos_new);
//     //         }

//     //         //            std::cout << "g_x : " << g_x << ", " << g_y << "\t\t min_g_xy : " << min_g_x << ", " << min_g_y << "\t\t" << std::hypot(min_g_x, min_g_y) << std::endl;

//     //         m_pso_result_path_points.push_back(std::make_pair(g_x, g_y));
//     //     }
//     // }
//     ~PSOPlanner()
//     {
//     }

// protected:
// private:
// };

// #endif /* pso_planner_hpp */
