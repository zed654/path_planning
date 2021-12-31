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

// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
std::vector<double> adj_nodes_x;
std::vector<double> adj_nodes_y;

class PSOPlanner
{
public:
    float w, c1, c2;
    int n_particles;
    int repeat_cnt;
    bool init_flag;
    double fitness_w1;
    double fitness_w2;

    std::vector<std::pair<double, double>> pso_result_path_points;
    std::pair<double, double> pso_target_xy;
    std::pair<double, double> closed_obsts_xy;

    std::vector<std::pair<double, double>> get_pso_result_path_points()
    {
        return this->pso_result_path_points;
    }

    std::pair<double, double> get_closed_obsts_xy()
    {
        return this->closed_obsts_xy;
    }
    float GetRandVal(int st_val_, int end_val_) // Get Rand Value ranged st_val_ <= x < end_val_
    {
        if (this->init_flag == true)
        {
            srand((unsigned)time(NULL)); //seed값으로 현재시간 부여
            this->init_flag = false;
        }
        int range = end_val_ - st_val_;
        double rand_val = rand() / (RAND_MAX + 1.0);
        rand_val = rand_val * range;
        rand_val += st_val_;
        return rand_val;
    }
    PSOPlanner(int width_, int height_, int init_pos_x_, int init_pos_y_)
    {
        std::pair<double, double> init_pos;
        init_pos = std::make_pair(0, 0);
        this->w = 1;  //0.25;
        this->c1 = 2; //2;
        this->c2 = 2; //2;
        this->n_particles = 20;
        this->repeat_cnt = 100;
        this->init_flag = true;
        this->fitness_w1 = 10;
        this->fitness_w2 = 1;

        pso_target_xy = std::make_pair(200, 200);

        closed_obsts_xy = std::make_pair(50, 50);

        std::vector<double> p_x, p_y;
        std::vector<double> min_p_x, min_p_y;
        p_x.assign(n_particles, 0);
        p_y.assign(n_particles, 0);
        min_p_x.assign(n_particles, 0);
        min_p_y.assign(n_particles, 0);
        double g_x = 0;
        double g_y = 0;

        //        std::vector<double> vel_x, vel_y;
        //        vel_x.push_back(0); vel_y.push_back(0);
        std::vector<std::vector<double>> vel_x(n_particles, std::vector<double>(1, 0));
        std::vector<std::vector<double>> vel_y(n_particles, std::vector<double>(1, 0));

        int min_g_x_index = 0;
        int min_g_y_index = 0;
        double min_g_x = 0;
        double min_g_y = 0;

        // Get X Init Pos Swarm
        std::vector<std::vector<double>> x_swarm(n_particles, std::vector<double>(1, 0));
        for (int i = 0; i < x_swarm.size(); i++)
            for (int j = 0; j < x_swarm[i].size(); j++)
            {
                x_swarm[i][j] = GetRandVal(0, 1) + init_pos.first;
            }

        // Get Y Init Pos Swarm
        std::vector<std::vector<double>> y_swarm(n_particles, std::vector<double>(1, 0));
        for (int i = 0; i < y_swarm.size(); i++)
            for (int j = 0; j < y_swarm[i].size(); j++)
            {
                y_swarm[i][j] = GetRandVal(0, 1) + init_pos.second;
            }

        for (int e = 0; e < repeat_cnt; e++)
        {
            // Update fitness_x
            std::vector<double> fitness_x, fitness_y;
            fitness_x.assign(n_particles, 0);
            fitness_y.assign(n_particles, 0);
            for (int i = 0; i < n_particles; i++)
            {
                int index_tmp = x_swarm[i].size() - 1;
                double x_tmp = x_swarm[i][index_tmp];
                double y_tmp = y_swarm[i][index_tmp];
                //                fitness_x[i] = this->fitness_w2 * std::fabs(pso_target_xy.first  - x_tmp);
                //                fitness_y[i] = this->fitness_w2 * std::fabs(pso_target_xy.second - y_tmp);

                fitness_x[i] = this->fitness_w1 * 1. / std::fabs(closed_obsts_xy.first - x_tmp) + this->fitness_w2 * std::fabs(pso_target_xy.first - x_tmp);
                fitness_y[i] = this->fitness_w1 * 1. / std::fabs(closed_obsts_xy.second - y_tmp) + this->fitness_w2 * std::fabs(pso_target_xy.second - y_tmp);
            }

            if (e == 0)
            {
                // Update Perosnal POS_x
                double fitness_x_tmp = 1e+10;
                for (int i = 0; i < n_particles; i++)
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
                for (int i = 0; i < n_particles; i++)
                {
                    // Update personal pos
                    p_y[i] = y_swarm[i][0];
                    min_p_y[i] = fitness_y[i]; ///

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
                for (int i = 0; i < n_particles; i++)
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
                for (int i = 0; i < n_particles; i++)
                {
                    if (min_p_x[i] < fitness_x_tmp)
                    {
                        fitness_x_tmp = min_p_x[i];
                        //                        min_g_x = min_p_x[i];
                        min_g_x_index = i;
                    }
                }

                // Update personal POS_y
                for (int i = 0; i < n_particles; i++)
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
                for (int i = 0; i < n_particles; i++)
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
            for (int i = 0; i < n_particles; i++)
            {
                int index_tmp = x_swarm[i].size() - 1;
                int vel_x_index_tmp = vel_x[i].size() - 1;
                //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
                //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
                double p_err = p_x[i] - x_swarm[i][index_tmp];
                double g_err = g_x - x_swarm[i][index_tmp];
                //                double vel_x_new = this->w * vel_x[vel_x.size()-1] + this->c1 * GetRandVal(0, 1) * p_err + this->c2 * GetRandVal(0, 1) * g_err;
                double vel_x_new = this->w * vel_x[i][vel_x_index_tmp] + this->c1 * GetRandVal(0, 1) * p_err + this->c2 * GetRandVal(0, 1) * g_err;
                if (vel_x_new > 10)
                    vel_x_new = 10;
                if (vel_x_new < -10)
                    vel_x_new = -10;

                double pos_new = vel_x_new + x_swarm[i][index_tmp];
                vel_x[i].push_back(vel_x_new);
                x_swarm[i].push_back(pos_new);
            }

            // Update Pos Y
            for (int i = 0; i < n_particles; i++)
            {
                int index_tmp = y_swarm[i].size() - 1;
                int vel_y_index_tmp = vel_y[i].size() - 1;

                //                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
                //                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
                double p_err = p_y[i] - y_swarm[i][index_tmp];
                double g_err = g_y - y_swarm[i][index_tmp];

                double param_1 = this->c1 * GetRandVal(0, 1);
                double param_2 = this->c2 * GetRandVal(0, 1);
                //                double vel_y_new = this->w * vel_y[vel_y.size()-1] + param_1 * p_err + param_2 * g_err;
                double vel_y_new = this->w * vel_y[i][vel_y_index_tmp] + param_1 * p_err + param_2 * g_err;
                if (vel_y_new > 10)
                    vel_y_new = 10;
                if (vel_y_new < -10)
                    vel_y_new = -10;
                //                double vel_y_new = this->w * vel_y[vel_y.size()-1] + this->c1 * GetRandVal(0, 1) * p_err + this->c2 * GetRandVal(0, 1) * g_err;
                double pos_new = vel_y_new + y_swarm[i][index_tmp];
                vel_y[i].push_back(vel_y_new);
                y_swarm[i].push_back(pos_new);
            }

            //            std::cout << "g_x : " << g_x << ", " << g_y << "\t\t min_g_xy : " << min_g_x << ", " << min_g_y << "\t\t" << std::hypot(min_g_x, min_g_y) << std::endl;

            pso_result_path_points.push_back(std::make_pair(g_x, g_y));
            //            if(std::hypot(g_x-pso_target_xy.first, g_y-pso_target_xy.second) < 1)
            //                break;
        }

        //        std::cout << "pso_target_xy : " << pso_target_xy.first << ", " << pso_target_xy.second << std::endl;
        //        for(int i = 0; i < repeat_cnt; i++)
        //        {
        //            std::cout << "Result : " << x_swarm[min_g_x_index][i] << ", " << y_swarm[min_g_x_index][i] << std::endl;
        //        }

        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////

        //        std::pair<double, double> init_pos;
        //        init_pos = std::make_pair(0, 0);
        //        this->w = 0.75; //0.25;
        //        this->c1 = 2; //2;
        //        this->c2 = 2; //2;
        //        this->n_particles = 10;
        //        this->repeat_cnt = 10;
        //        this->init_flag = true;
        //        this->fitness_w1 = 1;
        //        this->fitness_w2 = 1;

        //        std::pair<double, double> pso_target_xy;
        //        pso_target_xy = std::make_pair(11, 23);

        //        std::vector<double> p_x, p_y;
        //        std::vector<double> min_p_x, min_p_y;
        //        p_x.assign(n_particles, 0);         p_y.assign(n_particles, 0);
        //        min_p_x.assign(n_particles, 0);     min_p_y.assign(n_particles, 0);
        //        double g_x = 0;
        //        double g_y = 0;

        //        std::vector<double> vel_x, vel_y;
        //        vel_x.push_back(0); vel_y.push_back(0);

        //        int min_g_index = 0;
        //        double min_g_x = 0;
        //        double min_g_y = 0;

        //        // Get X Init Pos Swarm
        //        std::vector<std::vector<double>> x_swarm(n_particles, std::vector<double>(1, 0));
        //        for(int i = 0; i < x_swarm.size(); i++)
        //            for(int j = 0; j < x_swarm[i].size(); j++)
        //            {
        //                x_swarm[i][j] = GetRandVal(0, 1) + init_pos.first;
        //            }

        //        // Get Y Init Pos Swarm
        //        std::vector<std::vector<double>> y_swarm(n_particles, std::vector<double>(1, 0));
        //        for(int i = 0; i < y_swarm.size(); i++)
        //            for(int j = 0; j < y_swarm[i].size(); j++)
        //            {
        //                y_swarm[i][j] = GetRandVal(0, 1) + init_pos.second;
        //            }

        //        for(int e = 0; e < repeat_cnt; e++)
        //        {
        //            // Update Fitness
        //            std::vector<double> fitness;
        //            fitness.assign(n_particles, 0);
        //            for(int i = 0; i < n_particles; i++)
        //            {
        //                int index_tmp = x_swarm[i].size()-1;
        //                double x_tmp = x_swarm[i][index_tmp];
        //                double y_tmp = y_swarm[i][index_tmp];
        ////                fitness[i] = this->fitness_w1 * 1. / () + this->fitness_w2 * std::hypot(pso_target_xy.first-x_tmp, pso_target_xy.second-y_tmp);
        //                fitness[i] = this->fitness_w2 * std::hypot(pso_target_xy.first-x_tmp, pso_target_xy.second-y_tmp);
        ////                fitness[i] = std::pow(std::pow(x_tmp+1, 3) + 1, 2);
        //            }

        //            if(e == 0)
        //            {
        //                double fitness_tmp = 1e+10;
        //                for(int i = 0; i < n_particles; i++)
        //                {
        //                    // Update personal pos
        //                    p_x[i] = x_swarm[i][0];
        //                    p_y[i] = y_swarm[i][0];
        //                    min_p_x[i] = fitness[i];
        //                    min_p_y[i] = fitness[i];

        //                    // Update global pos 1
        //                    if(fitness[i] < fitness_tmp)
        //                    {
        //                        fitness_tmp = fitness[i];
        ////                        min_g_x = min_p_x[i];
        //                        min_g_index = i;
        //                    }
        //                }

        //                // Update global pos 2
        //                min_g_x = fitness[min_g_index];
        //                min_g_y = fitness[min_g_index];
        //                g_x = p_x[min_g_index];
        //                g_y = p_y[min_g_index];
        //            }
        //            else
        //            {
        //                // Update personal POSs
        //                for(int i = 0; i < n_particles; i++)
        //                {
        //                    // If new fitness is smaller than existing fitness, update personal.
        //                    if(min_p_x[i] > fitness[i])
        //                    {
        //                        int index_tmp = x_swarm[i].size()-1;
        //                        min_p_x[i] = fitness[i];
        //                        min_p_y[i] = fitness[i];
        //                        p_x[i] = x_swarm[i][index_tmp];
        //                        p_y[i] = y_swarm[i][index_tmp];
        //                    }
        //                }

        //                // Update global pos 1 among personal POSs
        //                double fitness_tmp = 1e+10;
        //                for(int i = 0; i < n_particles; i++)
        //                {
        //                    if(min_p_x[i] < fitness_tmp)
        //                    {
        //                        fitness_tmp = min_p_x[i];
        ////                        min_g_x = min_p_x[i];
        //                        min_g_index = i;
        //                    }
        //                }

        //                // Update global pos 2
        //                min_g_x = min_p_x[min_g_index];
        //                g_x = p_x[min_g_index];
        //                g_y = p_y[min_g_index];
        //            }

        //            // Update Pos X
        //            for(int i = 0; i < n_particles; i++)
        //            {
        //                int index_tmp = x_swarm[i].size()-1;
        ////                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
        ////                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
        //                double p_err = p_x[i] - x_swarm[i][index_tmp];
        //                double g_err = g_x - x_swarm[i][index_tmp];
        //                double vel_x_new = this->w * vel_x[vel_x.size()-1] + this->c1 * GetRandVal(0, 1) * p_err + this->c2 * GetRandVal(0, 1) * g_err;
        //                double pos_new = vel_x_new + x_swarm[i][index_tmp];
        //                vel_x.push_back(vel_x_new);
        //                x_swarm[i].push_back(pos_new);
        //            }

        //            // Update Pos Y
        //            for(int i = 0; i < n_particles; i++)
        //            {
        //                int index_tmp = y_swarm[i].size()-1;
        ////                double p_err = std::hypot(p_x[i]-x_swarm[i][index_tmp], p_y[i]-y_swarm[i][index_tmp]);
        ////                double g_err = std::hypot(g_x-x_swarm[i][index_tmp], g_y-y_swarm[i][index_tmp]);
        //                double p_err = p_y[i] - y_swarm[i][index_tmp];
        //                double g_err = g_y - y_swarm[i][index_tmp];

        //                double param_1 = this->c1 * GetRandVal(0, 1);
        //                double param_2 = this->c2 * GetRandVal(0, 1);
        //                double vel_y_new = this->w * vel_y[vel_y.size()-1] + param_1 * p_err + param_2 * g_err;
        ////                double vel_y_new = this->w * vel_y[vel_y.size()-1] + this->c1 * GetRandVal(0, 1) * p_err + this->c2 * GetRandVal(0, 1) * g_err;
        //                double pos_new = vel_y_new + y_swarm[i][index_tmp];
        //                vel_y.push_back(vel_y_new);
        //                y_swarm[i].push_back(pos_new);
        //            }

        //            std::cout << "g_x : " << g_x << ", " << g_y << "\t\t min_g_x : " << min_g_x << std::endl;

        //            if(std::hypot(g_x-pso_target_xy.first, g_y-pso_target_xy.second) < 1)
        //                break;
        //        }

        //        std::cout << "pso_target_xy : " << pso_target_xy.first << ", " << pso_target_xy.second << std::endl;
        //        for(int i = 0; i < repeat_cnt; i++)
        //        {
        //            std::cout << "Result : " << x_swarm[min_g_index][i] << ", " << y_swarm[min_g_index][i] << std::endl;
        //        }

        ////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////

        //        std::vector<double> p_pos;
        //        p_pos.assign(n_particles, 0);
        //        std::vector<double> min_p_pos;
        //        min_p_pos.assign(n_particles, 0);
        //        double g_pos = 0;

        //        std::vector<double> vel;
        //        vel.push_back(0);

        //        int min_g_index = 0;
        //        double min_g_pos = 0;

        //        // Get Init Pos Swarm
        //        std::vector<std::vector<double>> x_swarm(10, std::vector<double>(1, 0));
        //        for(int i = 0; i < x_swarm.size(); i++)
        //            for(int j = 0; j < x_swarm[i].size(); j++)
        //            {
        //                x_swarm[i][j] = GetRandVal(0, 1) + 5;
        //            }

        //        for(int e = 0; e < repeat_cnt; e++)
        //        {
        //            // Update Fitness
        //            std::vector<double> fitness;
        //            fitness.assign(n_particles, 0);
        //            for(int i = 0; i < fitness.size(); i++)
        //            {
        //                int index_tmp = x_swarm[i].size()-1;
        //                double x_tmp = x_swarm[i][index_tmp];
        //                fitness[i] = std::pow(std::pow(x_tmp+1, 3) + 1, 2);
        //            }

        //            if(e == 0)
        //            {
        //                // Update p_pos
        //                double fitness_tmp = 1e+10;
        //                for(int i = 0; i < n_particles; i++)
        //                {
        //                    p_pos[i] = x_swarm[i][0];
        //                    min_p_pos[i] = fitness[i];
        //                    if(min_p_pos[i] < fitness_tmp)
        //                    {
        //                        fitness_tmp = min_p_pos[i];
        ////                        min_g_pos = min_p_pos[i];
        //                        min_g_index = i;
        //                    }
        //                }

        //                // Update g_pos
        //                min_g_pos = min_p_pos[min_g_index];
        //                g_pos = p_pos[min_g_index];
        //            }
        //            else
        //            {
        //                double fitness_tmp = 1e+10;
        //                for(int i = 0; i < n_particles; i++)
        //                {
        //                    if(min_p_pos[i] > fitness[i])
        //                    {
        //                        int index_tmp = x_swarm[i].size()-1;
        //                        min_p_pos[i] = fitness[i];
        //                        p_pos[i] = x_swarm[i][index_tmp];
        //                    }
        //                }
        //                for(int i = 0; i < n_particles; i++)
        //                {
        //                    if(min_p_pos[i] < fitness_tmp)
        //                    {
        //                        fitness_tmp = min_p_pos[i];
        ////                        min_g_pos = min_p_pos[i];
        //                        min_g_index = i;
        //                    }
        //                }

        //                // Update g_pos
        //                min_g_pos = min_p_pos[min_g_index];
        //                g_pos = p_pos[min_g_index];
        //            }

        //            for(int i = 0; i < n_particles; i++)
        //            {
        //                int index_tmp = x_swarm[i].size()-1;
        //                double p_err = p_pos[i] - x_swarm[i][index_tmp];
        //                double g_err = g_pos - x_swarm[i][index_tmp];
        //                double vel_new = this->w * vel[vel.size()-1] + this->c1 * GetRandVal(0, 1) * p_err + this->c2 * GetRandVal(0, 1) * g_err;
        //                double pos_new = 0.9 * vel_new + x_swarm[i][index_tmp];
        //                vel.push_back(vel_new);
        //                x_swarm[i].push_back(pos_new);
        //            }

        //            std::cout << "g_pos : " << g_pos << "\t\t min_g_pos : " << min_g_pos << std::endl;
        //        }
    }
    ~PSOPlanner()
    {
    }

protected:
private:
};

#endif /* pso_planner_hpp */
