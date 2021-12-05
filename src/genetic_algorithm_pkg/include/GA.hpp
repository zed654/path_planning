#ifndef GA_hpp
#define GA_hpp


#include <iostream>
#include <vector>
#include <cmath>
#include <unistd.h>

// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
std::vector<double> adj_nodes_x;
std::vector<double> adj_nodes_y;

struct Node
{
    struct Cost
    {
        double g; // The sum of the calcuated costs
        double h; // The cost which will be spend
        double f; // The sum of the g and h
    };
    struct Pos
    {
        double x;
        double y;
    };
    struct Cost cost;
    struct Pos pos;
    bool close_flag;
    struct Node *ptr_parent_node;
};

class Astar
{
public:
    std::vector<struct Node> node;                 // Node
    std::vector<struct Node *> ptr_open_node_list; // Open Node List
    std::vector<double> local_x;
    std::vector<double> local_y;
    int grid_width;
    int grid_height;
    int cur_pos_x;
    int cur_pos_y;
    int init_pos_index;
    double threshold_distance_for_stop;

    Astar(int width_, int height_, int init_pos_x_, int init_pos_y_)
    {
        // Clipping
        if (width_ % 2 != 1)
        {
            std::cout << std::endl
                      << "Error from Astar's constructor" << std::endl;
            std::cout << "The width of Astar's Occupancy Grid Map have to set as odd number" << std::endl
                      << std::endl;
            exit(1);
        }

        this->grid_width = width_;
        this->grid_height = height_;
        this->threshold_distance_for_stop = 1; // 0.1 meter

        // Set the Size of Occupancy Grid Map
        this->node.assign(width_ * height_, {{0, 0, 0}, {0, 0}, 0, {}});

        // 초기 위치에서의 cost 계산 (xy는 Occupancy Grid Map 좌표계)
        int init_pos_x = init_pos_x_;
        int init_pos_y = init_pos_y_;
        this->init_pos_index = init_pos_x + (init_pos_y * width_);

        // 초기 위치 입력
        for (int i = 0; i < width_; i++)
            for (int j = 0; j < height_; j++)
            {
                node[i + (j * width_)].pos.x = i;
                node[i + (j * width_)].pos.y = j;
            }

        // 초기 위치를 Open Node List에 넣음.
        this->ptr_open_node_list.push_back(&this->node[init_pos_index]);

        // 초기 위치의 부모를 자신으로 설정
        this->node[init_pos_index].ptr_parent_node = &this->node[init_pos_index];
    }
    ~Astar()
    {
    }

    void ReturnPath(struct Node *arrival_node_)
    {
        if (&this->node[init_pos_index] != arrival_node_)
        {
            local_x.push_back(arrival_node_->pos.x);
            local_y.push_back(arrival_node_->pos.y);

            ReturnPath(arrival_node_->ptr_parent_node);
        }
    }

    bool GetResult(int target_x_, int target_y_)
    {
        double target_x_tmp = target_x_;
        double target_y_tmp = target_y_;

        // Open Node List 중 가장 작은 cost.f값의 인덱스 구하기 (A)
        double tmp = 65000.;
        int cur_i = 0;
        for (int i = 0; i < this->ptr_open_node_list.size(); i++)
        {
            if (tmp > this->ptr_open_node_list[i]->cost.f)
            {
                tmp = this->ptr_open_node_list[i]->cost.f;
                cur_i = i;
            }
        }

        // 최종 결과 반환
        if (this->ptr_open_node_list[cur_i]->cost.h < this->threshold_distance_for_stop) // cost.h는 목표지점까지의 uc_distance
        {
            // Cost값 0을 갖고있는 초기 위치가 아닐 때,
            if (this->ptr_open_node_list[cur_i] != &this->node[init_pos_index])
            {
                // Local Path 반환
                // 초기 위치의 주소 : &this->node[init_pos_index]
                // 현재 위치의 주소 : this->ptr_open_node_list[cur_i]
                // 현재-1 위치의 주소 : this->ptr_open_node_list[cur_i]->ptr_parent_node
                ReturnPath(this->ptr_open_node_list[cur_i]);

                return true;
            }
        }

        // (A)를 Closed List로
        this->ptr_open_node_list[cur_i]->close_flag = true;

        // (A)의 pos_xy값
        int cur_x_tmp = this->ptr_open_node_list[cur_i]->pos.x;
        int cur_y_tmp = this->ptr_open_node_list[cur_i]->pos.y;

        // (A)의 주변 8개에 대하여 작업
        int i_array[] = {cur_x_tmp + 1 + ((cur_y_tmp - 1) * this->grid_width),
                         cur_x_tmp + 1 + ((cur_y_tmp + 0) * this->grid_width),
                         cur_x_tmp + 1 + ((cur_y_tmp + 1) * this->grid_width),
                         cur_x_tmp + ((cur_y_tmp - 1) * this->grid_width),
                         cur_x_tmp + ((cur_y_tmp + 1) * this->grid_width),
                         cur_x_tmp - 1 + ((cur_y_tmp - 1) * this->grid_width),
                         cur_x_tmp - 1 + ((cur_y_tmp + 0) * this->grid_width),
                         cur_x_tmp - 1 + ((cur_y_tmp + 1) * this->grid_width)};

        // Open Node List의 개수를 받아, 해당 노드에 대해서만 8방향으로 접근하기 위한 변수.
        // 만일 변수를 따로 받지 않으면, 아래 부분 this->ptr_open_node_list.push에 의해 계산할수록 크기가 커짐.
        int open_node_check_size_tmp = this->ptr_open_node_list.size();

        // (A) 기준 여덟 방향의 노드들을 셋팅 (OpenNodeList에 넣기 & Cost계산 & Parent설정)
        for (int i = 0; i < 8; i++)
        {
            // Clipping for array size
            if (cur_x_tmp <= 0 | cur_x_tmp >= (this->grid_width - 1))
                continue;
            if (cur_y_tmp < 0 | cur_y_tmp >= (this->grid_height - 1))
                continue;
            if ((i_array[i] < 0) | (i_array[i] > this->grid_width * this->grid_height))
                continue;

            struct Node *ptr_open_node_cand_tmp = &this->node[i_array[i]];

            // (A)기준 여덟 방향의 노드들이 Closed Node면 계산하지 않는다.
            if (ptr_open_node_cand_tmp->close_flag == true)
                continue;

            // 8개의 후보가 Open Node Lists에 이미 올라와 있는애면 true
            bool open_node_list_compare_flag = false;
            for (int j = 0; j < open_node_check_size_tmp; j++)
                if (ptr_open_node_cand_tmp == this->ptr_open_node_list[j])
                    open_node_list_compare_flag = true;

            // 후보가 이미 Open Node Lists에 있으면
            if (open_node_list_compare_flag == true)
            {
                // 새로운 Cost.g 계산
                double cur_node_to_8_cand_node_distance = std::sqrt(std::pow((double)cur_x_tmp - ptr_open_node_cand_tmp->pos.x, 2) + std::pow((double)cur_y_tmp - ptr_open_node_cand_tmp->pos.y, 2));
                double new_cost_h_mh_distance = std::abs((double)target_x_tmp - ptr_open_node_cand_tmp->pos.x) + std::abs((double)target_y_tmp - ptr_open_node_cand_tmp->pos.y); // Manhattan Distance
                double new_cost_h_ec_distance = std::sqrt(std::pow((double)target_x_tmp - ptr_open_node_cand_tmp->pos.x, 2) + std::pow((double)target_y_tmp - ptr_open_node_cand_tmp->pos.y, 2)); // Euclidian Distance
                double new_cost_g_tmp = (this->ptr_open_node_list[cur_i]->cost.g + cur_node_to_8_cand_node_distance);// * 10;
                double new_cost_h_tmp = new_cost_h_mh_distance;
                double new_cost_f_tmp = new_cost_g_tmp + new_cost_h_tmp;

                // if(기존 cost.g > 새로운 cost.g)
                // 새로운 Cost가 더 작으면, cost 교체하고 부모노드 바꿈.
                // TODO: >를 >=로 바꿔서 돌려보기.
                if (ptr_open_node_cand_tmp->cost.g >= new_cost_g_tmp)
                {
                    // Cost를 새로 계산한 코스트들로 변경
                    ptr_open_node_cand_tmp->cost.g = new_cost_g_tmp;
                    ptr_open_node_cand_tmp->cost.h = new_cost_h_tmp;
                    ptr_open_node_cand_tmp->cost.f = new_cost_f_tmp;

                    // (A) 를 부모로
                    ptr_open_node_cand_tmp->ptr_parent_node = ptr_open_node_list[cur_i];
                }
            }
            else // 후보들이 Open Node Lists에 없던 애들이면
            {
                // Cost f, g, h 계산
                double cur_node_to_8_cand_node_distance = std::sqrt(std::pow((double)cur_x_tmp - ptr_open_node_cand_tmp->pos.x, 2) + std::pow((double)cur_y_tmp - ptr_open_node_cand_tmp->pos.y, 2));
                double new_cost_h_mh_distance = std::abs((double)target_x_tmp - ptr_open_node_cand_tmp->pos.x) + std::abs((double)target_y_tmp - ptr_open_node_cand_tmp->pos.y); // Manhattan Distance
                double new_cost_h_ec_distance = std::sqrt(std::pow((double)target_x_tmp - ptr_open_node_cand_tmp->pos.x, 2) + std::pow((double)target_y_tmp - ptr_open_node_cand_tmp->pos.y, 2)); // Euclidian Distance
                ptr_open_node_cand_tmp->cost.g = (this->ptr_open_node_list[cur_i]->cost.g + cur_node_to_8_cand_node_distance);// * 10;
                ptr_open_node_cand_tmp->cost.h = new_cost_h_mh_distance;
                ptr_open_node_cand_tmp->cost.f = ptr_open_node_cand_tmp->cost.h + ptr_open_node_cand_tmp->cost.g;

                // Open Node List에 넣기
                this->ptr_open_node_list.push_back(ptr_open_node_cand_tmp);

                // (A) 를 부모로
                ptr_open_node_cand_tmp->ptr_parent_node = ptr_open_node_list[cur_i];
            }

            // FIXME: Set to visualize
            // FIXME: Set to visualize
            // FIXME: Set to visualize
            // FIXME: Set to visualize
            // FIXME: Set to visualize
            adj_nodes_x.push_back(ptr_open_node_cand_tmp->pos.x);
            adj_nodes_y.push_back(ptr_open_node_cand_tmp->pos.y);
        }

        // (A)를 OpenNodeList에서 제거
        this->ptr_open_node_list.erase(this->ptr_open_node_list.begin() + cur_i);

        return false;
    }

protected:
private:
};


#endif /* GA_hpp */
