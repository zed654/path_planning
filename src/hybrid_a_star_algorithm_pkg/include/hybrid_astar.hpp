#ifndef hybrid_astar_hpp
#define hybrid_astar_hpp

#include <iostream>
#include <vector>
#include <cmath>
#include <unistd.h>

#include <Eigen-3.3/Eigen/Core>
#include <Eigen-3.3/Eigen/QR>

// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// FIXME: Set to Visualize
// For visualizer
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <monitor.h>

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
        double xn; // n -> now
        double yn;
        double xc; // c -> center
        double yc;
    };
    struct Cost cost;
    struct Pos pos;
    int cur_address;
    int parent_address;
};

// Process : NewNode -> 반올림(node[i].xn, node[i].yn)의 closed_flag -> 진행
// x = 반올림(node[i].xn)
// y = 반올림(node[i].yn)
// if(op[x][y].closed == true) ?
//  stop to propagate
// else
//  go propagation

// NewNode 증식 후에,
// x = 반올림(node[i].xc)
// y = 반올림(node[i].yc)
// op[x][y].closed = true;

// op[x][y].closed 는 어떻게 구성?
// x, y의 크기는 충분히 커야함.
// occupancy_points_flag[x][u] = 0;
// Node 안에 adj_occupancy_point 로 이동.
// 해당 point가 closed 되어있는지 확인.
// 기존 closed는? 제거.>?
class HybridAstar
{
public:
    std::vector<struct Node> node; // Node

    // Hybrid A*의 Node Renew는 Continous함. 따라서 Closed_flag를 위하여 Discrete한 Occupancy Grid Points를 사용.
    // oc_closed_flag가 1이면, 인접 Node들이 Closed Node 상태를 의미.
    // Renewed Node는 소수점 반올림을 통해 oc_closed_flag가 정의됨.
    std::vector<std::vector<bool>> oc_closed_flag;
    int oc_size_row;
    int oc_size_col;
    int negative_val_offset;
    Eigen::VectorXd WPs_coeff;
    double init_offset_x;
    double init_offset_y;
    // Result
    std::vector<double> local_x;
    std::vector<double> local_y;
    int grid_width;
    int grid_height;
    int cur_pos_x;
    int cur_pos_y;
    int init_pos_index;
    double threshold_distance_for_stop;

    double length_arc;

    double target_x;
    double target_y;

    // For Debug
    // For Debug
    // FIXME: 얘를 전역으로 빼면, mpc 연산(Vehicle Control단)에서 애러 발생
    cv::Mat test_img;
    std::vector<double> adj_nodes_x;
    std::vector<double> adj_nodes_y;
    std::vector<double> adj_nodes_x2;
    std::vector<double> adj_nodes_y2;
    std::vector<double> adj_nodes_x3;
    std::vector<double> adj_nodes_y3;
    //Eigen::VectorXd WPs_coeff;
    HybridAstar(double init_x_, double init_y_, double target_x_, double target_y_)
        : threshold_distance_for_stop(10),
          init_pos_index(0),
          oc_size_row(5000),
          oc_size_col(5000),
          negative_val_offset(3000),
          length_arc(10.3), //length_arc(5.3) (10.3) (20.3)
          init_offset_x(init_x_),
          init_offset_y(init_y_)
    {
        // ColxRow 크기의 Occuapcny Grid Map 선언 및 Closed Node 상태를 False로 선언.
        // oc_closed_flag[x][y] 이고, x의 사이즈는 oc_size_row, y의 사이즈는 oc_size_col
        oc_closed_flag.assign(oc_size_row, std::vector<bool>(oc_size_col, false));

        target_x = target_x_;
        target_y = target_y_;

        Node init_node_tmp;
        init_node_tmp.pos = {init_offset_x, init_offset_y, init_offset_x, init_offset_y - 800}; // Init Pos {xn, yn, xc, yc}
        // FIXME:
        // init_node_tmp.pos = {init_offset_x, init_offset_y, init_offset_x + 800, init_offset_y}; // Init Pos {xn, yn, xc, yc}
        init_node_tmp.cur_address = 0;
        init_node_tmp.parent_address = 0;
        init_node_tmp.cost.g = 0;
        init_node_tmp.cost.h = 0; //std::sqrt(pow(init_node_tmp.pos.xn - this->target_x, 2) + std::pow(init_node_tmp.pos.yn - this->target_y, 2));
        init_node_tmp.cost.f = 0; //init_node_tmp.cost.h + init_node_tmp.cost.g;

        // 초기 노드를 선언함
        this->node.push_back(init_node_tmp);

        // std::cout << "initialized" << std::endl;

        test_img = cv::Mat(2000, 2000, CV_8UC3);
        test_img.setTo(255);

#ifdef __GET_MIN_TURNING_RADIUS__
        double lr = 3.14;
        double lf = 1.56;
        double delta_f = M_PI / 6;
        double beta_tmp = std::atan(lr * std::tan(delta_f) / (lf + lr));
        beta_tmp = beta_tmp * M_PI / 180.; // 얘가 맞는지는 모르겠음. 근데 안해주면 너무 큼.
        double radius_tmp = std::sin(M_PI / 2 - delta_f) / std::sin(delta_f - beta_tmp) * lf;
        std::cout << "radius : " << radius_tmp << std::endl;
#endif
    }

    ~HybridAstar()
    {
    }

    bool Run(Eigen::VectorXd coeffs_, std::vector<std::pair<double, double>> obstacles_)
    {
        // Open Node List 중 가장 작은 cost.f값의 인덱스 구하기 (A)
        double tmp = 0;
        int cur_i = 0;
        int init_flag_tmp = true;
        bool node_opend_flag = false;
        for (int i = 0; i < this->node.size(); i++)
        {
            // Call the Closed State of Nodes
            int oc_x_tmp = std::round(this->node[i].pos.xn);
            int oc_y_tmp = std::round(this->node[i].pos.yn);
            if (oc_y_tmp < 0 | oc_x_tmp < 0)
            {
                std::cout << "Occupancy Grid Map Overflow" << std::endl;
                std::cout << "oc_xy_tmp : (" << oc_x_tmp << ", " << oc_y_tmp << ")" << std::endl;
            }
            if (oc_x_tmp < 0)
                oc_x_tmp += this->negative_val_offset;
            if (oc_y_tmp < 0)
                oc_y_tmp += this->negative_val_offset;

            // Open Node List 조건
            // if (this->node[i].close_flag == false)
            if (this->oc_closed_flag[oc_x_tmp][oc_y_tmp] == false)
            {
                node_opend_flag = true;
                // std::cout << "있음" << std::endl;
                if (init_flag_tmp == true)
                {
                    tmp = this->node[i].cost.f;
                    cur_i = i;
                    init_flag_tmp = false;
                }
                else if (tmp > this->node[i].cost.f)
                {
                    tmp = this->node[i].cost.f;
                    cur_i = i;
                }
            }
        }

        // 최종 결과 반환
        // If (Open Node List). node의 Index cur_i는 open_node_list일 수 밖에 없음.
        // if (this->node[cur_i].cost.h < this->threshold_distance_for_stop) // cost.h는 목표지점까지의 uc_distance
        double tmp_thresh2 = EuclideanDistance(this->node[cur_i].pos.xn - target_x, this->node[cur_i].pos.yn - target_y);
//        std::cout << (this->node[0].pos.xn-1000)/10 << "\t\t" << (this->node[0].pos.yn-1000)/10 << std::endl;
//        std::cout << (this->node[this->node.size()/2].pos.xn-1000)/10 << "\t\t" << (this->node[this->node.size()/2].pos.yn-1000)/10 << std::endl;
//        std::cout << (this->node[this->node.size()-1].pos.xn-1000)/10 << "\t\t" << (this->node[this->node.size()-1].pos.yn-1000)/10 << std::endl;
//        std::cout << "PolyEval Test :" << (PolyEval(coeffs_, 3)-1000)/10 << "\t\t" << PolyEval(coeffs_, 10) << "\t\t" << PolyEval(coeffs_, 15) << std::endl;
//        for(int i = 17; i < 70; i++)
//            std::cout << (PolyEval(coeffs_, i*10+1000)-1000)/10 << "\t\t";
//        std::cout << std::endl;

//        std::cout << "PolyEval Test :" << (PolyEval(coeffs_, 1170)-1000)/10 << "\t\t" << (PolyEval(coeffs_, 1300)-1000)/10 << "\t\t" << (PolyEval(coeffs_, 1800)-1000)/10 << std::endl;

//        std::cout << this->node[cur_i].pos.xn << "\t\t" << target_x << "\t\t" << this->node[cur_i].pos.yn << "\t\t" << target_y << std::endl;
        // double tmp_thresh2 = std::abs(this->node[cur_i].pos.xn - target_x);
        double tmp_thresh = EuclideanDistance(this->node[cur_i].pos.xn - 1000, this->node[cur_i].pos.yn - 1000); // Occupancy Grid를 넘어가는 현상을 막기 위하여 Grid의 크기인 1000을 사용.
        // if ((tmp_thresh < this->threshold_distance_for_stop) | (node_opend_flag == false)) // cost.h는 목표지점까지의 uc_distance
        // 첫 번째 인자와 두 번째 인자는 Cliping임. 세 번째 인자가 목표지점까지의 거리 기반.
        if ((1000 - tmp_thresh < this->threshold_distance_for_stop) | (node_opend_flag == false) | (tmp_thresh2 < this->threshold_distance_for_stop)) // cost.h는 목표지점까지의 uc_distance
        // if (tmp_thresh < this->threshold_distance_for_stop) // cost.h는 목표지점까지의 uc_distance
        {
            if (node_opend_flag == false)
            {
                std::cout << "LPP System Failure. All Node is Closed. Return the path and Renew" << std::endl;
            }
            // Cost값 0을 갖고있는 초기 위치가 아닐 때,
            if (&this->node[cur_i] != &this->node[init_pos_index])
            {
                // Local Path 반환
                // 초기 위치의 주소 : &this->node[init_pos_index].cur_address
                // 현재 위치의 주소 : this->node[cur_i].cur_address
                // 현재-1 위치의 주소 : this->node[cur_i]->parent_address
                ReturnPath(&this->node[cur_i]);

                return true;
            }
        }

        // TimeChecker tc;
        // tc.DeparturePointTime();
        // (A)위치에서 Kinematic Model 기반의 Neighborhood Nodes를 구하고, node에에 넣음
        std::vector<struct Node> updated_node_tmp = UpdateNode(&this->node[cur_i], coeffs_, obstacles_); // Set New Node
        node.insert(node.end(), updated_node_tmp.begin(), updated_node_tmp.end());                       // 주의 : vector의 insert, push_back 사용 시시 기존 주소가 변함함.
        // std::cout << "1 : " << tc.ArrivalPointTime() << std::endl;
        // New Nodes사이의 거리를 출력
        // for(int i = 0; i < updated_node_tmp.size()-1; i++)
        // {
        //     for(int j = i; j < updated_node_tmp.size(); j++)
        //     {
        //         double x_tmp = updated_node_tmp[i].pos.xn - updated_node_tmp[j].pos.xn;
        //         double y_tmp = updated_node_tmp[i].pos.yn - updated_node_tmp[j].pos.yn;

        //         std::cout << std::sqrt(std::pow(x_tmp,2) + std::pow(y_tmp,2)) << std::endl;
        //     }
        // }
        // std::cout << std::endl;
        // for (int i = 0; i < updated_node_tmp.size(); i++)
        // {
        //         double x_tmp = updated_node_tmp[i].pos.xn;
        //         double y_tmp = updated_node_tmp[i].pos.yn;
        //         std::cout << x_tmp << "\t\t" << y_tmp << std::endl;

        // }
        // std::cout << std::endl;
        //////////////////////////////////
        //////////////////////////////////
        // Debug (OpenCV Visualization) //
        //////////////////////////////////
        //////////////////////////////////
        // double offset_tmp = 4;
        // // 현재 노드 출력 (검은색)
        // cv::circle(test_img, cv::Point((this->node[cur_i].pos.xn - 900) * offset_tmp, (this->node[cur_i].pos.yn - 900) * offset_tmp), 10, cv::Scalar(0, 0, 0), 10);
        // // New Node 출력 (초록색)
        // // std::cout << "Size of New Node : " << updated_node_tmp.size() << std::endl;
        // for (int i = 0; i < updated_node_tmp.size(); i++)
        // {
        //     cv::circle(test_img, cv::Point((updated_node_tmp[i].pos.xn - 900) * offset_tmp, (updated_node_tmp[i].pos.yn - 900) * offset_tmp), 5, cv::Scalar(0, 255, 0), 1);
        //     double x_tmp = std::round(updated_node_tmp[i].pos.xn);
        //     double y_tmp = std::round(updated_node_tmp[i].pos.yn);
        //     // std::cout << "New Node Closed : " << this->oc_closed_flag[x_tmp][y_tmp] << std::endl;

        //     adj_nodes_x3.push_back(updated_node_tmp[i].pos.xn);
        //     adj_nodes_y3.push_back(updated_node_tmp[i].pos.yn);
        // }
        // // 전체 방문한 노드 출력 (진한청색)
        // adj_nodes_x2.push_back(this->node[cur_i].pos.xn);
        // adj_nodes_y2.push_back(this->node[cur_i].pos.yn);
        // for (int i = 0; i < adj_nodes_x2.size(); i++)
        // {
        //     cv::circle(test_img, cv::Point((adj_nodes_x2[i] - 900) * offset_tmp, (adj_nodes_y2[i] - 900) * offset_tmp), 2, cv::Scalar(100, 100, 0), 2);
        // }
        // // 모든 New Node들 출력
        // for (int i = 0; i < adj_nodes_x3.size(); i++)
        // {
        //     cv::circle(test_img, cv::Point((adj_nodes_x3[i] - 900) * offset_tmp, (adj_nodes_y3[i] - 900) * offset_tmp), 1, cv::Scalar(0, 0, 255), 1);
        // }

        // std::cout << "Size of All Nodes : " << node.size() << std::endl;
        // for (int i = 0; i < node.size(); i++)
        // {
        //     double x_tmp = std::round(node[i].pos.xn);
        //     double y_tmp = std::round(node[i].pos.yn);
        //     std::cout << "All Node Closed : " << this->oc_closed_flag[x_tmp][y_tmp] << "\t\tCost f : " << node[i].cost.f << "\t\tPos : (" << (node[i].pos.xn - 1000) / 10. << ", " << (node[i].pos.yn - 1000) / 10. << ")" << std::endl;
        // }
        // // std::cout << std::endl;
        // cv::imshow("test2232", test_img);
        // cv::waitKey(1);
        // test_img.setTo(255);
        // usleep(10);
        // usleep(50000);

        // (A) 위치에서 생성한 Neighborhood Nodes들 중, 장애물을 맞닥드린 Nodes는 제거.
        // TODO: (A) 위치에서 생성한 Neighborhood Nodes들 중, 장애물을 맞닥드린 Nodes는 제거.
        // TODO: Hybrid A*는 겹치는 부분에 의해 Cost가 다시 갱신되는 경우가 없나?? 어떻게 하나?? 그냥 무시하고??
        // -> 무시해도 될 듯. 이 부분이 연산량을 늘리는 부분일 듯 함. 중복되는 경우가 있다면, Kinematic Model 계산에 의해 0.00x 이상 차이가 날 것.

        // Call the Closed State of Nodes
        int oc_x_tmp = std::round(this->node[cur_i].pos.xn);
        int oc_y_tmp = std::round(this->node[cur_i].pos.yn);
        // for(int i = 0; i < this->node[cur_i])
        // FIXME: Overflow를 막기 위한 좀 더 확실한 방안 필요.
        if (oc_x_tmp < 0)
        {
            std::cout << "Negative value for oc_x_tmp : " << oc_x_tmp << std::endl;
            oc_x_tmp += this->negative_val_offset;
        }
        if (oc_y_tmp < 0)
        {
            std::cout << "Negative value for oc_y_tmp : " << oc_y_tmp << std::endl;
            oc_y_tmp += this->negative_val_offset;
        }

        // FIXME: Closed Flag 최적화
        // // (A)를 OpenNodeList에서 제거
        // int closed_map_size_tmp = 1;//std::floor(this->length_arc) - 4;
        // int i_init = (oc_x_tmp - closed_map_size_tmp);
        // int i_end = (oc_x_tmp + closed_map_size_tmp);
        // int j_init = (oc_y_tmp - closed_map_size_tmp);
        // int j_end = (oc_y_tmp + closed_map_size_tmp);
        // std::cout << "Set Closed Region : " << i_end - i_init << "\t\t" << i_init << "\t\t" << i_end << "\t\t" << j_init << "\t\t" << j_end << "\t\t" << std::endl;
        // for (int i = i_init; i <= i_end; i++)
        //     for (int j = j_init; j <= j_end; j++)
        //         this->oc_closed_flag[i][j] = true;

        // if(i_init > 0 && j_init > 0)
        // {
        // for(int i = i_init; i <= i_end; i++)
        // for(int j = j_init; j <= j_end; j++)
        //    this->oc_closed_flag[i][j] = true;
        // }
        // else
        //     this->oc_closed_flag[oc_x_tmp][oc_y_tmp] = true;

        // FIXME: Origin
        // (A)를 OpenNodeList에서 제거
        this->oc_closed_flag[oc_x_tmp][oc_y_tmp] = true;
        // 50cm 앞의 위치에서, New Node간 최대 거리는 10.37cm 이다.
        // 100cm 앞 위치에서, New Nodes간 최대 거리는 37.34cm이다.
        int nClosed = 1;
        for (int i = 1; i <= nClosed; i++)
        {
            this->oc_closed_flag[oc_x_tmp + i][oc_y_tmp] = true;
            this->oc_closed_flag[oc_x_tmp + i][oc_y_tmp + i] = true;
            this->oc_closed_flag[oc_x_tmp - i][oc_y_tmp] = true;
            this->oc_closed_flag[oc_x_tmp - i][oc_y_tmp - i] = true;
        }

        return false;
    }

    // coeffs_ 는 WPs를 Polyfit 한 것.
    std::vector<Node> UpdateNode(Node *stand_node, Eigen::VectorXd &coeffs_, std::vector<std::pair<double, double>> &obstacles_)
    {
        /////////////////////
        ///// Parameter /////
        /////////////////////
        std::vector<double> radius;
        // -30, -20, -10, 0, 10, 20, 30 Deg
        // double radius[] = {-4.16, -6.59, -13.61, 10000, 13.61, 6.59, 4.16};
        // 일반 대형세단 회전반경이 5.0~6.0m 정도 됨.
        // radius.push_back(-4.16); radius.push_back(-6.59); radius.push_back(-13.61); radius.push_back(1000);
        // radius.push_back(13.61); radius.push_back(6.59); radius.push_back(4.16);
        // radius.push_back(-4.16); radius.push_back(-8.96); radius.push_back(1000);
        // radius.push_back(8.96); radius.push_back(4.16);

        // Default
        // radius.push_back(-3.20);
        // radius.push_back(-8.00);
        // radius.push_back(100000);
        // radius.push_back(8.00);
        // radius.push_back(3.20);

        // New (Test)
        // FIXME: 생성할 Node의 개수를 골라야함.
        // radius.push_back(-3.20);
        // radius.push_back(-8.00);
        // radius.push_back(-12.00);
        // radius.push_back(-25.00);
        // radius.push_back(-50.00);
        // radius.push_back(100000);
        // radius.push_back(50.00);
        // radius.push_back(25.00);
        // radius.push_back(12.00);
        // radius.push_back(8.00);
        // radius.push_back(3.20);

        // New (Test)
        // FIXME: 생성할 Node의 개수를 골라야함.
        // radius.push_back(-3.20);
        // radius.push_back(-8.00);
        // radius.push_back(-12.00);
        // radius.push_back(100000);
        // radius.push_back(12.00);
        // radius.push_back(8.00);
        // radius.push_back(3.20);

        // 
//        radius.push_back(-20.0); // ??? FIXME:
        radius.push_back(-27.0); // 얘가 약 30도 인듯
        radius.push_back(-32.0);
        radius.push_back(-55.0);
        radius.push_back(-70.0);
        radius.push_back(-120.0);
        radius.push_back(100000);
        radius.push_back(120.0);
        radius.push_back(70.0);
        radius.push_back(55.0);
        radius.push_back(32.0);
        radius.push_back(27.0);
//        radius.push_back(20.0); // ??? FIXME:
        // double length_arc = 1.6; //0.97; // 1.6
        double cost_g_offset = 100; //0.35; // Origin : 100.0

        //////////////////////////////
        //// Set Node index & Pos ////
        //////////////////////////////
        // Input  : {init_xn, init_yn}, {init_xc, init_yc}, dir(얘는 후진의 경우?)
        // Output : {Xn_new, Yn_new, Xc_new, Yc_new}
        std::vector<Node> updated_node_list_tmp;
        Node init_tmp;
        init_tmp = *stand_node;

        // 후진 방향 Node 증식 제거용.
        double prev_xn_tmp = node[stand_node->parent_address].pos.xn;
        double prev_yn_tmp = node[stand_node->parent_address].pos.yn;
        double cur_xn_tmp = stand_node->pos.xn;
        double cur_yn_tmp = stand_node->pos.yn;

        // Init Node에서 앞뒤로 노드가 생성되는 문제를 막기 위해 사용
        if (prev_xn_tmp == cur_xn_tmp && prev_yn_tmp == cur_yn_tmp)
            prev_xn_tmp += -50;

        for (int i = 0; i < radius.size(); i++)
        {
            for (int j = 0; j < 2; j++)
            {
                Node tmp = {};
                tmp.pos = ReturnPOSnc({init_tmp.pos.xn, init_tmp.pos.yn, init_tmp.pos.xc, init_tmp.pos.yc}, radius[i], length_arc); // At this, pos_c mean heading.

                // Rewnew POSc (Including Heading info)
                init_tmp.pos.xc = tmp.pos.xc;
                init_tmp.pos.yc = tmp.pos.yc;

                // 후진 방향 Node 증식 제거용.
                // 후진 방향 Node 증식 제거용.
                // 후진 방향 Node 증식 제거용.
                double new_xn_tmp = tmp.pos.xn;
                double new_yn_tmp = tmp.pos.yn;
                double trans_prev_xn_tmp = prev_xn_tmp - cur_xn_tmp;
                double trans_prev_yn_tmp = prev_yn_tmp - cur_yn_tmp;
                double trans_new_xn_tmp = new_xn_tmp - cur_xn_tmp;
                double trans_new_yn_tmp = new_yn_tmp - cur_yn_tmp;
                double A_tmp = EuclideanDistance(trans_new_xn_tmp, trans_new_yn_tmp);
                double B_tmp = EuclideanDistance(trans_prev_xn_tmp, trans_prev_yn_tmp);
                double den_tmp = A_tmp * B_tmp;
                double cos_theta = (trans_prev_xn_tmp * trans_new_xn_tmp + trans_prev_yn_tmp * trans_new_yn_tmp) / den_tmp;
                double angle_deg = std::acos(cos_theta) * 180. / M_PI; // 얘는 안씀. cos_theta가 1.00001 혹은 -1.00001 값이 나오면 std::acos가 nan으로 발산하는 값을 리턴함.
                if (den_tmp == 0)
                {
                    updated_node_list_tmp.push_back(tmp);
                }
                else
                {
                    // if(angle_deg > 90)
                    if (cos_theta < 0) // angle_deg가 90도보다 큰 애들(=cos_theta<0) (전진쪽 노드)만 넣어준다는 의미. angle_deg를 안쓴 이유는 std::acos(-1.00001) 값이 나오면 nan을 리턴함.
                        updated_node_list_tmp.push_back(tmp);
                }
                // updated_node_list_tmp.push_back(tmp);
            }
        }

        // With Potential Field
        for (int i = 0; i < updated_node_list_tmp.size(); i++)
        {
            double U_att = 0; // attractive potential
            double U_rep = 0; // repulsive potential
            double U_total = 0;
            double ego_x_tmp = updated_node_list_tmp[i].pos.xn;
            double ego_y_tmp = updated_node_list_tmp[i].pos.yn;

            // WPs Polyfit Coeffs
            Eigen::VectorXd coeffs_tmp = coeffs_;

            // WPs Potential
            U_att = GetPotentialCost(coeffs_tmp, ego_x_tmp, ego_y_tmp);

            // Obstacle Potential
            for (int i = 0; i < obstacles_.size(); i++)
            {
                // Because of overflow.
                // Cost 1
                // double tmp = PotentialExp(ego_x_tmp - obstacles_[i].first, ego_y_tmp - obstacles_[i].second) * 10000000000000000000;
                // double tmp2 = tmp * 10;
                // U_rep += tmp2;

                // Cost 2 (GT에서 사용하던 값)
                // U_rep += PotentialExp(ego_x_tmp - obstacles_[i].first, ego_y_tmp - obstacles_[i].second) * 1000000000000000000; // GT 기반

                // Cost 3 (지은학생 Cluster Points 받은 값)
                // U_rep += PotentialExp(ego_x_tmp - obstacles_[i].first, ego_y_tmp - obstacles_[i].second) * 10000000; // 지은학생 LiDAR 기반 [a] (비스듬한 상황에서 아슬아슬.)
                // U_rep += PotentialExp(ego_x_tmp - obstacles_[i].first, ego_y_tmp - obstacles_[i].second) * 100000000; // 지은학생 LiDAR 기반 [b] ([a]에 0 1개 더 준 것. 얘도 약간 아슬)
                // U_rep += PotentialExp(ego_x_tmp - obstacles_[i].first, ego_y_tmp - obstacles_[i].second) * 1000000000; // 지은학생 LiDAR 기반 [c] ([b]에 0 1개 더 준 것)
                U_rep += PotentialExp(ego_x_tmp - obstacles_[i].first, ego_y_tmp - obstacles_[i].second) * 10000000000; // 지은학생 LiDAR 기반 [d] ([c]에 0 1개 더 준 것)
                // -> [a], [b] 모두 LiDAR를 보정하면 될 듯?
                
                // U_rep += PotentialExp(ego_x_tmp - obstacles_[i].first, ego_y_tmp - obstacles_[i].second) * 100000000;

                // if(U_rep > 5)
                // std::cout << U_rep << "\t\t" << ego_x_tmp-1000 << "\t\t" << ego_y_tmp - 1000 << std::endl;
            }
            // Total Potential Function Value
            U_total = U_rep + U_att;

            // FIXME: cost.g 추가 여부 판단하기.
            // FIXME: cost.g 추가 여부 판단하기.
            // FIXME: cost.g 추가 여부 판단하기.
            // cost.g 값이 클 수록 Potential을 무시하고 직진성이 강해짐
            updated_node_list_tmp[i].cost.g = 0; //stand_node->cost.g + length_arc / cost_g_offset; // FIXME: /cost_g_offset 개념 알아두기.
            updated_node_list_tmp[i].cost.h = U_total;
            updated_node_list_tmp[i].cost.f = updated_node_list_tmp[i].cost.g + updated_node_list_tmp[i].cost.h;

            updated_node_list_tmp[i].cur_address = this->node.size() + i;
            updated_node_list_tmp[i].parent_address = stand_node->cur_address;
        }

        return updated_node_list_tmp;
    }
    double EuclideanDistance(double x_, double y_)
    {
        return std::sqrt(x_ * x_ + y_ * y_);
    }
    double PotentialExp(double x_, double y_)
    {
        return std::exp(-std::sqrt(x_ * x_ + y_ * y_));
    }

    Node::Pos ReturnPOSnc(Node::Pos pos_, double radius_, double length_arc_)
    {
        Node::Pos pos_new_tmp;

        // POSn, Heading(POSr), radius, length로 POSn+1, Heading(POSr+1) 구하기
        double k = -1;
        // if (dir_change_flag_ == true)
        //     k = 1 + std::sqrt(radius_ * radius_ / ((POSn_.x - POSc_.x) * (POSn_.x - POSc_.x) + (POSn_.y - POSc_.y) * (POSn_.y - POSc_.y)));
        // else
        //     k = 1 - std::sqrt(radius_ * radius_ / ((POSn_.x - POSc_.x) * (POSn_.x - POSc_.x) + (POSn_.y - POSc_.y) * (POSn_.y - POSc_.y)));
        k = 1 + std::sqrt(radius_ * radius_ / ((pos_.xn - pos_.xc) * (pos_.xn - pos_.xc) + (pos_.yn - pos_.yc) * (pos_.yn - pos_.yc)));

        pos_new_tmp.xc = (pos_.xn - pos_.xc) * k + pos_.xc;
        pos_new_tmp.yc = (pos_.yn - pos_.yc) * k + pos_.yc;

        double theta_k = length_arc_ / radius_ * -1;
        pos_new_tmp.xn = std::cos(theta_k) * (pos_.xn - pos_new_tmp.xc) - std::sin(theta_k) * (pos_.yn - pos_new_tmp.yc) + pos_new_tmp.xc;
        pos_new_tmp.yn = std::sin(theta_k) * (pos_.xn - pos_new_tmp.xc) + std::cos(theta_k) * (pos_.yn - pos_new_tmp.yc) + pos_new_tmp.yc;

        return pos_new_tmp;
    }

    void ReturnPath(struct Node *arrival_node_)
    {
        // std::cout << "return pos : [" << arrival_node_->pos.xn << ", " << arrival_node_->pos.yn << "] \t\t node[init]_address : " << this->node[init_pos_index].cur_address << "\t\t ptr : " << arrival_node_->cur_address << std::endl;

        if (this->node[init_pos_index].cur_address != arrival_node_->cur_address)
        {
            // std::cout << "Cost f : " << arrival_node_->cost.f << "\t\tCost g : " << arrival_node_->cost.g << "\t\tCost h : " << arrival_node_->cost.h << std::endl;
            local_x.push_back(arrival_node_->pos.xn);
            local_y.push_back(arrival_node_->pos.yn);

            ReturnPath(&this->node[arrival_node_->parent_address]);
        }
    }

    Eigen::VectorXd PolyFit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
    {
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
            A(i, 0) = 1.0;

        for (int j = 0; j < xvals.size(); j++)
            for (int i = 0; i < order; i++)
                A(j, i + 1) = A(j, i) * xvals(j);

        auto Q = A.householderQr();
        auto result = Q.solve(yvals);

        return result;
    }

    double GetPotentialCost(Eigen::VectorXd coeffs_, double x_, double y_)
    {
        double x_tmp = (x_ - 1000) / 10.;
        double y_tmp = (y_ - 1000) / 10.;
        double offset_tmp = 1000;
        double wp_valley_param = 0.5; //1;
        double slope_param = 0.3; //1;
        double y_axis_lane_change_gradient_constant = 1;
        double Poly_Y_Eval = PolyEval(coeffs_, x_);
//        double valley_slope = std::hypot((x_-offset_tmp)/10., (y_-Poly_Y_Eval)/10.);
        double valley_slope = std::hypot(x_tmp, y_tmp);
        double WPsEval_dir = (y_-Poly_Y_Eval)/10.;
        double WPsEval = 0;

        int lane_change_dir = -1; // L_LaneChange(1), R_LaneChange(-1)
        double y_error = (y_-Poly_Y_Eval)/10.;
        if(lane_change_dir == 1)
        {
            if(WPsEval_dir > 0)
            {
                WPsEval = std::pow(y_error, 2) * y_axis_lane_change_gradient_constant;
//                WPsEval = std::pow(WPsEval_dir, 2) * y_axis_lane_change_gradient_constant;
            }
            else
            {
                WPsEval = std::pow(y_error, 2);
//                WPsEval = std::pow(WPsEval_dir, 2);
            }
        }
        else if(lane_change_dir == -1)
        {
            if(WPsEval_dir < 0)
            {
                WPsEval = std::pow(y_error, 2) * y_axis_lane_change_gradient_constant;
//                WPsEval = std::pow(WPsEval_dir, 2) * y_axis_lane_change_gradient_constant;
            }
            else
            {
                WPsEval = std::pow(y_error, 2);
//                WPsEval = std::pow(WPsEval_dir, 2);
            }
        }

        double tmp = WPsEval * wp_valley_param - valley_slope * slope_param;

        // x_ : Propagated Next Node X
        // y_ : Propagated Next Node Y

        // Poly_Y_Eval : 1000 Unit
        // x_ : 1000 Unit
        // y_ : 1000 Unit
        // y_-Poly_Y_Eval : 1 Unit
        // x_-offset_tmp : 1 Unit


//        double offset_tmp = 1000;
//        double wp_valley_param = 0.05;
//        double slope_param = 3;
//        double y_axis_lane_change_gradient_constant = 1;
//        double Poly_Y_Eval = PolyEval(coeffs_, x_);
//        double valley_slope = std::hypot(x_-offset_tmp, y_-Poly_Y_Eval);
//        double WPsEval_dir = y_-Poly_Y_Eval;
//        double WPsEval = 0;

//        int lane_change_dir = -1; // L_LaneChange(1), R_LaneChange(-1)
//        if(lane_change_dir == 1)
//        {
//            if(WPsEval_dir > 0)
//            {
//                WPsEval = std::pow(std::fabs(y_ - Poly_Y_Eval), 2) * y_axis_lane_change_gradient_constant;
////                WPsEval = std::pow(WPsEval_dir, 2) * y_axis_lane_change_gradient_constant;
//            }
//            else
//            {
//                WPsEval = std::pow(std::fabs(y_ - Poly_Y_Eval), 2);
////                WPsEval = std::pow(WPsEval_dir, 2);
//            }
//        }
//        else if(lane_change_dir == -1)
//        {
//            if(WPsEval_dir < 0)
//            {
//                WPsEval = std::pow(std::fabs(y_ - Poly_Y_Eval), 2) * y_axis_lane_change_gradient_constant;
////                WPsEval = std::pow(WPsEval_dir, 2) * y_axis_lane_change_gradient_constant;
//            }
//            else
//            {
//                WPsEval = std::pow(std::fabs(y_ - Poly_Y_Eval), 2);
////                WPsEval = std::pow(WPsEval_dir, 2);
//            }
//        }

//        double tmp = WPsEval * wp_valley_param - valley_slope * slope_param;

//        std::cout << "X : " << (x_-1000)/10. << "\t\tY : " << (y_-1000)/10. << "\t\tWPsValley : " << WPsEval << "\t\tValleySlope : " << valley_slope << std::endl;
        // FIXME: 후진 방향으로 경로가 생기지 않도록 코드를 구성한 것.
        if (x_ < offset_tmp)
            return 1000000;
        else
            return tmp;
    }
    double PolyEval(Eigen::VectorXd coeffs, double x)
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++)
            result += coeffs[i] * pow(x, i);

        return result;
    }

protected:
private:
};

#endif /* hybrid_astar_hpp */
