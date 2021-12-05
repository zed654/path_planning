 #include <iostream>
 #include <ros/ros.h>
 #include <visualization_msgs/Marker.h>
 #include <unistd.h>

 struct path_color
 {
     float r; // red
     float g; // blue
     float b; // green
 };

 void PathPubInit(visualization_msgs::Marker &marker_, int id_, float marker_type_, float scale_, path_color path_color_);

 struct Node
 {
     struct Cost
     {
         double g; // The sum of the calcuated costs
         double h; // The cost which will be spend
         double f; // The sum of the g and h
     };
     struct Cost cost;
     float pos_x;
     float pos_y;
     bool open_node_list_flag;
     bool close_flag;
     std::vector<struct Node *> linked_node;
     struct Node *prev;
 };

 class Graph
 {
 public:
     int width;
     int height;
     std::vector<struct Node> node;

     int start_pos_x;
     int start_pos_y;
     int current_node_index; // int이어야만함

     // Width X Heith 크기의 Occupancy Grid Map을 만들고
     // 인접 노드를 연결시킴 (node변수의 linked_node에)
     Graph(int width_, int height_)
     {
         width = width_;
         height = height_;
         result_local_x.push_back(0);
         result_local_y.push_back(0);
         result_local_xy_set_flag = false;

         if ((width % 2) == 0)
         {
             std::cout << "차량의 위치에 격자가 위치하지 않음. width는 홀수로 해야함" << std::endl;
             std::cout << "차량의 위치에 격자가 위치하지 않음. width는 홀수로 해야함" << std::endl;
             std::cout << "차량의 위치에 격자가 위치하지 않음. width는 홀수로 해야함" << std::endl;
         }

         // node[current_node_index] 부터 시작함.
         start_pos_x = width / 2;
         start_pos_y = 0;
         current_node_index = start_pos_x + start_pos_y;

         // Grid Size에 맞춰서 Node 생성.
         // 다만 클로즈 노드 리스트로 생성. (false부분)
         node.assign(width * height, {{0, 0, 0}, 0, 0, false, false, {}, NULL}); // vector에 NULL 넣으면 스텍 증가함.

         // pos값 입력
         for (int i = 0; i < width; i++)
             for (int j = 0; j < height; j++)
             {
                 node[i + (j * width)].pos_x = i;
                 node[i + (j * width)].pos_y = j;
             }

         // 초기 출발 노드 위치 출력
         std::cout << "init pos _ x : " << node[current_node_index].pos_x << "\t\t y : " << node[current_node_index].pos_y << std::endl;

         // 다중 연결리스트 생성
         for (int i = 1; i < width - 1; i++)
             for (int j = 1; j < height - 1; j++)
             {
                 int index_tmp = i + (j * width);
                 node[index_tmp].linked_node.push_back(&node[index_tmp - 1]);
                 node[index_tmp].linked_node.push_back(&node[index_tmp + width - 1]);
                 node[index_tmp].linked_node.push_back(&node[index_tmp + width]);
                 node[index_tmp].linked_node.push_back(&node[index_tmp + width + 1]);
                 node[index_tmp].linked_node.push_back(&node[index_tmp + 1]);
                 node[index_tmp].linked_node.push_back(&node[index_tmp - width + 1]);
                 node[index_tmp].linked_node.push_back(&node[index_tmp - width]);
                 node[index_tmp].linked_node.push_back(&node[index_tmp - width - 1]);
             }

         for (int i = 1; i < width - 1; i++)
         {
             int index_tmp = i;
             node[index_tmp].linked_node.push_back(&node[index_tmp - 1]);
             node[index_tmp].linked_node.push_back(&node[index_tmp + width - 1]);
             node[index_tmp].linked_node.push_back(&node[index_tmp + width]);
             node[index_tmp].linked_node.push_back(&node[index_tmp + width + 1]);
             node[index_tmp].linked_node.push_back(&node[index_tmp + 1]);
         }
         // // 다중 연결리스트 생성
         // int linked_index_tmp[8] = {-1, width - 1, width, width + 1, 1, -width + 1, -width, -width - 1};
         // for (int i = 0; i < width; i++)
         //     for (int j = 0; j < height; j++)
         //     {
         //         int index_tmp = i + (j * width);

         //         // index 0
         //         int prev_i = node[i - 1 + (j * width)].pos_x;
         //         int prev_j = node[i - 1 + (j * width)].pos_y;
         //         double length_0 = std::sqrt(std::pow(i - prev_i, 2) + std::pow(j - prev_j, 2));
         //         if (length_0 == 1)
         //         {
         //             node[index_tmp].linked_node.push_back(&node[index_tmp - 1]);
         //         }

         //         // index 1
         //         prev_i = node[i - 1 + ((j + 1) * width)].pos_x;
         //         prev_j = node[i - 1 + ((j + 1) * width)].pos_y;
         //         double length_1 = std::sqrt(std::pow(i - prev_i, 2) + std::pow(j - prev_j, 2));
         //         if (length_1<1.415 & length_1> 1.413)
         //         {
         //             node[index_tmp].linked_node.push_back(&node[index_tmp + width - 1]);
         //         }

         //         // index 2
         //         if (j < height - 1)
         //         {
         //             node[index_tmp].linked_node.push_back(&node[index_tmp + width]);
         //         }

         //         // index 3
         //         prev_i = node[i + 1 + ((j + 1) * width)].pos_x;
         //         prev_j = node[i + 1 + ((j + 1) * width)].pos_y;
         //         double length_3 = std::sqrt(std::pow(i - prev_i, 2) + std::pow(j - prev_j, 2));
         //         if (length_3<1.415 & length_3> 1.413)
         //         {
         //             node[index_tmp].linked_node.push_back(&node[index_tmp + width + 1]);
         //         }

         //         // index 4
         //         prev_i = node[i + 1 + (j * width)].pos_x;
         //         prev_j = node[i + 1 + (j * width)].pos_y;
         //         double length_4 = std::sqrt(std::pow(i - prev_i, 2) + std::pow(j - prev_j, 2));
         //         if (length_4 == 1)
         //         {
         //             node[index_tmp].linked_node.push_back(&node[index_tmp + 1]);
         //         }

         //         // index 5
         //         prev_i = node[i + 1 + ((j - 1) * width)].pos_x;
         //         prev_j = node[i + 1 + ((j - 1) * width)].pos_y;
         //         double length_5 = std::sqrt(std::pow(i - prev_i, 2) + std::pow(j - prev_j, 2));
         //         if (length_5<1.415 & length_5> 1.413)
         //         {
         //             node[index_tmp].linked_node.push_back(&node[index_tmp - width + 1]);
         //         }

         //         // index 6
         //         if (j > 0)
         //         {
         //             node[index_tmp].linked_node.push_back(&node[index_tmp - width]);
         //         }

         //         // index 7
         //         prev_i = node[i - 1 + ((j - 1) * width)].pos_x;
         //         prev_j = node[i - 1 + ((j - 1) * width)].pos_y;
         //         double length_7 = std::sqrt(std::pow(i - prev_i, 2) + std::pow(j - prev_j, 2));
         //         if (length_7<1.415 & length_7> 1.413)
         //         {
         //             node[index_tmp].linked_node.push_back(&node[index_tmp - width - 1]);
         //         }
         //     }
     }
     virtual ~Graph()
     {
     }
    
     bool grid_algorithm(int final_pos_x_, int final_pos_y_)
     {
         // 차량 좌표계 -> Occupancy Grid Map 좌표계
         float final_pos_x_tmp = final_pos_y_ + start_pos_x;
         float final_pos_y_tmp = final_pos_x_;

         // linked_node의 Cost 계산
         for (int i = 0; i < node[current_node_index].linked_node.size(); i++)
         {
             if (node[current_node_index].linked_node[i]->close_flag == false) // 열린 노드이면 ??
             {
                 float linked_node_x_tmp = node[current_node_index].linked_node[i]->pos_x;
                 float linked_node_y_tmp = node[current_node_index].linked_node[i]->pos_y;
                 float current_x_tmp = node[current_node_index].pos_x;
                 float current_y_tmp = node[current_node_index].pos_y;
                 // calc cost g, h, f
                 double linked_node_cost_g_tmp = node[current_node_index].cost.g + std::sqrt(std::pow(current_x_tmp - linked_node_x_tmp, 2) + std::pow(current_y_tmp - linked_node_y_tmp, 2));
                 double linked_node_cost_h_tmp = std::sqrt(std::pow(final_pos_x_tmp - linked_node_x_tmp, 2) + std::pow(final_pos_y_tmp - linked_node_y_tmp, 2));
                 double linked_node_cost_f_tmp = linked_node_cost_g_tmp + linked_node_cost_h_tmp;

                 // set cost g, h, f
                 node[current_node_index].linked_node[i]->cost.g = linked_node_cost_g_tmp;
                 node[current_node_index].linked_node[i]->cost.h = linked_node_cost_h_tmp;
                 node[current_node_index].linked_node[i]->cost.f = linked_node_cost_f_tmp;

                 // std::cout << i << "\t\t g : " << node[current_node_index].linked_node[i]->cost.g << "\t\t h : " << node[current_node_index].linked_node[i]->cost.h << "\t\t f : " << node[current_node_index].linked_node[i]->cost.f << std::endl;
             }
         }

         // 가장 작은 Cost의 인덱스 구하기
         unsigned int tmp = 64000;
         int linked_node_index_0_7_tmp = 0;
         for (int i = 0; i < node[current_node_index].linked_node.size(); i++)
         {
             if (node[current_node_index].linked_node[i]->close_flag == false)
             {
                 double cost_f_tmp = node[current_node_index].linked_node[i]->cost.f;
                 if (tmp > cost_f_tmp)
                 {
                     tmp = cost_f_tmp;
                     linked_node_index_0_7_tmp = i;
                 }
                 else if (tmp == cost_f_tmp)
                     if (node[current_node_index].linked_node[linked_node_index_0_7_tmp]->cost.h > node[current_node_index].linked_node[i]->cost.h)
                     {
                         linked_node_index_0_7_tmp = i;
                     }
             }
         }

         int renew_pos_x_tmp = node[current_node_index].linked_node[linked_node_index_0_7_tmp]->pos_x;
         int renew_pos_y_tmp = node[current_node_index].linked_node[linked_node_index_0_7_tmp]->pos_y;
         int next_node_index = renew_pos_x_tmp + renew_pos_y_tmp * width;

         // 이동한 노드(linked_node쪽)의 이전 노드를 현재 노드로 넣어줌.
         node[current_node_index].linked_node[linked_node_index_0_7_tmp]->prev = &node[current_node_index];

         // 노드를 이동하였으므로, 이동 전 노드를 close
         node[current_node_index].close_flag = true;

         // 현재 노드 최신으로 갱신해줌.
         // current_node_index += linked_index_tmp[linked_node_index_0_7_tmp];
         current_node_index = next_node_index;
         std::cout << next_node_index << std::endl;

         float current_x_tmp_ = node[current_node_index].pos_x;
         float current_y_tmp_ = node[current_node_index].pos_y;
         // std::cout << "x : " << current_x_tmp_ << "\t\t y : " << current_y_tmp_ << std::endl;

         // 차량 좌표계로 변환하여 반환
         result_local_x.push_back(current_x_tmp_ - start_pos_x);
         result_local_y.push_back(current_y_tmp_);
         if ((final_pos_x_tmp - node[current_node_index].pos_x) <= 1.415 & (final_pos_y_tmp - node[current_node_index].pos_y) <= 1.415)
         {
             // node.assign(width * height, {{0, 0, 0}, 0, 0, false, {}, NULL}); // vector에 NULL 넣으면 스텍 증가함.
             // current_node_index = start_pos_x + start_pos_y;
             // Graph(this->width, this->height);
             result_local_xy_set_flag = true;
             return true;
         }
         else
         {
             return false;
         }
     }

     void set_result_local_xy_set_flag(bool param_)
     {
         this->result_local_xy_set_flag = param_;
     }
     bool get_result_local_xy_set_flag()
     {
         return this->result_local_xy_set_flag;
     }
     std::vector<double> get_result_local_x()
     {
         return this->result_local_x;
     }
     std::vector<double> get_result_local_y()
     {
         return this->result_local_y;
     }

 protected:
 private:
     std::vector<double> result_local_x;
     std::vector<double> result_local_y;
     bool result_local_xy_set_flag;
 };

 int main(int argc, char **argv)
 {
     // int obstacle_pos_x[10] = {10, 11,}
     // Input FSD Data
     std::vector<std::vector<bool>> bj_pos;

     int grid_width = 301;  // 21
     int grid_height = 301; // 30
     bj_pos.assign(grid_width, std::vector<bool>(grid_height, 0));

     // A*를 위한 Occupancy Grid Map 생성
     Graph graph(grid_width, grid_height);
     double grid_unit = 1; // meter. 0.1은 0.1m 단위임. x, y값 1개당 grid_unit 미터임.

     // Obstacle 넣는 곳
     // close_flag == 0 이면 갈 수 있는곳, 1이면 못가는 곳
     for (int i = 0; i < graph.width; i++)
         for (int j = 0; j < graph.height; j++)
         {
             graph.node[i + (j * graph.width)].close_flag = bj_pos[i][j];
         }
     // 차량좌표계로 11미터 앞에 장애물 4개 설치
     graph.node[graph.start_pos_x - 1 / grid_unit + (11 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x + (11 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x + 1 / grid_unit + (11 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x + 2 / grid_unit + (11 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x + 2 / grid_unit + (10 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x + 2 / grid_unit + (9 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x + 2 / grid_unit + (8 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x - 1 / grid_unit + (10 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x - 1 / grid_unit + (9 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x - 1 / grid_unit + (8 * graph.width / grid_unit)].close_flag = 1;
     graph.node[graph.start_pos_x - 1 / grid_unit + (7 * graph.width / grid_unit)].close_flag = 1;

     // 최종 목표 위치 (차량 좌표계로 전달)
     int final_pos_x = 12;
     int final_pos_y = 0;

     // A* 알고리즘 구동
     while (!graph.grid_algorithm(final_pos_x, final_pos_y))
         ;

     // 계산된 Local Path Waypoint 받아오기
     std::vector<double> local_x;
     std::vector<double> local_y;
     if (graph.get_result_local_xy_set_flag() == true)
     {
         local_x = graph.get_result_local_y();
         local_y = graph.get_result_local_x();
         graph.set_result_local_xy_set_flag(false);
     }

     // RVIZ
     // RVIZ
     // RVIZ
     // RVIZ
     // RVIZ
     ros::init(argc, argv, "av_platform_node"); // 노드명 초기화
     ros::NodeHandle nh_marker;
     ros::Publisher marker_pub = nh_marker.advertise<visualization_msgs::Marker>("marker_local_path", 10);
     // ros::Rate r(500);

     visualization_msgs::Marker local_path_pos;
     int id_tmp = 0;
     path_color path_color_tmp = {1, 1, 0}; // 노란색
     float scale_tmp = 0.2;
     PathPubInit(local_path_pos, id_tmp, visualization_msgs::Marker::LINE_STRIP, scale_tmp, path_color_tmp);

     visualization_msgs::Marker obstacle_pos;
     id_tmp = 1;
     path_color_tmp = {1, 1, 0}; // 자주색?
     scale_tmp = 0.5;
     PathPubInit(obstacle_pos, id_tmp, visualization_msgs::Marker::POINTS, scale_tmp, path_color_tmp);

     int xxx = 20;
     int yyy = -10;
     while (ros::ok())
     {
         // sleep(1);
         yyy++;
         if (yyy >= 10)
         {
             yyy = -10;
             if (xxx < 30)
                 xxx++;
             else
                 xxx = 10;
         }

         // A*를 위한 Occupancy Grid Map 생성
         Graph graph(grid_width, grid_height);

         // Obstacle 넣는 곳
         // close_flag == 0 이면 갈 수 있는곳, 1이면 못가는 곳
         for (int i = 0; i < graph.width; i++)
             for (int j = 0; j < graph.height; j++)
             {
                 graph.node[i + (j * graph.width)].close_flag = bj_pos[i][j];
             }
         // 차량좌표계로 11미터 앞에 장애물 4개 설치
         graph.node[graph.start_pos_x - 1 / grid_unit + (11 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x + (11 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x + 1 / grid_unit + (11 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x + 2 / grid_unit + (11 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x + 2 / grid_unit + (10 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x + 2 / grid_unit + (9 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x + 2 / grid_unit + (8 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x - 1 / grid_unit + (10 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x - 1 / grid_unit + (9 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x - 1 / grid_unit + (8 * graph.width / grid_unit)].close_flag = 1;
         graph.node[graph.start_pos_x - 1 / grid_unit + (7 * graph.width / grid_unit)].close_flag = 1;

         // 최종 목표 위치 (차량 좌표계로 전달)
         final_pos_x = xxx;
         final_pos_y = yyy;

         // A* 알고리즘 구동
         while (!graph.grid_algorithm(final_pos_x, final_pos_y));

         // 계산된 Local Path Waypoint 받아오기
         std::vector<double> local_x;
         std::vector<double> local_y;
         if (graph.get_result_local_xy_set_flag() == true)
         {
             local_x = graph.get_result_local_y();
             local_y = graph.get_result_local_x();
             graph.set_result_local_xy_set_flag(false);
         }

         local_path_pos.points.clear();
         geometry_msgs::Point pos_tmp;
         for (int i = 0; i < local_x.size(); i++)
         {
             pos_tmp.x = local_x[i] * grid_unit;
             pos_tmp.y = local_y[i] * grid_unit;
             pos_tmp.z = 0.1;
             std::cout << pos_tmp.x << "\t\t" << pos_tmp.y << std::endl;
             local_path_pos.points.push_back(pos_tmp);
         }

         marker_pub.publish(local_path_pos);

         obstacle_pos.points.clear();
         for (int i = 0; i < grid_width; i++)
             for (int j = 0; j < grid_height; j++)
             {
                 if (graph.node[i + (j * graph.width)].close_flag == 1)
                 {
                     pos_tmp.x = graph.node[i + (j * graph.width)].pos_y * grid_unit;
                     pos_tmp.y = (graph.node[i + (j * graph.width)].pos_x - graph.start_pos_x) * grid_unit;
                     pos_tmp.z = 0.1;
                     obstacle_pos.points.push_back(pos_tmp);
                 }
             }

         marker_pub.publish(obstacle_pos);

         // r.sleep();

         usleep(10000);
         // f += 0.04;
     }

     return 0;
 }


void PathPubInit(visualization_msgs::Marker &marker_, int id_, float marker_type_, float scale_, path_color path_color_)
{
    marker_.header.frame_id = "velodyne";
    marker_.header.stamp = ros::Time::now();
    // marker_.ns = "points_and_line";
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.orientation.w = 1.0;
    marker_.type = marker_type_;

    marker_.id = id_;
    if (marker_type_ == visualization_msgs::Marker::POINTS)
    {
        marker_.scale.x = scale_;
        marker_.scale.y = scale_;
    }
    else if (marker_type_ == visualization_msgs::Marker::LINE_STRIP)
    {
        marker_.scale.x = scale_;
    }
    else if (marker_type_ == visualization_msgs::Marker::LINE_LIST)
    {
        marker_.scale.x = scale_;
    }
    else
    {
        marker_.scale.x = scale_;
        marker_.scale.y = scale_;
        marker_.scale.z = scale_;
    }

    marker_.scale.z = 0;

    marker_.color.r = path_color_.r;
    marker_.color.g = path_color_.g;
    marker_.color.b = path_color_.b;
    marker_.color.a = 1.0;
}
