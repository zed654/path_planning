#include <iostream>
#include <string>

// For Astar
//#include "astar.hpp"

// PSO Path Planner
#include "pso_planner.hpp"

// For visualizer
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "monitor.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

double PolyEval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
        result += coeffs[i] * pow(x, i);

    return result;
}

Eigen::VectorXd PolyFit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    //    assert(xvals.size() == yvals.size());
    //    assert(order >= 1 && order <= xvals.size() - 1);
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

int main(int argc, char **argv)
{
    std::cout << argc << std::endl;
    if(argc!=5)
    {
        std::cout << "The command should be comply with this format." << std::endl;
        std::cout << "Format : ros2 run pso_planner_pkg pso_planner_pkg [init width position] [init height position] [obstacles width position] [obstacles height position]" << std::endl;
        exit(1);
    }

    int init_width_position = std::stoi(argv[1]);
    int init_height_position = std::stoi(argv[2]);
    int obsts_width_position = std::stoi(argv[3]);
    int obsts_height_position = std::stoi(argv[4]);

    std::cout << init_width_position << ", " << init_height_position << "\t\t" << obsts_width_position << obsts_height_position << std::endl;

    // If push 'q', exit the progmram
    while (cv::waitKey(10) != 'q')
    {
    // Set Result Image to visualize
    cv::Mat result(800, 800, CV_8UC3);
    result.setTo(cv::Scalar(255, 255, 255));

    std::pair<double, double> pso_target_xy = std::make_pair(400, 400);
    std::pair<double, double> closed_obsts_xy = std::make_pair(obsts_width_position, obsts_height_position);
    std::pair<double, double> init_xy = std::make_pair(init_width_position, init_height_position);

    TimeChecker tc;
    tc.DeparturePointTime();
    PSOPlanner pso_search(init_xy, pso_target_xy, closed_obsts_xy);

    
    std::cout << "Duration Time : " << tc.ArrivalPointTime() << "ms" << std::endl;

    std::vector<std::pair<double, double>> pso_result_path_points = pso_search.get_pso_result_path_points();
    std::vector<std::vector<std::pair<double, double>>> pso_total_result_path_points = pso_search.get_pso_total_result_path_points();
    std::vector<std::pair<double, double>> global_best_position_array = pso_search.get_global_best_position_array();
    std::pair<int, int> global_best_array_index = pso_search.get_global_best_array_index();

    cv::line(result, cv::Point(closed_obsts_xy.first, closed_obsts_xy.second), cv::Point(closed_obsts_xy.first, closed_obsts_xy.second), cv::Scalar(0, 0, 255), 15, 8, 0);
    cv::line(result, cv::Point(init_xy.first, init_xy.second), cv::Point(init_xy.first, init_xy.second), cv::Scalar(0, 255, 0), 15, 8, 0);
    cv::line(result, cv::Point(pso_target_xy.first, pso_target_xy.second), cv::Point(pso_target_xy.first, pso_target_xy.second), cv::Scalar(0, 0, 0), 15, 8, 0);

    // for (int i = 0; i < pso_result_path_points.size() - 1; i++)
    // {
    //     cv::line(result, cv::Point(pso_result_path_points[i].first, pso_result_path_points[i].second), cv::Point(pso_result_path_points[i + 1].first, pso_result_path_points[i + 1].second), cv::Scalar(0, 0, 255), 1, 8, 0);
    // }

    // Global Best 이미지 출력
    int gb_draw_color = 0;
    int gb_draw_radius = 4;
    int gb_draw_thickness = 4;
    int gb_draw_linetype = 8;
    for (int i = 0; i < global_best_position_array.size(); i++)
    {
	    if(i == 0)
                gb_draw_color = 255;
            else
    	        gb_draw_color = 255/global_best_position_array.size()*(global_best_position_array.size()-i+1);
	//std::cout << global_best_position_array[i].first << ", " << global_best_position_array[i].second << std::endl;
        cv::circle(result, cv::Point(global_best_position_array[i].first, global_best_position_array[i].second), gb_draw_radius, cv::Scalar(gb_draw_color, 0, 0), gb_draw_thickness, gb_draw_linetype, 0);
    }
    //std::cout << std::endl;


    for (int i = 0; i < pso_total_result_path_points.size(); i++)
    {
        // std::cout << "Size : " << pso_total_result_path_points[i].size() << std::endl;
        for (int j = 0; j < pso_total_result_path_points[i].size() - 1; j++)
        {
		if(i == global_best_array_index.first)
            		cv::line(result, cv::Point(pso_total_result_path_points[i][j].first, pso_total_result_path_points[i][j].second), cv::Point(pso_total_result_path_points[i][j + 1].first, pso_total_result_path_points[i][j + 1].second), cv::Scalar(0, 0, 255), 2, 8, 0);
		else
            		cv::line(result, cv::Point(pso_total_result_path_points[i][j].first, pso_total_result_path_points[i][j].second), cv::Point(pso_total_result_path_points[i][j + 1].first, pso_total_result_path_points[i][j + 1].second), cv::Scalar(0, 255, 0), 1, 8, 0);
            // cv::line(result, cv::Point(pso_total_result_path_points[i][j].first, pso_total_result_path_points[i][j].second), cv::Point(pso_total_result_path_points[i][j].first, pso_total_result_path_points[i][j].second), cv::Scalar(0, 0, 255), 3, 8, 0);
            // std::cout << pso_total_result_path_points[i][j].first << ", " << pso_total_result_path_points[i][j].second << "\t\t";
        }
        // std::cout << std::endl;
    }

    int swarm_draw_color = 0;
    int swarm_draw_radius = 2;
    int swarm_draw_thickness = 2;
    int swarm_draw_linetype = 8;
    //std::cout << "i size : " << pso_total_result_path_points.size() << std::endl;
    for (int i = 0; i < pso_total_result_path_points.size(); i++)
    {
        for (int j = 0; j < pso_total_result_path_points[i].size(); j++)
        {
	    if(j == 0)
	    {
		swarm_draw_radius = 9;
		swarm_draw_thickness = 1;
                swarm_draw_color = 255;
		//std::cout << "j size : " << pso_total_result_path_points[i].size() << std::endl;
	    }
            else
	    {
		swarm_draw_radius = 2;
		swarm_draw_thickness = 2;
    	        swarm_draw_color = 255/pso_total_result_path_points[i].size()*(j+1);
	    }
	    cv::circle(result, cv::Point(pso_total_result_path_points[i][j].first, pso_total_result_path_points[i][j].second), swarm_draw_radius, cv::Scalar(0, swarm_draw_color, 0), swarm_draw_thickness, swarm_draw_linetype, 0);
            //std::cout << pso_total_result_path_points[i][j].first << ", " << pso_total_result_path_points[i][j].second << "\t\t";
        }
         //std::cout << std::endl;
    }




    //    cv::imshow("Input Obstacle Img", obstacle_img);
    cv::imshow("Result of PSO Path Planner", result);
    cv::imwrite("img/result/result.jpeg", result);
    }

    return 0;
}
