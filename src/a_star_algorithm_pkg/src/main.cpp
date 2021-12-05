#include <iostream>
#include <string>

// For Astar
#include "astar.hpp"

// For visualizer
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "monitor.h"
#include <Eigen-3.3/Eigen/Core>
#include <Eigen-3.3/Eigen/QR>
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

    ///////////////////////////////
    ///////////////////////////////
    ///////////////////////////////
    ///////////////////////////////
    if(argc!=2)
    {
        std::cout << "The command should be comply with this format." << std::endl;
        std::cout << "Format : rosrun [pkg] [img_num]" << std::endl;
        exit(1);
    }
    

    ////////////////////////
    //////// Praram ////////
    ////////////////////////
    // Set Input Image Name
    std::string img_count = argv[1];

    // // Set Size of Obstacle Image
    int obstacle_img_width = 960;
    int obstacle_img_height = 540;

    // // Set Size of Occupancy Grid Map, Astar
    int occupancy_grid_map_width = 959;
    int occupancy_grid_map_height = 538;

    ////////////////////////////////////
    ////////////////////////////////////
    ////////////////////////////////////
    //////// Set Obstacle Image ////////
    ////////////////////////////////////
    ////////////////////////////////////
    ////////////////////////////////////
    // Get Input Img from jpeg
    cv::Mat obstacle_img;
    obstacle_img = cv::imread("img/" + img_count + ".jpeg");
    // obstacle_img = cv::imread("img/" + img_count + ".png");
    if(!obstacle_img.data)
    {
        std::cout << "Can't find the image file" << std::endl;
        exit(1);
    }
    cv::resize(obstacle_img, obstacle_img, cv::Size(obstacle_img_width, obstacle_img_height));

    // OpenCV Coordinate -> Occupancy Grid Map Coordinate
    // OpenCV와 Astar의 Occuapncy Grid Map의 좌표계가 다르기 때문에 상하반전 작업
    cv::flip(obstacle_img, obstacle_img, 0);

    // Set Result Image to visualize
    cv::Mat result;
    result = obstacle_img.clone();


    std::vector<std::pair<double, double>> waypoints;
    double prev_x_tmp = 0;
    for(int i = 0; i < obstacle_img.cols; i++)
        for(int j = 0; j < obstacle_img.rows; j++)
        {
            double r_tmp = obstacle_img.at<cv::Vec3b>(j, i)[2];
            double g_tmp = obstacle_img.at<cv::Vec3b>(j, i)[1];
            double b_tmp = obstacle_img.at<cv::Vec3b>(j, i)[0];
            if((r_tmp + g_tmp + b_tmp) <= 20)
            {
                if(i >= 163)
                {
                    if(i - prev_x_tmp > 5)
                    {
                        // std::cout << obstacle_img.at<cv::Vec3b>(j, i) << "\t\t" << i << "\t\t" << j << std::endl;
                        waypoints.push_back(std::make_pair(i, j));
                        // cv::circle(result, cv::Point(i, j), 0, cv::Scalar(0,0,255), 2);
                    }
                    prev_x_tmp = i;
                }
            }
        }

    int init_pos_x = waypoints[0].first;
    int init_pos_y = waypoints[0].second;
    int target_x = waypoints[waypoints.size()-5].first;
    int target_y = waypoints[waypoints.size()-5].second;

    ///////////////////////
    ///////////////////////
    ///////////////////////
    //////// Astar ////////
    ///////////////////////
    ///////////////////////
    ///////////////////////
    TimeChecker tc[5];
    tc[0].DeparturePointTime();

    // Set Astar
    Astar astar(occupancy_grid_map_width, occupancy_grid_map_height, init_pos_x, init_pos_y);

    // std::vector<std::pair<double, double>> obstacles;
    for(int i = 0; i < obstacle_img.cols; i++)
        for(int j = 0; j < obstacle_img.rows; j++)
        {
            double r_tmp = obstacle_img.at<cv::Vec3b>(j, i)[2];
            double g_tmp = obstacle_img.at<cv::Vec3b>(j, i)[1];
            double b_tmp = obstacle_img.at<cv::Vec3b>(j, i)[0];
            // if((r_tmp + g_tmp + b_tmp) < 700 & (r_tmp + g_tmp + b_tmp) > 500)
            // if(((r_tmp + g_tmp + b_tmp) >= 400) & ((r_tmp + g_tmp + b_tmp) < 700))
            if( b_tmp > 215 & (r_tmp + g_tmp + b_tmp < 590))
            {
                std::cout << obstacle_img.at<cv::Vec3b>(j, i) << "\t\t" << i << "\t\t" << j << std::endl;
                // obstacles.push_back(std::make_pair(i, j));
                // cv::circle(result, cv::Point(i, j), 0, cv::Scalar(0,255,0), 5);
                // astar.node[i + (j * occupancy_grid_map_width)].close_flag = true;

                for(int l = -15; l < 15; l++)
                    for(int m = -15; m<15; m++)
                    {
                        // cv::circle(result, cv::Point(i+l, j+m), 0, cv::Scalar(0,255,0), 5);
                        astar.node[i+l + ((j+m) * occupancy_grid_map_width)].close_flag = true;
                        // obstacles.push_back(std::make_pair(i, j));
                    }
            }
        }


    // Set Obstacle in Occupancy Grid Map of Astar
    // for(int i = 0; i < obstacle_img.cols; i++)
    //     for(int j = 0; j < obstacle_img.rows; j++)
    //     {
    //         // Obstacle_value 값이 255*3보다 작으면, 장애물의 위치를 의미
    //         int obstacle_value =
    //                             obstacle_img.data[(i * 3 + 0) + (3 * j * obstacle_img.cols)] +
    //                             obstacle_img.data[(i * 3 + 1) + (3 * j * obstacle_img.cols)] +
    //                             obstacle_img.data[(i * 3 + 2) + (3 * j * obstacle_img.cols)];

    //         // Set Obstacle
    //         if (obstacle_value < 255*3)
    //         {
    //             int x_tmp = i;
    //             int y_tmp = j;
    //             astar.node[x_tmp + (y_tmp * occupancy_grid_map_width)].close_flag = true;
    //         }
    //     }

    // Clipping for init and final pos for existing obstacle
    if(astar.node[init_pos_x + (init_pos_y * occupancy_grid_map_width)].close_flag == true |
            astar.node[init_pos_x + (init_pos_y * occupancy_grid_map_width)].close_flag == true |
	    astar.node[target_x + (target_y * occupancy_grid_map_width)].close_flag == true |
            astar.node[target_x + (target_y * occupancy_grid_map_width)].close_flag == true)
    {
        std::cout << "The position of Init_xy or Final_xy was occupied by obstacle. Change the pos value" << std::endl;
        exit(0);
    }

   target_y = target_y - 10;
    // Do Astar
    // You can get the result from vectors of astar.local_xy
    while(!astar.GetResult(target_x,target_y))
    {
    }
    std::cout << "Loop Time : " << tc[0].ArrivalPointTime() << std::endl;



    std::vector<double> WPs_x_tmp2;
    std::vector<double> WPs_y_tmp2;
    for (int i = 0; i < waypoints.size(); i++)
    {
        WPs_x_tmp2.push_back(waypoints[i].first);
        WPs_y_tmp2.push_back(waypoints[i].second);
    }
    Eigen::Map<Eigen::VectorXd> x_tmp(&WPs_x_tmp2[0], WPs_x_tmp2.size());
    Eigen::Map<Eigen::VectorXd> y_tmp(&WPs_y_tmp2[0], WPs_y_tmp2.size());
    Eigen::VectorXd coeffs_tmp2 = PolyFit(x_tmp, y_tmp, 3);
//    Eigen::VectorXd coeffs_tmp2(4);
//    coeffs_tmp2 << 1595.485386, 0.3118997227, -0.0009691267892, 3.840002278e-07;
//    coeffs_tmp2 << 1262.65461834417, 0.0203069391261758, -1.20542449927122e-05, 2.37382369415847e-09;
//    1595.485386		0.3118997227		-0.0009691267892		3.840002278e-07
    double sum_of_error = 0;
    double sum_of_rms_square = 0;

    double max_error = 0;
    double min_error = 999999;
    for (int i = 0; i < astar.local_x.size(); i++)
    {
//        std::cout << astar.local_x[i]/10. << "\t\t" << astar.local_y[i]/10. << std::endl;

//        std::cout << (h_astar2.local_x[i]-1000)/10 << "\t\t";
            double error        = std::fabs(((PolyEval(coeffs_tmp2, astar.local_x[i]))/10. - (astar.local_y[i]/10.)));
            double rms_square   = std::pow(((PolyEval(coeffs_tmp2, astar.local_x[i])))/10. - (astar.local_y[i]/10.),2);
            sum_of_error += error;
            if(error > max_error) max_error = error;
            if(std::fabs(error) < min_error) min_error = error;
            sum_of_rms_square += rms_square;
            std::cout << error << "\t\t" << rms_square << "\t\t" << ((PolyEval(coeffs_tmp2, astar.local_x[i])))/10. << "\t\t" <<  astar.local_x[i]/10. << "\t\t" << astar.local_y[i]/10. << "\t\t" << PolyEval(coeffs_tmp2, astar.local_x[i]+1000) << std::endl;
//            std::cout << (h_astar2.local_x[i]-1000)/10 << "\t\t";
    }

    std::cout << std::endl;
    std::cout << "Max error : " << max_error << std::endl;
    std::cout << "Min error : " << min_error << std::endl;
    std::cout << "Sum of the error : " << sum_of_error << std::endl;
    std::cout << "Avg of the error : " << sum_of_error/astar.local_x.size() << std::endl;
    std::cout << "Sum of the RMS square : " << sum_of_rms_square << std::endl;
    std::cout << "Avg of the RMS square : " << sum_of_rms_square/astar.local_x.size() << std::endl;
    std::cout << "RMS error : " << std::sqrt(sum_of_rms_square/astar.local_x.size()) << std::endl;
    //    std::cout << "PolyEval Test :" << (PolyEval(coeffs_tmp2, 1170)-1000)/10 << "\t\t" << (PolyEval(coeffs_tmp2, 1300)-1000)/10 << "\t\t" << (PolyEval(coeffs_tmp2, 1800)-1000)/10 << std::endl;

    std::cout.precision(10);
    std::cout << "coeffs : ";
    for(int i = 0; i < coeffs_tmp2.size(); i++)
        std::cout << coeffs_tmp2[i] << "\t\t";
    std::cout << std::endl;

    ///////////////////////////
    ///////////////////////////
    ///////////////////////////
    ////// Visualization //////
    ///////////////////////////
    ///////////////////////////
    ///////////////////////////
/*
    for(int i = 0; i < adj_nodes_x.size(); i++)
    {
        int color_tmp = i % 255;
        cv::circle(result, cv::Point(adj_nodes_x[i], adj_nodes_y[i]), 0, cv::Scalar(0, 0, color_tmp), 1);
//        cv::imshow("Result of Astar", result);
//        cv::waitKey();
    }
*/

    
    for(int i = 0; i < astar.local_x.size()-1; i++)
    {
        // cv::circle(result, cv::Point(astar.local_x[i], astar.local_y[i]), 0, cv::Scalar(0,0,255), 1);
        cv::line(result, cv::Point(astar.local_x[i], astar.local_y[i]), cv::Point(astar.local_x[i+1], astar.local_y[i+1]), cv::Scalar(0,0,255), 1, 8, 0);
    }

    // Occupancy Grid Map Coordinate -> OpenCV Coordinate
    // OpenCV와 Astar의 Occuapncy Grid Map의 좌표계가 다르기 때문에 상하반전 작업
    // cv::resize(obstacle_img, obstacle_img, cv::Size(960, 540));
    // cv::resize(result, result, cv::Size(960, 540));
    cv::flip(obstacle_img, obstacle_img, 0);
    cv::flip(result, result, 0);

//    cv::imshow("Input Obstacle Img", obstacle_img);
    cv::imshow("Result of Astar", result);
    cv::imwrite("img/result/result.jpeg", result);
    
    // If push 'q', exit the progmram
    while(cv::waitKey(10)!='q')
    {

    }


    return 0;
}
