#ifndef DIJKSTRA_H_
#define DIJKSTRA_H_
#include <iostream>
#include <vector>
#include <pcl/point_types.h>

class dijkstra
{
    private:
        int M=100000;
        int N = 0;
        bool fail_to_find;

        std::vector<int> path,is;
        std::vector<double> distance;
        std::vector<std::vector<double>>destination;
        std::vector<std::vector<pcl::PointXYZ>> points;

        void init();
        void showpath(int start_pt,int end_pt);

    public:
        std::vector<int>line_size;
        std::vector<int> path_out;
        std::vector<int> pt_num_a_line;
        std::vector<pcl::PointXYZ>path_points;

        dijkstra(std::vector<std::vector<pcl::PointXYZ>> points_input);
        
        void find(int start_pt,int end_pt);

        void find_path_points();
        std::vector<int> get_pt_num_a_line();
        std::vector<pcl::PointXYZ> get_path_points();



    // double destination[N][N];//路程矩阵





};


#endif /* DIJKSTRA_H_ */