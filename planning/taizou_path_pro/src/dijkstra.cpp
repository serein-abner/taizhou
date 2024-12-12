// ————————————————
// 版权声明：本文为CSDN博主「九幽小班」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
// 原文链接：https://blog.csdn.net/qq_41559171/article/details/88031577
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include "dijkstra.h"
#include <fstream>
#include <iomanip>
// //find函数寻找最短路径
// void find(int N,int *is,double *distance,int *path ,double *destination[2][3]){
//     //最多进行N-1次循环
//     for(int k=0;k<N;k++){
//         double max=100000;//路径最大值
//         int u=0;//u为已经找到最短路径的节点,初始状态都为起点0
//         for(int i=0;i<N;i++){
//             if(is[i]==0 && distance[i]<max){
//                 u=i;
//                 max=distance[i];
//             }
//         }
//         is[u]=1;
//         //找出一点后更新其余点的最短路径
//         for(int y=0;y<N;y++){
//             if(is[y]==0 && distance[u]+destination[u][y]<distance[y]){
//                 distance[y]=distance[u]+destination[u][y];
//                 path[y]=u;
//             }
//         }
//     }
// }
// 输出最短路径
// void showpath(){
// for(int i=0;i<N;i++){
//     if(is[i]==1){
//         int k=i;
// 		if(k!=0)
// 		{
//         while(k!=0){
//             int k2=k+1;//使输出与标号对应
//             std::cout<<k2<<"<-";
//             k=path[k];
//         }//若前一节点不为起点则不断查询前一节点
// 		}
// 		else
// 		{
// 			int k2=k+1;//使输出与标号对应
//             std::cout<<k2<<"<-";
// 			k=path[k];
// 			while(k!=0){
//             int k2=k+1;//使输出与标号对应
//             std::cout<<k2<<"<-";
//             k=path[k];
// 			}
// 		}
//         std::cout<<"C"<<' '<<"Distance   "<<distance[i];//要改11111111111111
//         std::cout<<std::endl;//输出一项最短路径
//     }
//     else{
//         std::cout<<i<<"不能到达";
//         std::cout<<std::endl;
//     }
// }
// }
// void init(){}
// //主函数起到遍历初始值的效果
// int get_the_path(int start_pt,int end_pt,std::vector<std::vector<pcl::PointXYZ>> point){
//     int point_num = 0;
//     for(int i ;i<point.size();i++){
//         point_num = point_num + point[i].size();
//     }
//     int  M  = 100000;
//     int N  = point_num;//要改111111111111111
//     double distance[N];//最短路径
//     int path[N];//上一节点
//     int is[N];//是否选用该节点,0位未选用
//     double destination[N][N];//路程矩阵
//     for(int i =0;i<N;i++){
//         for(int j = 0;j<N;j++){
//             if(i==j)
//             destination[i][j] = 0;
//             else
//             destination[i][j] = M;}
//     }
//     std::vector<int>line_size;
//     int temp_cout = 0;
//     for(int i = 0;i<(point.size()/3);i++){
//         line_size.push_back(point[i*3].size());
//         for(int j = 0;j<(line_size[i]*2+1);j++){
//             destination[temp_cout+j][temp_cout+j+1] = 1;
//         }
//         destination[temp_cout+line_size[i]*2+1][temp_cout] = 1;
//         temp_cout =temp_cout + line_size[i];
//         // destination[temp_cout+line_size[i]*2+1][temp_cout] = 1;
//         // destination[temp_cout + point[i*3].size()][temp_cout] = 1;
//         // destination[temp_cout + point[i*3].size()+1][temp_cout+line_size[i]*2+2] = 1;
//         // std::vector<pcl::PointXYZ>line1_3 = point[i*3];
//         // std::vector<pcl::PointXYZ>line2_3 = point[i*3+1];
//         // std::vector<pcl::PointXYZ>line3_3 = point[i*3+2];
//         // for(int j =0;j<line1_3;j++){
//         // }
//     }
//     //开始时进行初始化遍历
//     for(int i=0;i<N;i++){
//         path[i]=0;//一开始所有点的前一点为起点
//         is[i]=0;//都未选用
//         distance[i]=destination[start_pt][i];//初始时路程都为起点道该点距离//要改1111111111111destination的一个参数
//     }
//     is[start_pt]=1;//第一个节点选用//要改1111111111111111
// /*开始查找*************************************************************************************************************/
//     //最多进行N-1次循环
//     for(int k=0;k<N;k++){
//         double max=100000;//路径最大值
//         int u=0;//u为已经找到最短路径的节点,初始状态都为起点0
//         for(int i=0;i<N;i++){
//             if(is[i]==0 && distance[i]<max){
//                 u=i;
//                 max=distance[i];
//             }
//         }
//         is[u]=1;
//         //找出一点后更新其余点的最短路径
//         for(int y=0;y<N;y++){
//             if(is[y]==0 && distance[u]+destination[u][y]<distance[y]){
//                 distance[y]=distance[u]+destination[u][y];
//                 if(y==2)
//                     std::cout<<"u = "<<u<<std::endl;
//                 path[y]=u;
//             }
//         }
//     }
// /*开始查找*************************************************************************************************************/
// /*显示最短路径********************************************************************************************************/
//     for(int i=0;i<N;i++){
//         if(is[i]==1){
//             int k=i;
// 			if(k!=0)
// 			{
//             while(k!=0){
//                 int k2=k+1;//使输出与标号对应
//                 std::cout<<k2<<"<-";
//                 k=path[k];
//             }//若前一节点不为起点则不断查询前一节点
// 			}
// 			else
// 			{
// 				int k2=k+1;//使输出与标号对应
//                 std::cout<<k2<<"<-";
// 				k=path[k];
// 				while(k!=0){
//                 int k2=k+1;//使输出与标号对应
//                 std::cout<<k2<<"<-";
//                 k=path[k];
// 				}
// 			}
//             std::cout<<"C"<<' '<<"Distance   "<<distance[i];//要改11111111111111
//             std::cout<<std::endl;//输出一项最短路径
//         }
//         else{
//             std::cout<<i<<"不能到达";
//             std::cout<<std::endl;
//         }
//     }
// /*显示最短路径********************************************************************************************************/
// }

dijkstra::dijkstra(std::vector<std::vector<pcl::PointXYZ>> points_input)
{
    points = points_input;
    std::cout << "points.size()=  " << points.size() << std::endl;
    for (int i = 0; i < points.size(); i++)
    {
        std::cout << "points[i].size()=  " << points[i].size() << std::endl;
        std::cout << "points[i].size()=  " << points[i].size() << "   "
                  << "points[i][0]=  " << points[i][0] << "   "
                  << "points[i][1]=  " << points[i][1] << "   " << std::endl;
        N = N + 2 * points[i].size(); // 计算节点总数量，每个节点两个方向，所以是2倍
        // N=N+points[i].size();
    }
    init(); // 构建graph，核心！！！
}

void dijkstra::init()
{
    std::cout << "!!!!! N=  " << N << std::endl;
    for (int i = 0; i < N; i++)
    {
        path.push_back(0);
        is.push_back(0);
        distance.push_back(M);
        std::vector<double> row;
        for (int j = 0; j < N; j++) 
        {
            if (i == j)
            {
                row.push_back(0);// 此处赋值为0，代表自己与自己的连接距离为0
            }
            else
            {
                row.push_back(M);// 此处赋值为M，代表没有直接连接关系
            }
        }
        destination.push_back(row);//邻接矩阵
        // std::cout<<"!!!!!   "<<destination.size()<<std::endl;
    }
    int temp_cout = 0;
    int sta_idx = 0;
    for (int i = 0; i < (points.size()); i++)
    {
        // int temp_cnt = 1;
        for (int j = sta_idx; j < (sta_idx + points[i].size() - 1); j++)//sta_idx是用来记录序号的,记录的是已经处理过的节点的数量，包括两个方向的节点
        {
            destination[j][j + 1] = 1;//节点与下一个节点的连接距离
            std ::cout << "destination[" << j << "][" << j + 1 << "] = 1" << std ::endl;

            destination[2 * sta_idx - j + points[i].size() * 2 - 1][2 * sta_idx - j + points[i].size() * 2 - 2] = 1;
            //说明：如果有A、B、C点，a1,b1,c1代表一个方向的路径点，把a2,b2,c2代表另一个方向的路径点，destination中的排列是a1,b1,c1，a2,b2,c2，所以...
            std ::cout << "destination[" << 2 * sta_idx - j + points[i].size() * 2 - 1 << "][" << 2 * sta_idx - j + points[i].size() * 2 - 2 << "] = 1" << std ::endl;
        }
        destination[sta_idx + points[i].size() - 1][sta_idx + points[i].size() * 2 - 1] = 0.1;//端点自旋
        destination[sta_idx + points[i].size()][sta_idx] = 0.1;//端点自旋
        destination[sta_idx + points[i].size() * 2 - 1][sta_idx + points[i].size() - 1] = 0.1;//端点自旋
        destination[sta_idx][sta_idx + points[i].size()] = 0.1;//端点自旋
        std ::cout << "!!!destination[" << sta_idx + points[i].size() - 1 << "][" << sta_idx + points[i].size() * 2 - 1 << "] = 1" << std ::endl;
        std ::cout << "!!!destination[" << sta_idx + points[i].size() << "][" << sta_idx << "] = 1" << std ::endl;
        sta_idx = sta_idx + (points[i].size() * 2);
    }
    if (points.size() != 1)
    {
        int temp_sta_id = 0;
        int next_sta_id = 0;
        for (int i = 0; i < points.size() - 1; i++)//此处写的跨道的端点间的连接
        {
            std::cout << "???" << std::endl;
            next_sta_id = temp_sta_id + points[i].size() * 2;
            destination[temp_sta_id + points[i].size() - 1][next_sta_id + points[i + 1].size() * 2 - 1] = 1;
            destination[temp_sta_id + points[i].size()][next_sta_id] = 1; // 111
            destination[next_sta_id + points[i + 1].size() - 1][temp_sta_id + points[i].size() * 2 - 1] = 1;
            destination[next_sta_id + points[i + 1].size()][temp_sta_id] = 1;
            temp_sta_id = temp_sta_id + points[i].size() * 2;
        }
    }

    // pt_num_a_line.push_back(temp_cout);
    // line_size.push_back(points[i].size()); // line_size保存的是每一行树里面,除左右端点的，单独一边的数量
    // for (int j = 0; j < (line_size[i] * 2 + 1); j++)
    // {
    //     destination[temp_cout + j][temp_cout + j + 1] = 1;
    // }
//
    // std::cout << "!!!!! N=  " << 200 << std::endl;
    // line_size.clear();
    // pt_num_a_line.clear();
    // int temp_cout = 0;
    // for(int i = 0;i<(points.size()/3);i++){
    //     pt_num_a_line.push_back(temp_cout);
    //     line_size.push_back(points[i*3].size());//line_size保存的是每一行树里面,除左右端点的，单独一边的数量
    //     for(int j = 0;j<(line_size[i]*2+1);j++){
    //         destination[temp_cout+j][temp_cout+j+1] = 1;
    //     }
    //     destination[temp_cout+line_size[i]*2+1][temp_cout] = 1;
    //     destination[temp_cout+line_size[i]-2][temp_cout+line_size[i]*3+1] = 1;
    //     destination[temp_cout+line_size[i]*3][temp_cout+line_size[i]*1-1] = 1;
    //     destination[temp_cout+line_size[i]*2-1][temp_cout+line_size[i]*4+2] = 1;
    //     destination[temp_cout+line_size[i]*4+1][temp_cout+line_size[i]*2] = 1;
    //     temp_cout =temp_cout + line_size[i]*2+2;//同一行树，第一圈
    //     /*****************************************************************************/
    //     for(int j = 0;j<(line_size[i]*2+1);j++){
    //         destination[temp_cout+j][temp_cout+j+1] = 1;
    //     }
    //     destination[temp_cout+line_size[i]*2+1][temp_cout] = 1;
    //     temp_cout =temp_cout + line_size[i]*2+2;//第二圈
    // }
    // std::cout << "!!!!! N=  " << 226 << std::endl;
    // pt_num_a_line.push_back(temp_cout);
    // for(int i=0;i<pt_num_a_line.size()-2;i++)
    // {
    //     std::cout<<"("<<pt_num_a_line[i]+line_size[i]<<","<<pt_num_a_line[i+1]+line_size[i+1]<<")"<<std::endl;
    //     std::cout<<"("<<pt_num_a_line[i+1]+line_size[i+1]*2+1<<","<<pt_num_a_line[i]+line_size[i]*2+1<<")"<<std::endl;
    //     std::cout<<"("<<pt_num_a_line[i+1]+line_size[i+1]*3+2<<","<<pt_num_a_line[i]+line_size[i]*3+2<<")"<<std::endl;
    //     std::cout<<"("<<pt_num_a_line[i]+line_size[i]*4+3<<","<<pt_num_a_line[i+1]+line_size[i+1]*4+3<<")"<<std::endl;
    //     destination[pt_num_a_line[i]+line_size[i]][pt_num_a_line[i+1]+line_size[i+1]] = 1;
    //     destination[pt_num_a_line[i+1]+line_size[i+1]*2+1][pt_num_a_line[i]+line_size[i]*2+1]= 1;
    //     destination[pt_num_a_line[i+1]+line_size[i+1]*3+2][pt_num_a_line[i]+line_size[i]*3+2]= 1;
    //     destination[pt_num_a_line[i]+line_size[i]*4+3][pt_num_a_line[i+1]+line_size[i+1]*4+3]= 1;
    // }

    /*可视化以文本的形式，无实际作用，只用于调试****************************************************************************************************/
    std::ofstream outfile("/home/hwkan/Music/231102_taizou_test/231108_tiazou_demo/show_destination.txt");
    if (outfile.is_open())
    {
        std::cout << "creat file!" << std::endl;

        for (int i = 0; i < destination[0].size(); i++)
        {
            outfile << std::setw(13) << std::setprecision(0) << std::fixed << std::to_string(i) << ',';
        }
        outfile << "\n";
        for (int i = 0; i < destination.size(); i++)
        {
            for (int j = 0; j < destination[i].size(); j++)
            {
                outfile << std::setw(13) << std::setprecision(6) << std::fixed << std::to_string(destination[i][j]) << ',';
            }
            outfile << "\n";
        }
    }
    else
        std::cout << "fail to creat file!" << std::endl;
    outfile.close();
    /*可视化以文本的形式****************************************************************************************************/
}

// //find函数寻找最短路径
void dijkstra::find(int start_pt, int end_pt)
{
    // if((start_pt<0)||(start_pt>pt_num_a_line[pt_num_a_line.size()-1]-1)||(end_pt<0)||(end_pt>pt_num_a_line[pt_num_a_line.size()-1]-1)){
    //     std::cout<<"start_pt or end_pt is wrong!!!"<<std::endl;
    //     return;
    // }

    // std::cout<<"2 part done!"<<std::endl;
    // 开始时进行初始化遍历
    for (int i = 0; i < N; i++)
    {
        path[i] = 0; // 一开始所有点的前一点为起点
        is[i] = 0;   // 都未选用

        distance[i] = destination[start_pt][i]; // 初始时路程都为起点道该点距离//要改1111111111111destination的一个参数
    }
    // is[start_pt]=1;//第一个节点选用//要改1111111111111111
    // std::cout<<"2 part done!"<<std::endl;

    // 最多进行N-1次循环
    for (int k = 0; k < N; k++)
    {

        // std::cout<<"is[5] ="<<is[5]<<std::endl;
        double max = 100000; // 路径最大值
        int u = 0;           // u为已经找到最短路径的节点,初始状态都为起点0
        for (int i = 0; i < N; i++)
        {
            if (is[i] == 0 && distance[i] < max)
            {

                u = i;
                max = distance[i];
                if (max == 0)
                    std::cout << "u = " << u << std::endl;
            }
        }
        // std::cout<<"u = "<<u<<std::endl;
        is[u] = 1;

        // 找出一点后更新其余点的最短路径
        for (int y = 0; y < N; y++)
        {
            // if(y==5)
            // {
            //     // std::cout<<"is[4]="<<is[4]<<std::endl;
            //     std::cout<<"u="<<u<<","<<"distance[u]="<<distance[u]<<","<<"destination[u][y]="<<destination[u][y]<<","<<"destination[y]="<<distance[y]<<std::endl;
            // }
            if (is[y] == 0 && distance[u] + destination[u][y] <= distance[y])
            {
                distance[y] = distance[u] + destination[u][y];
                // if()
                // if(y==5)
                // {std::cout<<"path[5]"<<std::endl;}
                //     {
                // std::cout<<"u = "<<u<<std::endl;
                // std::cout<<"u = "<<u<<std::endl;
                // std::cout<<"u = "<<u<<std::endl;
                // std::cout<<"y = "<<y<<std::endl;
                // }
                // else
                // std::cout<<"u = "<<u<<std::endl;
                path[y] = u;
            }
        }
    }
    // std::cout<<"2 part done!"<<std::endl;
    showpath(start_pt, end_pt);
    find_path_points();
}

void dijkstra::showpath(int start_pt, int end_pt)
{
    // for(int i=0;i<N;i++){
    //     std::cout <<i<< "   "<<path[i]<<  "   "<< is[i] <<std::endl;
    // }
    path_out.clear();
    std::cout << "distance[end_pt] = " << distance[end_pt] << std::endl;
    ;

    std::cout << end_pt << "<-";
    path_out.push_back(end_pt);

    while ((path[end_pt] != start_pt) && distance[end_pt])
    {
        end_pt = path[end_pt];
        std::cout << end_pt << "<-";
        path_out.push_back(end_pt);
    }
    std::cout << start_pt << std::endl;
    path_out.push_back(start_pt);
    // find_path_points();
}

void dijkstra::find_path_points()
{
    // std::cout<<"find path points"<<std::endl;
    path_points.clear();
    for (int j = 0; j < path_out.size(); j++)
    {
        int pt_idx = path_out[path_out.size() - j - 1];
        // int pt_idx_start=0;
        int temp_sum = 0;
        // if (i == (path_out.size()-1))
        // {
        //     std::cout << "pt_idx" << pt_idx << std::endl;
        // }
        for (int i = 0; i < points.size(); i++)
        {

            temp_sum = temp_sum + points[i].size() * 2;
            if ((pt_idx < temp_sum) && (pt_idx >= temp_sum - points[i].size())) // 在反向列
            {
                int tem_col_idx = pt_idx - temp_sum + points[i].size();

                path_points.push_back(points[i][tem_col_idx]);

                if (j == (path_out.size() - 1))
                {
                    // std::cout << "402i = " << i << std::endl;
                    // std::cout << "tem_col_idx = " << tem_col_idx << std::endl;
                }

                break;
            }
            if ((pt_idx < temp_sum) && (pt_idx < temp_sum - points[i].size())) // 在正向列
            {
                int tem_col_idx = pt_idx - temp_sum + points[i].size() * 2;
                path_points.push_back(points[i][tem_col_idx]);

                if (j == (path_out.size() - 1))
                {

                    // std::cout << "415i = " << i << std::endl;
                    // std::cout << "tem_col_idx = " << tem_col_idx << std::endl;
                }

                break;
            }
        }

        //     int line_idx = 0;
        //     for(int j=0;j<pt_num_a_line.size();j++){
        //         if (pt_idx>pt_num_a_line[j]-1){
        //             pt_idx_start=pt_num_a_line[j];
        //             line_idx = j;
        //             // std::cout<<"line_idx!!!"<<"  =  "<<line_idx<<std::endl;
        //         }
        //         //  std::cout<<"pt_num_a_line[j].size()"<<"  =  "<<pt_num_a_line[j]<<std::endl;
        //     }
        //     // std::cout<<"line_idx"<<"  =  "<<line_idx<<std::endl;
        //     int temp_line_size = line_size[line_idx];
        //     // std::cout<<"temp_line_size"<<"  =  "<<temp_line_size<<std::endl;
        //     // std::cout<<"pt_idx"<<"  =  "<<pt_idx<<std::endl;
        //     pt_idx = pt_idx-pt_idx_start;
        //     // std::cout<<"pt_idx"<<"  =  "<<pt_idx<<std::endl;
        //     if (pt_idx==temp_line_size||pt_idx==(2*temp_line_size+1)||pt_idx==(3*temp_line_size+2)||pt_idx==(4*temp_line_size+3)){
        //         path_points.push_back(points[line_idx*3+1][1-(pt_idx%temp_line_size)%2]);
        //         std::cout<<"get point"<<"  =  "<<line_idx*3+1<<","<<1-(pt_idx%temp_line_size)%2<<std::endl;
        //     }
        //     else if(pt_idx<(2*temp_line_size+2)){
        //         if(pt_idx<temp_line_size){
        //             path_points.push_back(points[line_idx*3+0][pt_idx]);
        //         }
        //         else{
        //             path_points.push_back(points[line_idx*3+2][temp_line_size*2-pt_idx]);
        //         }
        //     }
        //     else {
        //         pt_idx = pt_idx-2*temp_line_size-2;
        //         if(pt_idx<temp_line_size){
        //             path_points.push_back(points[line_idx*3+2][pt_idx]);
        //         }
        //         else{
        //             path_points.push_back(points[line_idx*3+0][temp_line_size*2-pt_idx]);
        //         }
        //     }
    }
}

std::vector<pcl::PointXYZ> dijkstra::get_path_points()
{
    return path_points;
}

std::vector<int> dijkstra::get_pt_num_a_line()
{
    return pt_num_a_line;
}