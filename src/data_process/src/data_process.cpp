#include <iostream>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h> 
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <unordered_set>
#include <dirent.h>
#include <sys/time.h>

#include <chrono>
#include <cuda_runtime.h>

#include "common_func/algorithm.h"
#include "data_process/image_process.h"


using namespace std;
static float floor_height = 0.0;
static int start_x, start_y;
static int end_x, end_y;
static int path_valid = 0; 
static int car_drive_direction = 255;

static int32_t getFileNum(const std::string &path) {   //需要用到<dirent.h>头文件
    int32_t fileNum=0;
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str())))
        return fileNum;
    while((ptr=readdir(pDir))!=0){
        if(strcmp(ptr->d_name,".")!=0&&strcmp(ptr->d_name,"..")!=0 )
           fileNum++;
    }
    closedir(pDir);
    return fileNum;
}

void voxelgrid_cuda(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSrc,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDst)
{
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  std::chrono::duration<double, std::ratio<1, 1000>> time_span =
     std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
  cudaStream_t stream = NULL;
  cudaStreamCreate ( &stream );

  unsigned int nCount = cloudSrc->width * cloudSrc->height;
  float *inputData = (float *)cloudSrc->points.data();



  std::cout << "\n------------checking CUDA ---------------- "<< std::endl;
  std::cout << "CUDA Loaded "
      << cloudSrc->width*cloudSrc->height
      << " data points from PCD file with the following fields: "
      << pcl::getFieldsList (*cloudSrc)
      << std::endl;

  float *input = NULL;
  cudaMallocManaged(&input, sizeof(float) * 4 * nCount, cudaMemAttachHost);
  cudaStreamAttachMemAsync (stream, input );
  cudaMemcpyAsync(input, inputData, sizeof(float) * 4 * nCount, cudaMemcpyHostToDevice, stream);
  cudaStreamSynchronize(stream);

  float *output = NULL;
  cudaMallocManaged(&output, sizeof(float) * 4 * nCount, cudaMemAttachHost);
  cudaStreamAttachMemAsync (stream, output );
  cudaStreamSynchronize(stream);

  cudaFilter filterTest(stream);
  FilterParam_t setP;
  FilterType_t type;


  unsigned int countLeft = 0;
  std::cout << "\n------------checking CUDA VoxelGrid---------------- "<< std::endl;

  type = VOXELGRID;

  setP.type = type;
  setP.voxelX = 1;
  setP.voxelY = 1;
  setP.voxelZ = 1;

  filterTest.set(setP);
  int status = 0;
  cudaDeviceSynchronize();
  t1 = std::chrono::steady_clock::now();
  status = filterTest.filter(output, &countLeft, input, nCount);
  cudaDeviceSynchronize();
  t2 = std::chrono::steady_clock::now();

  if (status != 0)
    return;
  time_span = std::chrono::duration_cast<std::chrono::duration<double, std::ratio<1, 1000>>>(t2 - t1);
  std::cout << "CUDA VoxelGrid by Time: " << time_span.count() << " ms."<< std::endl;
  std::cout << "CUDA VoxelGrid before filtering: " << nCount << std::endl;
  std::cout << "CUDA VoxelGrid after filtering: " << countLeft << std::endl;

  cloudDst->width = countLeft;
  cloudDst->height = 1;
  cloudDst->points.resize (cloudDst->width * cloudDst->height);

  int check = 0;
  for (std::size_t i = 0; i < cloudDst->size(); ++i)
  {
      cloudDst->points[i].x = output[i*4+0];
      cloudDst->points[i].y = output[i*4+1];
      cloudDst->points[i].z = output[i*4+2];
  }


  cudaFree(input);
  cudaFree(output);
  cudaStreamDestroy(stream);
}


int32_t separate_ground(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground,
    pcl::PointCloud<pcl::PointXYZ>::Ptr off_ground,
    int *rotate_angle)
{
    int num_points = input->points.size();
    cout << num_points << endl; //2063
    std::unordered_set<int> inliersResult;
    int angle = 0;
    int angle_valid_times = 0;

    int maxIterations = MAX_RANSIC_LOOP_TIMES; //迭代次数
    while (maxIterations--)  //
    {
        std::unordered_set<int> inliers;  //存放平面的内点，平面上的点也属于平面的内点
        //因为一开始定义inliers，内点并没有存在，所以随机在原始点云中随机选取了三个点作为点云的求取平面系数的三个点
        while (inliers.size() < 3)  //当内点小于3 就随机选取一个点 放入内点中 也就是需要利用到三个内点
        {
            inliers.insert(rand() % num_points);   //产生 0~num_points 中的一个随机数
        }
 
        // 需要至少三个点 才能找到地面
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = inliers.begin();  //auto 自动类型
        x1 = input->points[*itr].x;
        y1 = input->points[*itr].y;
        z1 = input->points[*itr].z;
        itr++;
        x2 = input->points[*itr].x;
        y2 = input->points[*itr].y;
        z2 = input->points[*itr].z;
        itr++;
        x3 = input->points[*itr].x;
        y3 = input->points[*itr].y;
        z3 = input->points[*itr].z;
 
        //计算平面系数
        float a, b, c, d, sqrt_abc;
        a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        d = -(a * x1 + b * y1 + c * z1);
        sqrt_abc = sqrt(a * a + b * b + c * c);

        //理论上地面和摄像头Y轴平行，因此计算地面和XZ平面的夹角粗略认为是摄像头的俯视角
        if(c == 0)
        {
            continue;
        }
        double camera_angle = atan(a / c);
        camera_angle *= (180/M_PI);
        //cout << "get angel: " << camera_angle <<endl;

        if((camera_angle < CAMERA_ROTATE_MIN) || (camera_angle > CAMERA_ROTATE_MAX))
        {
            continue;
        }

        angle += camera_angle;
        angle_valid_times++;

        //分别计算这些点到平面的距离
        for (int i = 0; i < num_points; i++)
        {
            if (inliers.count(i) > 0) //判断一下有没有内点
            { // that means: if the inlier in already exist, we dont need do anymore
                continue;
            }
            pcl::PointXYZ point = input->points[i];
            float x = point.x;
            float y = point.y;
            float z = point.z;

            float dist = fabs(a * x + b * y + c * z + d) / sqrt_abc; // calculate the distance between other points and plane
            float distanceTol = RANSIC_DISTANCE_THRESHOLD;
            if (dist < distanceTol)
            {
                inliers.insert(i); //如果点云中的点 距离平面距离的点比较远 那么该点则视为内点
            }
            //将inliersResult 中的内容不断更新，因为地面的点一定是最多的，所以迭代40次 找到的inliersResult最大时，也就相当于找到了地面
            //inliersResult 中存储的也就是地面上的点云
            if (inliers.size() > inliersResult.size())
            {
                inliersResult = inliers;  
            }
        }
        //cout << inliers.size() << endl;
    }
    //迭代结束后，所有属于平面上的内点都存放在inliersResult中
    //std::unordered_set<int> inliersResult;
    cout << inliersResult.size() << endl;  //1633
   


    //两片点云，一片存放地面，一片存放其他点   
    for (int i = 0; i < num_points; i++)
    {
        pcl::PointXYZ pt = input->points[i];
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;

        if (inliersResult.count(i))
        {
            //手动增加范围过滤无效点
            if((x < X_VALID_MIN) || (x > X_VALID_MAX) || (y < Y_VALID_MIN) || (y > Y_VALID_MAX) || (z < Z_VALID_MIN) || (z > Z_VALID_MAX))
            {
                continue;
            }

            //cout << i << " x: " << x << " y: " << y << " z: " <<z <<endl;
            ground->points.push_back(pt);
        }
        else
        {
            off_ground->points.push_back(pt);
        }
    }

    if(angle_valid_times != 0)
    {
        angle = angle / angle_valid_times;
    }

    *rotate_angle = -1 * angle;
    cout<<"final angle: " << *rotate_angle << endl;


    return 0;
}

int32_t get_aera_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, uchar image[][MAP_COL_NUM])
{
    int point_index = 0;
    int row = 0;
    int col = 0;

    memset(image, 0, MAP_ROW_NUM * MAP_COL_NUM);

    for(point_index = 0; point_index < input->points.size(); point_index++)
    {

        pcl::PointXYZ pt = input->points[point_index];
        if((pt.x < X_VALID_MIN) || (pt.x > X_VALID_MAX) || (pt.y < Y_VALID_MIN) || (pt.y > Y_VALID_MAX) || (pt.z < floor_height) || (pt.z > floor_height + CAR_HEIGHT_MAX))
        {

            continue;
        }

        if(output != NULL)
        {
            output->points.push_back(pt);
        }

        uint32_t row_pos = floor((pt.x - X_VALID_MIN) / (PIC_GRID));
        uint32_t col_pos = floor((pt.y - Y_VALID_MIN) / (PIC_GRID));
        row = (row_pos < MAP_ROW_NUM) ? row_pos : (MAP_ROW_NUM - 1);
        col = (col_pos < MAP_COL_NUM) ? col_pos : (MAP_COL_NUM - 1);

        image[row][col]+= POINT_ONCE_ACCU;
    }

    for(int i = 0; i < MAP_ROW_NUM; i++)
    {
        for(int j = 0; j < MAP_COL_NUM; j++)
        {
            image[i][j] = (image[i][j] > POINT_ONCE_ACCU) ? 255 : 0;

            //set obstacle in blind aera
            //1: y = SLOPE_POS * (x - MAP_COL_NUM / 2)
            if(i < round(X_BLIND_DIS / (PIC_GRID)))
            {
                image[i][j] = 255;
            }

            if((SLOPE_POS * i + j) < MAP_COL_NUM / 2)
            {
                image[i][j] = 255;
            }

            if((SLOPE_NEG * i + j) > MAP_COL_NUM / 2)
            {
                image[i][j] = 255;
            }

        }
    }

    return 0;
}

static float calculate_floor_hight(const pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    float total_height = 0.0;
    float x_min, x_max =0;
    float y_min, y_max = 0;
    float z_min, z_max = 0;
    int32_t num = input->points.size();

    for(int i = 0; i < num; i++)
    {
        pcl::PointXYZ pt = input->points[i];
        total_height += pt.z;

        x_min = (x_min < pt.x) ? x_min : pt.x;
        x_max = (x_max > pt.x) ? x_max : pt.x;
        y_min = (y_min < pt.y) ? y_min : pt.y;
        y_max = (y_max > pt.y) ? y_max : pt.y;
        z_min = (z_min < pt.z) ? z_min : pt.z;
        z_max = (z_max > pt.z) ? z_max : pt.z;
    }

    cout<<"x_min: " << x_min << " x_max: " << x_max << " y_min: " << y_min << " y_max: " << y_max << " z_min: " << z_min << " z_max: " << z_max << endl;

    return (total_height / num);
}



int main(int argc, char **argv)
{
    int32_t pcd_num = 0;
    int32_t pcd_index = 0;
    int32_t i = 0, j = 0;
    pcd_num = getFileNum(PCD_FILE_DIR);
    char file_name[100] = {0};
    cout << "total " << pcd_num << " .pcd file" << endl;

    #if(DEBUG_TIME_PRINT)
    struct timeval tv_begin, tv_tag1, tv_tag2, tv_tag3, tv_tag4, tv_tag5;
    #endif

    while(pcd_index < pcd_num)
    {
        memset(file_name, 0, 100);
        sprintf(file_name, PCD_FILE_DIR"cloud_pont_%d.pcd", pcd_index);
        cout<<"[begin] process pcf file: " << file_name<<endl;


        #if(DEBUG_TIME_PRINT)
        gettimeofday(&tv_begin, NULL);
        #endif

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cuda_cloud_sampled(new pcl::PointCloud<pcl::PointXYZ>);
        // 填入点云数据
        if(pcl::io::loadPCDFile(file_name, *cloud) < 0)
        {
            pcd_index++;
            continue;
        }

        std::cerr << "Cloud before filtering:" << std::endl;
        std::cerr << *cloud << std::endl;



        //sample by pcl
        pcl::VoxelGrid<pcl::PointXYZ> sor;//滤波处理对象
        sor.setInputCloud(cloud);
        sor.setLeafSize(SAMPLE_GRID, SAMPLE_GRID, SAMPLE_GRID);//设置滤波器处理时采用的体素大小的参数
        sor.filter(*cloud_sampled);  

        #if(DEBUG_TIME_PRINT)
        gettimeofday(&tv_tag1, NULL);
        cout<<"[sample time]: "<<  (tv_tag1.tv_sec*1000000 + tv_tag1.tv_usec) - (tv_begin.tv_sec*1000000 + tv_begin.tv_usec)<<endl;
        #endif

        //sample by cuda
        voxelgrid_cuda(cloud, cuda_cloud_sampled);

        //get ground using RANSIC
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_point(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr off_ground_point(new pcl::PointCloud<pcl::PointXYZ>);
        int rotate_angle = 0;
        separate_ground(cloud_sampled, ground_point, off_ground_point, &rotate_angle);

        #if(DEBUG_TIME_PRINT)
        gettimeofday(&tv_tag2, NULL);
        cout<<"[RANSAC time]: "<<  (tv_tag2.tv_sec*1000000 + tv_tag2.tv_usec) - (tv_tag1.tv_sec*1000000 + tv_tag1.tv_usec)<<endl;
        #endif

        //rotate as camera angle
        //rotate_angle = CAMERA_ROTATE_DEFAULT;
        float theta = M_PI * rotate_angle/180; // 弧度角
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
        //std::cout << transform.matrix() << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_transformed (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr off_ground_transformed (new pcl::PointCloud<pcl::PointXYZ> ());
        pcl::transformPointCloud (*ground_point, *ground_transformed, transform);
        pcl::transformPointCloud (*off_ground_point, *off_ground_transformed, transform);

        #if(DEBUG_TIME_PRINT)
        gettimeofday(&tv_tag3, NULL);
        cout<<"[rotate time]: "<<  (tv_tag3.tv_sec*1000000 + tv_tag3.tv_usec) - (tv_tag2.tv_sec*1000000 + tv_tag2.tv_usec)<<endl;
        #endif

        floor_height = calculate_floor_hight(ground_transformed);
        cout << "average floor height: " << floor_height << endl;

        //障碍物点云3D转2D（除去那些过比小车高、以及在地面以下的障碍物点），绘制区域二维地图
        /*
            对于具体地图中循迹，只关心一定范围内的地图信息，此处（二维）X-min~X_max、Y_min~Y_max的一个二维矩阵，网格大小:SAMPLE_GRID;
            比如只关注Y轴-2~2，X轴0~4米范围，SAMPLE_GRID为0.1米，则生成的局部地图是一个40*40大小的网格;
            生成网格步骤：
                在指定X.Y范围内遍历所有非地面点，若改点坐标落在网格内，且Z轴坐标满足障碍物条件（理论上是z轴为负数或者z轴值大于小车高度）,
                则该网格置1，若无有效障碍物点云或者该处无点云信息（视野盲区），则认为是有效路径提取网格
        */
        uchar map_temp[MAP_ROW_NUM][MAP_COL_NUM] = {0};
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_obstacle (new pcl::PointCloud<pcl::PointXYZ> ());
        get_aera_map(off_ground_transformed, filtered_obstacle, map_temp);

        #if(DEBUG_TIME_PRINT)
        gettimeofday(&tv_tag4, NULL);
        cout<<"[3D-2D time]: "<<  (tv_tag4.tv_sec*1000000 + tv_tag4.tv_usec) - (tv_tag3.tv_sec*1000000 + tv_tag3.tv_usec)<<endl;
        #endif

        //new 2D array to store map(get part map)

        uchar map_formal[MAP_VALID_ROW_NUM][MAP_VALID_COL_NUM] = {0};
        uint32_t base_row = floor(CAR_WITH_MAX / PIC_GRID);
        uint32_t base_col =  floor((MAP_COL_NUM - MAP_VALID_COL_NUM) / 2);

        cout << "1: " << base_row << " " << base_col <<endl;

        cv::Mat map_binary(MAP_VALID_ROW_NUM ,MAP_VALID_COL_NUM, CV_8UC1);
        for(i = 0; i < MAP_VALID_ROW_NUM; i++)
        {
            for(j = 0; j < MAP_VALID_COL_NUM; j++)
            {
                map_binary.at<uchar>(i, j) = (uchar)(map_temp[base_row + i][base_col + j]);
            }
        }
        cv::Mat obstacle_binary(MAP_VALID_ROW_NUM ,MAP_VALID_COL_NUM, CV_8UC1);
        //获取自定义核
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
            cv::Size(floor(CAR_WITH_MAX / PIC_GRID) ,floor(CAR_WITH_MAX / PIC_GRID))); //第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
        //膨胀操作
        cv::dilate(map_binary, obstacle_binary, element);


        //========    confirm start point and end point    ========//
        //confirm start point first
        start_x = 0; //start point should always be 1st row
        for(j = MAP_VALID_COL_NUM / 2; j > 0; j--)
        {
            if(obstacle_binary.at<uchar>(0, j) == 0)
            {
                start_y = j;
                break;
            }
        }
        cout<<"start point, row: " << 0 << " col: " << start_y <<endl;
        if(j == 0)
        {
            path_valid = 0;
        }

        //find
        bool find_end_success = false;
        for(i = MAP_VALID_ROW_NUM - 1; (i > start_y) && (find_end_success == false); i-=ROW_PATH_FIND_GAP)
        {
        #if CAR_DRIVE_DIR
            for(j = 0; (j < MAP_VALID_COL_NUM) && (find_end_success == false); j+=COL_PATH_FIND_GAP)
        #else
            for(j = MAP_VALID_COL_NUM - 1; (j >= 0) && (find_end_success == false); j-=COL_PATH_FIND_GAP)
        #endif    
            {
                //calculate the straight line between start point and end point
                //thus: (end_y - start_y) * (x - start_x) = (y - start_y) * (end_x - start_x)
                end_x = i;
                end_y = j;
                int row_index = 0;
                int col_index = 0;
                //check if the obstacle is in the straight line
                bool cur_find_failed = false;
                for(row_index = start_x + 1; (row_index < end_x) && (cur_find_failed == false); row_index++)
                {
                    for(col_index = 0; (col_index < MAP_VALID_COL_NUM) && (cur_find_failed == false); col_index++)
                    {
                        uchar value = obstacle_binary.at<uchar>(row_index, col_index);
                        if(value == 0xFF)
                        {
                            long abs_value = 0;
                            abs_value = abs(((row_index - start_x)*(end_y - start_y)) - ((col_index - start_y)*(end_x - start_x)));

                            if(abs_value <= IN_LINE_THRESHOLD)
                            {
                                cur_find_failed = true;
                                continue;
                            }
                        }
                    }
                }

                if(cur_find_failed==false)
                {
                    path_valid = 1;
                    find_end_success = true;
                }
            }

            #if(DEBUG_TIME_PRINT)
            gettimeofday(&tv_tag5, NULL);
            cout<<"[opencv proc time]: "<<  (tv_tag5.tv_sec*1000000 + tv_tag5.tv_usec) - (tv_tag4.tv_sec*1000000 + tv_tag4.tv_usec)<<endl;
            #endif

            #if(PLOT_PATH)
            int row_index = 0;
            int col_index = 0;
            for(row_index = start_x + 1; (row_index < end_x); row_index++)
            {
                for(col_index = 0; (col_index < MAP_VALID_COL_NUM); col_index++)
                {
                    uchar value = obstacle_binary.at<uchar>(row_index, col_index);
                    long abs_value = 0;
                    abs_value = abs(((row_index - start_x)*(end_y - start_y)) - ((col_index - start_y)*(end_x - start_x)));

                    if(abs_value <= IN_LINE_THRESHOLD)
                    {
                        obstacle_binary.at<uchar>(row_index, col_index) = 128;
                    }
                }
            }
            #endif


        }

        cout << "path_valid: " << path_valid << " end_point, row: " << end_x << " col: " << end_y << endl;

        #if(FIGURE_DEBUG)
        if(1)
        {

            pcl::visualization::PCLVisualizer viewer("test");
            //viewer.addCoordinateSystem();  
            int v1(0);
            int v2(1);
            int v3(2);
            int v4(3);
            viewer.createViewPort(0.0, 0.0, 0.25, 1.0, v1);
            viewer.setBackgroundColor(0, 0, 0, v1);
            viewer.createViewPort(0.25, 0.0, 0.5, 1.0, v2);
            viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);
            viewer.createViewPort(0.5, 0.0, 0.75, 1.0, v3);
            viewer.setBackgroundColor(0, 0, 0, v3);
            viewer.createViewPort(0.75, 0.0, 1, 1.0, v4);
            viewer.setBackgroundColor(0.5, 0.5, 0.5, v4);

            viewer.addCoordinateSystem (1.0);//添加法线  每个视图都有一组对应的法线

            //display original cloud points
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud_sampled, "x"); // 按照x字段进行渲染
            viewer.addPointCloud<pcl::PointXYZ>(cloud_sampled, fildColor, "cloud_sampled", v1);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_sampled");
            viewer.addText("sampled cloud point", 10, 10, "cloud_sampled", v1);

            //display ground point and off ground oint together
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ground_green(ground_point, 20, 180, 20);
            viewer.addPointCloud(ground_point, ground_green, "ground_point", v2);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ground_point");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_ground_oringe(off_ground_point, 250, 128, 10);
            viewer.addPointCloud(off_ground_point, off_ground_oringe, "off_ground_point", v2);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "off_ground_point");
            viewer.addText("separate floor and obstacle", 10, 10, "separate floor", v2);


            //display transformed ground point and off ground point together
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> trans_ground_green(ground_transformed, 20, 180, 20);
            viewer.addPointCloud(ground_transformed, trans_ground_green, "ground_transformed", v3);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ground_transformed");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> trans_off_ground_oringe(off_ground_transformed, 250, 128, 10);
            viewer.addPointCloud(off_ground_transformed, trans_off_ground_oringe, "off_ground_transformed", v3);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "off_ground_transformed");
            viewer.addText("separate floor and obstacle(transformed)", 10, 10, "separate floor", v3);


            //display transformed ground point and filtered obstacle together
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> trans_ground_green2(ground_transformed, 20, 180, 20);
            viewer.addPointCloud(ground_transformed, trans_ground_green2, "ground_transformed2", v4);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ground_transformed2");
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> filtered_obstacle_oringe(filtered_obstacle, 250, 128, 10);
            viewer.addPointCloud(filtered_obstacle, filtered_obstacle_oringe, "filtered_obstacle", v4);
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "filtered_obstacle");
            viewer.addText("separate floor and obstacle(transformed)", 10, 10, "separate floor", v4);


            cv::Mat resized_image;
            cv::Mat rotate_image;
            cv::resize(obstacle_binary, resized_image, cv::Size(MAP_ROW_NUM * 5, MAP_COL_NUM * 5), 0, 0, cv::INTER_AREA);
            cv:: flip(resized_image, rotate_image, -1);
            cv::imshow("camera", rotate_image);
            cv::waitKey(1500);

            while (!viewer.wasStopped ()) { // 在按下 "q" 键之前一直会显示窗口
              viewer.spinOnce ();
            }


        }
        #endif






        pcd_index++;
    }






  return 0;
}