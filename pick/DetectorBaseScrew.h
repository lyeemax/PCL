//
// Created by cobot on 19-2-15.
//

#ifndef BINPICKINGDETECTORMEDICINEBOX_DETECTORBASESCREW_H
#define BINPICKINGDETECTORMEDICINEBOX_DETECTORBASESCREW_H



//#include <vision/3D/interface/toolKit.h>
//#include <vision/3D/interface/Vision3DInterface.h>
#include <string>

#include <opencv2/opencv.hpp>

#include <pcl/surface/gp3.h>   //贪婪三角化重建。
#include <pcl/surface/poisson.h> //泊松重建。
#include <pcl/surface/marching_cubes_hoppe.h>//立方体重建法。
#include <pcl/surface/marching_cubes_rbf.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/io/pcd_io.h>
#include <sstream>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>  //随机样本一致性算法 分割方法
#include <pcl/segmentation/extract_clusters.h>  // extractEuclideanClusters
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <librealsense2/hpp/rs_pipeline.hpp>

//#define DEBUG_SCREW
namespace cVisionDL {
    struct ObjectPose {
        Eigen::Vector3d coordInCam;//外部接口, 相机坐标系下中心点坐标
        Eigen::Vector3d normalInCam;//外部接口，相机坐标系下法向量坐标
        Eigen::Vector3d longSideDirection;//外部接口，长边在相机坐标系下的方向矢量

        pcl::PointXYZ center;//表面中心
        pcl::Normal normal; //从表面向外
    };

    struct detectParam {
        //roi set
        int roiX;
        int roiY;
        int roiW;
        int roiH;

        //点云过滤参数
        float ptMinZ;
        float ptMaxZ;
        float ptMinX;
        float ptMaxX;
        float ptMinY;
        float ptMaxY;

        //下采样
        float downSampeX;
        float downSampeY;
        float downSampeZ;

        //滤波statisticalFilter参数
        int meanK;
        float mulStd;

        //欧式距离分割，去除螺丝，保留背景，用于重建
        float segdistanceThreshold;
        float segScrewMaxSize;
        float segBkgroundMaxSize;

        //滤波参数
        float filterRadius;
        float filterMinNum;

        //法向量计算参数
        double normalRadius;

        //三维重建参数
        //贪婪三角化参数
        float greedyConnectPointDisMax;//贪婪三角化连接点之间最大距离
        float greedySearchDisMax;//设置连接点最大距离
        float greedyNearestNeighborsNum;//设置可搜索邻域个数
        float greedySurfaceAngle;//设置法线方向偏离样本的最大角度
        float greedyMinInteriorAngle;//设置三角化得到的内角最小角度
        float greedyMaxInteriorAngle;//设置三角化得到的内角最小角度
        bool greedyNormalConsistency;//设置保证法线朝向一致

        //泊松表面重建Poisson
        bool poissonConfidence;//是否使用发想了的大小作为置信信息。如果false，所有法向量均归一化
        float poissonDegree;//设置参数degree[1,5],值越大越精细，耗时越久
        float poissonDepth;//树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
        float poissonIsoDivide;//用于提取ISO等值面的算法的深度
        bool poissonManifold;//是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
        bool poissonOutputPolygons;//是否输出多边形网格（而不是三角化移动立方体的结果）
//        float poissonSamplesPerNode;//设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
        float poissonScale;//设置用于重构的立方体直径和样本边界立方体直径的比率
        float poissonSolverDivide;//设置求解线性方程组的Gauss-Seidel迭代方法的深度




        //分割参数
        //最小尺寸,允许的点云簇的最小尺寸
        int segMinSize;
        //最大尺寸,允许的点云簇的最大尺寸
        int segMaxSize;
        //半径阈值, 典型值0.5
        float segResidualThreshold;
        //曲率阈值, 典型值0.05　
        float segCurvatureThreshold;
        //角度阈值，以度为单位，典型值5度
        float segThetaThresholdInDegree;
        //邻域阈值，指定用包含多少个点的邻域
        int segNeighborNumber;
        //分割的距离阈值
        float segMaxDis;

        //平面拟合的距离阈值
        float distanceThreshold;
        //最大迭代次数
        int maxIterations;
        //内点比例阈值
        float planeInliersRatio;

        //后处理参数
        float maxAngleInDegreeWithZ;
        float widthThreshold;
        float heightThreshold;
    };

    class DetectorBaseScrew {
    public:
        DetectorBaseScrew();

        //配置参数
        bool setParam(const std::string &fileFullName);
        void initRealsense();
        //初始数据下采样
        bool downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud);
        //三维重建
        bool reconstructPolygonMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud, pcl::PolygonMesh &triangles);//triangles//存储最终三角化的网络模型



        //0.1 由相机原始数据转化成，便于通过2D图像查找的mat格式的点云
        void generatePointCloud(const std::string &path_xy, const std::string &path_pointcloud,cv::Mat &pointCloud);
        //edit by ma
        void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, bool bSave);
        void matToPCD(const cv::Mat& pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool bSave=true);
        bool cropROI(const cv::Mat &imgOrg, const cv::Mat &pcOrg, cv::Mat &imgROI, cv::Mat &pcROI);


        //对输入点云进行裁剪，留下Xmin-Xmax/Ymin-Ymax/Zmin-Zmax范围内点云
        bool getRoi(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud);
        //edit by ma
        pcl::PointCloud<pcl::PointXYZ>::Ptr getRoiFixed(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud);
        bool getPlaneInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud ,pcl::PointIndices::Ptr &inliersIds,pcl::PointCloud<pcl::PointXYZ>::Ptr &inlierscloud,float &inliersSizeRatios);

        //对点云分割，得到去除螺丝的背景点云
        bool getbkground(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &bkgroundcloud , pcl::PointCloud<pcl::PointXYZ>::Ptr &targetcloud);

        bool getScrewCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointIndices::Ptr &otherinliersIds,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFilter);
        bool getScrewCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &screwCloud);
        //分割得到各个螺丝点云簇
        bool getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters);

        bool getScrewPose(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
        void initViewer(){
            viewer=boost::make_shared<pcl::visualization::PCLVisualizer>("viewer");
            poseViewer=boost::make_shared<pcl::visualization::PCLVisualizer>("getScrewPose");
        }
        pcl::visualization::PCLVisualizer::Ptr viewer;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> poseViewer;

    private:
        void initParam(const detectParam &param);

        bool loadParamFile(const std::string &fileFullName, detectParam &param);

        detectParam _param;
        rs2::pipeline pipe;
        rs2::config cfg;

    };
}


#endif //BINPICKINGDETECTORMEDICINEBOX_DETECTORBASESCREW_H
