//
// Created by cobot on 19-2-15.
//
#include <opencv2/opencv.hpp>
#include "DetectorBaseScrew.h"
#include <librealsense2/rs.hpp>



//重建方法选择
#define RECONSTRUCT_GREEY           //贪婪三角化
//#define RECONSTRUCT_POISSON         //泊松重建
//#define RECONSTRUCT_MARCHINGCUBES   //移动立方体重建
namespace cVisionDL {
    DetectorBaseScrew::DetectorBaseScrew() {
    }


    bool DetectorBaseScrew::getRoi(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud){
        std::vector<float> limit;
        limit.push_back(_param.ptMinX);
        limit.push_back(_param.ptMaxX);
        limit.push_back(_param.ptMinY);
        limit.push_back(_param.ptMaxY);
        limit.push_back(_param.ptMinZ);
        limit.push_back(_param.ptMaxZ);
        //  cubscrop 与cobsys的cVision3D::toolKit::passThroughFilter()相似
        pcl::CropBox<pcl::PointXYZ> filter;
        filter.setInputCloud(oricloud);
        // 设置体素栅格的大小为 1x1x1cm
        filter.setMin(Eigen::Vector4f(limit.at(0),limit.at(2),limit.at(4),1.0));
        //给定立体空间
        filter.setMax(Eigen::Vector4f(limit.at(1),limit.at(3),limit.at(5),1.0));


        filter.filter(*oricloud);
        std::cout<<"PCL downsample"<<oricloud->points.size()<<std::endl;
//        bool resFilter = cVision3D::toolKit::passThroughFilter(oricloud, limit, oricloud);
//        if(!resFilter){
//            return false;
//        }
        return true;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr DetectorBaseScrew::getRoiFixed(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<float> limit;
        limit.push_back(_param.ptMinX);
        limit.push_back(_param.ptMaxX);
        limit.push_back(_param.ptMinY);
        limit.push_back(_param.ptMaxY);
        limit.push_back(_param.ptMinZ);
        limit.push_back(_param.ptMaxZ);
        //  cubscrop 与cobsys的cVision3D::toolKit::passThroughFilter()相似
        pcl::CropBox<pcl::PointXYZ> filter;
        filter.setInputCloud(oricloud);
        // 设置体素栅格的大小为 1x1x1cm
        filter.setMin(Eigen::Vector4f(limit.at(0),limit.at(2),limit.at(4),1.0));
        //给定立体空间
        filter.setMax(Eigen::Vector4f(limit.at(1),limit.at(3),limit.at(5),1.0));

        filter.setNegative(false);

        filter.filter(*cloud);
       // std::cout<<"PCL downsample"<<cloud->points.size()<<std::endl;
//        bool resFilter = cVision3D::toolKit::passThroughFilter(oricloud, limit, oricloud);
//        if(!resFilter){
//            return false;
//        }
        return cloud;
    }


    bool DetectorBaseScrew::getPlaneInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud ,pcl::PointIndices::Ptr &inliersIds,pcl::PointCloud<pcl::PointXYZ>::Ptr &inlierscloud,float &inliersSizeRatios){

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); //存储输出的模型的系数
//        pcl::PointIndices::Ptr inliersIds (new pcl::PointIndices); //存储内点，使用的点 //创建分割对象
        pcl::SACSegmentation<pcl::PointXYZ> seg; //可选设置
        seg.setOptimizeCoefficients (true); //必须设置
        seg.setModelType (pcl::SACMODEL_PLANE); //设置模型类型，检测平面
        seg.setMethodType (pcl::SAC_RANSAC); //设置方法【聚类或随机样本一致性】
//        seg.setMaxIterations(100);
        seg.setDistanceThreshold(4);
        seg.setInputCloud (cloud);
        seg.segment (*inliersIds, *coefficients); //分割操作

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < inliersIds->indices.size(); ++i){
            inlierscloud->points.push_back(cloud->points[inliersIds->indices[i]]);
        }
        inlierscloud->width = inlierscloud->points.size();
        inlierscloud->height = 1;
        inlierscloud->is_dense = true;


        inliersSizeRatios = inliersIds->indices.size()/cloud->points.size();


//        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("getPlaneInlies"));
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color0(cloud, 255, 255, 255);
//        viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color0, "oricloud");
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud1, 255, 0, 0);
//        viewer->addPointCloud<pcl::PointXYZ> (cloud1, single_color3, "cloud1");
        return true;
    }

    bool DetectorBaseScrew::getbkground(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &bkgroundcloud , pcl::PointCloud<pcl::PointXYZ>::Ptr &targetcloud){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; //创建滤波器对象
        sor.setInputCloud(cloud); //设置待滤波的点云
        sor.setMeanK(_param.meanK); //设置在进行统计时考虑查询点临近点数
        sor.setStddevMulThresh(_param.mulStd); //设置判断是否为离群点的阀值，1个标准差以上就是离群点
        //sor.setNegative(true);         //保存离群点
        sor.filter(*cloudFilter); //存储
        // 以下是调用地 cobsys 程序
//        bool res = cVision3D::toolKit::statisticalFilter(cloud, _param.meanK, _param.mulStd, cloudFilter);
//        if(!res)
//            return false;

        //利用欧式聚类分割出各个点云簇，去除螺丝，保留其他
        std::vector<pcl::PointIndices> clusterIndices;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloudFilter);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
        ec.setClusterTolerance ( _param.segdistanceThreshold );                     // 设置近邻搜索的搜索半径为2cm
        ec.setMinClusterSize ( _param.segBkgroundMaxSize );                 //设置一个聚类需要的最少的点数目为100
        ec.setMaxClusterSize ( _param.segScrewMaxSize*1.5 );               //设置一个聚类需要的最大点数目为25000
        ec.setSearchMethod (tree);                    //设置点云的搜索机制  ec.setInputCloud (cloud_filtered);
        ec.extract (clusterIndices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中

//        cVision3D::toolKit::segmentEuclidean(cloudFilter, _param.segdistanceThreshold ,_param.segScrewMaxSize*1.5 ,_param.segBkgroundMaxSize,clusterIndices);
/*
        for (int i = 0; i < clusterIndices.size(); ++i) {
            for (int j = 0; j < clusterIndices[i].indices.size(); ++j) {
                bkgroundcloud->points.push_back(cloud->points[clusterIndices[i].indices[j]]);
            }
        }
        bkgroundcloud->width = bkgroundcloud->points.size();
        bkgroundcloud->height = 1;
        bkgroundcloud->is_dense = true;
*/
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter2(new pcl::PointCloud<pcl::PointXYZ>);
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer0(new pcl::visualization::PCLVisualizer("newCluster_inliers"));

        for (int i = 0; i < clusterIndices.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (unsigned int jj = 0; jj < clusterIndices[i].indices.size(); jj++) {
                newCluster->points.push_back(cloudFilter->points[clusterIndices[i].indices[jj]]);
            }
            newCluster->width = newCluster->points.size();
            newCluster->height = 1;
            newCluster->is_dense = true;

            *cloudFilter2 = *cloudFilter2 + *newCluster;

            pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster_inliers(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster_outliers(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointIndices::Ptr inliersIds (new pcl::PointIndices);
            float ratio = 0;
            getPlaneInliers(newCluster,inliersIds,newCluster_inliers,ratio);

            int r = rand()*255;
            int g = rand()*255;
            int b = rand()*255;

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorin(newCluster_inliers, r, g, b);
            std::string str;
            str = "cloud" + std::to_string(r+g+b);
            viewer0->addPointCloud<pcl::PointXYZ> (newCluster_inliers, single_colorin, str);

            *bkgroundcloud = *bkgroundcloud + *newCluster_inliers;

            getScrewCloud(newCluster,inliersIds,newCluster_outliers);
            *targetcloud = *targetcloud + *newCluster_outliers;
        }
        cloud = cloudFilter;
//        std::cout<<"bkgroundinliersIds.size() = "<<bkgroundinliersIds->indices.size()<<std::endl;

//        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("getbkground clustercloud111"));
//
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(targetcloud, 255, 255, 255);
//        viewer2->addPointCloud<pcl::PointXYZ> (targetcloud, single_color3, "targetcloud");


#ifdef DEBUG_SCREW
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("getbkground clustercloud"));
        for (int i = 0; i < clusterIndices.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (unsigned int jj = 0; jj < clusterIndices[i].indices.size(); jj++) {
                newCluster->points.push_back(cloudFilter->points[clusterIndices[i].indices[jj]]);
            }
            newCluster->width = newCluster->points.size();
            newCluster->height = 1;
            newCluster->is_dense = true;
            std::ostringstream ost1;
            ost1 << "cluster_preprocessed_" << i << ".ply";

            int r = rand()*255;
            int g = rand()*255;
            int b = rand()*255;
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(newCluster, r, g, b);
            viewer->addPointCloud<pcl::PointXYZ> (newCluster, single_color3, ost1.str());
        }
#endif
        return true;
    }

    bool DetectorBaseScrew::getScrewCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,pcl::PointIndices::Ptr &otherinliersIds,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFilter){

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(otherinliersIds);
        //除去平面之外的数据
        extract.setNegative(true);
        extract.filter(*cloudFilter);
        std::cout<<otherinliersIds->indices.size()<<std::endl;


//        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("getScrewCloud"));
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color0(cloudFilter, 255, 255, 255);
//        viewer->addPointCloud<pcl::PointXYZ> (cloudFilter, single_color0, "oricloud");
        return true;
    }

    bool DetectorBaseScrew::getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters){
        static int solve_num=0;
        for (int i = 0; i < solve_num; ++i) {
            viewer->removePointCloud(std::to_string(i));
        }
        viewer->removePointCloud("oricloud");
        std::vector<pcl::PointIndices> clusterIndices;
        //利用欧式聚类分割出各个点云簇，去除距离远的点，分离桌面与螺丝
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud);
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
        ec.setInputCloud(cloud) ;
        ec.setClusterTolerance (0.01);                     // 设置近邻搜索的搜索半径为2cm
        ec.setMinClusterSize (100);                 //设置一个聚类需要的最少的点数目为100
        ec.setMaxClusterSize (2500000);               //设置一个聚类需要的最大点数目为25000
        ec.setSearchMethod (tree);                    //设置点云的搜索机制  ec.setInputCloud (cloud_filtered);
        ec.extract (clusterIndices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color0(cloud, 0, 0, 255);
       viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color0, "oricloud");

        solve_num=clusterIndices.size();
       // std::cout<<solve_num<<std::endl;
        for (int i = 0; i < clusterIndices.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr screwcluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (int j = 0; j < clusterIndices[i].indices.size(); ++j) {
                screwcluster->points.push_back(cloud->points[clusterIndices[i].indices[j]]);
            }
            screwcluster->width = screwcluster->points.size();
            screwcluster->height = 1;
            screwcluster->is_dense = true;
            clusters.push_back(screwcluster);

            int r = rand()*255;
            int g = rand()*255;
            int b = rand()*255;

            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_colorin(screwcluster, r, g, b);
            std::string str;
            str = "cloud" + std::to_string(r+g+b);
            //viewer->addPointCloud<pcl::PointXYZ> (screwcluster, single_colorin, str);
            viewer->addPointCloud<pcl::PointXYZ> (screwcluster, single_colorin, std::to_string(i));
        }

        return true;
    }
    bool DetectorBaseScrew::getScrewPose(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,int i) {
        poseViewer->removePointCloud("cloud"+std::to_string(i-1));
        poseViewer->removeShape("plane"+std::to_string(i-1));
        //poseViewer->removePolygonMesh("plane");
        std::vector<float> planeCoeff;
        int inlierSize = 0;

//        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr modelP(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
//        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelP);
//        ransac.setDistanceThreshold(8);
//        ransac.setMaxIterations(100);
//        ransac.computeModel();

        pcl::PointIndices::Ptr inliersIds(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> seg; //可选设置
        seg.setOptimizeCoefficients(true); //必须设置
        seg.setModelType(pcl::SACMODEL_PLANE); //设置模型类型，检测平面
        seg.setMethodType(pcl::SAC_RANSAC); //设置方法【聚类或随机样本一致性】
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);
        seg.segment(*inliersIds, *coefficients); //分割操作


      //  std::cout<<"coefficients = "<<*coefficients<<std::endl;

        //1.计算各Cluster所在平面参数，并转换法向量方向
        if (coefficients->values[2] < 0){
            coefficients->values[0] *= -1;
            coefficients->values[1] *= -1;
            coefficients->values[2] *= -1;
            coefficients->values[3] *= -1;
        }
        planeCoeff.clear();
        planeCoeff.push_back(coefficients->values[0]);
        planeCoeff.push_back(coefficients->values[1]);
        planeCoeff.push_back(coefficients->values[2]);
        planeCoeff.push_back(coefficients->values[3]);

        //2.判断法向量与Z方向夹角，认为法向与Z轴夹角小于15-20度（待测试）为正常螺丝
        Eigen::Vector3d planeNormal(0, 0, 0);//平面法向方向
        Eigen::Vector3d nz(0, 0, 1);
        planeNormal << planeCoeff[0], planeCoeff[1], planeCoeff[2];

//        float theta = acos(planeNormal.dot(nz) / planeNormal.stableNorm()) * 180.0 / CV_PI;
        float theta = acos(planeNormal.dot(nz) / planeNormal.norm()) * 180.0 / CV_PI;
        if (theta > 15) {
      //      std::cout<<theta<<std::endl;
            return false;
        }

      //  std::cout<<"planeNormal.norm() = "<< planeNormal.norm()<<std::endl;
      //  std::cout<<"planeNormal.stableNorm() = "<<planeNormal.stableNorm()<<std::endl;
     //   std::cout<<"planeNormal = "<< planeNormal<<std::endl;


        //3.旋转点云至水平，求内点BoundingBox，认为螺丝长宽比例大于0.85(待测试)，现采集螺丝点云，边缘部分点云基本保留完整
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPt(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < inliersIds->indices.size(); ++j) {
            inlierPt->points.push_back(cloud->points[inliersIds->indices[j]]);
        }
        //3.1旋转点云至水平
        Eigen::Vector3d cNormal;
        cNormal = planeNormal.cross(nz);

        Eigen::AngleAxisd rotationVector(acos(planeNormal.dot(nz)), cNormal);
        Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
        rotationMatrix = rotationVector.toRotationMatrix();

        Eigen::Matrix4d transferM;
        transferM.block(0, 0, 3, 3) = rotationMatrix;
        transferM.row(3) << 0, 0, 0, 1;
        transferM.col(3) << 0, 0, 0, 1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPoint(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transformedPoint, transferM);
//        std::cout<<"transferM"<<transferM<<std::endl;

        //3.2计算BoundingBox，判断
        pcl::PointXYZ minpt, maxpt;
        pcl::getMinMax3D(*cloud, minpt, maxpt);
//        std::cout<<"bdBox.position"<<bdBox.position<<std::endl;
      //  std::cout<<"x.length = "<<(maxpt.x - minpt.x)<<std::endl;
     //   std::cout<<"y.length = "<<(maxpt.y - minpt.y)<<std::endl;
     //   std::cout<<"z.length = "<<(maxpt.z - minpt.z)<<std::endl;
//        std::cout<<"rotationMatrix = "<<bdBox.rotationMatrix<<std::endl;

        float Length = std::min((maxpt.x - minpt.x),(maxpt.y - minpt.y));
        float Width = std::max((maxpt.x - minpt.x),(maxpt.y - minpt.y));
        float ratioLength2Width =  Length/Width;//确保ratio <= 1
      //  std::cout<<"ratio = "<<ratioLength2Width<<std::endl;
        if(ratioLength2Width < 0.85) return false;

        Eigen::Vector4d center;
        center << (maxpt.x - minpt.x)/2 + minpt.x, (maxpt.y - minpt.y)/2 + minpt.y, (maxpt.z - minpt.z)/2 + minpt.z, 1;
        Eigen::Vector4d tCenter;
        tCenter = transferM.inverse() * center;//螺丝上表面中心位置
        std::cout<<"tCenter = "<<tCenter<<std::endl;
        std::cout<<"------------------------"<<std::endl;
        std::cout<<"The Transfer Matrix is:"<<std::endl;
        std::cout<<transferM<<std::endl;
        std::cout<<"------------------------"<<std::endl;


////        cVision3D::BoundingBox bdBox;
//        cVision3D::toolKit::computeBoundingBox(transformedPoint,bdBox);
//        std::cout<<"bdBox.minPoint"<<bdBox.minPoint<<std::endl;
//        std::cout<<"bdBox.maxPoint"<<bdBox.maxPoint<<std::endl;
//        std::cout<<"bdBox.position"<<bdBox.position<<std::endl;
//        std::cout<<"x.length = "<<(bdBox.maxPoint.x - bdBox.minPoint.x)<<std::endl;
//        std::cout<<"y.length = "<<(bdBox.maxPoint.y - bdBox.minPoint.y)<<std::endl;
//        std::cout<<"z.length = "<<(bdBox.maxPoint.z - bdBox.minPoint.z)<<std::endl;
//        std::cout<<"rotationMatrix = "<<bdBox.rotationMatrix<<std::endl;
//
//        float Length = std::min((bdBox.maxPoint.x - bdBox.minPoint.x),(bdBox.maxPoint.y - bdBox.minPoint.y));
//        float Width = std::max((bdBox.maxPoint.x - bdBox.minPoint.x),(bdBox.maxPoint.y - bdBox.minPoint.y));
//        float ratioLength2Width =  Length/Width;//确保ratio <= 1
//        std::cout<<"ratio = "<<ratioLength2Width<<std::endl;
//        if(ratioLength2Width < 0.85) return false;
//
//        Eigen::Vector4d center;
//        center << bdBox.position.x, bdBox.position.y, bdBox.position.z + bdBox.minPoint.z, 1;
//        Eigen::Vector4d tCenter;
//        tCenter = transferM.inverse() * center;//螺丝上表面中心位置
       // std::cout<<"tCenter = "<<tCenter<<std::endl;


        /*Eigen::Matrix3d rotationMatrix111,rotationMatrix222;
        rotationMatrix222 << bdBox.rotationMatrix(0,0),bdBox.rotationMatrix(0,1),bdBox.rotationMatrix(0,2)
                                            ,bdBox.rotationMatrix(1,0),bdBox.rotationMatrix(1,1),bdBox.rotationMatrix(1,2)
                                            ,bdBox.rotationMatrix(2,0),bdBox.rotationMatrix(2,1),bdBox.rotationMatrix(2,2);
        rotationMatrix111 =rotationMatrix*rotationMatrix222;
        std::cout<<"rotationMatrix222 = "<<rotationMatrix111<<std::endl;*/

        //4.判断螺丝点云的周围点云，若周围点云平均比螺丝点云低，认为是螺丝




        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
        poseViewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "cloud"+std::to_string(i));

//        planeCoeff.push_back(coefficients->values[0]);
//        planeCoeff.push_back(coefficients->values[1]);
//        planeCoeff.push_back(coefficients->values[2]);
//        planeCoeff.push_back(coefficients->values[3]);

        poseViewer->addPlane (*coefficients,"plane"+std::to_string(i));




//        inlierSize = inliersIds->indices.size();
//
//        float ratio = inlierSize * 1.0 / cloud->points.size();
//        std::cout << "ratio = " << ratio << std::endl;
//        std::cout << "inlierSize = " << inlierSize << std::endl;
//        std::cout << "cloud.size() = " << cloud->points.size() << std::endl;
//
//        if (cloud->size() == 63) {
//            if (ratio > 0.85) {
//                for (int i = 0; i < planeCoeff.size(); ++i) {
//                    std::cout << "circleCoeff.Data = " << planeCoeff[i] << std::endl;
//                }
//                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
//                for (size_t i = 0; i < inliersIds->indices.size(); ++i) {
//                    cloud1->points.push_back(cloud->points[inliersIds->indices[i]]);
//                }
//
//                boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(
//                        new pcl::visualization::PCLVisualizer("getScrewPose"));
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
//                viewer2->addPointCloud<pcl::PointXYZ>(cloud, single_color, "cloud");
//
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud1, 0, 255, 0);
//                viewer2->addPointCloud<pcl::PointXYZ>(cloud1, single_color2, "cloud11");
//
//
//            }
//        }


//        float radiusLimitsDown,radiusLimitsUp;

//        cVision3D::toolKit::extractCylinder(cloud,20,20,1,10,circleCoeff,inlierSize);


//        pcl::ModelCoefficients::Ptr cylinderCoeff0(new pcl::ModelCoefficients);
//        pcl::PointIndices::Ptr inliersIds(new pcl::PointIndices);
//        //为圆柱体分割创建分割对象，并设置所有参数
//        pcl::SACSegmentation<pcl::PointXYZ> seg; //可选设置
//        seg.setOptimizeCoefficients (true);         //设置对估计的模型系数需要进行优化
////        seg.setModelType (pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱型
//        seg.setModelType (pcl::SACMODEL_CIRCLE3D); //设置分割模型为圆柱型
//        seg.setMethodType (pcl::SAC_RANSAC);       //设置采用RANSAC作为算法的参数估计方法
//        seg.setMaxIterations (1000);              //设置迭代的最大次数10000
//        seg.setDistanceThreshold (8);           //设置内点到模型的距离允许最大值
//        seg.setRadiusLimits (1, 6);              //设置估计出的圆柱模型的半径范围
//        seg.setInputCloud (cloud);
//        seg.segment (*inliersIds, *cylinderCoeff0);
        //平面分割求解平面法向




//        cVision3D::toolKit::extractPlane(cloud,150,50,circleCoeff,inlierSize);
//        cVision3D::BoundingBox bdb;
//        cVision3D::toolKit::computeBoundingBox(cloud,bdb);
//        std::cout<<"boundingbox = "<<inlierSize<<std::endl;
//        inlierSize = inliersIds->indices.size();
//        float ratio = inlierSize*1.0/cloud->points.size();
//        std::cout<<"ratio = "<<ratio<<std::endl;
//        std::cout<<"inlierSize = "<<inlierSize<<std::endl;
//        std::cout<<"cloud.size() = "<<cloud->points.size()<<std::endl;


//        if (ratio > 0.85){
//            for (int i = 0; i < cylinderCoeff0->values.size(); ++i) {
////                std::cout<<"circleCoeff.Data = "<<circleCoeff[i]<<std::endl;
//                std::cout<<"cylinderCoeff0->value = "<<cylinderCoeff0->values[i]<<std::endl;
//
//            }
//            pcl::ModelCoefficients::Ptr cylinderCoeff1(new pcl::ModelCoefficients);
//            pcl::PointIndices::Ptr inliersIds1(new pcl::PointIndices);
//            pcl::SACSegmentation<pcl::PointXYZ> seg1; //可选设置
//            seg1.setOptimizeCoefficients (true);         //设置对估计的模型系数需要进行优化
//            seg1.setModelType (pcl::SACMODEL_CYLINDER); //设置分割模型为圆柱型
////        seg.setModelType (pcl::SACMODEL_CIRCLE3D); //设置分割模型为圆柱型
//            seg1.setMethodType (pcl::SAC_RANSAC);       //设置采用RANSAC作为算法的参数估计方法
//            seg1.setMaxIterations (100);              //设置迭代的最大次数10000
//            seg1.setDistanceThreshold (8);           //设置内点到模型的距离允许最大值
//            seg1.setRadiusLimits (0, 6);              //设置估计出的圆柱模型的半径范围
//            seg1.setInputCloud (cloud);
//            seg1.segment (*inliersIds1, *cylinderCoeff1);








//
//            double nx = cylinderCoeff0->values[4], ny = cylinderCoeff0->values[5], nz = cylinderCoeff0->values[6];
//            double cx = cylinderCoeff0->values[0],cy = cylinderCoeff0->values[1],cz = cylinderCoeff0->values[2];
//            double r = cylinderCoeff0->values[3];
//
//            double ux = ny, uy = -nx, uz = 0;
//            double vx = nx*nz,
//                    vy = ny*nz,
//                    vz = -nx*nx - ny*ny;
//
//            double sqrtU = sqrt(ux*ux + uy*uy + uz*uz);
//            double sqrtV = sqrt(vx*vx + vy*vy + vz*vz);
//
//            double ux_ = (1 / sqrtU)*ux;
//            double uy_ = (1 / sqrtU)*uy;
//            double uz_ = (1 / sqrtU)*uz;
//
//            double vx_ = (1 / sqrtV)*vx;
//            double vy_ = (1 / sqrtV)*vy;
//            double vz_ = (1 / sqrtV)*vz;
//
//            double xi, yi, zi;
//            double t = 0;
//            double angle = (t / 180.0)*CV_PI;
//            std::vector<double> x, y, z;
//
//            while (t < 360.0)
//            {
//                xi = cx + r*(ux_*cos(angle) + vx_*sin(angle));
//                yi = cy + r*(uy_*cos(angle) + vy_*sin(angle));
//                zi = cz + r*(uz_*cos(angle) + vz_*sin(angle));
//                x.push_back(xi);
//                y.push_back(yi);
//                z.push_back(zi);
//
//                t = t + 1;
//                angle = (t / 180.0)*CV_PI;
//            }
//            pcl::PointCloud<pcl::PointXYZ>::Ptr theroyCirclePoints(new pcl::PointCloud<pcl::PointXYZ>);
//
//            //定义cloudPoints大小,无序点云
//            theroyCirclePoints->resize(x.size());
//            for (int i = 0; i < x.size(); i++){
//                //将三维坐标赋值给PCL点云坐标
//                (*theroyCirclePoints)[i].x = x[i];
//                (*theroyCirclePoints)[i].y = y[i];
//                (*theroyCirclePoints)[i].z = z[i];
//            }






//            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("getScrewPose"));
////            viewer2->addCylinder(*cylinderCoeff1);
////            viewer2->addCircle(*cylinderCoeff);
//            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(theroyCirclePoints, 255, 0, 0);
//            viewer2->addPointCloud<pcl::PointXYZ> (theroyCirclePoints, single_color3, "targetcloud");
//
//            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud, 0, 255, 0);
//            viewer2->addPointCloud<pcl::PointXYZ> (cloud, single_color2, "targetcloud11");



//            cylinderCoeff1->values[0] = cylinderCoeff0->values[0];
//            cylinderCoeff1->values[1] = cylinderCoeff0->values[1];
//            cylinderCoeff1->values[2] = cylinderCoeff0->values[2];
//
//            cylinderCoeff1->values[3] = cylinderCoeff0->values[4];
//            cylinderCoeff1->values[4] = cylinderCoeff0->values[5];
//            cylinderCoeff1->values[5] = cylinderCoeff0->values[6];
//
//            cylinderCoeff1->values[6] = cylinderCoeff0->values[3];



//            cylinderCoeff = cylinderCoeff0;

//            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("getScrewPose"));
//            viewer2->addCylinder(*cylinderCoeff1);
////            viewer2->addCircle(*cylinderCoeff);
//            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud, 255, 255, 255);
//            viewer2->addPointCloud<pcl::PointXYZ> (cloud, single_color3, "targetcloud");
//        }
//        else return false;

        return true;
    }


   bool DetectorBaseScrew::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud ){

        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (oricloud);
        sor.setMeanK (200);
        sor.setStddevMulThresh (0.1);
        sor.filter (*oricloud);
       return  true;
       // cout<<sor_filtered->size()<<"  --final"<<endl;
//        bool res = cVision3D::toolKit::downSample(oricloud, _param.downSampeX,  _param.downSampeY,  _param.downSampeZ , oricloud);
        //std::cout<<"toolKit downsample"<<oricloud->points.size()<<std::endl;

    }

    bool DetectorBaseScrew::reconstructPolygonMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr &oricloud , pcl::PolygonMesh &triangles){

        std::cout<<"oricloud.size()=" <<oricloud->points.size()<<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        //下采样

//        // 创建滤波对象
//        pcl::VoxelGrid<pcl::PointXYZ> filter;
//        filter.setInputCloud(oricloud);
//        // 设置体素栅格的大小为 1x1x1cm
//        filter.setLeafSize(_param.downSampeX,  _param.downSampeY,  _param.downSampeZ );
//        filter.filter(*cloud);
//        std::cout<<"PCL downsample"<<cloud->points.size()<<std::endl;

        std::cout<<"normals start"<<std::endl;
        //估计法线
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;      //法线估计对象
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);   //存储估计的法线
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);  //定义kd树指针

//        cVision3D::toolKit::computeNormal(cloud,_param.normalRadius,normals);
//        std::cout<<"def "<<std::endl;
//        tree->setInputCloud (cloud);   //用cloud构建tree对象

//        std::cout<<"tree->setInputCloud "<<std::endl;
        n.setInputCloud (cloud);
//        std::cout<<"n.setInputCloud "<<std::endl;
        n.setSearchMethod (tree);
        n.setKSearch (_param.normalRadius);          //设置k搜索的k值为50

        n.compute (*normals);       //估计法线存储到其中

        std::cout<<"normals.size()="<<normals->points.size()<<std::endl;

        //组合cloud与normal
        //* normals should not contain the point normals + surface curvatures
        // Concatenate the XYZ and normal fields*   //连接字段
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

        // 定义搜索树对象
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud (cloud_with_normals);   //点云构建搜索树

        std::cout<<"reconstruct start"<<std::endl;
        #ifdef RECONSTRUCT_GREEY  //贪婪三角化
        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   //定义三角化对象
        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius (_param.greedyConnectPointDisMax);  //设置连接点之间的最大距离，（即是三角形最大边长）
        // 设置各参数值
        gp3.setMu (_param.greedySearchDisMax);  //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
        gp3.setMaximumNearestNeighbors (_param.greedyNearestNeighborsNum);    //设置样本点可搜索的邻域个数
        gp3.setMaximumSurfaceAngle(_param.greedySurfaceAngle*M_PI /180); // 设置某点法线方向偏离样本点法线的最大角度45
        gp3.setMinimumAngle(_param.greedyMinInteriorAngle*M_PI /180); // 设置三角化后得到的三角形内角的最小的角度为10
        gp3.setMaximumAngle(_param.greedyMaxInteriorAngle*M_PI /180); // 设置三角化后得到的三角形内角的最大角度为120
        gp3.setNormalConsistency(_param.greedyNormalConsistency);  //设置该参数保证法线朝向一致
        // Get result
        gp3.setInputCloud (cloud_with_normals);     //设置输入点云为有向点云
        gp3.setSearchMethod (tree2);   //设置搜索方式
        gp3.reconstruct (triangles);  //重建提取三角化
        // 附加顶点信息
        std::vector<int> parts = gp3.getPartIDs();
        std::vector<int> states = gp3.getPointStates();
        #endif
        #ifdef RECONSTRUCT_POISSON  //泊松重建
        //创建Poisson对象，并设置参数。泊松重建。
        pcl::Poisson<pcl::PointNormal> pn ;
        pn.setConfidence(false);//是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
        pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
        pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
        pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度。
        pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时，设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加。
        pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果） pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑。
        pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率
        pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度。
        //pn.setIndices();
        // 设置搜索方法和输入点云。
        pn.setSearchMethod(tree2);
        pn.setInputCloud(cloud_with_normals); //创建多变形网格，用于存储结果。
        pn.performReconstruction(triangles);
        #endif
        #ifdef RECONSTRUCT_MARCHINGCUBES  //移动立方体重建//效果不好
        //初始化MarchingCubes对象，并设置参数。移动立方体重建。
        pcl::MarchingCubes<pcl::PointNormal> *mc;
        mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
        /*
         if (hoppe_or_rbf == 0)
           mc = new pcl::MarchingCubesHoppe<pcl::PointNormal> ();
         else
         {
           mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();
           (reinterpret_cast<pcl::MarchingCubesRBF<pcl::PointNormal>*> (mc))->setOffSurfaceDisplacement (off_surface_displacement);
         }
       */
        mc->setIsoLevel (0.0f);
        mc->setGridResolution (50, 50, 50);
        mc->setPercentageExtendGrid (0.0f);
        mc->setInputCloud (cloud_with_normals);
        mc->reconstruct (triangles);
        #endif
    }

    ///////////////////////////////////////////////
    bool DetectorBaseScrew::setParam(const std::string &fileFullName) {
        detectParam param;
        if (loadParamFile(fileFullName, param)) {
            initParam(param);
            return true;
        } else {
            return false;
        }
    }

    void DetectorBaseScrew::initParam(const detectParam &param) {
        this->_param.roiX = param.roiX;
        this->_param.roiY = param.roiY;
        this->_param.roiW = param.roiW;
        this->_param.roiH = param.roiH;

        this->_param.ptMinZ = param.ptMinZ;
        this->_param.ptMaxZ = param.ptMaxZ;
        this->_param.ptMinX = param.ptMinX;
        this->_param.ptMaxX = param.ptMaxX;
        this->_param.ptMinY = param.ptMinY;
        this->_param.ptMaxY = param.ptMaxY;

        this->_param.downSampeX = param.downSampeX;
        this->_param.downSampeY = param.downSampeY;
        this->_param.downSampeZ = param.downSampeZ;

        this->_param.meanK = param.meanK;
        this->_param.mulStd = param.mulStd;

        this->_param.filterRadius = param.filterRadius;
        this->_param.filterMinNum = param.filterMinNum;

        this->_param.segdistanceThreshold = param.segdistanceThreshold;
        this->_param.segScrewMaxSize = param.segScrewMaxSize;
        this->_param.segBkgroundMaxSize = param.segBkgroundMaxSize;



        this->_param.normalRadius = param.normalRadius;


        //重建参数
        //贪婪三角化参数
        this->_param.greedyConnectPointDisMax = param.greedyConnectPointDisMax ;
        this->_param.greedySearchDisMax = param.greedySearchDisMax ;
        this->_param.greedyNearestNeighborsNum = param.greedyNearestNeighborsNum;
        this->_param.greedySurfaceAngle = param.greedySurfaceAngle;
        this->_param.greedyMinInteriorAngle = param.greedyMinInteriorAngle;
        this->_param.greedyMaxInteriorAngle = param.greedyMaxInteriorAngle;
        this->_param.greedyNormalConsistency = param.greedyNormalConsistency;
        //泊松表面重建Poisson
        this->_param.poissonConfidence = param.poissonConfidence;
        this->_param.poissonDegree = param.poissonDegree;
        this->_param.poissonDepth = param.poissonDepth;
        this->_param.poissonIsoDivide = param.poissonIsoDivide;
        this->_param.poissonManifold = param.poissonManifold;
        this->_param.poissonOutputPolygons = param.poissonOutputPolygons;
//        this->_param.poissonSamplesPerNode = param.poissonSamplesPerNode;
        this->_param.poissonScale = param.poissonScale;
        this->_param.poissonSolverDivide = param.poissonSolverDivide;



        this->_param.segCurvatureThreshold = param.segCurvatureThreshold;
        this->_param.segMaxSize = param.segMaxSize;
        this->_param.segMinSize = param.segMinSize;
        this->_param.segNeighborNumber = param.segNeighborNumber;
        this->_param.segResidualThreshold = param.segResidualThreshold;
        this->_param.segThetaThresholdInDegree = param.segThetaThresholdInDegree;
        this->_param.segMaxDis = param.segMaxDis;



        this->_param.distanceThreshold = param.distanceThreshold;
        this->_param.maxIterations = param.maxIterations;
        this->_param.planeInliersRatio = param.planeInliersRatio;

        this->_param.maxAngleInDegreeWithZ = param.maxAngleInDegreeWithZ;
        this->_param.widthThreshold = param.widthThreshold;
        this->_param.heightThreshold = param.heightThreshold;
    }

    bool DetectorBaseScrew::loadParamFile(const std::string &fileFullName, cVisionDL::detectParam &param) {
        cv::FileStorage fs(fileFullName, cv::FileStorage::READ);
        if (fs.isOpened()) {
            std::string s;
            param.roiX = fs["roiX"];
            param.roiY = fs["roiY"];
            param.roiW = fs["roiW"];
            param.roiH = fs["roiH"];
            param.ptMinZ = fs["ptMinZ"];
            param.ptMaxZ = fs["ptMaxZ"];
            param.ptMinX = fs["ptMinX"];
            param.ptMaxX = fs["ptMaxX"];
            param.ptMinY = fs["ptMinY"];
            param.ptMaxY = fs["ptMaxY"];
            param.downSampeX = fs["downSampeX"];
            param.downSampeY = fs["downSampeY"];
            param.downSampeZ = fs["downSampeZ"];
            param.meanK = fs["meanK"];
            param.mulStd = fs["mulStd"];
            param.segdistanceThreshold = fs["segdistanceThreshold"];
            param.segScrewMaxSize = fs["segScrewMaxSize"];
            param.segBkgroundMaxSize = fs["segBkgroundMaxSize"];


            //三维重建
            param.greedyConnectPointDisMax = fs["greedyConnectPointDisMax"];//贪婪三角化连接点之间最大距离
            param.greedySearchDisMax = fs["greedySearchDisMax"];//设置连接点最大距离
            param.greedyNearestNeighborsNum = fs["greedyNearestNeighborsNum"];//设置可搜索邻域个数
            param.greedySurfaceAngle =fs["greedySurfaceAngle"];//设置法线方向偏离样本的最大角度
            param.greedyMinInteriorAngle = fs["greedyMinInteriorAngle"];//设置三角化得到的内角最小角度
            param.greedyMaxInteriorAngle = fs["greedyMaxInteriorAngle"];//设置三角化得到的内角最小角度
            s = (std::string)fs["greedyNormalConsistency"];//设置保证法线朝向一致
            std::istringstream(s)>>param.greedyNormalConsistency;

            //泊松表面重建Poisson
            s = (std::string)fs["poissonConfidence"];
            std::istringstream(s)>>param.poissonConfidence;
            param.poissonDegree = fs["poissonDegree"];
            param.poissonDepth = fs["poissonDepth"];
            param.poissonIsoDivide = fs["poissonIsoDivide"];
            s = (std::string)fs["poissonManifold"];
            std::istringstream(s)>>param.poissonManifold;
            s = (std::string)fs["poissonOutputPolygons"];
            std::istringstream(s)>>param.poissonOutputPolygons;
//            param.poissonSamplesPerNode = fs["poissonSamplesPerNode"];
            param.poissonScale = fs["poissonScale"];
            param.poissonSolverDivide = fs["poissonSolverDivide"];






                                                                                                                                                                                                 param.filterRadius = fs["filterRadius"];
            param.filterMinNum = fs["filterMinNum"];
            param.normalRadius = fs["normalRadius"];
            param.segMinSize = fs["segMinSize"];
            param.segMaxSize = fs["segMaxSize"];
            param.segResidualThreshold = fs["segResidualThreshold"];
            param.segCurvatureThreshold = fs["segCurvatureThreshold"];
            param.segThetaThresholdInDegree = fs["segThetaThresholdInDegree"];
            param.segNeighborNumber = fs["segNeighborNumber"];
            param.segMaxDis = fs["segMaxDis"];
            param.distanceThreshold = fs["distanceThreshold"];
            param.maxIterations = fs["maxIterations"];
            param.planeInliersRatio = fs["planeInliersRatio"];
            param.maxAngleInDegreeWithZ = fs["maxAngleInDegreeWithZ"];
            param.widthThreshold = fs["widthThreshold"];
            param.heightThreshold = fs["heightThreshold"];
            return true;
        } else {
            return false;
        }
    }
    void DetectorBaseScrew::initRealsense(){
//        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 45);
//        cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 45);
//        cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 45);
        pipe.start(cfg);
    }
    void DetectorBaseScrew::generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, bool bSave) {

        cloud->clear();
        auto frames = pipe.wait_for_frames(500);
        std::cout<<frames.size()<<std::endl;
        auto depth = frames.get_depth_frame();
        rs2::pointcloud pc;
        rs2::points points;
        points = pc.calculate(depth);
        auto sp = points.get_profile().as<rs2::video_stream_profile>();
        cloud->width = sp.width();
        cloud->height = sp.height();
        cloud->is_dense = false;
        cloud->points.resize(points.size());
        auto ptr = points.get_vertices();
        for (auto& p : cloud->points)
        {
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
            ptr++;
        }
        if (bSave) {
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("totalCloud.pcd", *cloud);
        }
    }
    void DetectorBaseScrew::generatePointCloud(const std::string &path_xy, const std::string &path_pointcloud, cv::Mat &pointCloud) {
        std::ifstream coordinate_xys;
        std::vector<cv::Point> coordinate_xy;
        cv::Point coordinate_temp;
        int coordinate_temp_x = 0;
        int coordinate_temp_y = 0;
        coordinate_xys.open(path_xy);
        assert(coordinate_xys.is_open());
        while (coordinate_xys >> coordinate_temp_x >> coordinate_temp_y) {
            coordinate_temp.x = coordinate_temp_x;
            coordinate_temp.y = coordinate_temp_y;
            coordinate_xy.push_back(coordinate_temp);
        }
//        std::cout << coordinate_xy.size() << std::endl;

        std::ifstream pointcloud_xyzs;
        std::vector<cv::Point3f> pointcloud_xyz;
        cv::Point3f pointcloud_temp;
        float pointcloud_temp_x = 0.0;
        float pointcloud_temp_y = 0.0;
        float pointcloud_temp_z = 0.0;
        pointcloud_xyzs.open(path_pointcloud);
        assert(pointcloud_xyzs.is_open());
        while (pointcloud_xyzs >> pointcloud_temp_x >> pointcloud_temp_y >> pointcloud_temp_z) {
            pointcloud_temp.x = pointcloud_temp_x;
            pointcloud_temp.y = pointcloud_temp_y;
            pointcloud_temp.z = pointcloud_temp_z;
            pointcloud_xyz.push_back(pointcloud_temp);
        }
        std::cout << "original num of Points11: " << pointcloud_xyz.size() << std::endl;
//        pointCloud.at<cv::Vec3f>(2047,2447)[0] = 100;
        int k = 0;
        for (auto p : coordinate_xy) {
            pointCloud.at<cv::Vec3f>(2048 - p.x, p.y)[0] = pointcloud_xyz[k].x;
            pointCloud.at<cv::Vec3f>(2048 - p.x, p.y)[1] = pointcloud_xyz[k].y;
            pointCloud.at<cv::Vec3f>(2048 - p.x, p.y)[2] = pointcloud_xyz[k].z;
            ++k;
        }
        std::cout << "original num of Points: " << pointcloud_xyz.size() << std::endl;
    }

    void DetectorBaseScrew::matToPCD(const cv::Mat &pointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool bSave) {
        //转化成点云格式
        cloud->clear();
        for (unsigned int i = 0; i < pointCloud.rows; i++) {
            for (unsigned int j = 0; j < pointCloud.cols; j++) {
                if (pointCloud.at<cv::Vec3f>(i, j)[2] < 1) {
                    continue;
                } else {
                    pcl::PointXYZ point;
                    point.x = pointCloud.at<cv::Vec3f>(i, j)[0];
                    point.y = pointCloud.at<cv::Vec3f>(i, j)[1];
                    point.z = pointCloud.at<cv::Vec3f>(i, j)[2];
                    cloud->points.push_back(point);
                }
            }
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        std::cout << "cloud size " << cloud->points.size() << std::endl;
        if (bSave) {
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("totalCloud.pcd", *cloud);
        }
    }

    bool DetectorBaseScrew::cropROI(const cv::Mat &imgOrg, const cv::Mat &pcOrg, cv::Mat &imgROI, cv::Mat &pcROI) {
        imgROI = imgOrg(cv::Rect(_param.roiX, _param.roiY, _param.roiW, _param.roiH)).clone();
        pcROI = pcOrg(cv::Rect(_param.roiX, _param.roiY, _param.roiW, _param.roiH)).clone();
#ifdef DEBUG_MEDICINE_BOX
        cv::namedWindow("roiImg", CV_WINDOW_NORMAL);
        cv::imshow("roiImg", imgROI);
        cv::namedWindow("roiPt", CV_WINDOW_NORMAL);
        cv::imshow("roiPt", pcROI);
        cv::waitKey(1);
#endif
        return true;
    }
}