// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "view_helper.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include "rsconversion.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>

using namespace std;
static bool process=true;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getKeySym () == "r" && event.keyDown ())
    {
        std::cout << "r was pressed => start processing" << std::endl;
        process= true;

    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    return (viewer);
}

int main(int argc, char * argv[]) try {
    //signal( SIGINT, handler );
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    rs2::context                ctx;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    pcl::visualization::CloudViewer viewer ("test");
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe(ctx);
    rs2::colorizer colorizer;
    rs2::config cfg;

//    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 20);
//    cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 20);
//    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 20);
    // Start streaming with default recommended configuration
    //pipe.start(cfg);
    window app(480, 480,"RGB");
    // We'll keep track for the last frame of each stream available to make the presentation persistent
    std::map<int, rs2::frame> render_frames;


    pipe.start(cfg);

   // boost::shared_ptr<pcl::visualization::PCLVisualizer> acviewer=interactionCustomizationVis();
    while (!viewer.wasStopped()&app) {
        auto frames = pipe.wait_for_frames(10000);
       // boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        render_frames[color.get_profile().unique_id()] = colorizer.process(color);
       // render_frames.insert(std::pair<int,rs2::frame>(0,color));
        app.show(render_frames);
        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto pcl_points = points_to_PCLXYZ(points);
       // boost::this_thread::sleep (boost::posix_time::microseconds (10000));




        pcl::PointCloud<pcl::PointXYZ>::Ptr pass_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_points);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0,3);
        pass.filter(*pass_filtered);
        cout<<pass_filtered->size()<<" --first"<<endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud (pass_filtered);
        vox.setLeafSize (0.01f, 0.01f, 0.01f);
        vox.filter (*voxel_filtered);


        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (voxel_filtered);
        sor.setMeanK (5);
        sor.setStddevMulThresh (0.1);
        sor.filter (*sor_filtered);
        cout<<sor_filtered->size()<<"  --final"<<endl;

        viewer.showCloud(sor_filtered);



        // Present all the collected frames with openGl mosaic




    }
//       // gl_viewer(1280,640,"test",layers);


        return EXIT_SUCCESS;

}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
