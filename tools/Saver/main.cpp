#include <librealsense2/rs.hpp>
#include<iostream>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <sstream>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <algorithm>
#include <tuple>
#include <pcl/point_types.h>

using namespace std;

class Saver{
public:
    Saver(){
        pipe.start(cfg);
        frames = pipe.wait_for_frames(10000);
        // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        //auto color = frames.get_infrared_frame();
        //pc.map_to(color);
        depth = frames.get_depth_frame();
        color = frames.get_color_frame();
        pc.map_to(color);
        points = pc.calculate(depth);
    }
    void initRealsense(){
//        cfg.enable_stream(RS2_STREAM_COLOR, 1, 480, RS2_FORMAT_BGR8, 30);
//        cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
//        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);


        // render_frames[color.get_profile().unique_id()] = colorizer.process(color);
        // app.show(render_frames);
        // Generate the pointcloud and texture mappings


    }
    std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
    {
        // Get Width and Height coordinates of texture
        int width  = texture.get_width();  // Frame width in pixels
        int height = texture.get_height(); // Frame height in pixels

        // Normals to Texture Coordinates conversion
        int x_value = std::min(std::max(int(Texture_XY.u * width  + .5f), 0), width - 1);
        int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

        int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
        int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
        int Text_Index =  (bytes + strides);

        const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

        // RGB components to save in tuple
        int NT1 = New_Texture[Text_Index];
        int NT2 = New_Texture[Text_Index + 1];
        int NT3 = New_Texture[Text_Index + 2];

        return std::tuple<int, int, int>(NT1, NT2, NT3);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_PCLXYZRGB(){

        // Object Declaration (Point Cloud)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
        std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

        //================================
        // PCL Cloud Object Configuration
        //================================
        // Convert data captured from Realsense camera to Point Cloud
        auto sp = points.get_profile().as<rs2::video_stream_profile>();

        cloud->width  = static_cast<uint32_t>( sp.width()  );
        cloud->height = static_cast<uint32_t>( sp.height() );
        cloud->is_dense = false;
        cloud->points.resize( points.size() );

        auto Texture_Coord = points.get_texture_coordinates();
        auto Vertex = points.get_vertices();

        // Iterating through all points and setting XYZ coordinates
        // and RGB values
        for (int i = 0; i < points.size(); i++)
        {
            //===================================
            // Mapping Depth Coordinates
            // - Depth data stored as XYZ values
            //===================================
            cloud->points[i].x = Vertex[i].x;
            cloud->points[i].y = Vertex[i].y;
            cloud->points[i].z = Vertex[i].z;

            // Obtain color texture for specific point
            RGB_Color = RGB_Texture(color, Texture_Coord[i]);

            // Mapping Color (BGR due to Camera Model)
            cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
            cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
            cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>

        }

        return cloud; // PCL RGB Point Cloud generated
    }
    void generatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, bool bSave){
        std::cout<<"start generating "<<std::endl;
//        cloud->clear();
//       // viewer->removePointCloud("cloud");
//        auto frames = pipe.wait_for_frames(5000);
//        auto depth = frames.get_depth_frame();
//        rs2::pointcloud pc;
//        rs2::points points;
//        points = pc.calculate(depth);
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
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.5);
        pass.filter (*cloud);
        //viewer->addPointCloud(cloud,"cloud");
        //cout<<cloud->size()<<endl;
        if (bSave) {
            pcl::PCDWriter writer;
            writer.write<pcl::PointXYZ>("totalCloud.pcd", *cloud);
        }
        std::cout<<"end generating "<<std::endl;
    }

    void initViewer(){
        //viewer=boost::make_shared<pcl::visualization::PCLVisualizer>("viewer");
        oviewer=boost::make_shared<pcl::visualization::PCLVisualizer>("Orignal viewer");
    }

   // pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PCLVisualizer::Ptr oviewer;
private:
    rs2::pipeline pipe;
    rs2::points points;
    rs2::config cfg;
    rs2::pointcloud pc;
    rs2::frame color;
    rs2::frameset frames;
    rs2::frame depth;

};

int main(int argc,char **argv){
    Saver ac;
    ac.initRealsense();

    if(argc>1){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud=ac.points_to_PCLXYZRGB();
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.5);
        pass.filter (*cloud);
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZRGB>("ColorCloud.pcd", *cloud);
    }
    else{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ac.generatePointCloud(cloud, true);
    }






}