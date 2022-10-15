// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <stdint.h>
#include <unordered_map>

#include <python2.7/Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

namespace std {
  template <>
  class hash< cv::Point >{
  public :
    size_t operator()(const cv::Point &pixel_cloud ) const
    {
      return hash<std::string>()( std::to_string(pixel_cloud.x) + "|" + std::to_string(pixel_cloud.y) );
    }
  };
};

void init_numpy()
{
    import_array();
}

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

std::vector<std::vector<double>> read_calib(const std::string calib_path)
{
    std::ifstream calib_file(calib_path,std::ifstream::in);
    std::string s;
    std::string line;
    std::vector<double> calib_matrix;
    std::vector<std::vector<double>> calibs;
    while(std::getline(calib_file, line))
    {
    std::stringstream calib_stream(line);
    std::getline(calib_stream, s, ' ');
    for (std::size_t i = 0; i < 12; ++i)
    {
        std::getline(calib_stream, s, ' ');
        calib_matrix.push_back(stod(s));
    }
    calibs.push_back(calib_matrix);
    calib_matrix.clear();
    }
    return calibs;
}










int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_helper");

    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, output_bag_file,lidar_frame_id;
    n.getParam("lidar_frame_id", lidar_frame_id);
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';

    bool to_bag,publish_dense_depth;
    n.getParam("to_bag", to_bag);
    n.getParam("publish_dense_depth", publish_dense_depth);

    if (to_bag)
    {
        n.getParam("output_bag_file", output_bag_file);
        output_bag_file += std::string("kitti_")+sequence_number+std::string("_dense.bag");
        std::cout<<output_bag_file<<std::endl;
    }

    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/points_raw", 100);
    ros::Publisher pub_camera_info = n.advertise<sensor_msgs::CameraInfo>("/camera_info", 100);
    ros::Publisher pub_fuse_cloud = n.advertise<sensor_msgs::PointCloud2>("/points_fuse", 100);
    ros::Publisher pub_dense_cloud_rgb = n.advertise<sensor_msgs::PointCloud2>("/points_dense", 100);
    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);

    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/camera_init";
    odomGT.child_frame_id = "/aft_mapped";
    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/camera_init";

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 100);
    image_transport::Publisher pub_image_depth = it.advertise("/image_depth", 100);
    image_transport::Publisher pub_image_depth_dense = it.advertise("/image_depth_dense", 100);

    std::string ground_truth_path = "poses/" + sequence_number + ".txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);

    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    std::string calib_path = dataset_folder+"sequences/" + sequence_number + "/calib.txt";
    std::vector<std::vector<double>> calibs=read_calib(calib_path);

    Eigen::Matrix<double,3,4> P2;
    P2 <<calibs[2][0],calibs[2][1],calibs[2][2],calibs[2][3],
    calibs[2][4],calibs[2][5],calibs[2][6],calibs[2][7],
    calibs[2][8],calibs[2][9],calibs[2][10],calibs[2][11];

    Eigen::Matrix<double,4,4> T2;
    T2 <<1,0,0,P2(0,3)/P2(0,0),
                0,1,0,0,
                0,0,1,0,
                0,0,0,1;
    
    Eigen::Matrix<double,3,4> P3;
    P3 <<calibs[3][0],calibs[3][1],calibs[3][2],calibs[3][3],
    calibs[3][4],calibs[3][5],calibs[3][6],calibs[3][7],
    calibs[3][8],calibs[3][9],calibs[3][10],calibs[3][11];


    Eigen::Matrix<double,4,4> T3;
    T3 <<1,0,0,P3(0,3)/P3(0,0),
                0,1,0,0,
                0,0,1,0,
                0,0,0,1;

    Eigen::Matrix<double,4,4> Tr;
    Tr <<calibs[4][0],calibs[4][1],calibs[4][2],calibs[4][3],
    calibs[4][4],calibs[4][5],calibs[4][6],calibs[4][7],
    calibs[4][8],calibs[4][9],calibs[4][10],calibs[4][11],
    0,0,0,1;

    Eigen::Matrix<double,4,4> T_cam2_velo=T2*Tr;
    // Eigen::Matrix<double,4,4> T_cam3_velo=T3*Tr;

    Eigen::Matrix<double,3,4> project_matrix;
    project_matrix= P2*Tr;

    // Eigen::Vector4d p_cam(0,0,0,1);

    // double b_rgb = (T_cam3_velo.inverse()*p_cam - T_cam2_velo.inverse()*p_cam).norm()*1000;   

    // std::cout<<"b_rgb="<< b_rgb<<std::endl;   

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay);


        Py_Initialize();
        init_numpy();
        PyRun_SimpleString("import os");
        PyRun_SimpleString("import sys");
        PyRun_SimpleString("sys.path.append('/home/wenyu/catkin_ws/src/slammot/src')"); 
        PyObject* pModule =  PyImport_Import(PyString_FromString("depth_map_utils"));
        if (pModule == nullptr)
        {
            std::cout<<"error loading"<<std::endl;
            PyErr_Print();
            std::exit(1);
        }
        PyObject *pDict    =  PyModule_GetDict(pModule);
        PyObject* pFunc =PyDict_GetItemString(pDict, "fill_in_multiscale");
        PyObject* pArgs = PyTuple_New(1);
        std::cout<<"load funs"<<std::endl;


    while (std::getline(timestamp_file, line) && ros::ok())
    {
        float timestamp = stof(line);

        std::stringstream left_image_path;
        left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_2/"
                    << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str());

        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "sequences/" + sequence_number + "/velodyne/" 
                    << std::setfill('0') << std::setw(6) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<float> lidar_intensities;

        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> laser_cloud_RGB;

        cv::Mat depth_image = cv::Mat(left_image.size().height, left_image.size().width,  CV_16UC1,cv::Scalar(0));

        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            // lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            // lidar_intensities.push_back(lidar_data[i+3]);

            Eigen::Vector4d P_xyz(lidar_data[i],lidar_data[i + 1],lidar_data[i + 2],1);
            Eigen::Vector3d P_uv = project_matrix*P_xyz;
            P_uv << P_uv[0]/P_uv[2],P_uv[1]/P_uv[2],P_uv[2];

            int u = int(P_uv[0]);
            int v = int(P_uv[1]);

                if(u>=0 && u<left_image.size().width && v>=0 && v<left_image.size().height && lidar_data[i]>0)
                {
                    // pcl::PointXYZ point_1;
                    // point_1.x = lidar_data[i];
                    // point_1.y = lidar_data[i + 1];
                    // point_1.z = lidar_data[i + 2];
                    // projection_map.insert(std::pair<cv::Point, pcl::PointXYZ>(cv::Point(u, v), point_1));
                    cv::Vec3b rgb_pixel = left_image.at<cv::Vec3b>(v, u);
                    pcl::PointXYZRGB colored_3d_point;
                    colored_3d_point.x = lidar_data[i];
                    colored_3d_point.y = lidar_data[i + 1];
                    colored_3d_point.z = lidar_data[i + 2];
                    colored_3d_point.r = rgb_pixel[2];
                    colored_3d_point.g = rgb_pixel[1];
                    colored_3d_point.b = rgb_pixel[0];
                    laser_cloud_RGB.push_back(colored_3d_point);
                    depth_image.at<uint16_t>(v, u) =  uint16_t(P_uv[2]*256) ; 
                }

                    pcl::PointXYZI point;
                    point.x = lidar_data[i];
                    point.y = lidar_data[i + 1];
                    point.z = lidar_data[i + 2];
                    point.intensity = lidar_data[i + 3];
                    laser_cloud.push_back(point);
        }

            //depth completion and rgb color
            npy_intp Dims[2] = { left_image.size().height,left_image.size().width}; 
            PyObject *PyArray  = PyArray_SimpleNewFromData(2, Dims, NPY_UINT16, depth_image.data); 
            PyTuple_SetItem(pArgs, 0, PyArray); 

            cv::Mat dense_depth_image;
    
            if(publish_dense_depth)
            {
                PyObject* pReturn = PyObject_CallObject(pFunc, pArgs);
                uint16_t *data = (uint16_t *)PyByteArray_AsString(pReturn );
            // cv::Mat dense_depth_image(left_image.size().height,  left_image.size().width, CV_16UC1, data);
                dense_depth_image=cv::Mat(left_image.size().height,  left_image.size().width, CV_16UC1, data);
            }
             else
            {
                dense_depth_image=cv::Mat(left_image.size().height,  left_image.size().width, CV_16UC1, cv::Scalar(0));
            }





            pcl::PointCloud<pcl::PointXYZRGB> laser_cloud_dense;
       
            // Eigen::Isometry3d T_cam2_velo_inv=  T_cam2_velo.inverse();
            Eigen::Matrix<double,4,4> T_cam2_velo_inv=  T_cam2_velo.inverse();
            for(auto cv_row = 0; cv_row < dense_depth_image.size().height; cv_row++){
                for(auto cv_col = 0; cv_col < dense_depth_image.size().width; cv_col++){
                    if (dense_depth_image.at<uint16_t>(cv_row, cv_col) !=0)
                    {
                        pcl::PointXYZRGB dense_3d_point;
                        cv::Vec3b rgb_pixel = left_image.at<cv::Vec3b>(cv_row, cv_col);
                        Eigen::Vector4d P_cam(
                            uint16_t(dense_depth_image.at<uint16_t>(cv_row, cv_col)/256)*(cv_col-P2(0,2))/P2(0,0),
                            uint16_t(dense_depth_image.at<uint16_t>(cv_row, cv_col)/256)*(cv_row-P2(1,2))/P2(1,1),
                            uint16_t(dense_depth_image.at<uint16_t>(cv_row, cv_col)/256),
                            1);
                        Eigen::Vector4d P_velo = T_cam2_velo_inv*P_cam;
                        dense_3d_point.x = P_velo(0);
                        dense_3d_point.y = P_velo(1);
                        dense_3d_point.z = P_velo(2);
                        // dense_3d_point.intensity = 1;
                        dense_3d_point.r = rgb_pixel[2];
                        dense_3d_point.g = rgb_pixel[1];
                        dense_3d_point.b = rgb_pixel[0];
                        laser_cloud_dense.push_back(dense_3d_point);
                    }
                }
            }

        



    // //save pcd to bin file
    // std::ofstream out;
    // std::stringstream save_filename;
    // save_filename << dataset_folder << "sequences/" + sequence_number + "/dense/" 
    //                 << std::setfill('0') << std::setw(6) << line_num << ".bin";
    // out.open(save_filename.str(), std::ios::out | std::ios::binary);
    // std::cout << save_filename.str() << " saved" << std::endl;
    // int cloudSize = laser_cloud_dense.points.size();
    // for (int i = 0; i < cloudSize; ++i)
    // {
    //     float point_x = laser_cloud_dense.points[i].x;
    //     float point_y = laser_cloud_dense.points[i].y;
    //     float point_z = laser_cloud_dense.points[i].z;
    //     out.write(reinterpret_cast<const char *>(&point_x), sizeof(float));
    //     out.write(reinterpret_cast<const char *>(&point_y), sizeof(float));
    //     out.write(reinterpret_cast<const char *>(&point_z), sizeof(float));
    // }
    // out.close();

    // // save dense to pcd file
    // std::stringstream save_filename;
    // save_filename << dataset_folder << "sequences/" + sequence_number + "/dense_pcd/" 
    //                 << std::setfill('0') << std::setw(6) << line_num << ".pcd";
    // pcl::io::savePCDFileASCII (save_filename.str(), laser_cloud_dense);

    // // save sparse to pcd file
    // std::stringstream save_filename;
    // save_filename << dataset_folder << "sequences/" + sequence_number + "/sparse_pcd/" 
    //                 << std::setfill('0') << std::setw(6) << line_num << ".pcd";
    // pcl::io::savePCDFileASCII (save_filename.str(), laser_cloud_RGB);

    //     // save raw bin to pcd file
    // std::stringstream save_filename;
    // save_filename << dataset_folder << "sequences/" + sequence_number + "/raw_pcd/" 
    //                 << std::setfill('0') << std::setw(6) << line_num << ".pcd";
    // pcl::io::savePCDFileASCII (save_filename.str(), laser_cloud);


        // std::stringstream image_depth_path;
        // image_depth_path << "/home/wenyu/test_depth/"  << std::setfill('0') << std::setw(6) << line_num << ".png";
        // cv::imwrite(image_depth_path.str(),dense_depth_image);

        std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<double, 4, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }
        // gt_pose.row(3)<< 0,0,0,1;
        // gt_pose=T_cam2_velo_inv*gt_pose;
        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();
        odomGT.header.stamp = ros::Time().fromSec(timestamp);
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);

        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT.publish(pathGT);

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        laser_cloud_msg.header.frame_id = lidar_frame_id;
        pub_laser_cloud.publish(laser_cloud_msg);

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(laser_cloud_RGB, cloud_msg);
        cloud_msg.header = laser_cloud_msg.header;
        pub_fuse_cloud.publish(cloud_msg);

        std_msgs::Header image_header;
        image_header=laser_cloud_msg.header;
        image_header.frame_id="camera_link";

        if(publish_dense_depth)
        {
            sensor_msgs::PointCloud2 cloud_msg_dense_rgb;
            pcl::toROSMsg(laser_cloud_dense, cloud_msg_dense_rgb);
            cloud_msg_dense_rgb.header=laser_cloud_msg.header;
            pub_dense_cloud_rgb.publish(cloud_msg_dense_rgb);

            dense_depth_image.convertTo(dense_depth_image,CV_16UC1);
            sensor_msgs::ImagePtr image_depth_dense_msg = cv_bridge::CvImage(image_header, "16UC1", dense_depth_image).toImageMsg();
            pub_image_depth_dense.publish(image_depth_dense_msg);

        }


        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(image_header, "bgr8", left_image).toImageMsg();
        pub_image_left.publish(image_left_msg);

        depth_image.convertTo(depth_image,CV_16UC1);
        sensor_msgs::ImagePtr image_depth_msg = cv_bridge::CvImage(image_header, "16UC1", depth_image).toImageMsg();
        pub_image_depth.publish(image_depth_msg);


        sensor_msgs::CameraInfo lidar2image;
        lidar2image.header  =  laser_cloud_msg.header;
        lidar2image.header.frame_id = "/camera_init";

        //Extrinsics
        lidar2image.P={T_cam2_velo(0,0),T_cam2_velo(0,1),T_cam2_velo(0,2),T_cam2_velo(0,3),
                                        T_cam2_velo(1,0),T_cam2_velo(1,1),T_cam2_velo(1,2),T_cam2_velo(1,3),
                                        T_cam2_velo(2,0),T_cam2_velo(2,1),T_cam2_velo(2,2),T_cam2_velo(2,3)};

        //Intrinsics
        lidar2image.K={P2(0,0),P2(0,1),P2(0,2),
                                         P2(1,0),P2(1,1),P2(1,2),
                                         P2(2,0),P2(2,1),P2(2,2)};
        lidar2image.D={0,0,0,0,0};

        lidar2image.height=left_image.size().height;
        lidar2image.width=left_image.size().width;
        pub_camera_info.publish(lidar2image);
        
        if (to_bag)
        {
            bag_out.write("/image_left", ros::Time::now(), image_left_msg);
            bag_out.write("/image_depth", ros::Time::now(), image_depth_msg);
            // bag_out.write("/points_raw", ros::Time::now(), laser_cloud_msg);
            // bag_out.write("/points_fuse", ros::Time::now(), cloud_msg);
            bag_out.write("/camera_info", ros::Time::now(), lidar2image);
            // bag_out.write("/points_dense", ros::Time::now(), cloud_msg);
        }
        // std::cout<<"frame:"<<line_num<<std::endl;
        line_num ++;

        // printf("process dataset %f ms *************\n", t_whole.toc());

        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";

    if(publish_dense_depth)
    {
        Py_Finalize();
    }


    return 0;
}    

