/*****************************************************
 * JHU License stuff goes here
 *
 * **************************************************/

#include "Greedy/seg.h"
#include <sys/time.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <bits/algorithmfwd.h>

#include "Greedy/refine.hpp"

// ros communication stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>

//=========================================================================================================================

bool view_flag = false;

ros::Subscriber sub;
ros::Publisher pub_link;
ros::Publisher pub_node;

std::string link_mesh_name("data/link_uniform");
std::string node_mesh_name("data/node_uniform");

pcl::visualization::PCLVisualizer::Ptr viewer;

double linkWidth = 0.15;
double nodeWidth = 0.05;
double voxelSize = 0.003;

greedyObjRansac linkrec(linkWidth, voxelSize);
greedyObjRansac noderec(nodeWidth, voxelSize);
greedyObjRansac objrec(nodeWidth, voxelSize);

ModelT link_mesh, node_mesh;
std::vector<ModelT> mesh_set;

int method_id = 4;
std::string trial_id("0");
std::string path("data/ln_joint/");

void callback(const sensor_msgs::PointCloud2 &pc) {

  int t1, t2, avg_t;

  pcl::PointCloud<PointT>::Ptr scene_pc(new pcl::PointCloud<PointT>());

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(pc, pcl_pc);

  pcl::fromPCLPointCloud2(pcl_pc, *scene_pc);

  if( scene_pc->empty() == true )
    ROS_ERROR("Recieved empty point cloud message!");
  return;

  pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
  pcl::copyPointCloud(*scene_pc, *scene_xyz);
  if( view_flag == true )
  {
    viewer->removeAllPointClouds();
    viewer->addPointCloud(scene_xyz, "whole_scene");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "whole_scene");
  }
  std::vector<poseT> all_poses;
  std::vector<poseT> link_poses, node_poses;
  switch(method_id)
  {
    case 0:
      t1 = get_wall_time();
      objrec.StandardRecognize(scene_xyz, all_poses);
      t2 = get_wall_time();
      break;
    case 1:
      {
        t1 = get_wall_time();
        int pose_num = 0;
        int iter = 0;
        pcl::PointCloud<myPointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<myPointXYZ>());
        filtered_cloud = scene_xyz;
        while(true)
        {
          //std::cerr<< "Recognizing Attempt --- " << iter << std::endl;
          objrec.StandardRecognize(filtered_cloud, all_poses);
          if( all_poses.size() <= pose_num )
            break;
          else
            pose_num = all_poses.size();

          pcl::PointCloud<myPointXYZ>::Ptr model_cloud = objrec.FillModelCloud(all_poses);
          filtered_cloud = FilterCloud(filtered_cloud, model_cloud);

          iter++;
        }
        t2 = get_wall_time();
        //std::cerr<< "Recognizing Done!!!" << std::endl;
        break;
      }
    case 2:
      {
        //std::vector<poseT> tmp_poses;
        t1 = get_wall_time();
        objrec.GreedyRecognize(scene_xyz, all_poses);
        t2 = get_wall_time();
        //all_poses = RefinePoses(scene_xyz, mesh_set, tmp_poses);
        break;
      }
    case 3:
      {
        pcl::PointCloud<myPointXYZ>::Ptr link_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::PointCloud<myPointXYZ>::Ptr node_cloud(new pcl::PointCloud<myPointXYZ>());
        splitCloud(scene_pc, link_cloud, node_cloud);

        t1 = get_wall_time();
        //std::vector<poseT> link_poses, node_poses;
        linkrec.StandardRecognize(link_cloud, link_poses);
        noderec.StandardRecognize(node_cloud, node_poses);
        t2 = get_wall_time();

        all_poses.insert(all_poses.end(), link_poses.begin(), link_poses.end());
        all_poses.insert(all_poses.end(), node_poses.begin(), node_poses.end());
        break;
      }
    case 4:
      {
        pcl::PointCloud<myPointXYZ>::Ptr link_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::PointCloud<myPointXYZ>::Ptr node_cloud(new pcl::PointCloud<myPointXYZ>());
        splitCloud(scene_pc, link_cloud, node_cloud);

        t1 = get_wall_time();
        int pose_num = 0;
        std::vector<poseT> tmp_poses;
        int iter = 0;
        while(true)
        {
          //std::cerr<< "Recognizing Attempt --- " << iter << std::endl;
          list<AcceptedHypothesis> acc_hypotheses;

          linkrec.genHypotheses(link_cloud, acc_hypotheses);
          noderec.genHypotheses(node_cloud, acc_hypotheses);

          linkrec.mergeHypotheses(scene_xyz, acc_hypotheses, tmp_poses);

          if( tmp_poses.size() <= pose_num )
            break;
          else
            pose_num = tmp_poses.size();

          pcl::PointCloud<myPointXYZ>::Ptr link_model = linkrec.FillModelCloud(tmp_poses);
          pcl::PointCloud<myPointXYZ>::Ptr node_model = noderec.FillModelCloud(tmp_poses);

          if( link_model->empty() == false )
            link_cloud = FilterCloud(link_cloud, link_model);
          if( node_model->empty() == false)
            node_cloud = FilterCloud(node_cloud, node_model);
          iter++;
        }
        t2 = get_wall_time();
        //std::cerr<< "Recognizing Done!!!" << std::endl;
        all_poses = RefinePoses(scene_xyz, mesh_set, tmp_poses);
        break;
      }
    case 5:
      {
        pcl::PointCloud<myPointXYZ>::Ptr link_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::PointCloud<myPointXYZ>::Ptr node_cloud(new pcl::PointCloud<myPointXYZ>());
        splitCloud(scene_pc, link_cloud, node_cloud);

        t1 = get_wall_time();
        linkrec.GreedyRecognize(link_cloud, link_poses);
        noderec.GreedyRecognize(node_cloud, node_poses);
        t2 = get_wall_time();

        std::vector<poseT> tmp_poses;
        tmp_poses.insert(tmp_poses.end(), link_poses.begin(), link_poses.end());
        tmp_poses.insert(tmp_poses.end(), node_poses.begin(), node_poses.end());

        //all_poses = tmp_poses;
        all_poses = RefinePoses(scene_xyz, mesh_set, tmp_poses);
        break;
      }
    default:return;
  }

  if( view_flag == true )
  {
    switch(method_id)
    {
      case 0:
      case 1:
      case 2:
        objrec.visualize(viewer, all_poses);
        break;
      case 3:
      case 4:
      case 5:
        linkrec.visualize(viewer, link_poses);
        noderec.visualize(viewer, node_poses);
        break;
      default:return;
    }
    viewer->spin();
  }

  geometry_msgs::PoseArray links;
  geometry_msgs::PoseArray nodes;

  // publish poses as TF
  if (method_id < 3) {
    for (poseT &p: all_poses) {
      geometry_msgs::Pose pose;
      pose.position.x = p.shift[0];
      pose.position.y = p.shift[1];
      pose.position.z = p.shift[2];
      pose.orientation.x = p.rotation.x();
      pose.orientation.y = p.rotation.y();
      pose.orientation.z = p.rotation.z();
      pose.orientation.w = p.rotation.w();
      links.poses.push_back(pose);
    }
    pub_link.publish(links);
  } else {
    for (poseT &p: all_poses) {
      geometry_msgs::Pose pose;
      pose.position.x = p.shift[0];
      pose.position.y = p.shift[1];
      pose.position.z = p.shift[2];
      pose.orientation.x = p.rotation.x();
      pose.orientation.y = p.rotation.y();
      pose.orientation.z = p.rotation.z();
      pose.orientation.w = p.rotation.w();
      links.poses.push_back(pose);
    }
    for (poseT &p: all_poses) {
      geometry_msgs::Pose pose;
      pose.position.x = p.shift[0];
      pose.position.y = p.shift[1];
      pose.position.z = p.shift[2];
      pose.orientation.x = p.rotation.x();
      pose.orientation.y = p.rotation.y();
      pose.orientation.z = p.rotation.z();
      pose.orientation.w = p.rotation.w();
      nodes.poses.push_back(pose);
    }
    pub_link.publish(links);
    pub_link.publish(nodes);
  }


}

//std::string link_mesh_name("data/driller_uniform");
//std::string node_mesh_name("data/sander_uniform");

std::vector<poseT> RefinePoses(const pcl::PointCloud<myPointXYZ>::Ptr scene,
                               const std::vector<ModelT> &mesh_set, const std::vector<poseT> &all_poses);
//=========================================================================================================================

int main(int argc, char** argv)
{

  ros::init(argc,argv,"greedy_objrec_ransac_node");
  ros::NodeHandle nh;

  pcl::console::parse_argument(argc, argv, "--m", method_id);
  pcl::console::parse_argument(argc, argv, "--p", path);
  pcl::console::parse_argument(argc, argv, "--t", trial_id);

  bool view_flag = false;
  if( pcl::console::find_switch(argc, argv, "-v") == true )
    view_flag = true;

  if( view_flag == true )
  {
    viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setSize(640, 480);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
  }

  std::stringstream mm;
  mm << method_id;

  if( method_id >= 3 )
  {
    link_mesh = LoadMesh(link_mesh_name + ".obj", "link");
    node_mesh = LoadMesh(node_mesh_name + ".obj", "node");
    mesh_set.push_back(link_mesh);
    mesh_set.push_back(node_mesh);
  }

  switch(method_id)
  {
    case 0:
    case 1:
    case 2:
      objrec.AddModel(link_mesh_name, "link");
      objrec.AddModel(node_mesh_name, "node");
      break;
    case 3:
    case 4:
    case 5:
      linkrec.AddModel(link_mesh_name, "link");
      noderec.AddModel(node_mesh_name, "node");
      break;
    default:return 0;
  }

  if (method_id < 3) {
    pub_link = nh.advertise<geometry_msgs::PoseArray>("objects",1000);
  } else {
    pub_link = nh.advertise<geometry_msgs::PoseArray>("links",1000);
    pub_node = nh.advertise<geometry_msgs::PoseArray>("nodes",1000);
  }
  sub = nh.subscribe("points_in",1,callback);

  // send and recieve messages
  while (ros::ok()) {
    ros::spin();
  }

  return 0;
}

