// FYP-project use

//  Objective: Create custom service node with two types of service for GPD to generate grasp poses.
//    1) Sample_during_generation
//    2) Pre_sampled

//  Note: This is only used for 1 camera viewpoint

//  Service1: Requires 2 pointclouds as input. Segmented pointcloud for sampling done by gpd,
//use the sample points onto full pointcloud to generate valid grasp with context of the environment
//Segmented Pointcloud: PC of 1 object that want to grasp
//Full Pointcloud: PC of the environment perceived by the camera

//  Service2: Requires 1 pointcloud and sample points. Pre-sampled as sampling is done with another node
//Note: Similar to original grasp_detection_server node, but uses sample points instead of indices

#include <gpd_ros/grasp_detection_custom.h>

// Initialise Node
GraspDetectionCustom::GraspDetectionCustom(ros::NodeHandle& node) : frame_("")
{
  cloud_camera_ = NULL;

  // set camera viewpoint to default origin
  //std::vector<double> camera_position;
  //node.getParam("camera_position", camera_position);
  //view_point_ << camera_position[0], camera_position[1], camera_position[2];

  std::string cfg_file;
  node.param("config_file", cfg_file, std::string(""));
  grasp_detector_ = new gpd::GraspDetector(cfg_file);

  std::string rviz_topic;
  node.param("rviz_topic", rviz_topic, std::string("plot_grasps"));


  //rviz
  if (!rviz_topic.empty())
  {
    grasps_rviz_pub_ = node.advertise<visualization_msgs::MarkerArray>(rviz_topic, 1);  //additional
    rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);
    use_rviz_ = true;
  }
  else
  {
    use_rviz_ = false;
  }

  // Advertise ROS topic for detected grasps.
  grasps_pub_ = node.advertise<gpd_ros::GraspConfigList>("clustered_grasps", 10);

  node.getParam("workspace", workspace_);
}


bool GraspDetectionCustom::detectGrasps(gpd_ros::detect_grasps_custom::Request& req, gpd_ros::detect_grasps_custom::Response& res)
{
  ROS_INFO("Received service request ...");

  // 1. Initialize cloud camera.
  cloud_camera_ = NULL;

  // Set view points.
  Eigen::Matrix3Xd view_points(3,1);
  view_points.col(0) = view_point_;

  // Set point cloud.
  if (req.seg_cloud.fields.size() == 6 && req.seg_cloud.fields[3].name == "normal_x" && req.seg_cloud.fields[4].name == "normal_y"
    && req.seg_cloud.fields[5].name == "normal_z")
  {
    ROS_INFO("Setup Cloud with Normal");
    PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
    pcl::fromROSMsg(req.seg_cloud, *cloud);
    cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
    cloud_camera_header_ = req.seg_cloud.header;
    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points and normals.");
  }
  else
  {
    ROS_INFO("Setup Cloud with RGB");
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(req.seg_cloud, *cloud);
    cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
    cloud_camera_header_ = req.seg_cloud.header;
    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
  }

  frame_ = req.seg_cloud.header.frame_id;


  // 2. Preprocess the point cloud.
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // get samples
  //ROS_INFO("Extract Samples");
  std::vector<int> indices_t = cloud_camera_->getSampleIndices();

  Eigen::Matrix3Xd samples(3, cloud_camera_->getSampleIndices().size());

  //debug
  //ROS_INFO("EIGEN SIZE:");
  //std::cout << std::endl << samples.size() << std::endl;

  //ROS_INFO("Get Samples");
  int count = 0;
  for (int i: indices_t)
  {
    //std::cout << i << ' ';
    //add 3 elements into the row in order x,y,z
    samples.col(count) << cloud_camera_->getCloudProcessed()->points[i].x, cloud_camera_->getCloudProcessed()->points[i].y, cloud_camera_->getCloudProcessed()->points[i].z;
    count++;
  }

  //debug
  //ROS_INFO("Print Samples");
  //std::cout << std::endl << samples << std::endl;

  //ROS_INFO("setting up Samples");
  if (cloud_camera_->getSampleIndices().size() != 0)
  {
    cloud_camera_ = NULL;
    // Set point cloud full
    if (req.full_cloud.fields.size() == 6 && req.full_cloud.fields[3].name == "normal_x" && req.full_cloud.fields[4].name == "normal_y"
      && req.full_cloud.fields[5].name == "normal_z")
    {
      ROS_INFO("Setup Cloud with Normal");
      PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
      pcl::fromROSMsg(req.full_cloud, *cloud);
      cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
      cloud_camera_header_ = req.full_cloud.header;
      ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points and normals.");
    }
    else
    {
      ROS_INFO("Setup Cloud with RGB");
      PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
      pcl::fromROSMsg(req.full_cloud, *cloud);
      cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
      cloud_camera_header_ = req.full_cloud.header;
      ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
    }

    //ROS_INFO("Insert Samples");
    //cloud_camera_->setSampleIndices(indices_t);
    cloud_camera_->setSamples(samples);  //error here: something to do with samples generated using Eigen::Matrix3Xd (fixed with changing the flags in CMake (GPD, not GPD_ros))

    //cloud_camera_->subsample(30);       //to minimise sample size further but the number of sample can be set in the cfg file in GPD

    //ROS_INFO("Reprocess the point cloud.");
    grasp_detector_->preprocessPointCloud(*cloud_camera_);

    // 3. Detect grasps in the point cloud.
    //ROS_INFO("Detect grasps in the point cloud.");
    std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

    if (grasps.size() > 0)   //if the is grasp
    {
      // Visualize the detected grasps in rviz.
      if (use_rviz_)
      {
        rviz_plotter_->drawGrasps(grasps, frame_);
      }

      // Publish the detected grasps.
      gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
      res.grasp_configs = selected_grasps_msg;
      ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
      return true;
    }
    //otherwise return false
    ROS_WARN("No grasps detected!");
    return false;
  }
  else
  {
    ROS_WARN("No sample detected!");
    return false;
  }

}

bool GraspDetectionCustom::detectGraspsSampled(gpd_ros::detect_grasps_custom_sampled::Request& req, gpd_ros::detect_grasps_custom_sampled::Response& res)
{
  ROS_INFO("Received service request with sample...");

  // 1. Initialize cloud camera.
  // clean up
  if(!cloud_camera_) delete cloud_camera_;
  cloud_camera_ = NULL;

  // Set view points.
  Eigen::Matrix3Xd view_points(3,1);
  view_points.col(0) = view_point_;

  // Set point cloud.
  if (req.cloud.fields.size() == 6 && req.cloud.fields[3].name == "normal_x" && req.cloud.fields[4].name == "normal_y"
    && req.cloud.fields[5].name == "normal_z")
  {
    ROS_INFO("Setup Cloud with Normal");
    PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
    pcl::fromROSMsg(req.cloud, *cloud);
    cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
    cloud_camera_header_ = req.cloud.header;
    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points and normals.");
  }
  else
  {
    ROS_INFO("Setup Cloud with RGB");
    PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
    pcl::fromROSMsg(req.cloud, *cloud);
    cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);
    cloud_camera_header_ = req.cloud.header;
    ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points.");
  }

  frame_ = req.cloud.header.frame_id;


  // Set the samples at which to sample grasp candidates.
  //matrix 3-3inrow ,X-dynamic column size, d-double
  Eigen::Matrix3Xd samples(3, req.samples.samples.size());
  //3 element in row, n number of elements in column depending on number of samples
  for (int i=0; i < req.samples.samples.size(); i++)
  {
    //add 3 elements into the row in order x,y,z
    samples.col(i) << req.samples.samples[i].x, req.samples.samples[i].y, req.samples.samples[i].z;
  }

  cloud_camera_->setSamples(samples);  //error here: something to do with samples generated using Eigen::Matrix3Xd (fixed with changing the flags in CMake (GPD, not GPD_ros))


  ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
    << cloud_camera_->getSamples().cols() << " samples");

  // 2. Preprocess the point cloud.
  grasp_detector_->preprocessPointCloud(*cloud_camera_);

  // 3. Detect grasps in the point cloud.
  std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_);

  if (grasps.size() > 0)   //if the is grasp
  {
    // Visualize the detected grasps in rviz.
    if (use_rviz_)
    {
      rviz_plotter_->drawGrasps(grasps, frame_);
    }

    // Publish the detected grasps.
    gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
    res.grasp_configs = selected_grasps_msg;
    ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
    return true;
  }
  //otherwise return false
  ROS_WARN("No grasps detected!");
  return false;
}



int main(int argc, char** argv)
{
  // seed the random number generator
  std::srand(std::time(0));

  // initialize ROS
  ros::init(argc, argv, "detect_grasps_custom");
  ros::NodeHandle node("~");

  GraspDetectionCustom grasp_detection_custom(node);

  ros::ServiceServer service = node.advertiseService("detect_grasps", &GraspDetectionCustom::detectGrasps,
                                                     &grasp_detection_custom);

  //for other one
  ros::ServiceServer service2 = node.advertiseService("detect_grasps_sampled", &GraspDetectionCustom::detectGraspsSampled,
                                                      &grasp_detection_custom);

  ROS_INFO("Grasp detection service is waiting for a point cloud ...");

  ros::spin();

  return 0;
}
