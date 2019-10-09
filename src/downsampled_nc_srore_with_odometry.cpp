#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <pcl/common/transforms.h>
/* #include <pcl/filters/voxel_grid.h> */
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>

class DownsampledNCSroreWithOdometry{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_odom;
		/*publish*/
		ros::Publisher pub;
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"downsampled_nc_srore_with_odometry"};
		/*cloud*/
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_stored {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_now {new pcl::PointCloud<pcl::PointNormal>};
		/*odom*/
		nav_msgs::Odometry odom_now;
		nav_msgs::Odometry odom_last;
		/*flags*/
		bool first_callback_odom = true;
		bool pc_was_added = false;
		/*limit storing*/
		std::vector<size_t> list_num_scanpoints;
		/*parameters*/
		bool mode_limit_store;
		double pc_range;
		double leaf_size;
	public:
		DownsampledNCSroreWithOdometry();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Downsampling(pcl::PointCloud<pcl::PointNormal>::Ptr pc);
		void PassThrough(pcl::PointCloud<pcl::PointNormal>::Ptr pc_in, pcl::PointCloud<pcl::PointNormal>::Ptr pc_out, std::vector<double> range);
		void Visualization(void);
		void Publication(void);
};

DownsampledNCSroreWithOdometry::DownsampledNCSroreWithOdometry()
	: nhPrivate("~")
{
	sub_pc = nh.subscribe("/normals", 1, &DownsampledNCSroreWithOdometry::CallbackPC, this);
	sub_odom = nh.subscribe("/odom", 1, &DownsampledNCSroreWithOdometry::CallbackOdom, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/normals/stored", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");

	nhPrivate.param("mode_limit_store", mode_limit_store, true);
	nhPrivate.param("pc_range", pc_range, 10.0);
	nhPrivate.param("leaf_size", leaf_size, 0.2);
	std::cout << "mode_limit_store = " << mode_limit_store << std::endl;
	std::cout << "pc_range = " << pc_range << std::endl;
	std::cout << "leaf_size = " << leaf_size << std::endl;
}

void DownsampledNCSroreWithOdometry::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK PC" << std::endl;
	pcl::fromROSMsg(*msg, *cloud_now);
	cloud_stored->header.frame_id = msg->header.frame_id;
	pc_was_added = false;
}

void DownsampledNCSroreWithOdometry::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;
	odom_now = *msg;
	if(first_callback_odom)	odom_last = odom_now;
	else if(!pc_was_added){
		/*compute offset and rotation*/
		tf::Quaternion pose_now;
		tf::Quaternion pose_last;
		quaternionMsgToTF(odom_now.pose.pose.orientation, pose_now);
		quaternionMsgToTF(odom_last.pose.pose.orientation, pose_last);
		tf::Quaternion relative_rotation = pose_last*pose_now.inverse();
		relative_rotation.normalize();	
		Eigen::Quaternionf rotation(relative_rotation.w(), relative_rotation.x(), relative_rotation.y(), relative_rotation.z());
		tf::Quaternion q_global_move(
			odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
			odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
			odom_last.pose.pose.position.z - odom_now.pose.pose.position.z,
			0.0
		);
		tf::Quaternion q_local_move = pose_last.inverse()*q_global_move*pose_last;
		Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
		/*transform*/
		pcl::transformPointCloud(*cloud_stored, *cloud_stored, offset, rotation);
		*cloud_stored  += *cloud_now;
		pc_was_added = true;
		
		odom_last = odom_now;

		/*limit storing*/
		if(mode_limit_store){
			PassThrough(cloud_stored, cloud_stored, std::vector<double> {-pc_range, pc_range, -pc_range, pc_range});
		}
		/*downsampling*/
		Downsampling(cloud_stored);
	}
	first_callback_odom = false;

	Visualization();
	Publication();
}

void DownsampledNCSroreWithOdometry::Downsampling(pcl::PointCloud<pcl::PointNormal>::Ptr pc)
{
	pcl::ApproximateVoxelGrid<pcl::PointNormal> avg;
	avg.setInputCloud(pc);
	avg.setLeafSize((float)leaf_size, (float)leaf_size, (float)leaf_size);
	avg.filter(*pc);

	/* pcl::VoxelGrid<pcl::PointNormal> vg; */
	/* vg.setInputCloud(pc); */
	/* vg.setLeafSize(0.5f, 0.5f, 0.5f); */
	/* vg.filter(*pc); */
}

void DownsampledNCSroreWithOdometry::PassThrough(pcl::PointCloud<pcl::PointNormal>::Ptr pc_in, pcl::PointCloud<pcl::PointNormal>::Ptr pc_out, std::vector<double> range)
{
	pcl::PassThrough<pcl::PointNormal> pass;
	pass.setInputCloud(pc_in);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(range[0], range[1]);
	pass.filter(*pc_out);
	pass.setInputCloud(pc_out);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(range[2], range[3]);
	pass.filter(*pc_out);
}

void DownsampledNCSroreWithOdometry::Visualization(void)
{
	viewer.removeAllPointClouds();

	/*normals*/
	viewer.addPointCloudNormals<pcl::PointNormal>(cloud_stored, 1, 0.5, "cloud_stored");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "cloud_stored");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "cloud_stored");
	
	viewer.spinOnce();
}

void DownsampledNCSroreWithOdometry::Publication(void)
{
	sensor_msgs::PointCloud2 ros_pc_out;
	pcl::toROSMsg(*cloud_stored, ros_pc_out);
	ros_pc_out.header.stamp = odom_now.header.stamp;
	pub.publish(ros_pc_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "downsampled_nc_srore_with_odometry");
	
	DownsampledNCSroreWithOdometry downsampled_nc_srore_with_odometry;

	ros::spin();
}