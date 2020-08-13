#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

class PCStoreWithOdometry{
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
		pcl::visualization::PCLVisualizer viewer{"pc_store_with_odometry"};
		/*cloud*/
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_stored {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_now {new pcl::PointCloud<pcl::PointNormal>};
		/*flags*/
		bool pc_was_added = false;
		/*pub info*/
		std::string frame_id_pub;
		ros::Time time_pub;
		/*parameters*/
		double leaf_size;
	public:
		PCStoreWithOdometry();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Downsampling(pcl::PointCloud<pcl::PointNormal>::Ptr pc);
		void Visualization(void);
		void Publication(void);
};

PCStoreWithOdometry::PCStoreWithOdometry()
	: nhPrivate("~")
{
	sub_pc = nh.subscribe("/normals", 1, &PCStoreWithOdometry::CallbackPC, this);
	sub_odom = nh.subscribe("/odom", 1, &PCStoreWithOdometry::CallbackOdom, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/normals/stored", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");

	nhPrivate.param("leaf_size", leaf_size, 0.5);
	std::cout << "leaf_size = " << leaf_size << std::endl;
}

void PCStoreWithOdometry::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK PC" << std::endl;

	pcl::fromROSMsg(*msg, *cloud_now);
	pc_was_added = false;
}

void PCStoreWithOdometry::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;

	frame_id_pub = msg->header.frame_id;
	time_pub = msg->header.stamp;
	if(!pc_was_added){
		/* #<{(|downsampling|)}># */
		/* Downsampling(cloud_now); */
		/*transform*/
		Eigen::Vector3f offset(
			msg->pose.pose.position.x,
			msg->pose.pose.position.y,
			msg->pose.pose.position.z
		);
		Eigen::Quaternionf rotation(
			msg->pose.pose.orientation.w,
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z
		);
		// pcl::transformPointCloud(*cloud_now, *cloud_now, offset, rotation);
		pcl::transformPointCloudWithNormals(*cloud_now, *cloud_now, offset, rotation);

		*cloud_stored  += *cloud_now;
		pc_was_added = true;

		/*downsampling*/
		Downsampling(cloud_stored);

		Visualization();
		Publication();
	}
}

void PCStoreWithOdometry::Downsampling(pcl::PointCloud<pcl::PointNormal>::Ptr pc)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp (new pcl::PointCloud<pcl::PointNormal>);
	pcl::VoxelGrid<pcl::PointNormal> vg;
	vg.setInputCloud(pc);
	vg.setLeafSize((float)leaf_size, (float)leaf_size, (float)leaf_size);
	vg.filter(*tmp);
	*pc = *tmp;
}

void PCStoreWithOdometry::Visualization(void)
{
	viewer.removeAllPointClouds();

	/*normals*/
	viewer.addPointCloudNormals<pcl::PointNormal>(cloud_stored, 1, 0.5, "cloud_stored");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "cloud_stored");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "cloud_stored");
	
	viewer.spinOnce();
}

void PCStoreWithOdometry::Publication(void)
{
	sensor_msgs::PointCloud2 ros_pc_out;
	pcl::toROSMsg(*cloud_stored, ros_pc_out);
	ros_pc_out.header.frame_id = frame_id_pub;
	ros_pc_out.header.stamp = time_pub;
	pub.publish(ros_pc_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_store_with_odometry");
	
	PCStoreWithOdometry pc_store_with_odometry;

	ros::spin();
}
