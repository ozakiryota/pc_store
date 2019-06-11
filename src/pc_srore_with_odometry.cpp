#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <pcl/common/transforms.h>

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
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stored {new pcl::PointCloud<pcl::PointXYZI>};
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_now {new pcl::PointCloud<pcl::PointXYZI>};
		/*odom*/
		nav_msgs::Odometry odom_now;
		nav_msgs::Odometry odom_last;
		/*flags*/
		bool first_callback_odom = true;
		bool pc_was_added = false;
		/*limit storing*/
		bool mode_limit_store = true;
		int limited_num_scans = 20;
		std::vector<size_t> list_num_scanpoints;

	public:
		PCStoreWithOdometry();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Visualization(void);
		void Publication(void);
};

PCStoreWithOdometry::PCStoreWithOdometry()
	: nhPrivate("~")
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &PCStoreWithOdometry::CallbackPC, this);
	sub_odom = nh.subscribe("/odom", 1, &PCStoreWithOdometry::CallbackOdom, this);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/stored", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");

	nhPrivate.param("mode_limit_store", mode_limit_store, true);
	nhPrivate.param("limited_num_scans", limited_num_scans, 20);
}

void PCStoreWithOdometry::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK PC" << std::endl;
	if(pc_was_added)	cloud_now->points.clear();
	pcl::fromROSMsg(*msg, *cloud_now);
	cloud_stored->header.frame_id = msg->header.frame_id;
	pc_was_added = false;
}

void PCStoreWithOdometry::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;
	odom_now = *msg;
	if(first_callback_odom)	odom_last = odom_now;
	else if(!pc_was_added){
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
		pcl::transformPointCloud(*cloud_stored, *cloud_stored, offset, rotation);
		*cloud_stored  += *cloud_now;
		pc_was_added = true;
		
		odom_last = odom_now;
		
		/*limit storing*/
		if(mode_limit_store){
			list_num_scanpoints.push_back(cloud_now->points.size());
			if(list_num_scanpoints.size()>limited_num_scans){
				cloud_stored->points.erase(cloud_stored->points.begin(), cloud_stored->points.begin() + list_num_scanpoints[0]);
				list_num_scanpoints.erase(list_num_scanpoints.begin());
			}
			cloud_stored->width = cloud_stored->points.size();
			cloud_stored->height = 1;

			std::cout << "limit storing: true" << std::endl;
			std::cout << "number of stored scans: " << list_num_scanpoints.size() << std::endl;
		}
	}
	first_callback_odom = false;

	Visualization();
	Publication();
}

void PCStoreWithOdometry::Visualization(void)
{
	viewer.removeAllPointClouds();

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_stored, "intensity"); 
	viewer.addPointCloud<pcl::PointXYZI>(cloud_stored, intensity_distribution, "cloud");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	
	viewer.spinOnce();
}

void PCStoreWithOdometry::Publication(void)
{
	sensor_msgs::PointCloud2 ros_pc_out;
	pcl::toROSMsg(*cloud_stored, ros_pc_out);
	ros_pc_out.header.stamp = odom_now.header.stamp;
	pub.publish(ros_pc_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_store_with_odometry");
	
	PCStoreWithOdometry pc_store_with_odometry;

	ros::spin();
}
