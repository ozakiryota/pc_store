#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

class PcStore{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber sub_pc_;
		ros::Subscriber sub_odom_;
		/*publisher*/
		ros::Publisher pub_pc_;
		/*pc*/
		sensor_msgs::PointCloud2 pc_submsg_;
		sensor_msgs::PointCloud2 pc_pubmsg_;
		/*odom*/
		nav_msgs::Odometry odom_last_;
		/*list*/
		std::vector<std::string> field_list_;
		std::vector<size_t> num_scanpoints_list_;
		/*flag*/
		bool got_first_pc_ = false;
		bool got_new_pc_ = false;
		bool got_first_odom_ = false;
		/*parameter*/
		int max_num_buffering_scan_;
		double downsampling_leafsize_;

	public:
		PcStore();
		void listUpPointType(void);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void checkTypeAndStorePC(nav_msgs::Odometry odom_now);
		template<typename CloudPtr> void storePC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now);
		template<typename CloudPtr> void storeNC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now);
		template<typename CloudPtr, typename PointT> void downsampling(CloudPtr pc, PointT no_use);
		template<typename CloudPtr> void erasePoints(CloudPtr pc);
		void publication(void);
};

PcStore::PcStore()
	: nh_private_("~")
{
	std::cout << "--- pc_store_with_odometry ---" << std::endl;
	/*parameter*/
	nh_private_.param("max_buffering_scan", max_num_buffering_scan_, -1);
	std::cout << "max_num_buffering_scan_ = " << max_num_buffering_scan_ << std::endl;
	nh_private_.param("downsampling_leafsize", downsampling_leafsize_, -1.0);
	std::cout << "downsampling_leafsize_ = " << downsampling_leafsize_ << std::endl;
	/*subscriber*/
	sub_pc_ = nh_.subscribe("/point_cloud", 1, &PcStore::callbackPC, this);
	sub_odom_ = nh_.subscribe("/odom", 1, &PcStore::callbackOdom, this);
	/*publisher*/
	pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud/stored", 1);
	/*initialize*/
	listUpPointType();
}

void PcStore::listUpPointType(void)
{
	field_list_ = {
		"x y z",
		"x y z intensity",
		"x y z intensity ring time",	//velodyne
		"x y z strength",
		"x y z normal_x normal_y normal_z curvature",
		"x y z intensity normal_x normal_y normal_z curvature"
	};
}

void PcStore::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if(!got_first_pc_){
		pc_pubmsg_ = *msg;
		got_first_pc_ = true;
		return;
	}

	pc_submsg_ = *msg;
	got_new_pc_ = true;
}

void PcStore::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(!got_first_odom_){
		odom_last_ = *msg;
		got_first_odom_ = true;
		return;
	}
	if(!got_new_pc_)	return;

	checkTypeAndStorePC(*msg);
	publication();
	
	/*reset*/
	odom_last_ = *msg;
	got_new_pc_ = false;
}

void PcStore::checkTypeAndStorePC(nav_msgs::Odometry odom_now)
{
	std::string fields = pcl::getFieldsList(pc_submsg_);

	if(fields == field_list_[0]){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZ>);
		storePC(pc_now, pc_store, odom_now);
	}
	else if(fields == field_list_[1] || fields == field_list_[2]){
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZI>);
		storePC(pc_now, pc_store, odom_now);
	}
	else if(fields == field_list_[3]){
		pcl::PointCloud<pcl::InterestPoint>::Ptr pc_now (new pcl::PointCloud<pcl::InterestPoint>);
		pcl::PointCloud<pcl::InterestPoint>::Ptr pc_store (new pcl::PointCloud<pcl::InterestPoint>);
		storePC(pc_now, pc_store, odom_now);
	}
	else if(fields == field_list_[4]){
		pcl::PointCloud<pcl::PointNormal>::Ptr pc_now (new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointNormal>::Ptr pc_store (new pcl::PointCloud<pcl::PointNormal>);
		storeNC(pc_now, pc_store, odom_now);
	}
	else if(fields == field_list_[5]){
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZINormal>);
		storeNC(pc_now, pc_store, odom_now);
	}
	else{
		std::cout << "This point-type is not supported: fields = " << fields << std::endl;
		exit(1);
	}
}

template<typename CloudPtr>
void PcStore::storePC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now)
{
	/*ROS -> PCL*/
	pcl::fromROSMsg(pc_submsg_, *pc_now);
	pcl::fromROSMsg(pc_pubmsg_, *pc_store);
	/*prepare*/
	tf::Quaternion q_ori_now;
	tf::Quaternion q_ori_last;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_ori_now);
	quaternionMsgToTF(odom_last_.pose.pose.orientation, q_ori_last);
	tf::Quaternion q_rel_rot = q_ori_last*q_ori_now.inverse();
	q_rel_rot.normalize();	
	Eigen::Quaternionf rotation(q_rel_rot.w(), q_rel_rot.x(), q_rel_rot.y(), q_rel_rot.z());
	tf::Quaternion q_global_move(
		odom_last_.pose.pose.position.x - odom_now.pose.pose.position.x,
		odom_last_.pose.pose.position.y - odom_now.pose.pose.position.y,
		odom_last_.pose.pose.position.z - odom_now.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_ori_last.inverse()*q_global_move*q_ori_last;
	Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
	/*transform*/
	pcl::transformPointCloud(*pc_store, *pc_store, offset, rotation);
	/*store*/
	*pc_store  += *pc_now;
	pc_store->header.stamp = pc_now->header.stamp;
	/*erase*/
	if(max_num_buffering_scan_ > 0){
		num_scanpoints_list_.push_back(pc_now->points.size());
		erasePoints(pc_store);
	}
	/*downsampling*/
	if(downsampling_leafsize_ > 0)	downsampling(pc_store, pc_store->points[0]);
	/*PCL -> ROS*/
	pcl::toROSMsg(*pc_store, pc_pubmsg_);
}

template<typename CloudPtr>
void PcStore::storeNC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now)
{
	/*ROS -> PCL*/
	pcl::fromROSMsg(pc_submsg_, *pc_now);
	pcl::fromROSMsg(pc_pubmsg_, *pc_store);
	/*prepare*/
	tf::Quaternion q_ori_now;
	tf::Quaternion q_ori_last;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_ori_now);
	quaternionMsgToTF(odom_last_.pose.pose.orientation, q_ori_last);
	tf::Quaternion q_rel_rot = q_ori_last*q_ori_now.inverse();
	q_rel_rot.normalize();	
	Eigen::Quaternionf rotation(q_rel_rot.w(), q_rel_rot.x(), q_rel_rot.y(), q_rel_rot.z());
	tf::Quaternion q_global_move(
		odom_last_.pose.pose.position.x - odom_now.pose.pose.position.x,
		odom_last_.pose.pose.position.y - odom_now.pose.pose.position.y,
		odom_last_.pose.pose.position.z - odom_now.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_ori_last.inverse()*q_global_move*q_ori_last;
	Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
	/*transform*/
	pcl::transformPointCloudWithNormals(*pc_store, *pc_store, offset, rotation);
	/*store*/
	*pc_store  += *pc_now;
	pc_store->header.stamp = pc_now->header.stamp;
	/*erase*/
	if(max_num_buffering_scan_ > 0){
		num_scanpoints_list_.push_back(pc_now->points.size());
		erasePoints(pc_store);
	}
	/*downsampling*/
	if(downsampling_leafsize_ > 0)	downsampling(pc_store, pc_store->points[0]);
	/*convert*/
	pcl::toROSMsg(*pc_store, pc_pubmsg_);
}

template<typename CloudPtr, typename PointT>
void PcStore::downsampling(CloudPtr pc, PointT no_use)
{
	typedef pcl::PointCloud<PointT> Cloud;
	CloudPtr tmp(new Cloud);
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(pc);
	vg.setLeafSize(downsampling_leafsize_, downsampling_leafsize_, downsampling_leafsize_);
	vg.filter(*tmp);
	*pc = *tmp;
}

template<typename CloudPtr>
void PcStore::erasePoints(CloudPtr pc)
{
	if(downsampling_leafsize_ > 0){
		int limit_numpoints = num_scanpoints_list_[0]*max_num_buffering_scan_;
		int erase_numpoints = pc->points.size() - limit_numpoints;
		if(erase_numpoints > 0)	pc->points.erase(pc->points.begin(), pc->points.begin() + erase_numpoints);
		num_scanpoints_list_.erase(num_scanpoints_list_.begin());
		std::cout << "number of stored points: " << pc->points.size() << " / " << limit_numpoints << std::endl;
	}
	else{
		if(num_scanpoints_list_.size() > (size_t)max_num_buffering_scan_){
			pc->points.erase(pc->points.begin(), pc->points.begin() + num_scanpoints_list_[0]);
			num_scanpoints_list_.erase(num_scanpoints_list_.begin());
		}
		std::cout << "number of stored scans: " << num_scanpoints_list_.size() << " / " << max_num_buffering_scan_ << std::endl;
	}
	pc->width = pc->points.size();
	pc->height = 1;
}

void PcStore::publication(void)
{
	pub_pc_.publish(pc_pubmsg_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_store_with_odometry");
	
	PcStore pc_store_with_odometry;

	ros::spin();
}
