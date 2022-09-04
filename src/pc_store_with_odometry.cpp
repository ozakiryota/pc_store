#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class PcStore{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber pc_sub_;
		ros::Subscriber odom_sub_;
		/*publisher*/
		ros::Publisher pc_pub_;
		/*pc*/
		sensor_msgs::PointCloud2 sub_pc_msg_;
		sensor_msgs::PointCloud2 pub_pc_msg_;
		/*odom*/
		nav_msgs::Odometry last_odom_;
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
		double passthrough_range_;

	public:
		PcStore();
		void listUpPointType(void);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void getOdomDiff(const nav_msgs::Odometry& curr_odom, Eigen::Quaternionf& rotation, Eigen::Vector3f& offset);
		void checkTypeAndStorePC(const Eigen::Quaternionf& rotation, const Eigen::Vector3f& offset);
		template<typename CloudPtr> void threadForPC(CloudPtr curr_pc, CloudPtr store_pc, const Eigen::Quaternionf& rotation, const Eigen::Vector3f& offset);
		template<typename CloudPtr> void threadForNC(CloudPtr curr_pc, CloudPtr store_pc, const Eigen::Quaternionf& rotation, const Eigen::Vector3f& offset);
		template<typename CloudPtr, typename PointT> void storePC(const CloudPtr curr_pc, const CloudPtr& store_pc, PointT no_use);
		void publishMsg(void);
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
	nh_private_.param("passthrough_range", passthrough_range_, -1.0);
	std::cout << "passthrough_range_ = " << passthrough_range_ << std::endl;
	/*subscriber*/
	pc_sub_ = nh_.subscribe("/point_cloud", 1, &PcStore::callbackPC, this);
	odom_sub_ = nh_.subscribe("/odom", 1, &PcStore::callbackOdom, this);
	/*publisher*/
	pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud/stored", 1);
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
		pub_pc_msg_ = *msg;
		got_first_pc_ = true;
		return;
	}

	sub_pc_msg_ = *msg;
	got_new_pc_ = true;
}

void PcStore::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(!got_first_odom_){
		last_odom_ = *msg;
		got_first_odom_ = true;
		return;
	}
	if(!got_new_pc_)	return;

	Eigen::Quaternionf rotation;
	Eigen::Vector3f offset;
	getOdomDiff(*msg, rotation, offset);

	checkTypeAndStorePC(rotation, offset);
	publishMsg();
	
	/*reset*/
	last_odom_ = *msg;
	got_new_pc_ = false;
}

void PcStore::getOdomDiff(const nav_msgs::Odometry& curr_odom, Eigen::Quaternionf& rotation, Eigen::Vector3f& offset)
{
	/*rotation*/
	tf::Quaternion q_curr_ori;
	tf::Quaternion q_last_ori;
	quaternionMsgToTF(curr_odom.pose.pose.orientation, q_curr_ori);
	quaternionMsgToTF(last_odom_.pose.pose.orientation, q_last_ori);
	tf::Quaternion q_rel_rot = q_last_ori*q_curr_ori.inverse();
	q_rel_rot.normalize();
	rotation = Eigen::Quaternionf(q_rel_rot.w(), q_rel_rot.x(), q_rel_rot.y(), q_rel_rot.z());

	/*offset*/
	tf::Quaternion q_global_move(
		last_odom_.pose.pose.position.x - curr_odom.pose.pose.position.x,
		last_odom_.pose.pose.position.y - curr_odom.pose.pose.position.y,
		last_odom_.pose.pose.position.z - curr_odom.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_last_ori.inverse()*q_global_move*q_last_ori;
	offset = Eigen::Vector3f(q_local_move.x(), q_local_move.y(), q_local_move.z());
}

void PcStore::checkTypeAndStorePC(const Eigen::Quaternionf& rotation, const Eigen::Vector3f& offset)
{
	std::string fields = pcl::getFieldsList(sub_pc_msg_);

	if(fields == field_list_[0]){
		pcl::PointCloud<pcl::PointXYZ>::Ptr curr_pc (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr store_pc (new pcl::PointCloud<pcl::PointXYZ>);
		threadForPC(curr_pc, store_pc, rotation, offset);
	}
	else if(fields == field_list_[1] || fields == field_list_[2]){
		pcl::PointCloud<pcl::PointXYZI>::Ptr curr_pc (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr store_pc (new pcl::PointCloud<pcl::PointXYZI>);
		threadForPC(curr_pc, store_pc, rotation, offset);
	}
	else if(fields == field_list_[3]){
		pcl::PointCloud<pcl::InterestPoint>::Ptr curr_pc (new pcl::PointCloud<pcl::InterestPoint>);
		pcl::PointCloud<pcl::InterestPoint>::Ptr store_pc (new pcl::PointCloud<pcl::InterestPoint>);
		threadForPC(curr_pc, store_pc, rotation, offset);
	}
	else if(fields == field_list_[4]){
		pcl::PointCloud<pcl::PointNormal>::Ptr curr_pc (new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointNormal>::Ptr store_pc (new pcl::PointCloud<pcl::PointNormal>);
		threadForNC(curr_pc, store_pc, rotation, offset);
	}
	else if(fields == field_list_[5]){
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr curr_pc (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr store_pc (new pcl::PointCloud<pcl::PointXYZINormal>);
		threadForNC(curr_pc, store_pc, rotation, offset);
	}
	else{
		std::cout << "This point-type is not supported: fields = " << fields << std::endl;
		exit(1);
	}
}

template<typename CloudPtr>
void PcStore::threadForPC(CloudPtr curr_pc, CloudPtr store_pc, const Eigen::Quaternionf& rotation, const Eigen::Vector3f& offset)
{
	/*ROS -> PCL*/
	pcl::fromROSMsg(sub_pc_msg_, *curr_pc);
	pcl::fromROSMsg(pub_pc_msg_, *store_pc);
	/*transform*/
	pcl::transformPointCloud(*store_pc, *store_pc, offset, rotation);
	/*store*/
	storePC(curr_pc, store_pc, store_pc->points[0]);
}

template<typename CloudPtr>
void PcStore::threadForNC(CloudPtr curr_pc, CloudPtr store_pc, const Eigen::Quaternionf& rotation, const Eigen::Vector3f& offset)
{
	/*ROS -> PCL*/
	pcl::fromROSMsg(sub_pc_msg_, *curr_pc);
	pcl::fromROSMsg(pub_pc_msg_, *store_pc);
	/*transform*/
	pcl::transformPointCloudWithNormals(*store_pc, *store_pc, offset, rotation);
	/*store*/
	storePC(curr_pc, store_pc, store_pc->points[0]);
}

template<typename CloudPtr, typename PointT>
void PcStore::storePC(const CloudPtr curr_pc, const CloudPtr& store_pc, PointT no_use)
{
	/*add*/
	*store_pc  += *curr_pc;
	store_pc->header.stamp = curr_pc->header.stamp;
	/*downsampling*/
	if(downsampling_leafsize_ > 0){
		pcl::VoxelGrid<PointT> vg;
		vg.setInputCloud(store_pc);
		vg.setLeafSize(downsampling_leafsize_, downsampling_leafsize_, downsampling_leafsize_);
		vg.filter(*store_pc);
	}
	else if(max_num_buffering_scan_ > 0){
		num_scanpoints_list_.push_back(curr_pc->points.size());
		if(num_scanpoints_list_.size() > (size_t)max_num_buffering_scan_){
			store_pc->points.erase(store_pc->points.begin(), store_pc->points.begin() + num_scanpoints_list_[0]);
			num_scanpoints_list_.erase(num_scanpoints_list_.begin());
			store_pc->width = store_pc->points.size();
			store_pc->height = 1;
		}
		std::cout << "number of stored scans: " << num_scanpoints_list_.size() << " / " << max_num_buffering_scan_ << std::endl;
	}
	/*filter*/
	if(passthrough_range_ > 0){
		pcl::PassThrough<PointT> pt;
		/*x*/
		pt.setInputCloud(store_pc);
		pt.setFilterFieldName("x");
		pt.setFilterLimits(-passthrough_range_, passthrough_range_);
		pt.filter(*store_pc);
		/*y*/
		pt.setInputCloud(store_pc);
		pt.setFilterFieldName("y");
		pt.setFilterLimits(-passthrough_range_, passthrough_range_);
		pt.filter(*store_pc);
		/*z*/
		pt.setInputCloud(store_pc);
		pt.setFilterFieldName("z");
		pt.setFilterLimits(-passthrough_range_, passthrough_range_);
		pt.filter(*store_pc);
	}
	/*convert*/
	pcl::toROSMsg(*store_pc, pub_pc_msg_);
}

void PcStore::publishMsg(void)
{
	pc_pub_.publish(pub_pc_msg_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_store_with_odometry");
	
	PcStore pc_store_with_odometry;

	ros::spin();
}
