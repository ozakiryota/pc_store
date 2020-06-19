#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <pcl/common/transforms.h>

// template<typename PointT>
class PCStore{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscribe*/
		ros::Subscriber _sub_pc;
		ros::Subscriber _sub_odom;
		/*publish*/
		ros::Publisher _pub_pc;
		/*pc*/
		// std::vector<sensor_msg::PointCloud2> _pc_record;
		sensor_msgs::PointCloud2 _pc_submsg;
		sensor_msgs::PointCloud2 _pc_pubmsg;
		/*odom*/
		nav_msgs::Odometry _odom_last;
		/*list*/
		std::vector<std::string> _list_fields;
		std::vector<size_t> _list_num_scanpoints;
		/*flag*/
		bool _got_first_pc = false;
		bool _got_new_pc = false;
		bool _got_first_odom = false;
		/*parameter*/
		int _scan_limit;

	public:
		PCStore();
		void listUpPointType(void);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void checkTypeAndStorePC(nav_msgs::Odometry odom_now);
		template<typename CloudPtr> void storePC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now);
		template<typename CloudPtr> void storeNC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now);
		template<typename CloudPtr> void erasePoints(CloudPtr pc);
		void publication(void);
};

PCStore::PCStore()
	: _nhPrivate("~")
{
	/*parameter*/
	_nhPrivate.param("scan_limit", _scan_limit, -1);
	std::cout << "_scan_limit = " << _scan_limit << std::endl;
	/*subscriber*/
	_sub_pc = _nh.subscribe("/cloud", 1, &PCStore::callbackPC, this);
	_sub_odom = _nh.subscribe("/odom", 1, &PCStore::callbackOdom, this);
	/*publisher*/
	_pub_pc = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/stored", 1);
	/*initialize*/
	listUpPointType();
}

void PCStore::listUpPointType(void)
{
	_list_fields = {
		"x y z",
		"x y z intensity",
		"x y z intensity ring",	//velodyne
		"x y z strength",
		"x y z normal_x normal_y normal_z curvature",
		"x y z intensity normal_x normal_y normal_z curvature"
	};
}

void PCStore::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if(!_got_first_pc){
		_pc_pubmsg = *msg;
		_got_first_pc = true;
		return;
	}

	_pc_submsg = *msg;
	_got_new_pc = true;
}

void PCStore::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(!_got_first_odom){
		_odom_last = *msg;
		_got_first_odom = true;
		return;
	}
	if(!_got_new_pc)	return;

	checkTypeAndStorePC(*msg);
	/*publication*/
	publication();
	/*reset*/
	_odom_last = *msg;
	_got_new_pc = false;
}

void PCStore::checkTypeAndStorePC(nav_msgs::Odometry odom_now)
{
	std::string fields = pcl::getFieldsList(_pc_submsg);

	if(fields == _list_fields[0]){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(_pc_submsg, *pc_now);
		pcl::fromROSMsg(_pc_pubmsg, *pc_store);
		storePC(pc_now, pc_store, odom_now);
	}
	else if(fields == _list_fields[1] || fields == _list_fields[2]){
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(_pc_submsg, *pc_now);
		pcl::fromROSMsg(_pc_pubmsg, *pc_store);
		storePC(pc_now, pc_store, odom_now);
	}
	else if(fields == _list_fields[3]){
	}
	else if(fields == _list_fields[4]){
		pcl::PointCloud<pcl::PointNormal>::Ptr pc_now (new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointNormal>::Ptr pc_store (new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromROSMsg(_pc_submsg, *pc_now);
		pcl::fromROSMsg(_pc_pubmsg, *pc_store);
		storeNC(pc_now, pc_store, odom_now);
	}
	else if(fields == _list_fields[5]){
	}
	else{
		std::cout << "This point-type is not supported: fields = " << fields << std::endl;
		exit(1);
	}
}

template<typename CloudPtr>
void PCStore::storePC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now)
{
	/*prepare*/
	tf::Quaternion q_ori_now;
	tf::Quaternion q_ori_last;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_ori_now);
	quaternionMsgToTF(_odom_last.pose.pose.orientation, q_ori_last);
	tf::Quaternion q_rel_rot = q_ori_last*q_ori_now.inverse();
	q_rel_rot.normalize();	
	Eigen::Quaternionf rotation(q_rel_rot.w(), q_rel_rot.x(), q_rel_rot.y(), q_rel_rot.z());
	tf::Quaternion q_global_move(
		_odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
		_odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
		_odom_last.pose.pose.position.z - odom_now.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_ori_last.inverse()*q_global_move*q_ori_last;
	Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
	/*transform*/
	pcl::transformPointCloud(*pc_store, *pc_store, offset, rotation);
	/*store*/
	*pc_store  += *pc_now;
	pc_store->header.stamp = pc_now->header.stamp;
	_list_num_scanpoints.push_back(pc_now->points.size());
	/*erase*/
	if(_scan_limit > 0)	erasePoints(pc_store);
	/*convert*/
	pcl::toROSMsg(*pc_store, _pc_pubmsg);
}

template<typename CloudPtr>
void PCStore::storeNC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now)
{
	/*prepare*/
	tf::Quaternion q_ori_now;
	tf::Quaternion q_ori_last;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_ori_now);
	quaternionMsgToTF(_odom_last.pose.pose.orientation, q_ori_last);
	tf::Quaternion q_rel_rot = q_ori_last*q_ori_now.inverse();
	q_rel_rot.normalize();	
	Eigen::Quaternionf rotation(q_rel_rot.w(), q_rel_rot.x(), q_rel_rot.y(), q_rel_rot.z());
	tf::Quaternion q_global_move(
		_odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
		_odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
		_odom_last.pose.pose.position.z - odom_now.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_ori_last.inverse()*q_global_move*q_ori_last;
	Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
	/*transform*/
	pcl::transformPointCloudWithNormals(*pc_store, *pc_store, offset, rotation);
	/*store*/
	*pc_store  += *pc_now;
	pc_store->header.stamp = pc_now->header.stamp;
	_list_num_scanpoints.push_back(pc_now->points.size());
	/*erase*/
	if(_scan_limit > 0)	erasePoints(pc_store);
	/*convert*/
	pcl::toROSMsg(*pc_store, _pc_pubmsg);
}

template<typename CloudPtr>
void PCStore::erasePoints(CloudPtr pc)
{
	if(_list_num_scanpoints.size() > (size_t)_scan_limit){
		pc->points.erase(pc->points.begin(), pc->points.begin() + _list_num_scanpoints[0]);
		_list_num_scanpoints.erase(_list_num_scanpoints.begin());
	}
	pc->width = pc->points.size();
	pc->height = 1;

	std::cout << "limit storing: true" << std::endl;
	std::cout << "number of stored scans: " << _list_num_scanpoints.size() << std::endl;
}

void PCStore::publication(void)
{
	_pub_pc.publish(_pc_pubmsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_store_with_odometry");
	
	PCStore pc_store_with_odometry;

	ros::spin();
}