#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LowerPCHz{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscribe*/
		ros::Subscriber _sub_pc;
		/*publish*/
		ros::Publisher _pub_pc;
		/*pc*/
		sensor_msgs::PointCloud2 _pc_submsg;
		sensor_msgs::PointCloud2 _pc_pubmsg;
		/*list*/
		std::vector<std::string> _list_fields;
		/*flag*/
		bool _got_first_pc = false;
		/*parameters*/
		double _target_hz;

	public:
		LowerPCHz();
		void listUpPointType(void);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		bool isOverTargetHz(void);
		void checkTypeAndConvert(void);
		template<typename CloudPtr> void aggregatePC(CloudPtr pc_now, CloudPtr pc_store);
		void publication(void);
};

LowerPCHz::LowerPCHz()
	: _nhPrivate("~")
{
	std::cout << "--- lower_pc_hz ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("target_hz", _target_hz, 20.0);
	std::cout << "_target_hz = " << _target_hz << std::endl;
	/*subscriber*/
	_sub_pc = _nh.subscribe("/cloud", 1, &LowerPCHz::callbackPC, this);
	/*publisher*/
	_pub_pc = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/lower_hz", 1);
	/*initialize*/
	listUpPointType();
}

void LowerPCHz::listUpPointType(void)
{
	_list_fields = {
		"x y z",
		"x y z intensity",
		"x y z intensity ring"	//velodyne
		// "x y z strength",
		// "x y z normal_x normal_y normal_z curvature",
		// "x y z intensity normal_x normal_y normal_z curvature"
	};
}

void LowerPCHz::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if(!_got_first_pc){
		_pc_pubmsg = *msg;
		_got_first_pc = true;
		return;
	}

	_pc_submsg = *msg;

	if(isOverTargetHz()){
		publication();
		_pc_pubmsg = _pc_submsg;
	}
	else{
		checkTypeAndConvert();
	}
}

bool LowerPCHz::isOverTargetHz(void)
{
	double duration = (_pc_submsg.header.stamp - _pc_pubmsg.header.stamp).toSec();
	double hz = 1.0/duration;
	if(hz > _target_hz)	return true;
	else	return false;
}

void LowerPCHz::checkTypeAndConvert(void)
{
	std::string fields = pcl::getFieldsList(_pc_submsg);

	if(fields == _list_fields[0]){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZ>);
		aggregatePC(pc_now, pc_store);
	}
	else if(fields == _list_fields[1] || fields == _list_fields[2]){
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZI>);
		aggregatePC(pc_now, pc_store);
	}
	else{
		std::cout << "This point-type is not supported: fields = " << fields << std::endl;
		exit(1);
	}
}

template<typename CloudPtr>
void LowerPCHz::aggregatePC(CloudPtr pc_now, CloudPtr pc_store)
{
	pcl::fromROSMsg(_pc_submsg, *pc_now);
	pcl::fromROSMsg(_pc_pubmsg, *pc_store);

	*pc_store += *pc_now;

	pcl::toROSMsg(*pc_store, _pc_pubmsg);	
}

void LowerPCHz::publication(void)
{
	_pub_pc.publish(_pc_pubmsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lower_pc_hz");
	
	LowerPCHz lower_pc_hz;

	ros::spin();
}
