#include <ros/ros.h>
#include <tf/transform_listener.h>
using namespace std;
int main(int argc, char** argv){
ros::init(argc, argv, "robot_location");
ros::NodeHandle node;
tf::TransformListener listener;
ros::Rate rate(2.0);
listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), 
ros::Duration(10.0));
while (ros::ok()){
tf::StampedTransform transform;
try {   
int i=0;         
listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), 
transform);
while(i<1){
double x = transform.getOrigin().x();
double y = transform.getOrigin().y();
cout << "Current position: (" << x << "," << y << ")" << endl;
i++;
}
} catch (tf::TransformException &ex) {
ROS_ERROR("%s",ex.what());
}
rate.sleep();
}
return 0;
}
