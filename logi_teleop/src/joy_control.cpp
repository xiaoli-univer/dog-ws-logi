#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


geometry_msgs::Twist twist;
float LinearMax,AngularMax;
float LinearMax_origin,AngularMax_origin;
int forward, turn, add_vel, dec_vel, add_ang, dec_ang, reset;

const char* msg = R"(
Reading from the Logitech joy and Publishing to Twist!
---------------------------
Moving around:
   Left axis:   Front & Back
   Right axis:  Turn left & Turn right
---------------------------
Y/A : increase/decrease only linear speed by 20%
B/X : increase/decrease only angular speed by 20%
CTRL-C to quit
)";

void callback(const sensor_msgs::Joy::ConstPtr &data)
{
    // Flight mode, left -> linear, right -> angular
    twist.linear.x = data->axes[forward]*LinearMax;
    twist.angular.z = data->axes[turn]*AngularMax;
    // Set the LinearMax and AngularMax
    LinearMax += data->buttons[add_vel]*0.1;
    LinearMax -= data->buttons[dec_vel]*0.1;
    AngularMax += data->buttons[add_ang]*0.1;
    AngularMax -= data->buttons[dec_ang]*0.1;
    if(data->buttons[reset]==1)
    {
        LinearMax = LinearMax_origin;
        AngularMax = AngularMax_origin;        
    }
}

int main(int argc,char** argv)
{
// Init rosnode
    ros::init(argc,argv,"joyControl");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    ros::Subscriber joysub = nh.subscribe( "joy",10,callback);

// set param of maximum of linear and angular vel
    ros::param::param<float>("~/MaxLinear",LinearMax,0.5);  //* Deliver param by param server
    ros::param::param<float>("~/MaxAngular",AngularMax,0.5); //* Default value: Linear(0.5)  Angular(0.5)
    ros::param::param<int>("~/forward",forward,1); 
    ros::param::param<int>("~/turn",turn,3); 
    ros::param::param<int>("~/add_vel",add_vel,3); 
    ros::param::param<int>("~/dec_vel",dec_vel,0); 
    ros::param::param<int>("~/add_ang",add_ang,1); 
    ros::param::param<int>("~/dec_ang",dec_ang,2);
    ros::param::param<int>("~/reset",reset,7);

// Save the two param to init
    LinearMax_origin = LinearMax;
    AngularMax_origin = AngularMax;
// print msg
    printf("%s", msg);
    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        cmd_pub.publish(twist);
        printf("Current: MaxSpeed %f    MaxTurn %f \r",LinearMax,AngularMax);
        r.sleep();
    }
    return 0;
}