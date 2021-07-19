/**
 * Servono due subscriber e un publisher. L'utente scrive sul topic cmd_vel_user il comando di velocità
 * che vuole mandare al robot, nel main prendiamo con i due subscriber il comando di velocità e i dati del laserscan per
 * scrivere in una struct il comando di velocità effettivo da mandare al robot. Il publisher prende questo dato
 * e lo manda al robot sul topic cmd_vel che non lo manda a sbattere.
 */ 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h" 
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/message_filter.h" 
#include "message_filters/subscriber.h" 

float vel_user_x, vel_user_y, obstacle_x = 0, obstacle_y = 0; //variabili globali che servono al publisher //
laser_geometry::LaserProjection projector;
tf::StampedTransform obstacle;
tf::StampedTransform listener;
Eigen::Vector2f p;

void cmd_vel_user_callback(const geometry_msgs::Twist::ConstPtr& msg){
    vel_user_x = msg.linear.x;
    vel_user_y = msg.linear.y;
}
void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud cloud;
    try{
        
        // Get laser transform in odom frame using the tf listener
        projector.transformLaserScanToPointCloud("base_link",*scan, cloud,listener);
        listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10,0));
        listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), obstacle);
        Eigen::Isometry2f T = convertPose2D(obstacle);
        
        for(auto& point : cloud->points){//modifica usa Eigen e matrice isometria
            float norm = Math.sqrt(xi*xi + yi*yi);
            p(0) = point.x;
            p(1) = point.y;
            p = T * p; //ostacolo nel sistema di riferimento del robot
            obstacle_x += p(0)/norm;//due divisioni per norm?
            obstacle_y += p(1)/norm;
        }
    }
    catch (tf::TransformException& e){
        std::cout << e.what();
        ros::Duration(1.0).sleep();
        return;
    }    
}

int main(int argc, char **argv){
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "listener");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber sub_vel = n.subscribe("cmd_vel_user", 1000, cmd_vel_user_callback);
    ros::Subscriber sub_laser = n.subscribe("laser_scan", 1000, laser_scan_callback);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        //std_msgs::String msg;
        geometry_msgs::Twist msg;
        float vel_robot = vel_user + vel_obstacle;
        //msg.linear = vel_robot;

        
        pub_vel.publish(msg);

        //ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}