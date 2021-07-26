/**
 * Servono due subscriber e un publisher. L'utente scrive sul topic /cmd_vel_user il comando di velocità
 * che vuole mandare al robot, nel main prendiamo con i due subscriber il comando di velocità e i dati del laserscan per
 * scrivere in un twist il comando di velocità effettivo da mandare al robot. Il publisher manda il twist al robot 
 * sul topic cmd_vel che non lo manda a sbattere.
 */ 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h" 
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/message_filter.h" 
#include "message_filters/subscriber.h" 
#include <Eigen/Geometry>
#include <cmath>

float vel_user_x = 0, vel_user_y = 0, obstacle_x = 0, obstacle_y = 0; //variabili globali che servono al publisher 
float angular_z = 0;
Eigen::Isometry2f convertPose2D(const tf::StampedTransform& t) {
    double yaw,pitch,roll;
    tf::Matrix3x3 mat =  t.getBasis();
    mat.getRPY(roll, pitch, yaw);
    Eigen::Isometry2f T;
    T.setIdentity();
    Eigen::Matrix2f R;
    R << std::cos(yaw), -std::sin(yaw),
        std::sin(yaw), std::cos(yaw);
    T.linear() = R;
    T.translation() = Eigen::Vector2f(t.getOrigin().x(), t.getOrigin().y());
    return T;
}

void cmdveluserCallback(const geometry_msgs::Twist::ConstPtr& msg){//funzione di callback del subscriber per il comando di velocità dell'utente
    vel_user_x = msg->linear.x;
    vel_user_y = msg->linear.y;
    angular_z = msg -> angular.z;
    std::cerr << "vel_user_x: " << vel_user_x << std::endl;
    std::cerr << "vel_user_y: " << vel_user_y << std::endl;
    std::cerr << "angular_z: " << angular_z << std::endl;
    return;
}
void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){//funzione di callback del subscriber per il laserscan
    try{
        sensor_msgs::PointCloud cloud;
        laser_geometry::LaserProjection projector;
        Eigen::Vector2f p;
        tf::StampedTransform obstacle;
        tf::TransformListener listener;
        
        projector.transformLaserScanToPointCloud("base_laser_link",*scan, cloud, listener);
        listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10,0));
        listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), obstacle);
        Eigen::Isometry2f T = convertPose2D(obstacle); //matrice per trasformare le coordinate degli ostacoli 
                                                       //(dal sistema di riferimento del laser a quello del robot)
        
        for(auto& point : cloud.points){//itero sugli ostacoli per calcolare il loro contributo sulla forza risultante 
            float norm = sqrt(point.x*point.x + point.y*point.y);
            p(0) = point.x;
            p(1) = point.y;
            p = T * p; //ostacolo nel sistema di riferimento del robot
            obstacle_x += (p(0)/norm)/norm;
            obstacle_y += (p(1)/norm)/norm;
        }
        return;
    }
    catch (tf::TransformException& e){
        std::cout << e.what();
        ros::Duration(1.0).sleep();
        return;
    }    
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "progetto");

    ros::NodeHandle n;
    ros::Subscriber sub_vel = n.subscribe("/cmd_vel_user", 1000, cmdveluserCallback);
    ros::Subscriber sub_laser = n.subscribe("/base_scan", 1, laserscanCallback);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()){
        ros::spinOnce();
        geometry_msgs::Twist msg;
        if(vel_user_x == 0 && vel_user_y == 0 && angular_z == 0){
            continue;
        }
        /*
        std::cerr << "velocità utente x: " << vel_user_x << std::endl;
        std::cerr << "velocità utente y: " << vel_user_y << std::endl;*/
        float rot = obstacle_y/5000;
        if(angular_z && vel_user_x == 0){
            obstacle_x = (obstacle_x/800);
        }
        else if(angular_z){
            obstacle_x = (obstacle_x/800)*vel_user_x; 
        }
        else{
            obstacle_x = (obstacle_x/5000)*vel_user_x; 
        }
        if(angular_z && vel_user_y == 0){
            obstacle_y = obstacle_y/800;
        }
        else if(angular_z){
            obstacle_y = (obstacle_y/800)*vel_user_y;
        }
        else{
            obstacle_y = rot*vel_user_y;
        }
        std::cerr << "ostacoli x: " << obstacle_x << std::endl;
        std::cerr << "ostacoli y: " << obstacle_y << std::endl;
        float vel_out_x = vel_user_x - obstacle_x;
        float vel_out_y = vel_user_y - obstacle_y;
        //velocità effettiva da scrivere sul topic /cmd_vel
        msg.linear.x = vel_out_x;
        msg.linear.y = vel_out_y;
        //per far roteare il robot in prossimità di un ostacolo
        std::cerr << "rot: " << rot << std::endl;
        if(count){
            if(abs(rot) < 0.5)
                msg.angular.z = (sqrt(vel_user_x*vel_user_x + vel_user_y*vel_user_y)*obstacle_x)/10 + angular_z;
            else
                msg.angular.z = (sqrt(vel_user_x*vel_user_x + vel_user_y*vel_user_y)*obstacle_x) + angular_z;
        }
        //msg.angular.z = (sqrt(vel_user_x*vel_user_x + vel_user_y*vel_user_y)*rot)/4;
        pub_vel.publish(msg);
        vel_user_x = 0;
        vel_user_y = 0;
        obstacle_x = 0;
        obstacle_y = 0;
        angular_z = 0;
        rot = 0;
        //std::cerr << "cmd vel aggiornato: " <<msg << std::endl;
        loop_rate.sleep();
        ++count;   
    }
    ros::spin();

    return 0;
}