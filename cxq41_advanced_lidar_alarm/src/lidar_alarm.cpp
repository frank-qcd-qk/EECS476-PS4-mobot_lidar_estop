// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <std_srvs/Trigger.h>

const float ACCEPTABLE_SIDE_CLEARANCE = 0.25; //* Set the acceptable side clearance
const float ACCEPTABLE_forward_CLEARANCE = 2.0; //* Set the acceptable side clearance
float fovAngle = atan2f(ACCEPTABLE_SIDE_CLEARANCE,ACCEPTABLE_forward_CLEARANCE);

// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
float angle_min_=0.0;
float angle_max_=0.0;
float angle_increment_=0.0;
float range_min_ = 0.0;
float range_max_ = 0.0;
bool laser_alarm_=false;
bool laser_alarm_prev_state = false;

int rightSideIndex = -1;
int centerSideIndex = -1;
int leftSideIndex = -1;
int totalLaserPoints = -1;
int topRightIndex = -1;
int topLeftIndex = -1;

//! Emergency Stop Configuration
ros::ServiceClient estop_client_;
std_srvs::Trigger estop_srv_;
ros::ServiceClient clear_estop_client_;
std_srvs::Trigger clear_estop_srv_;

// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle
//! Added function for specifically checking if the future path is clear"
bool pathClearanceCheck(const sensor_msgs::LaserScan& laser_scan, float acceptableSideClearance, float acceptableForwardClearance){
    if(rightSideIndex == -1 || centerSideIndex==-1 || leftSideIndex==-1 || totalLaserPoints == -1){
        return true;
    }else
    {
        for(int i=0; i<=totalLaserPoints; i++){
            //* Data Aquisition
            float currentReadout = laser_scan.ranges[i];
            if(std::isinf(currentReadout) || std::isnan(currentReadout)){
                continue;
            }
            //ROS_INFO("-------------------------");
            //ROS_INFO("Current Raw Readout is %i:%f",i,currentReadout);
            //!Right
            //*Right_Back section
            if(i<rightSideIndex){
            //ROS_INFO("Right_Back");
                float currentAngle = angle_max_ - (float)(3.1415/2) - i*angle_increment_; //gives us angle for conversion
                currentAngle = fabs(currentAngle);
                float calculatedSideAcctual = cos(currentAngle) * currentReadout;
                //ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",currentAngle,calculatedSideAcctual);
                if(calculatedSideAcctual<acceptableSideClearance){
                    //ROS_WARN("Right_Back Warning! No Clearance!");
                    return(true);
                }
            }

            //*Right_front_bottom section
            if(i<topRightIndex && i>=rightSideIndex){
            //ROS_INFO("Right_front_bottom");
                float currentAngle = i*angle_increment_ - (angle_max_-(3.1415/2)); //gives us angle for conversion
                currentAngle = fabs(currentAngle);
                float calculatedSideAcctual = cos(currentAngle) * currentReadout;
                //ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",currentAngle,calculatedSideAcctual);
                if(calculatedSideAcctual<acceptableSideClearance){
                    //ROS_WARN("Right_front_bottom Warning! No Clearance!");
                    return(true);
                }
            }

            //*Right_front_top section
            if(i>=topRightIndex && i<=centerSideIndex){
            //ROS_INFO("Right_front_top");
                float currentAngle = angle_max_-i*angle_increment_; //gives us angle for conversion
                currentAngle = fabs(currentAngle);
                float calculatedSideAcctual = cos(currentAngle) * currentReadout;
                //ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",currentAngle,calculatedSideAcctual);
                if(calculatedSideAcctual<acceptableForwardClearance){
                    //ROS_WARN("Right_front_top Warning! No Clearance!");
                    return(true);
                }
            }
            //!Left
            //*Left_front_top section
            if(i>=centerSideIndex && i<=topLeftIndex){
            //ROS_INFO("Left_front_top");
                float currentAngle = i*angle_increment_-angle_max_; //gives us angle for conversion
                currentAngle = fabs(currentAngle);
                float calculatedSideAcctual = cos(currentAngle) * currentReadout;
                //ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",currentAngle,calculatedSideAcctual);
                if(calculatedSideAcctual<acceptableForwardClearance){
                    //ROS_WARN("Left_front_top Warning! No Clearance!");
                    return(true);
                }
            }

            //*left_front_bottom section
            if(i<=leftSideIndex && i>=topLeftIndex){
            //ROS_INFO("left_front_bottom");
                float currentAngle = (3.1415/2)-(fovAngle)-(i*angle_increment_-angle_max_-fovAngle); //gives us angle for conversion
                currentAngle = fabs(currentAngle);
                float calculatedSideAcctual = cos(currentAngle) * currentReadout;
                //ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",currentAngle,calculatedSideAcctual);
                if(calculatedSideAcctual<acceptableSideClearance){
                    //ROS_WARN("left_front_bottom Warning! No Clearance!");
                    return(true);
                }
            }
         
            //*Left_Back section
            if(i>leftSideIndex){
            //ROS_INFO("Left_Back");
                float currentAngle = i*angle_increment_-angle_max_-(3.1415/2); //gives us angle for conversion
                currentAngle = fabs(currentAngle);
                float calculatedSideAcctual = cos(currentAngle) * currentReadout;
                //ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",currentAngle,calculatedSideAcctual);
                if(calculatedSideAcctual<acceptableSideClearance){
                    //ROS_WARN("Left_Back Warning! No Clearance!");
                    return(true);
                }
            }
        }
        return(false);
    }
}


void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (centerSideIndex<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        //!Index calculation
        //*Total Laser Points
        totalLaserPoints = (int) ((angle_max_-angle_min_)/angle_increment_);
        ROS_INFO("TotalLaserPoints is: %i",totalLaserPoints);
        //*Center Point:
        centerSideIndex = (int) ((0.0 -angle_min_)/angle_increment_);        
        ROS_INFO("Calculated Center Index is: %i",centerSideIndex);
        //*Sharp Right and Sharp Left
        float rightAngle = angle_max_-(3.1415/2);
        float leftAngle = angle_min_+(3.1415/2);
        rightSideIndex = (int)(rightAngle/angle_increment_);
        leftSideIndex = totalLaserPoints + (int)(leftAngle/angle_increment_);
        ROS_INFO("Calculated: Right Index is: %i Left Index is: %i",rightSideIndex,leftSideIndex);
        //*Top Right and Top Left Calculation
        topRightIndex = centerSideIndex-((int)(fovAngle / angle_increment_));
        topLeftIndex = centerSideIndex+((int)(fovAngle / angle_increment_));
        ROS_INFO("Calculated: TOP Right Index is: %i TOP Left Index is: %i",topRightIndex,topLeftIndex);
    }
    //! Process laser
    laser_alarm_ = pathClearanceCheck(laser_scan,ACCEPTABLE_SIDE_CLEARANCE,ACCEPTABLE_forward_CLEARANCE);
    ROS_INFO("Laser Alarm is %i",laser_alarm_);
    if(laser_alarm_ && ! laser_alarm_prev_state){
        ROS_WARN("MAY DAY! MAY DAY! MAY DAY!");
        estop_client_.call(estop_srv_);
        ROS_WARN("E-Stopped.....");
    }else if(laser_alarm_prev_state && !laser_alarm_)
    {
        ROS_WARN("E-Stop Lifted!");
        clear_estop_client_.call(clear_estop_srv_);
    }
    laser_alarm_prev_state =laser_alarm_;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback); //Subscribe to Lidar Messages
    estop_client_ = nh.serviceClient<std_srvs::Trigger>("estop_service");
    clear_estop_client_ = nh.serviceClient<std_srvs::Trigger>("clear_estop_service");

    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

