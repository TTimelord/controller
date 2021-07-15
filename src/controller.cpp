#include <signal.h>
#include <std_msgs/String.h>
#include "ros/ros.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/Int32Stamped.h>
 
using namespace std;
#define TIME_STEP 8    //时钟
#define NMOTORS 20       //电机数量
#define robot_unique_name "/THMOS/"   //名字
 
ros::NodeHandle *n;

static int controllerCount;
static vector<string> controllerList; 

ros::ServiceClient timeStepClient;          //时钟通讯客户端
webots_ros::set_int timeStepSrv;            //时钟服务数据

ros::ServiceClient set_velocity_client[NMOTORS];     //速度设置客户端
webots_ros::set_float set_velocity_srv;     //速度设置数据

ros::ServiceClient set_position_client[NMOTORS];     //位置设置客户端
webots_ros::set_float set_position_srv;     //位置设置数据

bool turnLeft = true; 



double speeds[NMOTORS]={0.0};         //电机速度值
double positions[NMOTORS]={0.0};         //电机位置值

// 匹配电机名
static const char* motorNames[NMOTORS] ={
  "neck", "head", "L_arm_1", "L_arm_2", "L_arm_3", "R_arm_1", "R_arm_2", "R_arm_3",
  "L_leg_1", "L_leg_2", "L_leg_3", "L_leg_4", "L_leg_5", "L_leg_6",  
  "R_leg_1", "R_leg_2", "R_leg_3", "R_leg_4", "R_leg_5", "R_leg_6"
};

static const char* positionSensorNames[NMOTORS] ={
  "neck_sensor", "head_sensor", "L_arm_1_sensor", "L_arm_2_sensor", "L_arm_3_sensor", "R_arm_1_sensor", "R_arm_2_sensor", "R_arm_3_sensor",
  "L_leg_1_sensor", "L_leg_2_sensor", "L_leg_3_sensor", "L_leg_4_sensor", "L_leg_5_sensor", "L_leg_6_sensor",  
  "R_leg_1_sensor", "R_leg_2_sensor", "R_leg_3_sensor", "R_leg_4_sensor", "R_leg_5_sensor", "R_leg_6_sensor"
};

/*******************************************************
* Function name ：updateSpeed
* Description   ：将速度请求以set_float的形式发送给set_velocity_srv
* Parameter     ：无
* Return        ：无
**********************************************************/
void updateSpeed() {   
    for (int i = 0; i < NMOTORS; ++i) {
        // 更新速度
        set_velocity_srv.request.value = -speeds[i];
        set_velocity_client[i].call(set_velocity_srv);
    }
}

/*******************************************************
* Function name ：updatePosition
* Description   ：将位置请求以set_float的形式发送给set_velocity_srv
* Parameter     ：无
* Return        ：无
**********************************************************/
void updatePosition() {   
    for (int i = 0; i < NMOTORS; ++i) {
        // 更新位置
        set_position_srv.request.value = positions[i];
        set_position_client[i].call(set_position_srv);
    }
}

/*******************************************************
* Function name ：controllerNameCallback
* Description   ：控制器名回调函数，获取当前ROS存在的机器人控制器
* Parameter     ：
        @name   控制器名
* Return        ：无
**********************************************************/
void controllerNameCallback(const std_msgs::String::ConstPtr &name) { 
    controllerCount++; 
    controllerList.push_back(name->data);//将控制器名加入到列表中
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());

}
/*******************************************************
* Function name ：quit
* Description   ：退出函数
* Parameter     ：
        @sig   信号
* Return        ：无
**********************************************************/
void quit(int sig) {
    ROS_INFO("User stopped the node.");
    timeStepSrv.request.value = 0; 
    timeStepClient.call(timeStepSrv); 
    ros::shutdown();
    exit(0);
}

void timerCallback(const ros::TimerEvent&){
    if(turnLeft){
        positions[0] += 0.2;
        updatePosition();
        std_msgs::String msg;
        std::stringstream ss;
        ss << "turning left, position = " << positions[0];
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        if(positions[0] > 2.0)
            turnLeft = false;
    }
    else{
        positions[0] -= 0.2;
        updatePosition();
        ROS_INFO("turning right");
        if(positions[0] < -2.0)
            turnLeft = true;
    }
}


/*******************************************************
* Function name ：键盘返回函数
* Description   ：当键盘动作，就会进入此函数内
* Parameter     ：
        @value   返回的值
* Return        ：无
**********************************************************/
// void keyboardDataCallback(const webots_ros::Int32Stamped::ConstPtr &value)
// {
//     // 发送控制变量
//     int send =0;
//     ROS_INFO("sub keyboard value = %d",value->data);
//     switch (value->data)
//     {
//         // 左转
//         case 65:
//             if(positions[0] < 3.14)
//             {
//                 positions[0] += 0.1;    
//             }
//             send = 1;
//             ROS_INFO("turn left");
//             break;
//         // 抬头
//         case 87:
//             positions[1] += 0.1;
//             send = 1;
//             ROS_INFO("turn up");
//             break;
//         // 右转
//         case 68:
//             if(positions[0] > -3.14)
//             {
//                 positions[0] -= 0.1;    
//             }
//             send = 1;
//             ROS_INFO("turn right");
//             break;
//         // 低头
//         case 83:
//             positions[1] += 0.1;
//             send = 1;
//             ROS_INFO("turn down");
//             break;
//         // 停止
//         case 32:
//             positions[0] = 0;
//             positions[1] = 0;
//             send = 1;
//             break;
//         default:
//             send = 0; 
//             break;
//     }
//     //当接收到信息时才会更新positions
//     if (send)
//     {
//         updatePosition();
//         send=0;
//     } 
// }

int main(int argc, char **argv) {
    setlocale(LC_ALL, ""); // 用于显示中文字符
    string controllerName;
    // 在ROS网络中创建一个名为robot_init的节点
    ros::init(argc, argv, "robot_init", ros::init_options::AnonymousName);
    n = new ros::NodeHandle;
    // 截取退出信号
    signal(SIGINT, quit);

    // 订阅webots中所有可用的model_name
    ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }
    ros::spinOnce();
    // 服务订阅time_step和webots保持同步
    timeStepClient = n->serviceClient<webots_ros::set_int>(string(robot_unique_name) + "robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

    // 如果在webots中有多个控制器的话，需要让用户选择一个控制器
    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        cout << "Choose the # of the controller you want to use:\n";
        cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
        controllerName = controllerList[wantedController - 1];
        else {
        ROS_ERROR("Invalid number for controller choice.");
        return 1;
        }
    }
    ROS_INFO("Using controller: '%s'", controllerName.c_str());
    // 退出主题，因为已经不重要了
    nameSub.shutdown();

    //初始化电机
    for (int i = 0; i < NMOTORS; ++i) {
        // position速度控制时设置为缺省值INFINITY   
        // set_position_client = n->serviceClient<webots_ros::set_float>(string(robot_unique_name) + string(motorNames[i]) + string("/set_position"));   
        // set_position_srv.request.value = INFINITY;
        // if (set_position_client.call(set_position_srv) && set_position_srv.response.success)     
        //     ROS_INFO("Position set to INFINITY for motor %s.", motorNames[i]);   
        // else     
        //     ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);
        // position设置为0
        set_position_client[i] = n->serviceClient<webots_ros::set_float>(string(robot_unique_name) + string(motorNames[i]) + string("/set_position"));   
        set_position_srv.request.value = 0.0;
        if (set_position_client[i].call(set_position_srv) && set_position_srv.response.success)     
            ROS_INFO("Position set to 0 for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_position on motor %s.", motorNames[i]);
        
        // velocity初始速度设置为2.0   
        set_velocity_client[i] = n->serviceClient<webots_ros::set_float>(string(robot_unique_name) + string(motorNames[i]) + string("/set_velocity"));   
        set_velocity_srv.request.value = 2.0;   
        if (set_velocity_client[i].call(set_velocity_srv) && set_velocity_srv.response.success == 1)     
            ROS_INFO("Velocity set to 2.0 for motor %s.", motorNames[i]);   
        else     
            ROS_ERROR("Failed to call service set_velocity on motor %s.", motorNames[i]);
    }   

     // 使能 camera
    ros::ServiceClient set_camera_client;
    webots_ros::set_int camera_srv;
    set_camera_client = n->serviceClient<webots_ros::set_int>(string(robot_unique_name)+"Camera/enable");
    camera_srv.request.value = 4 * TIME_STEP;
    if (set_camera_client.call(camera_srv) && camera_srv.response.success == 1)     
        ROS_INFO("camera set sucsessfully");   
    else     
        ROS_ERROR("Failed to set camera");
    
    // 使能position sensor
    // ros::ServiceClient set_positionSensor_client;
    // webots_ros::set_int positionSensor_srv;
    // set_positionSensor_client = n->serviceClient<webots_ros::set_int>(string(robot_unique_name)+motorNames[i]+"/enable");
    // camera_srv.request.value = TIME_STEP;
    // if (set_camera_client.call(camera_srv) && camera_srv.response.success == 1)     
    //     ROS_INFO("camera set sucsessfully");   
    // else     
    //     ROS_ERROR("Failed to set camera");


    
    // 服务订阅键盘
    ros::ServiceClient keyboardEnableClient;
    webots_ros::set_int keyboardEnablesrv;
   
    // keyboardEnableClient = n->serviceClient<webots_ros::set_int>(string(robot_unique_name)+"/keyboard/enable");
    // keyboardEnablesrv.request.value = TIME_STEP;
    // if (keyboardEnableClient.call(keyboardEnablesrv) && keyboardEnablesrv.response.success)
    // {
    //     ros::Subscriber keyboardSub;
    //     keyboardSub = n->subscribe(string(robot_unique_name) + "/keyboard/key",1,keyboardDataCallback);
    //     while (keyboardSub.getNumPublishers() == 0) {}
    //     ROS_INFO("Keyboard enabled.");
    //     ROS_INFO("control head：");
    //     ROS_INFO("  ↑  ");
    //     ROS_INFO("← ↓ →");
    //     ROS_INFO("stop：space");
    //     ROS_INFO("Use the arrows in Webots window to move the robot.");
    //     ROS_INFO("Press the End key to stop the node.");
    //     while (ros::ok()) {   
    //         ros::spinOnce();
    //         if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
    //         {  
    //             ROS_ERROR("Failed to call service time_step for next step.");     
    //             break;   
    //         }   
    //         ros::spinOnce();
    //     } 
    // }
    // else
    // ROS_ERROR("Could not enable keyboard, success = %d.", keyboardEnablesrv.response.success);
    //退出时时钟清零

    ros::Timer timer = n->createTimer(ros::Duration(0.064,0), timerCallback);

    while (ros::ok()) {   
        ros::spinOnce();
        // if(turnLeft && )
        if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
        {  
            ROS_ERROR("Failed to call service time_step for next step.");     
            break;   
        }   
        ros::spinOnce();
    }
    
    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown(); 
    return 0;
}
