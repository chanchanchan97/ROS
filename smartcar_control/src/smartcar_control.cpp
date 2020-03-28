#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <algorithm>
#include <termio.h>
#include <linux/input.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <geometry_msgs/Twist.h>

using namespace std;

int x = 0, y = 0;
int th = 0;
int status = 0;
int num = 0;
int acc = 0.1;
double target_speed = 0;
double target_turn = 0;
double control_speed = 0;
double control_turn = 0;
double speed = 0.2;
double turn = 0.75;
double angle= 0;
double angular_speed = 0;
char key;
int in;
double i = 0, j = 0;
struct timeval tv;
struct termios new_settings;
struct termios stored_settings;
static int peek_character = -1;

void init_keyboard()
{
    tcgetattr(0,&stored_settings);
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	tcgetattr(0,&stored_settings);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&new_settings);
}
 
void close_keyboard()
{
    tcsetattr(0, TCSANOW, &new_settings);
}

int kbhit()
{
    unsigned char ch;
    int nread;
 
    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1) 
    {
        peek_character = ch;
        return 1;
    }
    return 0;
}

int readch()
{
    char ch;
 
    if(peek_character != -1) 
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}

int main(int argc, char** argv)
{
	const double degree = M_PI/180; 
	 
    ros::init(argc, argv, "smartcar_controller");
    
    ros::NodeHandle n;
    
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 5);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    
    
    ros::Rate loop_rate(200);
    
    init_keyboard();

    // message declarations
    sensor_msgs::JointState joint_state;
    geometry_msgs::Twist twist;
    
    int count = 0;

    while (1) 
    {
    	if( kbhit() )
    		key = readch();
    	else
    		key = 0;
    
		if( key == 101 )//'e'
		{
			x = 1;
			th = -1;
			angle = 30;
			i += angular_speed;
			num = 0;
		}
		else if( key == 113 )//'q'
		{
			x = 1;
			th = 1;
			angle = -30;
			i += angular_speed;
			num = 0;
		}
		else if( key == 119 )//'w'
		{
			x = 1;
			th = 0;
			angle = 0;
			i += angular_speed;
			num = 0;
		}
		else if( key == 115 )//'s'
		{
			x= -1;
			th = 0;
			angle = 0;
			i -= angular_speed;
			num = 0;
		}
		else if( key == 122 )//'z'
		{
			x= -1;
			th = -1;
			angle = -30;
			i -= angular_speed;
			num = 0;
		}
		else if( key == 99 )//'c'
		{
			x= -1;
			th = 1;
			angle = 30;
			i -= angular_speed;
			num = 0;
		}
		else if( key == 32 )//' '
		{
			x = 0;
			th = 0;
			angle = 0;
			control_speed = 0;
			control_turn = 0;
		}
		else
		{
			num = num + 1;
			if( num > 4 )
			{
				x = 0;
				th = 0;
				angle = 0;
			}
			if( key == '\x03' )
				break;
		}
		
        target_speed = speed * x;
        target_turn = turn * th;

		if( target_speed > control_speed )
			control_speed = min( target_speed, control_speed + 0.02 );
		else if( target_speed < control_speed )
			control_speed = max( target_speed, control_speed - 0.02 );
		else
			control_speed = target_speed;

		if( target_turn > control_turn )
			control_turn = min( target_turn, control_turn + 0.1 );
		else if( target_turn < control_turn )
			control_turn = max( target_turn, control_turn - 0.1 );
		else
			control_turn = target_turn;
			
		angular_speed = control_speed /  0.025;

    	//update joint_state
		twist.linear.x = control_speed; 
		twist.linear.y = 0; 
		twist.linear.z = 0;
		twist.angular.x = 0; 
		twist.angular.y = 0; 
		twist.angular.z = control_turn;
    	
        //update joint_state
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(6);
		joint_state.position.resize(6);
		joint_state.name[0]="left_front_wheel_to_bridge";
		joint_state.position[0] = i * degree;
		joint_state.name[1] ="right_front_wheel_to_bridge";
		joint_state.position[1] = i * degree;
		joint_state.name[2] ="left_back_wheel_joint";
		joint_state.position[2] = i * degree;
		joint_state.name[3] ="right_back_wheel_joint";
		joint_state.position[3] = i * degree;
		joint_state.name[4] ="left_bridge_to_bridge";
		joint_state.position[4] = angle * degree;
		joint_state.name[5] ="right_bridge_to_bridge";
		joint_state.position[5] = angle * degree;
		
		cout<<"x: "<<x<<", y: "<<y<<", control_speed: "<<control_speed<<", control_turn: "<<control_turn<<"\n"<<endl;

        //send the joint state and transform
		joint_pub.publish(joint_state);
		
		vel_pub.publish(twist);

        // This will adjust as needed per iteration
		loop_rate.sleep();
    }
    
    close_keyboard();

    return 0;
	
}


