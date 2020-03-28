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
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h> 

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
double speed = 4.5;
double left_speed = 0, right_speed = 0;
double left_angle = 0, servo_angle = 0, right_angle = 0;
double degree = M_PI/180, T = 0.16, L = 0.16;
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

void ackermann_steering_control(double velocity, double angle)
{
	double radian, outside_speed, inside_speed;
	double inside_radius, outside_radius, inside_angle, outside_angle;
	
	radian = angle * degree;
	
	if( angle > 0 )
	{
		inside_radius = L / tan(radian) - T / 2;
		outside_radius = L / tan(radian) + T / 2;
	}
	else
	{
		outside_radius = L / tan(radian) - T / 2;
		inside_radius = L / tan(radian) + T / 2;
	}
	
	outside_speed = velocity * ( 1 + T * tan(abs(radian)) / ( 2 * L ) );
	inside_speed = velocity * ( 1 - T * tan(abs(radian)) / ( 2 * L ) );
	
	inside_angle = atan( L / inside_radius );
	outside_angle = atan( L / outside_radius );
	
	cout<<"outside_speed: "<<outside_speed<<" outside_angle: "<<outside_angle<<" inside_speed: "<<inside_speed<<" inside_angle: "<<inside_angle<<endl;
	
	if( angle > 0 )
	{
		left_angle = outside_angle;
		left_speed = outside_speed;
		
		right_angle = inside_angle;
		right_speed = inside_speed;
	}
	else
	{
		right_angle = outside_angle;
		right_speed = outside_speed;
		
		left_angle = inside_angle;
		left_speed = inside_speed;
	}
}
	
int main(int argc, char** argv)
{
    ros::init(argc, argv, "smartcar_controller");
    
    ros::NodeHandle n;
    
    ros::Publisher left_vel_pub = n.advertise<std_msgs::Float64>("/rear_left_velocity_controller/command", 10);
    ros::Publisher right_vel_pub = n.advertise<std_msgs::Float64>("/rear_right_velocity_controller/command", 10);
    ros::Publisher left_steer_pub = n.advertise<std_msgs::Float64>("/left_bridge_position_controller/command", 10);
    ros::Publisher right_steer_pub = n.advertise<std_msgs::Float64>("/right_bridge_position_controller/command", 10);
    
    ros::Rate loop_rate(200);
    
    init_keyboard();

    // message declarations
    std_msgs::Float64 left_steer_position;
    std_msgs::Float64 right_steer_position;
    std_msgs::Float64 left_vel;
    std_msgs::Float64 right_vel;
    
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
			servo_angle = 25;
			num = 0;
		}
		else if( key == 113 )//'q'
		{
			x = 1;
			servo_angle = -25;
			num = 0;
		}
		else if( key == 119 )//'w'
		{
			x = 1;
			servo_angle = 0;
			num = 0;
		}
		else if( key == 115 )//'s'
		{
			x= -1;
			servo_angle = 0;
			num = 0;
		}
		else if( key == 122 )//'z'
		{
			x= -1;
			servo_angle = -25;
			num = 0;
		}
		else if( key == 99 )//'c'
		{
			x= -1;
			servo_angle = 25;
			num = 0;
		}
		else if( key == 32 )//' '
		{
			x = 0;
			servo_angle = 0;
			control_speed = 0;
			control_turn = 0;
		}
		else
		{
			num = num + 1;
			if( num > 4 )
			{
				x = 0;
				servo_angle = 0;
			}
			if( key == '\x03' )
				break;
		}
		
        target_speed = speed * x;

		if( target_speed > control_speed )
			control_speed = min( target_speed, control_speed + 0.02 );
		else if( target_speed < control_speed )
			control_speed = max( target_speed, control_speed - 0.02 );
		else
			control_speed = target_speed;
			
		ackermann_steering_control( control_speed, servo_angle );

    	//update joint_state
		left_vel.data = left_speed; 
		right_vel.data = right_speed;
    	
        //update joint_state
		left_steer_position.data = left_angle;
		right_steer_position.data = right_angle;
		
		cout<<"left_speed: "<<left_speed<<", right_speed: "<<right_speed<<", left_angle: "<<left_angle<<", right_angle: "<<right_angle<<"\n"<<endl;

        //send the joint state and transform
		left_vel_pub.publish(left_vel);
		right_vel_pub.publish(right_vel);
		
		left_steer_pub.publish(left_steer_position);
		right_steer_pub.publish(right_steer_position);

        // This will adjust as needed per iteration
		loop_rate.sleep();
    }
    
    close_keyboard();

    return 0;
	
}

