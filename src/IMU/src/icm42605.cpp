#include "IMU/i2c.h"
#include "IMU/registre.h"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <ros/ros.h>
#include <errno.h>
#include <cstring>
#include <time.h>
#include <ros/ros.h>


#define N 10    // Number of iteration for calculating the Acceleration's and Gyro's Offsets
#define K 0.8  // 

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_15_125DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz, AODR_1000Hz, AODR_2000Hz, AODR_4000Hz, AODR_8000Hz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1000Hz, GODR_2000Hz, GODR_4000Hz, GODR_8000Hz
*/


//Variables pour calcule quaternion
#define betaDef		0.5f		// 2 * proportional gain
volatile float sampleFreq = 512.0f;		// sample frequency in Hz
volatile float beta = betaDef;		// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
double Ts = 1;                                                        //Sampeling Period
//___________________________________________

batmon_driver bat(3, I2CADDRESS3);

uint8_t Ascale = AFS_2G, Gscale = GFS_250DPS, AODR = AODR_1000Hz, GODR = GODR_1000Hz;

int16_t ICM42605Data[6];   // Stores the 16-bit signed sensor output

// ICM42605 ICM42605(&i2c_0); // instantiate ICM42605 class


// debug
bool debug = false;

// variables
bool initialized;

float accData[3];

float gyroData[3];


bool init_sensor()
{
/*  if (debug)
    Serial.println("\nCommence init sequence ...");
  if (debug)
    Serial.println("OK");
  if (debug)
    Serial.print("Initializing sensor......");
  
*/
  
    bat.reset(); // software reset ICM42605 to default registers
    bat.init(Ascale, Gscale, AODR, GODR); // sofware init ICM42605
   // bat.offsetBias(accData,gyroData);
    return true;
  
}

//fonctions pour calcule de quaternion :
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
float invSqrt(float x);


int main(int argc, char **argv)                                                                    
{
     float a_x =0, a_y = 0, a_z = 0, g_x = 0, g_y = 0, g_z = 0;           //Relative acceleration and gyro
     float old_a_x = 0, old_a_y = 0, old_a_z = 0, old_g_z = 0;                          //Old relative acceleration
     float x = 0, y = 0, w = 0;                                            //Coordiantes
     float vitesse_x = 0, vitesse_y = 0, vitesse_z = 0;                    //Absolute speed
     float acceleration_x = 0, acceleration_y = 0, acceleration_z = 0;     //Absolute acceleration
     double Ts = 1;                                                        //Sampeling Period
     float offset_a_x = 0 ;
     float offset_a_y = 0 ;
     float offset_a_z = 0 ; 
     float offset_g_x = 0 ; 
     float offset_g_y = 0 ;
     float offset_g_z = 0 ;
     struct timespec old_time, new_time;
     clock_gettime(CLOCK_REALTIME, &old_time);
     clock_gettime(CLOCK_REALTIME, &new_time);

//initializing ross stuff
    ros::init(argc, argv, "raspi_batmon");
    ros::NodeHandle nh;
    ROS_INFO("hello");
    
    init_sensor();
    ros::Duration(1).sleep(); // Wait for all registers to reset
    
    for (int i = 0; i < N; i++)
	{ 
    bat.readData(ICM42605Data);
    
    offset_a_x = (float)ICM42605Data[0];
    offset_a_y = (float)ICM42605Data[1];
    offset_a_z = (float)ICM42605Data[2];

    offset_g_x = (float)ICM42605Data[3];
    offset_g_y = (float)ICM42605Data[4];
    offset_g_z = (float)ICM42605Data[5]; 

    a_x += (offset_a_x * bat._aRes ) * 9.80665;
    a_y += (offset_a_y * bat._aRes ) * 9.80665;
    a_z += (offset_a_z * bat._aRes ) * 9.80665;
    g_x += (offset_g_x * bat._gRes ) ;
    g_y += (offset_g_y * bat._gRes ) ;
    g_z += (offset_g_z * bat._gRes ) ;
    //ROS_INFO("|%f | %f |%f | %f |%f | %f | \n ", g_x, g_y, g_z, a_x, a_y, a_z );
	}
    offset_a_x = a_x/N;
    offset_a_y = a_y/N;
    offset_a_z = a_z/N;
    offset_g_x = g_x/N;
    offset_g_y = g_y/N;
    offset_g_z = g_z/N; 

    ros::Rate loop_rate(500); 
    while (ros::ok())
	{

	bat.readData(ICM42605Data);
        clock_gettime(CLOCK_REALTIME, &new_time);
	Ts = ((new_time.tv_sec - old_time.tv_sec) * 1e6 + (new_time.tv_nsec- old_time.tv_nsec) / 1e3) / 1e6;
	clock_gettime(CLOCK_REALTIME, &old_time);
    
	a_x = (float)ICM42605Data[0];
	a_y = (float)ICM42605Data[1];
	a_z = (float)ICM42605Data[2];

	g_x = (float)ICM42605Data[3];
	g_y = (float)ICM42605Data[4];
	g_z = (float)ICM42605Data[5];
       //ROS_INFO("|%f | %f |%f | %f |%f | %f | \n ", g_x, g_y, g_z, a_x, a_y, a_z ); 
	a_x = (a_x * bat._aRes ) * 9.80665 - offset_a_x;
	a_y = (a_y * bat._aRes ) * 9.80665 - offset_a_y;
	a_z = (a_z * bat._aRes ) * 9.80665 - offset_a_z;
	g_x = (g_x * bat._gRes )  - offset_g_x;
	g_y = (g_y * bat._gRes )  - offset_g_y;
	g_z = (g_z * bat._gRes )  - offset_g_z;

	 ROS_INFO("|%f | %f |%f | %f |%f | %f | \n ", g_x, g_y, g_z, a_x, a_y, a_z ); 

	//ROS_INFO("       | TS = %e |\n", Ts);
	

	//Calculate Quaternions with Madgwick filter
	MadgwickAHRSupdateIMU(g_x, g_y, g_z, a_x, a_y, a_z);

	//Filter a_x, a_y, a_z with KALMAN FILTER
	a_x = K * a_x + (1 - K) * old_a_x;
	a_y = K * a_y + (1 - K) * old_a_y;
	a_z = K * a_z + (1 - K) * old_a_z;
	g_z = K * g_z + (1 - K) * old_g_z;
	old_a_x = a_x;
	old_a_y = a_y;
	old_a_z = a_z;
	old_g_z = g_z;
	
	//ROS_INFO("|%f | %f |%f | %f |%f | %f | \n ", g_x, g_y, g_z, a_x, a_y, a_z );

	acceleration_x = (q0*q0+q1*q1-q2*q2-q3*q3) * a_x + 2*(q1*q2-q0*q3)            * a_y + 2*(q0*q2+q1*q3)             * a_z;
	acceleration_y = 2*(q1*q2+q0*q3)           * a_x + (q0*q0*-q1*q1+q2*q2-q3*q3) * a_y + 2*(q2*q3-q0*q1)             * a_z;
	//acceleration_z = 2*(q1*q3-q0*q2)           * a_x + 2*(q0*q1+q2*q3)              a_y + (q0*q0-q1*q1-q2*q2+q3*q3) * a_z;
	ROS_INFO("|%f | %f |\n ", acceleration_x,acceleration_y );
	vitesse_x += acceleration_x * Ts;
	vitesse_y += acceleration_y * Ts;
	//vitesse_z += acceleration_z * Ts;
        ROS_INFO("|%f | %f |\n ", acceleration_x,acceleration_y );
	x += 0.5 * acceleration_x * Ts * Ts + vitesse_x * Ts;
	y += 0.5 * acceleration_y * Ts * Ts + vitesse_y * Ts;
	//z += 0.5 * acceleration_z * Ts * Ts + vitesse_z * Ts;

	w = (w + g_z * Ts);
	if ( w < -360)
	{
	w += 360;
	}
	if ( w > +360)
	{
	w -= 360;
	}
	ROS_INFO("                                                         | X = %f | Y = %f | W = %f |", x, y, w);

	// construct odometry messages and publish it


	

	ros::Duration(0.01).sleep();
	}
}

//---------------------------------------------------------------------------------------------------
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (Ts);
	q1 += qDot2 * (Ts);
	q2 += qDot3 * (Ts);
	q3 += qDot4 * (Ts);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


