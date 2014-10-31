#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>		// I2C 
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>			// Various Libraries
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <termios.h>
#include <wiringPi.h>			//GPIO pins
#include <assert.h>
#include <wiringSerial.h>   		//GPIO pins
#include <tgmath.h>
#include <stdbool.h>
#include <syslog.h>
#include <time.h>
#include <libgen.h>
#include <signal.h>
#include <assert.h>

#include </home/pi/Desktop/Sailbot/gpsd-master/gps.h>			// GPSD Specific Libraries
#include </home/pi/Desktop/Sailbot/gpsd-master/gpsd_config.h>
#include </home/pi/Desktop/Sailbot/gpsd-master/gpsdclient.h>
#include </home/pi/Desktop/Sailbot/gpsd-master/revision.h>

/*
   -~- SAILBOT MACROS -~-
*/

#define winch_correction 0		// Corrects SailWinch
#define servo_correction -6		// Corrects servo to center
#define wind_sensor_correction 10	// Corrects Wind Sensor 
#define actuation_constant 1		// Speed of servo actuation
#define windward_angle 45		// The preferred angle of the boat headed to windward	
#define downwind_angle 135		// The preferred angle of the boat on a broad reach
#define tack_angle 45			// Angle to Tack the boat on
#define point_proximity_radius 10       // Defines the acceptable radius of waypoint to be considered "achieved" in meters
#define winch_min 80			// Defines the minimum point of sailwinch travel (closehauled position)
#define winch_max 148			// Defines the maximum point of sailwinch travel (broadreach position)
#define servo_hard_port	30		// Defines the maximum point of rudder travel (to port)
#define servo_hard_starboard 150	// Defines the maximum point of rudder travel (to starboard)
#define pi 3.1415926535898		// Pi

/*
   -~- SAILBOT FUNCTION DECLARATIONS -~-
*/

//void transmitToXBee(double gpsLong, double gpsLat, double boatHeading, double boatSpeed, double windHeading, double windSpeed, double roll, double pitch, double yaw);
void sail(int heading, int direction, int wind);
void read_buffer(void);
void write_buffer(void);
int boundary_detect(double boat_latitude, double boat_longitude);
int autonomous(void);
int starboard(int wind_direction);
int direction_to_next_point (double boat_latitude, double boat_longitude, double next_latitude, double next_longitude);
int point_proximity(double boat_latitude, double boat_longitude, double next_latitude, double next_longitude);

/*
   -~- SAILBOT GLOBAL VARIABLES -~-
*/

static const char *fileName = "/dev/i2c-1";				// Name of the port we will be using
static time_t status_timer;
static struct fixsource_t source;
static struct gps_data_t gpsdata;					// GPS Structure
unsigned char buf[5];							// Buffer for data being read/ written on the i2c bus
char scr[128], *s;
char buffer[7];
char *ptr;
int fd = 0, i =0, j=0, tack_timer = 0;					// File descrition
int address = 0x02;							// address of the SD21 shifted right one bit
int readBytes;
int servo = 90, winch = 90;
int delta_heading_angle = 0, delta_heading_wind = 0, delta_wind_direction = 0, distance = 0;
int wind_direction = 0;
int waypoint_number = 0;
int starboard_flag = 0;
int preferred_tack = 0;
int preferred_tack_is_to_starboard_of_wind = 0;
int current_desired_heading = 0;
int delta_heading_desired = 0;
double z = 0;
double x = 0;
double gps_waypoints[100] = { 	37.216769,-80.005349,
				37.217452,-80.003410,
				37.217490,-79.999750,
				37.222401,-79.997001,
				37.222525,-79.991690,
				37.208420,-80.008580,
				0,0,
				0,0 };      			 // Array of GPS waypoints            

/*
   -~- SAILBOT MAIN FUNCTION -~-
*/

int main(int argc, char **argv)
{
	printf("\033c");
	printf(" - Begin VT SailBOT Autonomous Navigation Routine. - \n\n");		// Indicate beginning of routine

  	wiringPiSetup();					// necessary for GPS interfacing
	
   	pinMode(1, INPUT);
   	pinMode(0, OUTPUT);			// Demonstrates functionality of WiringPi libary (issue "gpio readall" command from terminal)

    	digitalWrite(0, HIGH);
	usleep(100000);				// Resets Arduino Uno
    	digitalWrite(0, LOW);
	usleep(100000);
  	digitalWrite(0, HIGH);

	if ((fd = open(fileName, O_RDWR)) < 0) 
	{							// Open I2c port for reading and writing
		printf("Failed to open I2C port!\n");
		return 1;
	}

	if (ioctl(fd, I2C_SLAVE, address) < 0) 
	{							// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave!\n");
		return 1;
	}

	sleep(2);

	gps_close(&gpsdata);			// Close GPS just in case

	unsigned int flags = WATCH_ENABLE;		// Flags for GPSD
	char tbuf[CLIENT_DATE_MAX+1];

	int count = 0, track = 0, status = 0, heading = 0;
	double latitude = 0, longitude = 0, unix_time = 0;
	double roll = 0, pitch = 0, yaw = 0, speed = 0;
								 
	if (optind < argc) 		// Grok the server, port, and device
	{					
		gpsd_source_spec(argv[optind], &source);
	} 
	else
		gpsd_source_spec(NULL, &source);

	if (gps_open("127.0.0.1", "2947", &gpsdata) != 0) 	// Open the GPSD stream
	{
		printf("No gpsd running or network error!\n");
		return 0;	
	}
    	else
		printf("GPS Port Successfully opened! \n\n");
		sleep(2);
	
        status_timer = time(NULL);
    	if (source.device != NULL)
		flags |= WATCH_DEVICE;

    	(void)gps_stream(&gpsdata, flags, source.device);		// Stream GPS Parameters	

	while(1)								// Infinite loop
	{							
		usleep(100000); 						// Changes frequency of sampling (10 Hz)
		printf("\033c");					// Clear terminal							
		printf("~ VT SailBOT ~ \n\n");

		if (gps_read(&gpsdata) == -1) 
		{							// Print error if GPS isn't read
			printf("GPS not read.\n");
	    	} 
	   	else 
		{
			if (isnan(gpsdata.fix.latitude) != 0)			// Don't use GPS data without a Fix
				printf("No GPS Fix.\n");
			else
			{
				latitude = gpsdata.fix.latitude;		// Get Basic GPS parameters 
				longitude = gpsdata.fix.longitude;
				unix_time = gpsdata.fix.time;
				track = gpsdata.fix.track;
				speed = gpsdata.fix.speed;

				heading = gpsdata.attitude.heading;		// Specific to Hemisphere H-102 GPS
				roll = gpsdata.attitude.roll;
				pitch = gpsdata.attitude.pitch;
				yaw = gpsdata.attitude.yaw;
				status = gpsdata.attitude.mag_st;
				
				if (status == 0)
					heading = track;
		
				if((isnan(gpsdata.attitude.heading) != 0) || (heading > 360 || heading < 0)) 
   					heading = 0;
				if((isnan(gpsdata.fix.track) != 0) || (track > 360 || track < 0)) 
   					track = 0;
				if((isnan(gpsdata.attitude.roll) != 0) || (roll > 360 || roll < 0)) 
   					roll = 0;
				if((isnan(gpsdata.attitude.pitch) != 0) || (pitch > 360 || pitch < 0)) 
   					pitch = 0;
				if((isnan(gpsdata.attitude.yaw) != 0) || (yaw > 360 || yaw < 0)) 
   					yaw = 0;
			}    	
		count++;	
		}
				
		read_buffer();			// Read wind direction from I2C buffer

		speed = speed*2.23694;		// Speed in MPH
		//speed = speed*1.94384; 	// Speed in Knots 

		int absolute_wind_direction = heading + wind_direction;
		absolute_wind_direction = (absolute_wind_direction + 360) % 360;

		if(point_proximity(latitude, longitude, gps_waypoints[waypoint_number], gps_waypoints[waypoint_number+1]))
	        	waypoint_number = waypoint_number + 2;		// Step through Array if point is achieved 

		sail(heading, direction_to_next_point(latitude, longitude, gps_waypoints[waypoint_number], gps_waypoints[waypoint_number+1]), absolute_wind_direction);
	
		printf("	Loop Iterations: %d   \n\n", count);			// Print GPS Parameters
		printf("	Time: %s\n", unix_to_iso8601((timestamp_t)time(NULL), tbuf, sizeof(tbuf)));
		printf("	GPS time (seconds): %f\n\n",unix_time);
		printf("	Boat Latitude: %lf\n", latitude);
		printf("	Boat Longitude: %lf\n", longitude);
		printf("		Next Latitude: %lf\n", gps_waypoints[waypoint_number]);
		printf("		Next Longitude: %lf\n\n", gps_waypoints[waypoint_number + 1]);
		printf("~ Crucial SailBOT Parameters ~\n\n");
		printf("	Heading (degrees,N): %d\n", heading);
		printf("	Track (degrees,N): %d\n", track);		
		printf("	Wind Direction (degrees,N): %u\n",absolute_wind_direction);
		printf("	Direction to Next Waypoint (degrees,N): %d\n\n", direction_to_next_point(latitude, longitude, gps_waypoints[waypoint_number], gps_waypoints[waypoint_number+1]));
		printf("	Difference between heading and direction: %d\n", delta_heading_angle);
		printf("	Difference between heading and wind: %d\n", delta_heading_wind);
		printf("	Difference between direction and wind: %d\n\n", delta_wind_direction);
		printf("	Distance to Next Waypoint (m): %d\n\n", distance);
		printf("~ Other Parameters ~\n\n");
		printf("	Speed (mph): %lf\n", speed);
		printf("	Compass Lock Status: %d\n", status);
		printf("	Roll (degrees): %lf\n", roll);
		printf("	Pitch (degrees): %lf\n", pitch);
		printf("	Yaw (degrees): %lf\n", yaw);
		printf("	Wind Direction (degrees,B): %d\n", wind_direction);
		printf("	Next Waypoint: %u\n", waypoint_number/2+1);
		printf("	Autonomous: %u\n", autonomous());
		printf("	Starboard: %u\n", starboard(wind_direction));
		printf("	Servo and Winch are: %u %u\n\n", servo, winch);  

		write_buffer();			// Update Servo/Winch positions                   
	}
	return 0;
}


/*
   -~- SAILBOT FUNCTIONS -~-
*/


// Contains all sailing logic
void sail(int heading, int direction, int wind)	
{
	double heading_radians = heading * pi/180;
	double angle_radians = direction * pi/180;
	double wind_radians = wind * pi/180;
	double desired_radians = current_desired_heading *pi/180;

	delta_heading_angle = 180/pi * acos(cos(heading_radians)*cos(angle_radians) + sin(heading_radians)*sin(angle_radians));
	delta_heading_wind = 180/pi * acos(cos(heading_radians)*cos(wind_radians) + sin(heading_radians)*sin(wind_radians));
	delta_wind_direction = 180/pi * acos(cos(angle_radians)*cos(wind_radians) + sin(angle_radians)*sin(wind_radians));
	delta_heading_desired = 180/pi * acos(cos(heading_radians)*cos(desired_radians) + sin(heading_radians)*sin(desired_radians));


	//if the waypoint's sailable
	if(delta_wind_direction > 45){
		//you can now tack and sail directly to the target - break out of a preferred tack if you're on one
		preferred_tack = 0;

		//sail off the wind
		current_desired_heading = direction;
	}

	else if(delta_wind_direction <= 45 && preferred_tack == 0)				// Sail Upwind
	{
					// Vector Math (definition of cross product)
		z = -(cos((pi/180)*direction)*sin((pi/180)*wind) - sin((pi/180)*direction)*cos((pi/180)*wind));

		if(z >= 0)		// Starboard Case
		{
			current_desired_heading = ((wind + 45 + 360) % 360);
            preferred_tack_is_to_starboard_of_wind = 1;
		}
		else			// Port Case
		{
			current_desired_heading = ((wind - 45 + 360) % 360);
            preferred_tack_is_to_starboard_of_wind = 0;
		}

		//you're on a preferred tack
		preferred_tack = 1;	
	}

	else if(preferred_tack){
		//test which way of the wind you're facing
        
		//current_desired_heading = 45 degrees off the wind in the current direction
	}

	
	
	x = -(cos((pi/180)*current_desired_heading)*sin((pi/180)*heading) - sin((pi/180)*current_desired_heading)*cos((pi/180)*heading));

	if(x >= 0)		
	{
		servo = 90 + actuation_constant * delta_heading_desired;		// Turn Rudder one way
	}
	else
	{
		servo = 90 - actuation_constant * delta_heading_desired;		// Turn Rudder other way
	}
	
	int winch_wind_direction = 0;
	
	if ((wind_direction > 179) && (wind_direction < 360))
		winch_wind_direction = 180 - ((wind + 360) % 180);

	winch = ((winch_max - winch_min)/(downwind_angle- windward_angle)) * (winch_wind_direction - windward_angle) + winch_min;

}


// Function to receive the Wind Direction and Speed from I2C buffer
void read_buffer()
{
	if (read(fd, buf, 3) != 3)
	{									// Read back data from Arduino
		printf("Unable to read from slave!\n");
		    digitalWrite(0, HIGH);
			usleep(100000);
		    digitalWrite(0, LOW);
			usleep(100000);
		    digitalWrite(0, HIGH);
			usleep(3000000);
	}

	else 
	{
		wind_direction = buf[1];		// read from buffer
		if (buf[2]) wind_direction += 256;
		
		//wind_direction = wind_direction *(359/327) - (5026/327);

		//if (wind_direction < 0)
		//	wind_direction = 0;

		//if(wind_direction > 359)
		//	wind_direction = 359;

		//wind_direction = (wind_direction + 360 + wind_sensor_correction) % 360;
	}	
}

// Function to write Servo and Winch positions to I2C buffer
void write_buffer()
{
	if (winch <= winch_min)
		winch = winch_min;

	if (winch >= winch_max)
		winch = winch_max;

	if (servo >= servo_hard_starboard)
		servo = servo_hard_starboard;
		
	if (servo <= servo_hard_port)
		servo = servo_hard_port;

	winch = winch + winch_correction;
	servo = servo + servo_correction;					// Apply servo correction factor

	buf[0] = winch;								// Write servo position
	buf[1] = servo;	
}
	
/*
This function returns 1 if the boat has violated it's physical boundaries as defined by the function

Jeff Witten - 05/26/14

*/

int boundary_detect(double boat_latitude, double boat_longitude)
{
	// boundaries go here
	return 0;
}

/*

This function takes as input the next gps coordinate that the boat is suppossed to achieve and calculates the compass
heading to the waypoint

Jeff Witten - 05/26/14

10/08/14 --> Corrected the logic and changed abs() to fabs(), works for linearized gps coordinates

*/
int direction_to_next_point (double boat_latitude, double boat_longitude, double next_latitude, double next_longitude)
{
	double dist_lat = 111132.954 - 559.822*cos(2*boat_latitude) + 1.1175*cos(4*boat_latitude);
	double dist_lon = pi/180*6367449*cos(boat_latitude);
	double lat_meter = fabs(fabs(next_latitude) - fabs(boat_latitude))*dist_lat;
	double lon_meter = fabs(fabs(next_longitude) - fabs(boat_longitude))*dist_lon;
	double difference = (lat_meter/lon_meter);
	double	degree = atan(difference);	
	degree = fabs(degree*180/pi);
	int angle = 0;
	
           if(next_latitude >= boat_latitude && next_longitude >= boat_longitude)			//Quadrant I
                angle = 90 - degree;
		   else if(next_latitude <= boat_latitude && next_longitude >= boat_longitude)			//Quadrant II
                angle = 90 + degree;
		   else if(next_latitude <= boat_latitude && next_longitude <= boat_longitude)			//Quadrant III
                angle = 270 - degree;
		   else if(next_latitude >= boat_latitude && next_longitude <= boat_longitude)			//Quadrant IV
                angle = 270 + degree;

	return angle;
}

/*

This function takes as input the gps coordinates of the boat and the gps coordinates of a waypoint that the boat desires to approach
The function then sets a flag based on whether or not the boat is within a predetermined perimeter of the waypoint

	Approach:
1.) Latitude and Longitude of San Francisco = 37.7844 N, 122.4167 W
2.) At 40 degrees North: 1 degree latitude = 111.03 km, 1 degree longitude = 85.39 km
3.) 111.03 = 85.39 * (1.30027) - used to correct approximately rectangular lat/lon grid to approximately square
4.) Through unit analysis, 1 meter = 0.0000111509 degrees longitude 

Jeff Witten - 03/27/14

*/

int point_proximity(double boat_latitude, double boat_longitude, double next_latitude, double next_longitude)
{
	int number = 0;

	double dist_lat = 111132.954 - 559.822*cos(2*boat_latitude) + 1.1175*cos(4*boat_latitude);
	double dist_lon = pi/180*6367449*cos(boat_latitude);
	double lat_meter = fabs(fabs(next_latitude) - fabs(boat_latitude))*dist_lat;
	double lon_meter = fabs(fabs(next_longitude) - fabs(boat_longitude))*dist_lon;
	distance = sqrt(pow(lat_meter, 2) + pow(lon_meter,2));

	if (distance <= point_proximity_radius)
		number = 1;
	else
		number = 0;
	
	return number;
}

// Writes starboard as 1 if boat is on a starboard tack, 0 on port
int starboard(int wind_direction)
{
	int starboard_value = 0;

	if (wind_direction >= 0 && wind_direction <= 179)
		starboard_value = 1;
	else
		starboard_value = 0;

	return starboard_value;
}

int autonomous(void)
{
	int autonomous = 0;

	if (digitalRead(1))
		autonomous = 1;			// Change autonomous mode flag based on boat's condition
	else
		autonomous = 0;

	return autonomous;
}
