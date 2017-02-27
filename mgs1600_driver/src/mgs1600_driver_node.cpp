#include "ros/ros.h"
#include "std_msgs/String.h"

//#include "mgs1600_fields.h"
//#include "mgs1600_vector.h"
#include <mgs1600_driver/mgs1600_fields.h>
#include <mgs1600_driver/mgs1600_vector.h>
#include <string> 
#include <stdint.h>
#include <stdlib.h>
#include <cereal_port/CerealPort.h>
#include <sstream>
#include <cstring>
#include <time.h>



#define DEFAULT_BAUDRATE 		115200
#define DEFAULT_USB_PORT_NAME 	"/dev/ttyACM0"
#define REPLY_SIZE 		130 // max caracteres a ler 
#define TIMEOUT  		10   // ms - tempo de espera 
#define READ_TIMOUT		70

#define REPEAT_LAST_QUERY 				1
#define REPEAT_LAST_QUARY_PERIODICLY 	2		
#define CLEAR_QUARY						3

#define SENSOR_PERIOD 	10 //[ms] - min time possible
#define MAGNED_ARRAY_ALL 0x00

#define BOTH_TRACKS	0
#define LEFT_TRACK 	1
#define RIGHT_TRACK	2

#define Both_MARKERS	0
#define LEFT_MARKER 	1
#define RIGHT_MARKER	2

#define ALL_AXIS 			0
#define X_AXIS				1
#define Y_AXIS				2
#define Z_AXIS				3
 
#define TRACK_DETECT	 	 2
#define INTERNAL_SENSORS 	 4
#define SELECTED_TRACK	 	 5
#define TRACKS				 6
#define GYRO				 8
#define MARKERS				 9

#define SET_ALL_SENSORS		 0x3F
#define SET_TRACK_DETECT     0x01
#define SET_INTERNAL_SENSORS 0x02
#define SET_SELECTED_TRACK	 0x04
#define SET_TRACKS			 0x08
#define SET_GYRO			 0x10
#define SET_MARKERS			 0x20

#define SET_SENSORS_TO_READ	SET_SELECTED_TRACK

#define MAX_FRAQUENCY 100 //Hz
#define RATE MAX_FRAQUENCY

struct parsingStruct 
{
	uint16_t cmd;
	int arg[16];
	bool flagEchoReceived;
	int nElements;
};

class mgs1600
{
 private:

    ros::NodeHandle nh_;
    cereal::CerealPort port_;
 public:
    mgs1600(){}
    ~mgs1600(){ port_.close();}
   
   
    bool openPort(std::string & portName, int baudrate);
    bool readSensor(char * recvMsg);
    bool WriteToSensor(char * msg);
	bool ClearQueryBuffer(void);
	bool SetPeriodicQuery(uint32_t period);
	bool ReadTrackDetect(int * value);
	bool ReadSelectedTrack(int * value);
	bool ReadTracks(uint16_t sensors,int * arraySensorsValue);
	bool ReadGyro(uint16_t sensors,int * arraySensorsValue);
	bool ReadMarkers(uint16_t sensors,int * arraySensorsValue);
	bool setQueryConfig(uint16_t query, unsigned int period);
	bool SetPeriodicSensorQuery(uint16_t sensoresToSet, uint32_t period);
	bool GetheringSensorData(mgs1600_driver::mgs1600_fields * sample, uint32_t sensorSet);
	
	bool ReadInternalSensors(uint16_t sensors,int * arraySensorsValue);
	bool swapCharacter(char * buffer, char findCharacter, char swapCharacter);
	
	bool RecvSamples(struct parsingStruct * parsedLine);
	bool SendRequest(std::string sendMsg,struct parsingStruct * parsedLine );
	bool CmdParsing(std::string cmd, std::string * array);
	uint16_t BreakMsgInLines(std::string cmd, std::string * array);
	bool ParsingLine(std::string line,struct parsingStruct * parsedLine);
    
};





/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "mgs1600_driver");

  ros::NodeHandle n,nh("~");

  ros::Publisher sendor_pub = n.advertise<mgs1600_driver::mgs1600_fields>("mgs1600", 1000);

  std::string port_name = DEFAULT_USB_PORT_NAME;
  int baudrate = DEFAULT_BAUDRATE;
  int sensors_set = SET_ALL_SENSORS;
  
  mgs1600 magnet_sensor; // class do sensor magnetico mgs1600

  // load parameters from launch file

  nh.getParam("comm_port",port_name);
  nh.getParam("baudrate",baudrate);
  nh.getParam("sensor2read",sensors_set);
 
  printf("sensor2read %d:",sensors_set);
  // abrir a porta USB do sensor magnetico
  if(!magnet_sensor.openPort(port_name,baudrate))
  {
    ROS_INFO("error - usb port");
	return 0; 
  }
  ROS_INFO("Sensor is conected");
  
  // Select the sensors to be set
  
  //ROS_INFO("Sensor set %d",sensors_set);
  
  if(!magnet_sensor.SetPeriodicSensorQuery(sensors_set,SENSOR_PERIOD))
  {
    ROS_INFO("NO Sensores query set");
  	return false;
  }
  ROS_INFO("Sensors set");


  ros::Rate loop_rate(RATE); // frenquencia máxima 
  while (ros::ok())
  {
  
  	mgs1600_driver::mgs1600_fields sample;
	magnet_sensor.GetheringSensorData(&sample,sensors_set);

	sample.stamp = ros::Time::now();

	if(fabs(sample.LeftTrack) > fabs(sample.RightTrack))
	{
		sample.ErrorLength = sample.LeftTrack;
	}else
	{
		sample.ErrorLength = sample.RightTrack;
	}

	sendor_pub.publish(sample);
    ros::spinOnce();
    
    loop_rate.sleep();
  }

  
  return 0;
}




bool mgs1600::GetheringSensorData(mgs1600_driver::mgs1600_fields * sample, uint32_t sensorSet)
{

	struct parsingStruct  parsed_line;
    parsed_line.cmd 	= 0;
    uint8_t flag		= 0;
    bool flag_time_out 	= false;
    double delta 		= 0;
    ros::Time first 	= ros::Time::now();

    memset(&parsed_line.arg,0,sizeof(parsed_line.arg));

 	while((flag != sensorSet))
 	{
 
 		delta = ros::Time::now().toSec() - first.toSec();
 		if(delta > 0.06)
 		{
 			flag_time_out = true;
 			break;
 		}
		if(RecvSamples(&parsed_line))
		{
			switch(parsed_line.cmd)
			{
				case TRACK_DETECT:
					flag |= SET_TRACK_DETECT;
					sample->TrackDetect = parsed_line.arg[0];
					break;
				case INTERNAL_SENSORS:
					flag |= SET_INTERNAL_SENSORS;
					for(int i = 0; i< parsed_line.nElements;i++)
					{
						sample->InternalSensors[i]=(int32_t)parsed_line.arg[i];
					}
					break;
				case SELECTED_TRACK:
					flag |= SET_SELECTED_TRACK;
					sample->SelectedTrack = parsed_line.arg[0];
					break;
				case TRACKS:
					flag |= SET_TRACKS;
					sample->LeftTrack = parsed_line.arg[0];
					sample->RightTrack = parsed_line.arg[1];
					break;
				case GYRO:
					flag |= SET_GYRO;
					sample->GyroX = parsed_line.arg[0];
					sample->GyroY = parsed_line.arg[1];
					sample->GyroZ = parsed_line.arg[2];
					break;
				case MARKERS:
					flag |= SET_MARKERS;
					sample->LeftMarker = parsed_line.arg[0];
					sample->RightMarker = parsed_line.arg[1];
					break;
				default:
					ROS_INFO("erro ");
			}
		}else
		{
			
		}
		
	}

}

bool mgs1600::openPort(std::string & portName, int baudRate)
{
	try
	{
		port_.open(portName.c_str(),baudRate);
	}catch(cereal::TimeoutException& e)
	{
		return false;
	}
	return true;

}

bool  mgs1600::readSensor(char * reply)
{

	memset(reply,'\0',sizeof(reply));
	try
	{
		port_.read(reply,REPLY_SIZE,TIMEOUT);

	}catch(cereal::TimeoutException& e)
	{
		return false;
	}
	if(!swapCharacter(reply,'\r', '*'))
	{
		ROS_INFO("No r found");
	}
	return true;
}

bool mgs1600::WriteToSensor(char * msg)
{
	port_.flush();
	try
	{
		port_.write(msg,strlen(msg));
	}catch(cereal::TimeoutException& e)
	{
		return false;
	}
	return true;

}

bool mgs1600::RecvSamples(struct parsingStruct * parsedLine)
{
	char reply[REPLY_SIZE];
	memset(&reply,'\0',sizeof(reply));
	if(!readSensor(reply))
	{
		return false;
	}

	//ROS_INFO("received msg %s",reply);
	std::string reply_frame(reply); // implentar desde o incício string
	std::string reply_frame_broken_in_lines[10];
	uint16_t nsub_str = BreakMsgInLines(reply_frame,reply_frame_broken_in_lines);
	if(nsub_str <= 0)
	{
		return false; // no reply msg 
	}
	
	std::string line=reply_frame_broken_in_lines[0];
	parsedLine->nElements = 0;
	ParsingLine(line,parsedLine);

	return true;
	
}

bool mgs1600::SendRequest(std::string sendMsg,struct parsingStruct * parsedLine)
{
	char reply[REPLY_SIZE];
	memset(&reply,'\0',sizeof(reply));
	
	if(!WriteToSensor((char *)sendMsg.c_str()))
	{
		return false;
	}
	if(!readSensor(reply))
	{
		return false;
	}
	
	std::string reply_frame(reply); // implentar desde o incício string
	std::string reply_frame_broken_in_lines[10];
	uint16_t nsub_str = BreakMsgInLines(reply_frame,reply_frame_broken_in_lines);
	if(nsub_str <= 0)
	{
		return false; // no reply msg 
	}
	
	std::string echo = reply_frame_broken_in_lines[0] + std::string("\r"); //adicionar \r porque a msg enviada tem um
	
	if(sendMsg != echo)
	{
		parsedLine->flagEchoReceived = false; // redondante
		ROS_INFO("echo doesnt match %s",echo.c_str() );
		return false; // msg recebida não corresponde à esperada
	}
	parsedLine->flagEchoReceived = true;
	
	if(nsub_str+1 <= 1)
	{
		return false; // tem no máximo 1 comando == ECHO
	}
	std::string line=reply_frame_broken_in_lines[1];

	parsedLine->nElements = 0;
	ParsingLine(line,parsedLine);
	//ROS_INFO("received msg %s",reply);
	return true;

}



bool mgs1600::ClearQueryBuffer(void)
{
	std::string cmd("# C\r");
	struct parsingStruct  parsed_sensor_reply;
	
	if(!SendRequest(cmd,&parsed_sensor_reply))
	{
		return false;
	}
			
	
	if(!parsed_sensor_reply.flagEchoReceived)
	{
		ROS_INFO("No echo received");
		return false;
	}
	
	return true;

}




bool mgs1600::SetPeriodicQuery(uint32_t period)
{
	std::string cmd("# ");
	char time_cstr[5];
	
	sprintf(time_cstr,"%d\r",period);
	cmd+=time_cstr;
	//ROS_INFO("msg %s",cmd.c_str());
	struct parsingStruct  parsed_sensor_reply;
	
	if(!SendRequest(cmd,&parsed_sensor_reply))
	{
		return false;
	}
			
	
	if(!parsed_sensor_reply.flagEchoReceived)
	{
		ROS_INFO("No echo received");
		return false;
	}
	
	return true;
}





bool mgs1600::SetPeriodicSensorQuery(uint16_t sensoresToSet, uint32_t period)
{
	int value;
	int sensor_array[16];
	// configurar o modo de leitura do sensor
	if(!ClearQueryBuffer())
	{
		return false;
	} 
	if(sensoresToSet & SET_SELECTED_TRACK)
	{
		if(!ReadSelectedTrack(&value))
		{
			return false;
		}
		
	}if(sensoresToSet & SET_TRACK_DETECT)
	{
		if(!ReadTrackDetect(&value))
		{
			return false;
		}
		
	}if(sensoresToSet & SET_INTERNAL_SENSORS)
	{
		if(!ReadInternalSensors(MAGNED_ARRAY_ALL,sensor_array))
		{
			return false;
		}
		
	}if(sensoresToSet & SET_TRACKS)
	{
		
		memset(&sensor_array,0,sizeof(sensor_array));
		
		if(!ReadTracks(BOTH_TRACKS,sensor_array))
		{
			return false;
		}
	}if(sensoresToSet & SET_GYRO)
	{
		memset(&sensor_array,0,sizeof(sensor_array));
		if(!ReadGyro(ALL_AXIS,sensor_array))
		{
		 	return false;
		}
		
	}if(sensoresToSet & SET_MARKERS)
	{
		memset(&sensor_array,0,sizeof(sensor_array));
		if(!ReadMarkers(Both_MARKERS,sensor_array))
		{
			return false;
		}
		
	}
	if(!SetPeriodicQuery(period))
	{
		return false;
	}
	return true;

}






bool mgs1600::ReadTrackDetect(int * value)
{

	std::string cmd("?D\r");

	struct parsingStruct  parsed_sensor_reply;
	
	if(!SendRequest(cmd,&parsed_sensor_reply))
	{
		return false;
	}
			
	
	if(parsed_sensor_reply.cmd != TRACK_DETECT)
	{
		
		return false;
	}
	
	if(parsed_sensor_reply.nElements > 0 && parsed_sensor_reply.nElements < 2 )
	{
		*value = parsed_sensor_reply.arg[0];
	}
	return true;
}



bool mgs1600::ReadSelectedTrack(int * value)
{

	std::string cmd("?T\r");


	struct parsingStruct  parsed_sensor_reply;
	
	if(!SendRequest(cmd,&parsed_sensor_reply))
	{
		return false;
	}
			
	
	if(parsed_sensor_reply.cmd != SELECTED_TRACK)
	{
		
		return false;
	}
	
	if(parsed_sensor_reply.nElements > 0 && parsed_sensor_reply.nElements < 2 )
	{
		*value = parsed_sensor_reply.arg[0];
	}
	return true;
	
}


bool mgs1600::ReadInternalSensors(uint16_t sensors,int * arraySensorsValue)
{

	char cstr[3];
	std::string cmd("?MZ");
	
	if(sensors > 0 && sensors <= 16)
	{
		sprintf(cstr," %d\r",sensors);
		cmd+=cstr;
	}else
	{
		cmd+='\r';
	}
	

	std::string sensor_answer;
	struct parsingStruct  parsed_sensor_reply;
	
	if(!SendRequest(cmd,&parsed_sensor_reply))
	{
		return false;
	}
			
	
	
	if(parsed_sensor_reply.cmd != INTERNAL_SENSORS)
	{
		
		return false;
	}
	
	for(int i = 0; i <  parsed_sensor_reply.nElements; i++)
	{
		arraySensorsValue[i]=parsed_sensor_reply.arg[i];
	}
	return true;
	
}


bool mgs1600::ReadTracks(uint16_t sensors,int * arraySensorsValue)
{

	char cstr[3];
	std::string cmd("?TS");
	if(sensors > 0 && sensors <= 2)
	{
		sprintf(cstr," %d\r",sensors);
		cmd+=cstr;
	}else
	{
		cmd+='\r';
	}
	

	std::string sensor_answer;
	struct parsingStruct  parsed_sensor_reply;
	
	if(!SendRequest(cmd,&parsed_sensor_reply))
	{
		return false;
	}
			
	
	//ROS_INFO("cmd %d",parsed_sensor_reply.cmd);
	if(parsed_sensor_reply.cmd != TRACKS)
	{
		
		return false;
	}
	
	for(int i = 0; i <  parsed_sensor_reply.nElements; i++)
	{
		arraySensorsValue[i]=parsed_sensor_reply.arg[i];
	}
	return true;
	
}

bool mgs1600::ReadGyro(uint16_t sensors,int * arraySensorsValue)
{

	char cstr[3];
	std::string cmd("?GY");
	if(sensors > 0 && sensors <= 3)
	{
		sprintf(cstr," %d\r",sensors);
		cmd+=cstr;
	}else
	{
		cmd+='\r';
	}
	

	std::string sensor_answer;
	struct parsingStruct  parsed_sensor_reply;
	
	if(!SendRequest(cmd,&parsed_sensor_reply))
	{
		return false;
	}
			
	
	
	if(parsed_sensor_reply.cmd != GYRO)
	{
		
		return false;
	}
	
	for(int i = 0; i <  parsed_sensor_reply.nElements; i++)
	{
		arraySensorsValue[i]=parsed_sensor_reply.arg[i];
	}
	return true;
	
}

bool mgs1600::ReadMarkers(uint16_t sensors,int * arraySensorsValue)
{

	char cstr[3];
	std::string cmd("?M");
	if(sensors > 0 && sensors <= 3)
	{
		sprintf(cstr," %d\r",sensors);
		cmd+=cstr;
	}else
	{
		cmd+='\r';
	}
	

	std::string sensor_answer;
	struct parsingStruct  parsed_sensor_reply;
	
	if(!SendRequest(cmd,&parsed_sensor_reply))
	{
		return false;
	}
			
	
	
	if(parsed_sensor_reply.cmd != MARKERS)
	{
		
		return false;
	}
	
	for(int i = 0; i <  parsed_sensor_reply.nElements; i++)
	{
		arraySensorsValue[i]=parsed_sensor_reply.arg[i];
	}
	return true;
	
}


// substitui o um caracter dentro de um array por outro
bool mgs1600::swapCharacter(char * buffer, char findCharacter, char swapCharacter)
{
	bool flag_found_character = false;
	for(int i = 0;i<strlen(buffer);i++)
	{
		if(buffer[i] == findCharacter)
		{
			buffer[i] = swapCharacter;
			flag_found_character = true;	
		}
	}
	
	return flag_found_character;
}


uint16_t mgs1600::BreakMsgInLines(std::string cmd, std::string * array)
{
	 // proviório
	int character_counter=0;
	std::size_t index[16];
	int str_length = cmd.length();

	if(cmd.length() <= 0)
	{
		return false; // string is empty
	}
	for(int i = 0; i < str_length; i++)
	{
		index[character_counter] = cmd.find('*',i);
		if(index[character_counter] == std::string::npos)
		{
			return false;
		}
		i=(int)index[character_counter];
		character_counter++;
	}
	for(int j = 0; j < character_counter;j++)
	{
		int begin_str_index = (index[j-1]+1);
		int sub_str_length =index[j] - begin_str_index;

		if(j == 0)
		{
			array[j] = cmd.substr(0,index[j]);
		}else
		{
			array[j] = cmd.substr(begin_str_index,sub_str_length);
		}
		//ROS_INFO("str %d %s",j,array[j].c_str());
	}
	return character_counter;
}



bool mgs1600::ParsingLine(std::string line,struct parsingStruct * parsedLine)
{
	//ROS_INFO("I'm parsing");

	if(line.length() <= 1)
	{
		ROS_INFO("Line to short");
		return false;
	}
	
	size_t equal_position_index = line.find('=');
	if(equal_position_index == std::string::npos)
	{
		ROS_INFO("no = found");
		return false;
	}
	
	std::string cmd 		= line.substr(0,equal_position_index);
	int character_counter	=	0;
	int str_length 			= line.length();
	std::size_t index[16];
	
	for(int i = 0; i < str_length; i++)
	{
		index[character_counter] = line.find(':',i);
		if(index[character_counter] == std::string::npos)
		{
			//ROS_INFO("Found  %d", character_counter);
			break;
		}
		i=(int)index[character_counter];
		character_counter++;
	}
	
	std::string value;
	for(int j = 0; j <= character_counter;j++)
	{
		int begin_str_index;
		int sub_str_length;
		
		if(j==0) // primeiro 
		{
			begin_str_index = equal_position_index+1;
			sub_str_length = index[j] - begin_str_index;
			
		}else if(j ==  character_counter) // ultimo
		{
			begin_str_index =index[j-1]+1;
			sub_str_length = str_length -begin_str_index ;
		}
		else // entremédios
		{
			begin_str_index =index[j-1]+1;
			sub_str_length = index[j] - begin_str_index;
		}
		
		value = line.substr(begin_str_index,sub_str_length);
		
		parsedLine->arg[j] = atoi(value.c_str());
		
		//ROS_INFO("str %d %s int %d",j,value.c_str(),parsedLine->arg[j]);
	}
	
	parsedLine->nElements = character_counter+1;
	
	if(cmd == std::string("MZ"))
	{
		parsedLine->cmd = INTERNAL_SENSORS;
		
	}else if(cmd == std::string("T"))
	{
		parsedLine->cmd = SELECTED_TRACK;
	}else if(cmd == std::string("D"))
	{
		parsedLine->cmd = TRACK_DETECT;
	}else if(cmd == std::string("TS"))
	{
		parsedLine->cmd = TRACKS;
	}else if(cmd == std::string("GY"))
	{
		parsedLine->cmd = GYRO;
	}else if(cmd == std::string("M"))
	{
		parsedLine->cmd = MARKERS;
	}
	//ROS_INFO("cmd %d %d",parsedLine->cmd,parsedLine->nElements);
	return true;
}

