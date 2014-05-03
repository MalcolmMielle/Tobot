/*
//First edition by old000: C.H.Hung , NTU,EE97,master student
//Second edition by kenji: Y.C.Lin , NTU,EE97,master student
//          Idea from code by W.L.Hsu, NTU,EE98,PHD student

// MOUAHAHHAHA MALCOLM IS THERE ! :D
*/
#ifndef _SERIALPORTCONTROL_HPP
#define _SERIALPORTCONTROL_HPP
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <deque>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <string>
#include <stdexcept>
#include <SerialStream.h>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"

int testin;

//That must be parameters !!!
//#define TICKNUM 500000
//#define PORT "/dev/ttyUSB0"

class SerialPortControl{
	protected :
	//ATTRIBUTS 
	LibSerial::SerialStream _motor;
	int _lEnco;
	int _rEnco;
	float _targetSRW; //Speed in rpm
	float _targetSLW;
	float _measuredSRW; //Speed of right wheel mesured
	float _measuredSLW;
	double _nTick;
	std::string _LWHEEL;
	std::string _RWHEEL;
	bool _verbose;
	bool _readcorrectly;
	
	public:
	SerialPortControl(int maxspeed) : _lEnco(0),_rEnco(0),_targetSRW(0),_targetSLW(0),_measuredSRW(0),_measuredSLW(0), _verbose(false), _readcorrectly(true){
		ros::param::get("/TobotDriver/NodeleftWheel", _LWHEEL);
		ros::param::get("/TobotDriver/NodeRightWheel", _RWHEEL);
		ros::param::get("/TobotDriver/TICKNUM", _nTick);
		std::string PORT;
		ros::param::get("/TobotDriver/Port", PORT);
		double Baud;
		ros::param::get("/TobotDriver/BaudRate", Baud);
		
		_motor.Open(PORT);		
		//8 data bits
		//1 stop bit
		//No parity
		 if ( ! _motor.good() ){
			std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
			<< "Error: Could not open serial port "<<PORT
			<< std::endl ;
			exit(1) ;
		}
		
		if(Baud==115200){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
			std::cout<<"Going for value of 115200"<<std::endl;
		}
		else if(Baud==1200){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_1200);
			std::cout<<"Going for value of 1200"<<std::endl;
		}
		else if(Baud==2400){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_2400);
			std::cout<<"Going for value of 2400"<<std::endl;
		}
		else if(Baud==19200){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_19200);
			std::cout<<"Going for value of 19200"<<std::endl;
		}
		else if(Baud==38400){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_38400);
			std::cout<<"Going for value of 38400"<<std::endl;
		}
		else if(Baud==57600){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_57600);
			std::cout<<"Going for value of 57600"<<std::endl;
		}
		else if(Baud==9600){
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
			std::cout<<"Going for default value of 9600"<<std::endl;
		}
		else{
			std::cout<<"Value incorrect. Going for default value of 9600"<<std::endl;
			_motor.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);
		}
			  
		if ( ! _motor.good() ){
			std::cerr << "Error: Could not set the baud rate." <<std::endl;
			exit(1);
		}		
		
		
		_motor.SetCharSize(LibSerial::SerialStreamBuf::CHAR_SIZE_8);
		if ( ! _motor.good() ){
			std::cerr << "Error: Could not set the char size." <<std::endl;
			exit(1);
		}
		_motor.SetParity( LibSerial::SerialStreamBuf::PARITY_NONE ) ;
		if ( ! _motor.good() ){
			std::cerr << "Error: Could not set the parity." <<std::endl;
			exit(1);
		}
		_motor.SetFlowControl(LibSerial::SerialStreamBuf::FLOW_CONTROL_NONE ) ;
		if ( ! _motor.good() ){
			std::cerr << "Error: Could not set the control." <<std::endl;
			exit(1);
		}
		_motor.SetNumOfStopBits(1) ;
		if ( ! _motor.good() ){
			std::cerr << "Error: Could not set the stopbit." <<std::endl;
			exit(1);
		}
		
		//setMax Min speed
		writeDisable(); //Stop the motor in case the robot was still moving
		writeEnable();
		writeHome(); //Set the initial position to zero ;)
		writeMaxSpeed(maxspeed);
		writeMinSpeed(-maxspeed);

	}
	
	~SerialPortControl(){
		_motor.Close();
	}
	
	/**************************************************
	Accessors
	**************************************************/
	int getLencoder(){return _lEnco;}
	int getRencoder(){return _rEnco;}
	float getTargetSLW(){return _targetSLW;}
	float getTargetSRW(){return _targetSRW;}
	float getMeasuredSLW(){return _measuredSLW;}
	float getMeasuredSRW(){return _measuredSRW;}
	LibSerial::SerialStream& getMotor(){return _motor;}
	double getTickNumber(){return _nTick;}
	bool getReadState(){return _readcorrectly;}
	/**************************************************
	Read values in the controller
	**************************************************/
	int readLencoder(); //Left wheel node 0
	int readRencoder(); //Right wheel node 1
	int readTargetSLW();
	int readTargetSRW();
	int readRealSRW();
	int readRealSLW();
	int readEncoderResolution();

	/**************************************************
	Set values Manually
	**************************************************/	
	void setTargetSLW(float s){_targetSLW=s;}
	void setTargetSRW(float s){_targetSRW=s;}
	void setVerbose(){_verbose=true;}
	void setReadState(){_readcorrectly=true;}
	
	/**************************************************
	Write values in the controller
	**************************************************/
	void writeEnable();
	void writeDisable();
	void writeGoEncoderIndex();
	void writeHome();
	void writeAcc(int acc);
	void writeDec(int dec);
	void writeMaxSpeed(int ms){
		try{
			std::string MaxSpeed=boost::lexical_cast<std::string>(ms);
			MaxSpeed="SP"+MaxSpeed+"\n";
			writePort(MaxSpeed);
		}
		catch(boost::bad_lexical_cast& blc){
		  std::cout << "Exception in Max Speed" << blc.what() << std::endl;
		  _readcorrectly=false;
		  scanf("%d",&testin);
		}
	}
	void writeMinSpeed(int ms){
		try{
			std::string MaxSpeed=boost::lexical_cast<std::string>(ms);
			MaxSpeed="MV"+MaxSpeed+"\n";
			writePort(MaxSpeed);
		}
		catch(boost::bad_lexical_cast& blc){
		  std::cout << "Exception in Min Speed" << blc.what() << std::endl;
		  scanf("%d",&testin);
		  _readcorrectly=false;
		}
	}
	void writeSpeed(int ms){
		try{
			std::string MaxSpeed=boost::lexical_cast<std::string>(ms);
			MaxSpeed="V"+MaxSpeed+"\n";
			writePort(MaxSpeed);
		}
		catch(boost::bad_lexical_cast& blc){
		  std::cout << "Exception in Speed" << blc.what() << std::endl;
		  scanf("%d",&testin);
		  _readcorrectly=false;
		}
	}
	void writeTargetSRW(int ms){
		try{
			std::string MaxSpeed=boost::lexical_cast<std::string>(ms);
			MaxSpeed=_RWHEEL+"V"+MaxSpeed+"\n";
			writePort(MaxSpeed);
		}
		catch(boost::bad_lexical_cast& blc){
		  std::cout << "Exception in Right wheel Speed" << blc.what() << std::endl;
		  scanf("%d",&testin);
		  _readcorrectly=false;
		}
	}
	void writeTargetSLW(int ms){
		try{
			std::string MaxSpeed=boost::lexical_cast<std::string>(ms);
			MaxSpeed=_LWHEEL+"V"+MaxSpeed+"\n";
			writePort(MaxSpeed);
		}
		catch(boost::bad_lexical_cast& blc){
		  std::cout << "Exception in Left wheel Speed" << blc.what() << std::endl;
		  scanf("%d",&testin);
		  _readcorrectly=false;
		}	
	}
	void writePoseRelativeR(int ms){
		try{
			std::string pos=boost::lexical_cast<std::string>(ms);
			pos=_RWHEEL+"LR"+pos+"\n";
			writePort(pos);
		}
		catch(boost::bad_lexical_cast& blc){
		  std::cout << "Exception in Relative Right position" << blc.what() << std::endl;
		  scanf("%d",&testin);
		  _readcorrectly=false;
		}
	}
	void writePoseRelativeL(int ms){
		try{
			std::string pos=boost::lexical_cast<std::string>(ms);
			pos=_LWHEEL+"LR"+pos+"\n";
			writePort(pos);
		}
		catch(boost::bad_lexical_cast& blc){
		  std::cout << "Exception in Relative Left position" << blc.what() << std::endl;
		  scanf("%d",&testin);
		  _readcorrectly=false;
		}
	}
	void writePoseAbsoluteR(int ms){
		try{
			std::string pos=boost::lexical_cast<std::string>(ms);
			pos=_RWHEEL+"LA"+pos+"\n";
			writePort(pos);
		}
		catch(boost::bad_lexical_cast& blc){
		  std::cout << "Exception in Absolute Right Position" << blc.what() << std::endl;
		  scanf("%d",&testin);
		  _readcorrectly=false;
		}

	}
	void writePoseAbsoluteL(int ms){
		try{
			std::string pos=boost::lexical_cast<std::string>(ms);
			pos=_LWHEEL+"LA"+pos+"\n";
			writePort(pos);
		}
		catch(boost::bad_lexical_cast& blc){
		  std::cout << "Exception in Absolute Left Position" << blc.what() << std::endl;
		  scanf("%d",&testin);
		  _readcorrectly=false;
		}
	}
	void writeMove(){
		std::string s="M\n";
		writePort(s);
	}
	/**************************************************
	Basic serial operations
	**************************************************/
	
	void writePort(std::string& str);
	int readPort();

	/**************************************************
	Updates
	**************************************************/	
	void update(int speedRwheel, int speedLwheel);
};

/*********************************************************************************************************
FONCTIONS
*********************************************************************************************************/

/********************************Read values in the controller**************************************/

inline int SerialPortControl::readLencoder(){
	std::string yo("POS\n");
	yo=_LWHEEL+yo;
	writePort(yo);
	return readPort();
}

inline int SerialPortControl::readRencoder(){
	std::string yo("POS\n");
	yo=_RWHEEL+yo;
	writePort(yo);
	return readPort();
}

inline int SerialPortControl::readEncoderResolution(){
	std::string yo("GENCRES\n");
	writePort(yo);
	return readPort();
	
}


inline int SerialPortControl::readTargetSRW(){
	std::string speed2("GV\n");
	speed2=_RWHEEL+speed2;
	writePort(speed2);
	return readPort();
}

inline int SerialPortControl::readTargetSLW(){
	std::string speed2("GV\n");
	speed2=_LWHEEL+speed2;
	writePort(speed2);
	return readPort();
}


inline int SerialPortControl::readRealSRW(){	
	std::string speed2("GN\n");
	speed2=_RWHEEL+speed2;
	writePort(speed2);
	return readPort();
}

inline int SerialPortControl::readRealSLW(){	
	std::string speed2("GN\n");
	speed2=_LWHEEL+speed2;
	writePort(speed2);
	return readPort();

}

/*****************************CFULHABER CONTROLER CONFIGURATION*******************************/

inline void SerialPortControl::writeEnable(){
	std::string enable("en\n");
	SerialPortControl::writePort(enable);
}

inline void SerialPortControl::writeDisable(){
	std::string enable("di\n");
	SerialPortControl::writePort(enable);
}

inline void SerialPortControl::writeGoEncoderIndex(){
	std::string enable("GOIX\n");
	SerialPortControl::writePort(enable);
}
inline void SerialPortControl::writeHome(){
	std::string enable("HO\n");
	SerialPortControl::writePort(enable);
}
inline void SerialPortControl::writeAcc(int acc){
	std::string accc=boost::lexical_cast<std::string>(acc);
	std::string enable("AC"+accc+"\n");
	SerialPortControl::writePort(enable);
}
inline void SerialPortControl::writeDec(int dec){
	std::string decc=boost::lexical_cast<std::string>(dec);
	std::string enable("DEC"+decc+"\n");
	SerialPortControl::writePort(enable);
}

/****************************Basic functon*******************************************/

inline void SerialPortControl::writePort(std::string& str){
	_motor << str;//lpBufferToWrite;
	if(_verbose==true){
		std::cout << "wrote the demand "<<str.c_str()<<" written " << std::endl;
	}
}

inline int SerialPortControl::readPort(){
	std::string i;
	if(_verbose==true){
		if(_motor.rdbuf()->in_avail() >0){	
			_motor >> i;
			if(i.size()==0){i="fail";}
			std::cout<<"we read "<<i<<std::endl;
			try{
				return boost::lexical_cast<int>(i);
			}
			catch(boost::bad_lexical_cast& blc){
			  std::cout << "Exception in readPort" << blc.what()<< " this is the reason" <<i<< "VOILA" << std::endl;
			  scanf("%d",&testin);
			  _readcorrectly=false;
			}
		}
	}
	else{
		if(_motor.rdbuf()->in_avail() >0){
			_motor >> i;
			if(i.size()==0){i="fail";}
			try{
				return boost::lexical_cast<int>(i);
			}
			catch(boost::bad_lexical_cast& blc){
			  std::cout << "Exception in readPort" << blc.what() << std::endl;
	  		  scanf("%d",&testin);
	  		  _readcorrectly=false;
			}
		}
	}
	
}

/**************************Mise à jour des vitesse*******************************************/
/*Speeds must be in rpm at that point*/
inline void SerialPortControl::update(int speedRwheel, int speedLwheel){
	if(_verbose==true){
		std::cout<<"ON FAIT DES TRUCS COOLS AUX MOTEURS..."<<std::endl;
	}
	_targetSRW=speedRwheel;
	_targetSLW=speedLwheel;	
	
	//TO REMOVE
	//writeSpeed(speedRwheel);
	//Write the target speeds
	
	writeTargetSRW(speedRwheel);
	writeTargetSLW(speedLwheel);
	
}


#endif
