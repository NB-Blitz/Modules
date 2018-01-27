/*
 * BlitzLogger.cpp
 *
 *  Created on: Aug 5, 2017
 *      Author: Sam
 */
#include <BlitzLogger.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>


/* Constructor for BlitzLogger Class
 *
 * Logs everything from your input and below
 *
 * @parameter: logLevel
 *
 * Error = 0,
 * Warning = 1,
 * Info = 2,
 * Debug = 3,
 * Trace = 4
 */
FRC::BlitzLogger::BlitzLogger(int logLevel)
{
	this->logLevel = logLevel;
}

void FRC::BlitzLogger::init()
{
	std::string dateTime;
	dateTime = "/home/lvuser/";
	dateTime.append(spaceToUnderscore(getTimeStamp()));
	dateTime.append(".txt");
	SmartDashboard::PutString("StringTest", dateTime);
	logfile = fopen(dateTime.c_str(), "a");
}

std::string FRC::BlitzLogger::spaceToUnderscore(std::string text)
{
	for(int i = 0; i < text.length(); i++)
	{
		if(text[i] == ' ')
		{
			text[i] = '_';
		}
		else if(text[i] == ':')
		{
			text[i] = '_';
		}
	}
	return text;
}

void FRC::BlitzLogger::close()
{
	fflush(logfile);
	fclose(logfile);
}

std::string FRC::BlitzLogger::getTimeStamp()
{
	std::chrono::system_clock::time_point p = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(p);
	std::string timeStamp = ctime(&t);
	timeStamp = timeStamp.substr(0,timeStamp.length()-1);
	return timeStamp;
}

std::string FRC::BlitzLogger::getTicksElapsed()
{

}

void FRC::BlitzLogger::log(std::string currentStage, std::string level, std::string message)
{
	std::string timeStamp = FRC::BlitzLogger::getTimeStamp();
	if(logfile != NULL)
	{
		fputs("[", logfile);
		fputs(timeStamp.c_str(), logfile);//writes timestamp to file
		fputs("]", logfile);
		fputs("", logfile); //Separating time stamp and message type
		fputs("[", logfile);
		fputs(currentStage.c_str(), logfile); //writes timestamp to file
		fputs("]", logfile);
		fputs("", logfile); //Separating time stamp and message type
		fputs("[", logfile);
		fputs(level.c_str(), logfile); //writes timestamp to file
		fputs("]", logfile);
		fputs("", logfile); //Separating message type and message
		fputs(message.c_str(), logfile); //Writes message to file
		fputc('\r\n', logfile); //Sets writing location to next line for next log message
	}
}

void FRC::BlitzLogger::error(std::string currentStage, std::string message)
{
	if(logLevel >= Error)
	{
		log(currentStage, "Error", message);
	}
}

void FRC::BlitzLogger::warning(std::string currentStage, std::string message)
{
	if(logLevel >= Warning)
	{
		log(currentStage, "Warning", message);
	}
}

void FRC::BlitzLogger::info(std::string currentStage, std::string message)
{
	if(logLevel >= Error)
	{
		log(currentStage, "Info", message);
	}
}

void FRC::BlitzLogger::debug(std::string currentStage, std::string message)
{
	if(logLevel >= Debug)
	{
		log(currentStage, "Debug", message);
	}
}

void FRC::BlitzLogger::trace(std::string currentStage, std::string message)
{
	if(logLevel >= Trace)
	{
		log(currentStage, "Trace", message);
	}
}
