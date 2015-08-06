/*
author Jake Fountain
This code is part of mocap-kinect experiments.
The sensor plant is responsible for fusing multiple measurements*/
#include <armadillo>
#include <chrono>
#include <string>
#include "MocapStream.h"

#ifndef AUTOCAL_SENSOR_PLANT
#define AUTOCAL_SENSOR_PLANT

namespace autocal {
	
	class SensorPlant{
		std::map<std::string, MocapStream> streams;
	public:
		void addStream(const std::string& name, const MocapStream& s){
			streams[name] = s;
		}

		void addMeasurement(const std::string& name, const TimeStamp& timeStamp, const unsigned int& rigidBodyId, const arma::vec3& position, const arma::mat33& rotation);
	};

}
#endif