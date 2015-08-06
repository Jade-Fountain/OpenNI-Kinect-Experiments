/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "SensorPlant.h"

namespace autocal {
	
	void SensorPlant::addMeasurement(const std::string& name, 
									 const TimeStamp& timeStamp, 
									 const unsigned int& rigidBodyId, 
									 const arma::vec3& position, 
									 const arma::mat33& rotation)
	{
		if(streams.count(name) == 0){
			streams[name] = MocapStream();
		}
		streams[name].setRigidBodyInFrame(timeStamp, rigidBodyId, position, rotation);
	}


}
