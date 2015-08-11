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
		
		std::map<MocapStream::RigidBodyID,std::map<MocapStream::RigidBodyID,float>> linkWeights;
		
		TimeStamp lastLoadedTime;

		std::map<std::pair<std::string,std::string>, utility::math::matrix::Transform3D> groundTruthTransforms;


	public:
		void addStream(const std::string& name, const MocapStream& s){
			streams[name] = s;
		}

		void addMeasurement(const std::string& name, const TimeStamp& timeStamp, const unsigned int& rigidBodyId, const arma::vec3& position, const arma::mat33& rotation);
		
		void updateCorrelation();
		
		std::vector<std::pair<int,int>> getCorrelations(std::string stream_name_1, std::string stream_name_2, TimeStamp now);

		std::map<MocapStream::RigidBodyID,float> multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2);

		float likelihood(float error){
			return std::exp(-error * error / 10);
		}

		void markStartOfStreams();

		void setGroundTruthTransform(std::string streamA, std::string streamB, utility::math::matrix::Transform3D mapAtoB);

		std::map<int, utility::math::matrix::Transform3D> getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now);


	};

}
#endif