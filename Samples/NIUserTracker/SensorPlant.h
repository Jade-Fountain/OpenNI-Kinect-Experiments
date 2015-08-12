/*
author Jake Fountain
This code is part of mocap-kinect experiments.
The sensor plant is responsible for fusing multiple measurements*/
#include <armadillo>
#include <chrono>
#include <string>
#include "MocapStream.h"
#include "MocapRecording.h"

#ifndef AUTOCAL_SENSOR_PLANT
#define AUTOCAL_SENSOR_PLANT

namespace autocal {
	
	class SensorPlant{
		
		std::map<MocapStream::RigidBodyID,std::map<MocapStream::RigidBodyID,float>> linkWeights;
		
		std::map<std::pair<std::string,std::string>, utility::math::matrix::Transform3D> groundTruthTransforms;

	public:
		MocapRecording mocapRecording;

		void addStream(const std::string& name, const MocapStream& s){
			mocapRecording.getStream(name) = s;
		}
		
		void updateCorrelation();
		
		std::vector<std::pair<int,int>> getCorrelations(std::string stream_name_1, std::string stream_name_2, TimeStamp now);

		std::map<MocapStream::RigidBodyID,float> multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2);

		float likelihood(float error){
			return std::exp(-error * error / 10);
		}

		void setGroundTruthTransform(std::string streamA, std::string streamB, utility::math::matrix::Transform3D mapAtoB, bool useTruth = false);

		void convertToGroundTruth(std::string streamA, std::string streamB);

		std::map<int, utility::math::matrix::Transform3D> getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now);

	};

}
#endif