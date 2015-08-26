/*
author Jake Fountain
This code is part of mocap-kinect experiments.
The sensor plant is responsible for fusing multiple measurements*/
#include <armadillo>
#include <chrono>
#include <string>
#include <set>
#include <map>
#include "MocapStream.h"
#include "MocapRecording.h"
#include "CalibrationTools.h"
#include "Correlator.h"

#ifndef AUTOCAL_SENSOR_PLANT
#define AUTOCAL_SENSOR_PLANT

namespace autocal {
	
	class SensorPlant{
		
		std::map<MocapStream::RigidBodyID,std::map<MocapStream::RigidBodyID,float>> linkWeights;
		
		std::map<std::pair<std::string,std::string>, utility::math::matrix::Transform3D> groundTruthTransforms;

		std::map<std::pair<std::string,std::string> ,Correlator> correlators;

	public:
		SensorPlant():correlationStats(){}

		MocapRecording mocapRecording;

		void addStream(const std::string& name, const MocapStream& s){
			mocapRecording.getStream(name) = s;
		}
				
		std::vector<std::pair<int,int>> getCorrelations(std::string stream_name_1, std::string stream_name_2, TimeStamp now);
		
		std::vector<std::pair<int,int>> getCorrelationsOfInvariants(std::string stream_name_1, std::string stream_name_2, TimeStamp now);
		
		std::vector<std::pair<int,int>> matchStreams(std::string stream_name_1, std::string stream_name_2, TimeStamp now);

		std::map<MocapStream::RigidBodyID,float> multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2);

		void setGroundTruthTransform(std::string streamA, std::string streamB, utility::math::matrix::Transform3D mapAtoB, bool useTruth = false);

		void convertToGroundTruth(std::string streamA, std::string streamB);

		std::map<int, utility::math::matrix::Transform3D> getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now);


		//Stores statistics for correlating the data streams of interest
		//TODO: refactor or remove old methods
		std::map<std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID>, arma::mat > correlationStats;
	};

}
#endif