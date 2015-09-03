/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "SensorPlant.h"
#include "utility/math/matrix/Transform3D.h"
#include "arma_xn_tools.h"
#include <math.h>
#include <set>
#include <XnCppWrapper.h>

namespace autocal {
	using utility::math::matrix::Transform3D;

	std::vector<std::pair<int,int>> SensorPlant::matchStreams(std::string stream_name_1, std::string stream_name_2, TimeStamp now){
		// std::cout << "FRAME BEGIN"  << std::endl;
		std::vector<std::pair<int,int>> empty_result;

		MocapStream& stream2 = mocapRecording.getStream(stream_name_2);
		
		std::pair<std::string,std::string> hypothesisKey({stream_name_1,stream_name_2});

		//Initialise eliminated hypotheses if necessary
		if(correlators.count(hypothesisKey) == 0){
			correlators[hypothesisKey] = Correlator();
		}
		auto& correlator = correlators[hypothesisKey];

		//Check we have data to compare
		if(stream2.size() == 0){
			return empty_result;
		}

		std::map<MocapStream::RigidBodyID, Transform3D> currentState2 = stream2.getCompleteStates(now);
		std::map<MocapStream::RigidBodyID, Transform3D> currentState1;
		
		//if we simulate the data, derive it from the second stream
		if(simulate){
			currentState1 = stream2.getCompleteSimulatedStates(now, {18,12}, simParams.front());
		} else {
			MocapStream& stream1 = mocapRecording.getStream(stream_name_1);
			if(stream1.size() == 0) return empty_result;
     		currentState1 = stream1.getCompleteStates(now);
		}

		//Update statistics
		for(auto& state1 : currentState1){
			//For each rigid body to be matched
			MocapStream::RigidBodyID id1 = state1.first;
			for(auto& state2 : currentState2){
				//For each rigid body to match to
				MocapStream::RigidBodyID id2 = state2.first;

				correlator.addData(id1, state1.second, id2, state2.second);	
			}
		}

		//Compute correlations
		if(correlator.sufficientData()){
			correlator.compute();
		}

		// std::cout << "FRAME END"  << std::endl;

		std::vector<std::pair<int,int>> correlations = correlator.getBestCorrelations();

		//Compute correct guesses:
		for (auto& cor : correlations){
			correctGuesses += int(cor.first == 1 && cor.second == 18 ) +
							  int(cor.first == 2 && cor.second == 12 );
		}
		totalGuesses += correlations.size();

		return correlations;

	}

	std::map<MocapStream::RigidBodyID,float> SensorPlant::multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2){
		std::map<MocapStream::RigidBodyID,float> result;
		float learningRate = 0.1;
		for(auto& x : m1){
			if(m2.count(x.first) != 0){
				//Exponential filter
				// result[x.first] = (1-learningRate) * m1[x.first] + learningRate * m2[x.first];

				//Probabilistic decay filter
				result[x.first] = m1[x.first] * std::pow(m2[x.first],1-learningRate);
			} else {
				result[x.first] = m1[x.first];
			}
		}
		//Add in keys present in m2 but not m1
		for(auto& x : m2){
			if(m1.count(x.first) == 0){
				result[x.first] = m2[x.first];
			}
		}
		return result;
	}

	void SensorPlant::setGroundTruthTransform(std::string streamA, std::string streamB, Transform3D mapAtoB, bool useTruth){
		groundTruthTransforms[std::make_pair(streamA, streamB)] = mapAtoB;
		//HACK CORRECTION
		groundTruthTransforms[std::make_pair(streamA, streamB)].translation() += arma::vec3{-0.38,0,0};
		// std::cout << "groundTruthTransforms \n" << groundTruthTransforms[std::make_pair(streamA, streamB)]<<  std::endl;

		if(useTruth){
			convertToGroundTruth(streamA, streamB);
			//Set to identity
			groundTruthTransforms[std::make_pair(streamA, streamB)] = Transform3D();
		}
	}

	void SensorPlant::convertToGroundTruth(std::string streamA, std::string streamB){
		auto key = std::make_pair(streamA, streamB);
		
		if(groundTruthTransforms.count(key) != 0 && mocapRecording.streamPresent(streamA)){
			//Get the transform between coordinate systems
			Transform3D streamToDesiredBasis = groundTruthTransforms[key];

			for(auto& frame : mocapRecording.getStream(streamA).frameList()){
				//Loop through and record transformed rigid body poses
				for (auto& rb : frame.second.rigidBodies){
					Transform3D T = streamToDesiredBasis * rb.second.pose;
					rb.second.pose = T;
				}
			}
		} else {
			std::cout << "WARNING: ATTEMPTING TO ACCESSING GROUND TRUTH WHEN NONE EXISTS!!!" << std::endl;
		}


	}

	
	std::map<int, Transform3D> SensorPlant::getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now){
		std::map<int, Transform3D> truth;
		auto key = std::make_pair(stream, desiredBasis);
		if(groundTruthTransforms.count(key) != 0 && mocapRecording.streamPresent(stream)){
			//Get the transform between coordinate systems
			Transform3D streamToDesiredBasis = groundTruthTransforms[key];

			//Get the latest data 
			MocapStream::Frame latestFrame = mocapRecording.getStream(stream).getFrame(now);

			//Loop through and record transformed rigid body poses
			for (auto& rb : latestFrame.rigidBodies){
				truth[rb.first] = streamToDesiredBasis * rb.second.pose;
			}
		} else {
			std::cout << "WARNING: ATTEMPTING TO ACCESSING GROUND TRUTH WHEN NONE EXISTS!!!" << std::endl;
		}

		return truth;
	}

	void SensorPlant::next(){
		for(auto& c : correlators){
			c.second.reset();
		}
		if(simParams.size()!=0){
			MocapStream::SimulationParameters s = simParams.front();
			simParams.pop();
			std::cerr << "Finished simulating: " << s.latency_ms << " " << s.noise.angle_stddev << " " << s.noise.disp_stddev;
		}
		std::cerr << " Fraction correct: " << correctGuesses << " " << totalGuesses << " " <<  float(correctGuesses) / float(totalGuesses) << std::endl;
		correctGuesses = 0;
		totalGuesses = 0;
	}

	void SensorPlant::setLatencyNoiseSimParameters(float l1, float l2, int lN,
									  MocapStream::SimulationParameters::Noise n1, MocapStream::SimulationParameters::Noise n2, int nN){
		simParams = std::queue<MocapStream::SimulationParameters>();//clear queue
		float lStep = (l2 - l1) / (lN-1);
		float angleStep = (n2.angle_stddev - n1.angle_stddev) / (nN-1);
		float dispStep = (n2.disp_stddev - n1.disp_stddev) / (nN-1);
		for(int i = 0; i < lN; i++){
			float l = l1 + i*lStep;
			for(int j = 0; j < nN; j++){
				float angle = n1.angle_stddev + j * angleStep;
				float disp = n1.disp_stddev + j * dispStep;

				MocapStream::SimulationParameters s;
				s.latency_ms = l;
				s.noise.angle_stddev = angle;
				s.noise.disp_stddev = disp;
				simParams.push(s);
			}
		}
	}

















}
