/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "SensorPlant.h"
#include "utility/math/matrix/Transform3D.h"
#include "arma_xn_tools.h"
#include <math.h>
#include <XnCppWrapper.h>

namespace autocal {
	using utility::math::matrix::Transform3D;
	

	std::vector<std::pair<int,int>> SensorPlant::getCorrelations(std::string stream_name_1, std::string stream_name_2, TimeStamp now){
		std::vector<std::pair<int,int>> correlations;

		MocapStream stream1 = mocapRecording.getStream(stream_name_1);
		MocapStream stream2 = mocapRecording.getStream(stream_name_2);

		//Check we have data to compare
		if(stream1.size() == 0 || stream2.size() == 0){
			return correlations;
		}

		std::map<MocapStream::RigidBodyID, Transform3D>
			invariates1 = stream1.getInvariates(now);

		std::map<MocapStream::RigidBodyID, Transform3D>
			invariates2 = stream2.getInvariates(now);

		//match invariates and store correlations
		//TODO: parallelisable?
		
		for(auto& inv1 : invariates1){
			std::map<MocapStream::RigidBodyID,float> weight_map;
			for(auto& inv2 : invariates2){
				//Total transform norm:
				// float error = std::fabs(Transform3D::norm(inv2.second.i()) - Transform3D::norm(inv1.second));
				
				//DEBUG
				if(int(inv1.first) == 1 && int(inv2.first) == (XN_SKEL_RIGHT_SHOULDER)){
					//Correct hypothesis
					arma::vec3 d1 = inv1.second.translation();
					arma::vec3 d2 = inv2.second.translation();
					std::cout << d1[0] << " " << d1[1] << " " << d1[2] << " " << d2[0] << " " << d2[1] << " " << d2[2] << std::endl;
				}
				
				//Just position norm
				float error = std::abs(arma::norm(inv2.second.translation()) - arma::norm(inv1.second.translation()));
				weight_map[inv2.first] = likelihood(error);
			}
			if(weight_map.size()!=0){
				if(linkWeights.count(inv1.first) == 0){
					linkWeights[inv1.first] = weight_map;
				}else{
					linkWeights[inv1.first] = multiply(linkWeights[inv1.first],weight_map);
					// linkWeights[inv1.first] = weight_map;
				}
			}
		}

		//Sort and get most likely match for each rigidBody
		if(linkWeights.size() > 0){
			for(auto& link : linkWeights){
				//Resort weights by flipping map
				std::multimap<float,MocapStream::RigidBodyID> sortedWeights = flip_map(link.second);
				//Get largest weight (reverse iterator begins at largest)
				auto highestWeightLink = sortedWeights.rbegin();
				//Push back pair matching link RB id and highest weight link ID
				correlations.push_back(std::make_pair(link.first,highestWeightLink->second));

				// // //DEBUG
				// std::cout << "RB ID = " << int(link.first) << std::endl;
				// std::cout << "highestWeight ID = " << highestWeightLink->second << std::endl;
				// std::cout << "highestWeight value = " << highestWeightLink->first << std::endl;
			}
		}

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
					//Hack correction
					rb.second.pose.translation() += arma::vec3{-0.38,0,0};
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


















}
