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
	
	void SensorPlant::addMeasurement(const std::string& name, 
									 const TimeStamp& timeStamp, 
									 const unsigned int& rigidBodyId, 
									 const arma::vec3& position, 
									 const arma::mat33& rotation)
	{	
		//Create a new stream if one with this name doesnt exist
		if(streams.count(name) == 0){
			std::cout << "Initialising mocap stream: " << name << std::endl;
			streams[name] = MocapStream(name);
		}
		streams[name].setRigidBodyInFrame(timeStamp, rigidBodyId, position, rotation);
		if(lastLoadedTime == 0){
			//If we havent set the stream start yet
			lastLoadedTime = timeStamp;
			markStartOfStreams();
		} else {
			lastLoadedTime = timeStamp;
		}
	}

	std::vector<std::pair<int,int>> SensorPlant::getCorrelations(std::string stream_name_1, std::string stream_name_2, TimeStamp now){
		std::vector<std::pair<int,int>> correlations;

		MocapStream stream1 = streams[stream_name_1];
		MocapStream stream2 = streams[stream_name_2];

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
				float error = Transform3D::norm(inv2.second.i() * inv1.second);
				
				// //DEBUG
				// if(int(inv1.first) == 1 && int(inv2.first) == (XN_SKEL_LEFT_HIP)){
				// 	//Correct hypothesis
				// 	arma::vec3 d1 = inv1.second.translation();
				// 	arma::vec3 d2 = inv2.second.translation();
				// 	std::cout << d1[0] << " " << d1[1] << " " << d1[2] << " " << d2[0] << " " << d2[1] << " " << d2[2] << std::endl;
				// }
				
				//Just position norm
				// float error = std::abs(arma::norm(inv2.second.translation()) - arma::norm(inv1.second.translation()));
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

		// //Debug
		// for(auto& m : streams){
		// 	std::cout << m.second.toString() << std::endl;
		// }

		//Sort and get most likely match for each rigidBody
		if(linkWeights.size() > 0){
			for(auto& link : linkWeights){
				//Resort weights by flipping map
				std::multimap<float,MocapStream::RigidBodyID> sortedWeights = flip_map(link.second);
				//Get largest weight (reverse iterator begins at largest)
				auto highestWeightLink = sortedWeights.rbegin();
				//Push back pair matching link RB id and highest weight link ID
				correlations.push_back(std::make_pair(link.first,highestWeightLink->second));

				// //DEBUG
				// std::cout << "weights = ";
				// for (auto& weight : link.second){
				// 	std::cout << "[" << weight.first << " : " << weight.second << "], ";
				// }
				// std::cout << std::endl;
				// std::cout << "sorted weights = ";
				// for (auto& weight : sortedWeights){
				// 	std::cout << "[" << weight.second << " : " << weight.first << "], ";
				// }
				// std::cout << std::endl;
				// std::cout << "int(link.first) = " << int(link.first) << std::endl;
				// std::cout << "highestWeight ID = " << highestWeightLink->second << std::endl;
				// std::cout << "highestWeight value = " << highestWeightLink->first << std::endl;
			}
		}

		// for(auto& cor : correlations){
		// 	std::cout << stream_name_1 << "(RB "<< cor.first << ") matches " << stream_name_2 << "(RB " << cor.second << ")" << std::endl;
		// }

		return correlations;
	}

	std::map<MocapStream::RigidBodyID,float> SensorPlant::multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2){
		std::map<MocapStream::RigidBodyID,float> result;
		float learningRate = 0.5;
		for(auto& x : m1){
			if(m2.count(x.first) != 0){
				result[x.first] = (1-learningRate) * m1[x.first] + learningRate * m2[x.first];
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

	void SensorPlant::markStartOfStreams(){
		for(auto& stream : streams){
			stream.second.markStart(lastLoadedTime);
		}
	}

	void SensorPlant::setGroundTruthTransform(std::string streamA, std::string streamB, Transform3D mapAtoB){
		groundTruthTransforms[std::make_pair(streamA, streamB)] = mapAtoB;
	}

	
	std::map<int, Transform3D> SensorPlant::getGroundTruth(std::string stream, std::string desiredBasis, TimeStamp now){
		std::map<int, Transform3D> truth;
		auto key = std::make_pair(stream, desiredBasis);
		if(groundTruthTransforms.count(key) != 0 && streams.count(stream) != 0){
			//Get the transform between coordinate systems
			Transform3D streamToDesiredBasis = groundTruthTransforms[key];

			//Get the latest data 
			MocapStream::Frame latestFrame = streams[stream].getFrame(now);

			//Loop through and record transformed rigid body poses
			for (auto& rb : latestFrame.rigidBodies){
				// std::cout << "RigidBody " << rb.first << " has transform\n" << rb.second.getTransform();

				truth[rb.first] = streamToDesiredBasis * rb.second.getTransform();
				// std::cout << "after transform\n" << truth[rb.first];
			}
		} else {
			std::cout << "WARNING: ATTEMPTING TO ACCESSING GROUND TRUTH WHEN NONE EXISTS!!!" << std::endl;
		}

		return truth;
	}


















}
