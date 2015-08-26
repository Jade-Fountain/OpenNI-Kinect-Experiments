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
	
	// //Correlate stream 1 to stream 2
	// std::vector<std::pair<int,int>> SensorPlant::getCorrelations(std::string stream_name_1, std::string stream_name_2, TimeStamp now){
	// 	std::vector<std::pair<int,int>> correlations;

	// 	MocapStream& stream1 = mocapRecording.getStream(stream_name_1);
	// 	MocapStream& stream2 = mocapRecording.getStream(stream_name_2);
	// 	std::pair<std::string,std::string> hypothesisKey({stream_name_1,stream_name_2});

	// 	//Initialise eliminated hypotheses if necessary
	// 	if(eliminatedHypotheses.count(hypothesisKey) == 0){
	// 		eliminatedHypotheses[hypothesisKey] = std::set< std::pair<MocapStream::RigidBodyID,MocapStream::RigidBodyID> >();
	// 	}

	// 	//Check we have data to compare
	// 	if(stream2.size() == 0){
	// 		return correlations;
	// 	}


	// 	std::map<MocapStream::RigidBodyID, arma::vec>
	// 		currentState2 = stream2.getStates(now);
		
	// 	std::map<MocapStream::RigidBodyID, arma::vec> currentState1;
		
	// 	if(stream_name_1 == "fake_mocap"){
	// 		currentState1 = stream2.getSimulatedStates(now, {18,12});
	// 	} else {
	// 		stream1 = mocapRecording.getStream(stream_name_1);
	// 		if(stream1.size() == 0) return correlations;
 //     		currentState1 = stream1.getStates(now);
	// 	}

	// 	//Update statistics
	// 	for(auto& state1 : currentState1){
	// 		//For each rigid body to be matched
	// 		MocapStream::RigidBodyID id1 = state1.first;

	// 		float max_score = 0;
	// 		int max_score_id = 0;

	// 		for(auto& state2 : currentState2){
	// 			//For each rigid body to match to
	// 			MocapStream::RigidBodyID id2 = state2.first;

	// 			//Check if this hypothesis has been eliminated


	// 			//Generate key for the map of correlationStats
	// 			std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID> key = {id1,id2};
	// 			if(eliminatedHypotheses[hypothesisKey].count(key) != 0) continue;

	// 			arma::vec statVec = arma::join_cols(state1.second,state2.second);
	// 			int size1 = state1.second.n_rows;
	// 			int size2 = state2.second.n_rows;
	// 			int vecSize = size1 + size2;
	// 			if(correlationStats.count(key) == 0){
	// 				//Initialise if key missing. (true => calculate cov)
	// 				correlationStats[key] = arma::mat(statVec);
	// 				if(max_score == 0) {
	// 					max_score_id = id2;
	// 				}
	// 			} else {
	// 				//Add current stats to the vector
	// 				if(correlationStats[key].n_cols >= 100){
	// 					correlationStats[key] = arma::join_rows(correlationStats[key].cols(1,correlationStats[key].n_cols-1)
	// 															,statVec);
	// 				} else {
	// 					correlationStats[key] = arma::join_rows(correlationStats[key],statVec);
	// 					continue;
	// 				}					

	// 				//Get current data:
	// 				arma::mat mat1 = correlationStats[key].rows(0, size1-1);
	// 				arma::mat mat2 = correlationStats[key].rows(size1, size1+size2-1);

	// 				//------------------------------------
	// 				//				METHOD 1
	// 				//------------------------------------
	// 				// //MEASURE UNITARY MATRIX METHOD - shouldn't work but does for some reason
	// 				arma::mat B = mat2 * arma::pinv(mat1);

	// 				//B should be a unitary matrix if we have the correct hypothesis
	// 				arma::mat zeros = arma::eye(size1,size2) - B * B.t();
	// 				// float score = likelihood(arma::max(arma::max(zeros)));
	// 				float score = likelihood(arma::sum(arma::sum(arma::abs(zeros)))/10);
	// 				//------------------------------------

	// 				//Filter scores
	// 				float learningRate = 1;
	// 				//Init score to 1
	// 				if(scores.count(key) == 0){
	// 					scores[key] = 1;
	// 				}
	// 				scores[key] = score * std::pow(scores[key],1-learningRate);

	// 				std::cout << "score[" << id1 << "," << id2 << "] = " << scores[key] << std::endl;
	// 				if(scores[key] < 0.001){
	// 					std::cout << "Eliminated: " << std::endl;
	// 					std::cout << "B =\n" << B << std::endl;
	// 					std::cout << "zeros =\n" << zeros << std::endl;
	// 					eliminatedHypotheses[hypothesisKey].insert(key);
	// 				}
					

	// 				if(scores[key] > max_score){
	// 					max_score = scores[key];
	// 					max_score_id = id2;
	// 				}
	// 			}
				
	// 		}
	// 		if(max_score > 0.1){
	// 			correlations.push_back({id1,max_score_id});
	// 		}
	// 	}


	// 	return correlations;
	// }

	// //This doesnt really work.
	// std::vector<std::pair<int,int>> SensorPlant::getCorrelationsOfInvariants(std::string stream_name_1, std::string stream_name_2, TimeStamp now){
	// 	std::vector<std::pair<int,int>> correlations;

	// 	MocapStream stream1 = mocapRecording.getStream(stream_name_1);
	// 	MocapStream stream2 = mocapRecording.getStream(stream_name_2);

	// 	//Check we have data to compare
	// 	if(stream1.size() == 0 || stream2.size() == 0){
	// 		return correlations;
	// 	}

	// 	std::map<MocapStream::RigidBodyID, arma::vec>
	// 		invariates1 = stream1.getInvariates(now);

	// 	std::map<MocapStream::RigidBodyID, arma::vec>
	// 		invariates2 = stream2.getInvariates(now);

	// 	//match invariates and store correlations
	// 	//TODO: parallelisable?
		
	// 	for(auto& inv1 : invariates1){
	// 		std::map<MocapStream::RigidBodyID,float> weight_map;
	// 		if(int(inv1.first) == 1){
	// 			std::cout << inv1.second[0] << " ";
	// 		}
	// 		for(auto& inv2 : invariates2){
	// 			//DEBUG
	// 			if(int(inv1.first) == 1){
	// 				std::cout << inv2.second[0] << " ";
	// 			}
				
	// 			//Just rotation size comparison
	// 			float error = arma::norm(inv2.second - inv1.second);
	// 			weight_map[inv2.first] = likelihood(error);
	// 		}
			
	// 		// if(int(inv1.first) == 1) std::cout << std::endl;

	// 		if(weight_map.size()!=0){
	// 			if(linkWeights.count(inv1.first) == 0){
	// 				linkWeights[inv1.first] = weight_map;
	// 			}else{
	// 				linkWeights[inv1.first] = multiply(linkWeights[inv1.first],weight_map);
	// 				// linkWeights[inv1.first] = weight_map;
	// 			}
	// 		}
	// 	}

	// 	//Sort and get most likely match for each rigidBody
	// 	if(linkWeights.size() > 0){
	// 		for(auto& link : linkWeights){
	// 			//Resort weights by flipping map
	// 			std::multimap<float,MocapStream::RigidBodyID> sortedWeights = flip_map(link.second);
	// 			//Get largest weight (reverse iterator begins at largest)
	// 			auto highestWeightLink = sortedWeights.rbegin();
	// 			//Push back pair matching link RB id and highest weight link ID
	// 			correlations.push_back(std::make_pair(link.first,highestWeightLink->second));

	// 			// // //DEBUG
	// 			// std::cout << "RB ID = " << int(link.first) << std::endl;
	// 			// std::cout << "highestWeight ID = " << highestWeightLink->second << std::endl;
	// 			// std::cout << "highestWeight value = " << highestWeightLink->first << std::endl;
	// 		}
	// 	}

	// 	return correlations;
	// }

	std::vector<std::pair<int,int>> SensorPlant::matchStreams(std::string stream_name_1, std::string stream_name_2, TimeStamp now){
		std::cout << "FRAME BEGIN"  << std::endl;
		std::vector<std::pair<int,int>> empty_result;

		MocapStream& stream1 = mocapRecording.getStream(stream_name_1);
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
		if(stream_name_1 == "fake_mocap"){
			currentState1 = stream2.getCompleteSimulatedStates(now, {18,12});
		} else {
			stream1 = mocapRecording.getStream(stream_name_1);
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

		std::cout << "FRAME END"  << std::endl;

		return correlator.getBestCorrelations();
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
		// groundTruthTransforms[std::make_pair(streamA, streamB)].translation() += arma::vec3{-0.38,0,0};

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


















}
