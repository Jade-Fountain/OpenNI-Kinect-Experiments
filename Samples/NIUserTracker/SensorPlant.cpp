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
	
	//Correlate stream 1 to stream 2
	std::vector<std::pair<int,int>> SensorPlant::getCorrelations(std::string stream_name_1, std::string stream_name_2, TimeStamp now){
		std::vector<std::pair<int,int>> correlations;

		MocapStream& stream1 = mocapRecording.getStream(stream_name_1);
		MocapStream& stream2 = mocapRecording.getStream(stream_name_2);
		std::pair<std::string,std::string> hypothesisKey({stream_name_1,stream_name_2});

		//Initialise eliminated hypotheses if necessary
		if(eliminatedHypotheses.count(hypothesisKey) == 0){
			eliminatedHypotheses[hypothesisKey] = std::set< std::pair<MocapStream::RigidBodyID,MocapStream::RigidBodyID> >();
		}

		//Check we have data to compare
		if(stream2.size() == 0){
			return correlations;
		}


		std::map<MocapStream::RigidBodyID, arma::vec>
			currentState2 = stream2.getStates(now);
		
		std::map<MocapStream::RigidBodyID, arma::vec> currentState1;
		
		if(stream_name_1 == "fake_mocap"){
			currentState1 = stream2.getSimulatedStates(now, {18,12});
		} else {
			stream1 = mocapRecording.getStream(stream_name_1);
			if(stream1.size() == 0) return correlations;
     		currentState1 = stream1.getStates(now);
		}

		//Update statistics
		for(auto& state1 : currentState1){
			//For each rigid body to be matched
			MocapStream::RigidBodyID id1 = state1.first;

			float max_score = 0;
			int max_score_id = 0;

			for(auto& state2 : currentState2){
				//For each rigid body to match to
				MocapStream::RigidBodyID id2 = state2.first;

				//Check if this hypothesis has been eliminated


				//Generate key for the map of correlationStats
				std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID> key = {id1,id2};
				if(eliminatedHypotheses[hypothesisKey].count(key) != 0) continue;

				arma::vec statVec = arma::join_cols(state1.second,state2.second);
				int size1 = state1.second.n_rows;
				int size2 = state2.second.n_rows;
				int vecSize = size1 + size2;
				if(correlationStats.count(key) == 0){
					//Initialise if key missing. (true => calculate cov)
					correlationStats[key] = arma::mat(statVec);
					if(max_score == 0) {
						max_score_id = id2;
					}
				} else {
					//Add current stats to the vector
					if(correlationStats[key].n_cols >= 100){
						correlationStats[key] = arma::join_rows(correlationStats[key].cols(1,correlationStats[key].n_cols-1)
																,statVec);
					} else {
						correlationStats[key] = arma::join_rows(correlationStats[key],statVec);
						continue;
					}					

					//Get current data:
					arma::mat mat1 = correlationStats[key].rows(0, size1-1);
					arma::mat mat2 = correlationStats[key].rows(size1, size1+size2-1);

					//------------------------------------
					//				METHOD 1
					//------------------------------------
					// //MEASURE UNITARY MATRIX METHOD - shouldn't work but does for some reason
					arma::mat B = mat2 * arma::pinv(mat1);

					//B should be a unitary matrix if we have the correct hypothesis
					arma::mat zeros = arma::eye(size1,size2) - B * B.t();
					// float score = likelihood(arma::max(arma::max(zeros)));
					float score = likelihood(arma::sum(arma::sum(arma::abs(zeros)))/10);
					//------------------------------------

					//Filter scores
					float learningRate = 1;
					//Init score to 1
					if(scores.count(key) == 0){
						scores[key] = 1;
					}
					scores[key] = score * std::pow(scores[key],1-learningRate);

					std::cout << "score[" << id1 << "," << id2 << "] = " << scores[key] << std::endl;
					if(scores[key] < 0.001){
						std::cout << "Eliminated: " << std::endl;
						std::cout << "B =\n" << B << std::endl;
						std::cout << "zeros =\n" << zeros << std::endl;
						eliminatedHypotheses[hypothesisKey].insert(key);
					}
					

					if(scores[key] > max_score){
						max_score = scores[key];
						max_score_id = id2;
					}
				}
				
			}
			if(max_score > 0.1){
				correlations.push_back({id1,max_score_id});
			}
		}


		return correlations;
	}

	//This doesnt really work.
	std::vector<std::pair<int,int>> SensorPlant::getCorrelationsOfInvariants(std::string stream_name_1, std::string stream_name_2, TimeStamp now){
		std::vector<std::pair<int,int>> correlations;

		MocapStream stream1 = mocapRecording.getStream(stream_name_1);
		MocapStream stream2 = mocapRecording.getStream(stream_name_2);

		//Check we have data to compare
		if(stream1.size() == 0 || stream2.size() == 0){
			return correlations;
		}

		std::map<MocapStream::RigidBodyID, arma::vec>
			invariates1 = stream1.getInvariates(now);

		std::map<MocapStream::RigidBodyID, arma::vec>
			invariates2 = stream2.getInvariates(now);

		//match invariates and store correlations
		//TODO: parallelisable?
		
		for(auto& inv1 : invariates1){
			std::map<MocapStream::RigidBodyID,float> weight_map;
			// if(int(inv1.first) == 1){
			// 	std::cout << inv1.second[0] << " ";
			// }
			for(auto& inv2 : invariates2){
				//DEBUG
				// if(int(inv1.first) == 1){
				// 	std::cout << inv2.second[0] << " ";
				// }
				
				//Just position norm
				float error = arma::norm(inv2.second - inv1.second);
				weight_map[inv2.first] = likelihood(error);
			}
			
			// if(int(inv1.first) == 1) std::cout << std::endl;

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

	std::vector<std::pair<int,int>> SensorPlant::matchStreams(std::string stream_name_1, std::string stream_name_2, TimeStamp now){
		std::vector<std::pair<int,int>> correlations;

		MocapStream& stream1 = mocapRecording.getStream(stream_name_1);
		MocapStream& stream2 = mocapRecording.getStream(stream_name_2);
		std::pair<std::string,std::string> hypothesisKey({stream_name_1,stream_name_2});

		//Initialise eliminated hypotheses if necessary

		if(eliminatedHypotheses.count(hypothesisKey) == 0){
			
			eliminatedHypotheses[hypothesisKey] = std::set< std::pair<MocapStream::RigidBodyID,MocapStream::RigidBodyID> >();
		}

		//Check we have data to compare
		if(stream2.size() == 0){
			return correlations;
		}


		std::map<MocapStream::RigidBodyID, Transform3D>
			currentState2 = stream2.getCompleteStates(now);
		
		std::map<MocapStream::RigidBodyID, Transform3D> currentState1;
		
		if(stream_name_1 == "fake_mocap"){
			currentState1 = stream2.getCompleteSimulatedStates(now, {18,12});
		} else {
			stream1 = mocapRecording.getStream(stream_name_1);
			if(stream1.size() == 0) return correlations;
     		currentState1 = stream1.getCompleteStates(now);
		}


		//Update statistics
		for(auto& state1 : currentState1){
			//For each rigid body to be matched
			MocapStream::RigidBodyID id1 = state1.first;
			float totalScore = 0;

			float max_score = 0;
			int max_score_id = -1;

			for(auto& state2 : currentState2){
				//For each rigid body to match to
				MocapStream::RigidBodyID id2 = state2.first;

				//Generate key for the map of correlationStats
				std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID> key = {id1,id2};

				//Check if this hypothesis has been eliminated
				if(eliminatedHypotheses[hypothesisKey].count(key) != 0) continue;

				if(recordedStates.count(key) == 0){
					//Initialise if key missing. (true => calculate cov)
					recordedStates[key] = std::pair<std::vector<utility::math::matrix::Transform3D>, std::vector<utility::math::matrix::Transform3D>>();
					if(max_score == 0) {
						max_score_id = id2;
					}
				} else {
					//Add current stats to the vector
					int number_of_samples = 10;
					if(recordedStates[key].first.size() < number_of_samples){
						recordedStates[key].first.push_back(state1.second);
						recordedStates[key].second.push_back(state2.second);
						continue;
					}

					//Fit data
					auto result = CalibrationTools::solveHomogeneousDualSylvester(recordedStates[key].first,recordedStates[key].second);

					auto X = result.first;
					auto Y = result.second;

					//Calculate reprojection error as score
					// arma::mat totalError = arma::zeros(4,4);
					float totalError = 0;

					for(int i = 0; i < recordedStates[key].first.size(); i++){
						const Transform3D& A = recordedStates[key].first[i];
						const Transform3D& B = recordedStates[key].second[i];
						// totalError += arma::abs( A * X - Y * B);
						totalError += Transform3D::norm((A * X).i() * (Y * B));
					}

					// float score = likelihood(arma::sum(arma::sum(arma::abs(totalError)))/(number_of_samples));
					float score = likelihood(totalError / float(number_of_samples));

					//Init score to 1
					if(scores.count(key) == 0){
						scores[key] = 1;
					}
					//Filter scores
					// float learningRate = 0.1;
					// scores[key] = score * std::pow(scores[key],1-learningRate);
					
					//weight decay
					scores[key] = score * scores[key];
					
					std::cout << "score[" << id1 << "," << id2 << "] = " << scores[key] << std::endl;

					totalScore += scores[key];
					
					if(scores[key] > max_score){
						max_score = scores[key];
						max_score_id = id2;
					}

					if(recordedStates[key].first.size() >= number_of_samples){
						recordedStates[key].first.clear();
						recordedStates[key].second.clear();
					}

				}
				
			}
			//Normalise scores
			if(totalScore != 0){
				for (auto& s : scores){
					if(s.first.first == id1){
						s.second = s.second / totalScore;
						if(s.second < 0.01 && eliminatedHypotheses[hypothesisKey].count(s.first) == 0){
							std::cout << "Eliminated: [" << id1 << "," << s.first.second << "]" << std::endl;
							eliminatedHypotheses[hypothesisKey].insert(s.first);
						}						
						// std::cout << "normalised score[" << s.first.first << "," << s.first.second << "] = " << s.second << std::endl;
					}
				}
			}
			//Push back highest score
			if(max_score_id = -1){
				correlations.push_back({id1,max_score_id});
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
		//HACK CORRECTION
		groundTruthTransforms[std::make_pair(streamA, streamB)].translation() += arma::vec3{-0.38,0,0};

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
