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

		//Check we have data to compare
		if(stream1.size() == 0 || stream2.size() == 0){
			return correlations;
		}

		std::map<MocapStream::RigidBodyID, arma::vec>
			currentState1 = stream1.getStates(now);

		std::map<MocapStream::RigidBodyID, arma::vec>
			currentState2 = stream2.getStates(now);

		arma::mat scores(int(currentState1.end()->first),
						 int(currentState2.end()->first), 
						 arma::fill::zeros);

		//Update statistics
		for(auto& state1 : currentState1){
			//For each rigid body to be matched
			MocapStream::RigidBodyID id1 = state1.first;

			float max_score = 0;
			int max_score_id = 0;

			for(auto& state2 : currentState2){
				//For each rigid body to match to
				MocapStream::RigidBodyID id2 = state2.first;

				//Generate key for the map of correlationStats
				std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID> key = {id1,id2};

				arma::vec statVec = arma::join_cols(state1.second,state2.second);
				if(correlationStats.count(key) == 0){
					//Initialise if key missing. (true => calculate cov)
					arma::running_stat_vec<arma::vec> X(true);
					correlationStats[key] = X;
					correlationStats[key](statVec);
					max_score = 0;
					max_score_id = id2;
				} else {
					//Add current stats to the vector
					correlationStats[key](statVec);

					//Get current covariance:
					arma::mat covariance = correlationStats[key].cov();

					int size1 = state1.second.n_rows;
					int size2 = state2.second.n_rows;
					arma::mat correlation = covariance.submat(0,size1,size1-1,size1+size2-1);
					arma::mat selfCorr = covariance.submat(0,0,size1-1,size1-1);
					
					try{
						arma::mat selfCorInv = selfCorr.i();
						
						// std::cout << "correlation =\n" << correlation << std::endl;
						// std::cout << "selfCorr =\n" << selfCorr << std::endl;
						// std::cout << "selfCorInv =\n" << selfCorInv << std::endl;
						//Measure unitary matrix
						arma::mat33 B = correlation * selfCorInv;

						//B should be a unitary matrix if we have the correct hypothesis
						arma::mat33 zeros = arma::eye(3,3) - B * B.t();
						float score = likelihood(arma::max(arma::max(zeros)));

						//EIGENVALUE COMPARISON
						// arma::cx_vec eigval = eig_gen( correlation ); 
						// arma::cx_vec eigvalSelf = eig_gen( selfCorr ); 

						// float score = arma::norm(eigval-eigvalSelf);
						std::cout << "score[" << id1 << "," << id2 << "] = " << score << std::endl;
						// std::cout << "zeros =\n" << zeros << std::endl;

						if(score > max_score){
							max_score = score;
							max_score_id = id2;
						}
					} catch (std::runtime_error){
						// std::cout << __FILE__ << __LINE__ <<  " - selfCorr not invertible" << std::endl;
					}
				}
				
			}
			correlations.push_back({id1,max_score_id});
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

		MocapStream stream1 = mocapRecording.getStream(stream_name_1);
		MocapStream stream2 = mocapRecording.getStream(stream_name_2);

		//Check we have data to compare
		if(stream1.size() == 0 || stream2.size() == 0){
			return correlations;
		}

		std::map<MocapStream::RigidBodyID, std::vector<Transform3D>> data1;
		std::map<MocapStream::RigidBodyID, std::vector<Transform3D>> data2;

		std::set<TimeStamp> timestamps;

		//For each currently measured rigid body in stream one
		for(auto& rbIterator : stream1.getFrame(now).rigidBodies){
			//Get the id
			MocapStream::RigidBodyID rigidBodyID = rbIterator.first;
			//Initialise the transform vector
			data1[rigidBodyID] = std::vector<Transform3D>();
			
			//Compute iterator for now
			auto nowIterator = stream1.getUpperBoundIter(now);
			//Loop over previous frames 
			for(auto iter = stream1.begin(); iter != nowIterator; iter++){
				if(iter->second.rigidBodies.count(rigidBodyID) != 0){
					Transform3D& pose = iter->second.rigidBodies[rigidBodyID].pose;
					data1[rigidBodyID].push_back(pose);

					TimeStamp t = iter->first;
					timestamps.insert(t);
				}
			}	
		}
		// for(auto& rbIterator : stream2.getFrame(now).rigidBodies){
		// 	MocapStream::RigidBodyID rigidBodyID = rbIterator->first;
		// 	data2[rigidBodyID] = std::vector<Transform3D>();
		// }

		// for(auto& data : data1){
		// 	MocapStream::RigidBodyID rbID = data->first;

		// }

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
