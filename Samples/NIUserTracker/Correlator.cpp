/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "Correlator.h"

namespace autocal {

	using utility::math::matrix::Transform3D;
	using utility::math::matrix::Rotation3D;
	using utility::math::geometry::UnitQuaternion;


		void Correlator::addData(MocapStream::RigidBodyID id1, Transform3D T1, MocapStream::RigidBodyID id2, Transform3D T2){
			//Generate key for the map of correlationStats
			std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID> key = {id1,id2};

			//Check if this hypothesis has been eliminated
			if(eliminatedHypotheses.count(key) != 0) return;

			if(recordedStates.count(key) == 0){
				//Initialise if key missing. (true => calculate cov)
				recordedStates[key] = std::pair<std::vector<utility::math::matrix::Transform3D>, std::vector<utility::math::matrix::Transform3D>>();
			} else {
				//Add current stats to the vector
				bool noRecordedStates = recordedStates[key].first.empty() || recordedStates[key].second.empty();
				
				Transform3D lastTransform1 = recordedStates[key].first.empty() ? Transform3D() : recordedStates[key].first.back().i();
				float diff1 = Transform3D::norm(lastTransform1 * T1);
				
				Transform3D lastTransform2 = recordedStates[key].second.empty() ? Transform3D() : recordedStates[key].second.back().i();
				float diff2 = Transform3D::norm(lastTransform2 * T2);

					//DEBUG
					// std::cout << "number of samples = " << recordedStates[key].first.size() << std::endl;
					// std::cout << "newness of measurement 1 = " << diff1 << std::endl;
					// std::cout << "newness of measurement 2 = " << diff2 << std::endl;

				if( noRecordedStates ||
					diff1 > difference_threshold || 
					diff2 > difference_threshold)
				{
					if(recordedStates[key].first.size() >= number_of_samples){
						recordedStates[key].first.erase(recordedStates[key].first.begin());
						recordedStates[key].first.push_back(T1);
						recordedStates[key].second.erase(recordedStates[key].second.begin());
						recordedStates[key].second.push_back(T2);
						//Now we are ready to compute
						computableStreams.insert(key);
					} else {
						recordedStates[key].first.push_back(T1);
						recordedStates[key].second.push_back(T2);
						return;
					}
				}

			}
		}

		void Correlator::eliminateAndNormalise(std::map<MocapStream::RigidBodyID,float> totalScores){
			//Normalise scores and eliminate low scores
			for (auto& s : scores){
				const auto& pairID = s.first;
				const auto& id1 = pairID.first;
				float& score = s.second;
				if(eliminatedHypotheses.count(pairID) != 0) continue;
				
				if(totalScores[id1] != 0){
					//Normalise
					score = score / totalScores[id1];
					//Eliminate
					if(score < elimination_score_threshold && eliminatedHypotheses.count(pairID) == 0){
						eliminatedHypotheses.insert(pairID);
						std::cout << "Eliminated: [" << pairID.first << "," << pairID.second << "]" << std::endl;
					}						
				}
			}
		}

		bool Correlator::sufficientData(){
			for(auto& hypothesis : recordedStates){
				const auto& key = hypothesis.first;
				if(computableStreams.count(key) == 0) return false;
			}
			return true;
		}

		void Correlator::compute(){
			std::map<MocapStream::RigidBodyID,float> totalScores;
			for(auto& hypothesis : recordedStates){
				//Extract data from its confusing encrypted data structure
				const auto& key = hypothesis.first;
				const auto& id1 = key.first;
				const auto& id2 = key.second;
				std::vector<Transform3D> states1 = hypothesis.second.first;
				std::vector<Transform3D> states2 = hypothesis.second.second;
				
				//Check whether or not we need to check this hypothesis anymore
				if(eliminatedHypotheses.count(key) != 0) continue;

				//Init total scores if necessary
				if(totalScores.count(id1) == 0){
					totalScores[id1] = 0;
				}
				
				//Fit data
				auto result = CalibrationTools::solveHomogeneousDualSylvester(states1,states2);

				auto X = result.first;
				auto Y = result.second;

				//Calculate reprojection error as score
				float totalError = 0;

				for(int i = 0; i < states1.size(); i++){
					const Transform3D& A = states1[i];
					const Transform3D& B = states2[i];
					totalError += Transform3D::norm((A * X).i() * (Y * B));
				}

				float score = likelihood(totalError / float(number_of_samples));

				//Init score to 1 if not recorded or set at zero
				if(scores.count(key) == 0 || scores[key] == 0){
					scores[key] = 1;
				}

				//weight decay
				scores[key] = score * scores[key];

				std::cout << "score[" << id1 << "," << id2 << "] = " << scores[key]  <<  "  error = " << totalError << std::endl;

				totalScores[id1] += scores[key];
			}

			eliminateAndNormalise(totalScores);
		}

		std::vector<std::pair<int,int>> Correlator::getBestCorrelations(){
			//TODO: this
			std::map<MocapStream::RigidBodyID,MocapStream::RigidBodyID> bestMatches;
			std::map<MocapStream::RigidBodyID,float> bestScores;

			for(const auto& s : scores){
				const auto& key = s.first;
				const float& score = s.second;
				if(eliminatedHypotheses.count(key) != 0) continue;
				
				const auto& id1 = key.first;
				const auto& id2 = key.second;
				
				if(bestScores.count(id1) == 0){
					bestScores[id1] = score;
					bestMatches[id1] = id2;
				}
				
				if(score > bestScores[id1]){
					bestScores[id1] = score;
					bestMatches[id1] = id2;
				}
			}
			std::vector<std::pair<int,int>> result;
			for(auto& match : bestMatches){
				result.push_back(std::make_pair(int(match.first),int(match.second)));
			}
			return result;
		}

}






