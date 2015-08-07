/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "SensorPlant.h"
#include "utility/math/matrix/Transform3D.h"
#include "arma_xn_tools.h"
#include <math.h>

namespace autocal {
	using utility::math::matrix::Transform3D;
	
	void SensorPlant::addMeasurement(const std::string& name, 
									 const TimeStamp& timeStamp, 
									 const unsigned int& rigidBodyId, 
									 const arma::vec3& position, 
									 const arma::mat33& rotation)
	{
		if(streams.count(name) == 0){
			streams[name] = MocapStream();
		}
		streams[name].setRigidBodyInFrame(timeStamp, rigidBodyId, position, rotation);
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
				float error = Transform3D::norm(inv2.second.i() * inv1.second);
				weight_map[inv2.first] = likelihood(error);
			}
			if(linkWeights.count(inv1.first) == 0){
				linkWeights[inv1.first] = weight_map;
			}else{
				linkWeights[inv1.first] = multiply(linkWeights[inv1.first],weight_map);
			}
		}

		//Sort and get most likely match for each rigidBody
		if(linkWeights.size() > 0){
			for(auto& link : linkWeights){
				std::cout << "int(link.first)" << int(link.first) << std::endl;
				auto sortedWeights = flip_map(link.second);
				MocapStream::RigidBodyID highestWeightLink = sortedWeights.lower_bound(1)->second;
				correlations.push_back(std::make_pair(link.first,highestWeightLink));
			}
		}

		return correlations;
	}

	std::map<MocapStream::RigidBodyID,float> SensorPlant::multiply(std::map<MocapStream::RigidBodyID,float> m1, std::map<MocapStream::RigidBodyID,float> m2){
		std::map<MocapStream::RigidBodyID,float> result;
		for(auto& x : m1){
			if(m2.count(x.first) != 0){
				result[x.first] = m1[x.first] * m2[x.first];
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



}
