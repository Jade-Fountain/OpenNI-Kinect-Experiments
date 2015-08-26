/*
author Jake Fountain
This code is part of mocap-kinect experiments*/
#include <armadillo>
#include <chrono>
#include <dirent.h>
#include <map>
#include <set>
#include "MocapStream.h"
#include "CalibrationTools.h"
#include "arma_xn_tools.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"

#ifndef AUTOCAL_CORRELATOR
#define AUTOCAL_CORRELATOR

namespace autocal {

	class Correlator
	{
	public:
		Correlator(){};
		~Correlator(){};
		
	private:
		//CONFIG
		int number_of_samples = 10;

		float difference_threshold = 0.1;

		float elimination_score_threshold = 0.001;


		//STATE

		//States for matchStreams
		std::map<std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID>, 
				 	std::pair<std::vector<utility::math::matrix::Transform3D>, std::vector<utility::math::matrix::Transform3D>>>
				 	 recordedStates;
		//Stores scores for the matchings
		std::map<std::pair<MocapStream::RigidBodyID, MocapStream::RigidBodyID>, float> scores;
		//Stores the matches which have been deduced incorrect
		std::set<std::pair<MocapStream::RigidBodyID, 
						   MocapStream::RigidBodyID>> eliminatedHypotheses;
		
		std::set<std::pair<MocapStream::RigidBodyID, 
							   MocapStream::RigidBodyID>> computableStreams;
		
	public:

		void addData(MocapStream::RigidBodyID id1, utility::math::matrix::Transform3D T1, MocapStream::RigidBodyID id2, utility::math::matrix::Transform3D T2);

		void eliminateAndNormalise(std::map<MocapStream::RigidBodyID,float> totalScores);

		bool sufficientData();

		void compute();

		std::vector<std::pair<int,int>> getBestCorrelations();

		float likelihood(float error){
			return std::exp(-error * error);
		}

	};


}
#endif