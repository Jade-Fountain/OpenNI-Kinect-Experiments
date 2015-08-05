/*
author Jake Fountain
This code is part of mocap-kinect experiments*/
#include <armadillo>
#include <chrono>
#include <dirent.h>
#include <map>

#ifndef AUTOCAL_MOCAP_STREAM
#define AUTOCAL_MOCAP_STREAM

namespace autocal {
	
	class MocapStream {
	public:
		struct RigidBody{
			unsigned int id;
			arma::vec3 position;
			arma::mat33 rotation;
		};

		struct Frame {
			std::vector<RigidBody> rigidBodies;
		};

	private:
		std::map<long long int, Frame> stream;

		long long int getCount(std::chrono::system_clock::time_point t){
			return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
		}

		Frame createFrame(arma::mat m);

	public:
		bool loadMocapData(std::string folder_path, std::chrono::system_clock::time_point start_time, std::chrono::system_clock::time_point end_time);

		Frame getFrame(std::chrono::system_clock::time_point start_time);
	};

}
#endif