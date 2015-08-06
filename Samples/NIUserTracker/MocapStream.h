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
		typedef unsigned int RigidBodyID;
		typedef long long int TimeStamp;

		struct RigidBody{
			arma::vec3 position;
			arma::mat33 rotation;
		};

		struct Frame {
			std::map<RigidBodyID, RigidBody> rigidBodies;
		};

	private:
		std::map<TimeStamp, Frame> stream;

		TimeStamp getTimeStamp(const std::chrono::system_clock::time_point& t){
			return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
		}

		Frame createFrame(arma::mat m);

	public:
		bool loadMocapData(std::string folder_path, const std::chrono::system_clock::time_point& start_time, const std::chrono::system_clock::time_point& end_time);

		bool setRigidBodyInFrame(const std::chrono::system_clock::time_point&& frame_time, const unsigned int& id, const arma::vec3& position, const arma::mat33& rotation);

		Frame getFrame(const std::chrono::system_clock::time_point& start_time);
	};

}
#endif