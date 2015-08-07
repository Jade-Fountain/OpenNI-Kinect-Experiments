/*
author Jake Fountain
This code is part of mocap-kinect experiments*/
#include <armadillo>
#include <chrono>
#include <dirent.h>
#include <map>
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"

#ifndef AUTOCAL_MOCAP_STREAM
#define AUTOCAL_MOCAP_STREAM

namespace autocal {
	
	typedef long long int TimeStamp;

	class MocapStream {
	public:
		typedef unsigned int RigidBodyID;

		//TODO generalise to other sensors, not just rigid bodies
		struct RigidBody {
			arma::vec3 position;
			arma::mat33 rotation;

			utility::math::matrix::Transform3D getTransform(){
				auto T = utility::math::matrix::Transform3D(utility::math::matrix::Rotation3D(rotation));
				T.translation() = position;
				return T;
			}
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
		int size() {return stream.size();}

		bool loadMocapData(std::string folder_path, const TimeStamp& start_time, const std::chrono::system_clock::time_point& end_time);

		bool setRigidBodyInFrame(const std::chrono::system_clock::time_point& frame_time, const unsigned int& id, const arma::vec3& position, const arma::mat33& rotation);
		
		bool setRigidBodyInFrame(const TimeStamp& frame_time, const unsigned int& id, const arma::vec3& position, const arma::mat33& rotation);

		Frame getFrame(const std::chrono::system_clock::time_point& start_time);

		std::map<MocapStream::RigidBodyID, utility::math::matrix::Transform3D> getInvariates(TimeStamp now);

	};

}
#endif