/*
author Jake Fountain
This code is part of mocap-kinect experiments*/
#include <armadillo>
#include <chrono>
#include <dirent.h>
#include <map>
#include "arma_xn_tools.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"

#ifndef AUTOCAL_MOCAP_STREAM
#define AUTOCAL_MOCAP_STREAM

namespace autocal {
	
	typedef long long int TimeStamp;

	class MocapStream {
	public:
		typedef unsigned int RigidBodyID;

		//TODO generalise to other sensors, not just rigid bodies
		struct RigidBody {
			utility::math::matrix::Transform3D pose;
		};

		struct Frame {
			std::map<RigidBodyID, RigidBody> rigidBodies;

			std::string toString();
		};

	private:
		std::map<TimeStamp, Frame> stream;

		std::string stream_name;

		TimeStamp getTimeStamp(const std::chrono::system_clock::time_point& t){
			return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
		}

		Frame createFrame(arma::mat m);

		TimeStamp streamStart;

	public:
		//Constructors
		MocapStream() : stream_name(""){}

		MocapStream(std::string name) : stream_name(name){}

		//Accessors and small utilities
		void markStart(TimeStamp t){
			streamStart = t;
		}

		int size() const {return stream.size();}
		
		std::string name() const {return stream_name;}

		std::string toString();

		std::map<TimeStamp, Frame>& frameList(){return stream;}
		
		//Frame retrieval
		Frame getFrame(const std::chrono::system_clock::time_point& start_time);
		Frame getFrame(const TimeStamp& start_time);

		//Heavy functions
		bool loadMocapData(std::string folder_path, const TimeStamp& start_time, const std::chrono::system_clock::time_point& end_time);

		bool setRigidBodyInFrame(const std::chrono::system_clock::time_point& frame_time, const unsigned int& id, const utility::math::matrix::Transform3D& pose);
		
		bool setRigidBodyInFrame(const TimeStamp& frame_time, const unsigned int& id, const utility::math::matrix::Transform3D& pose);

		std::map<MocapStream::RigidBodyID, arma::vec> getInvariates(TimeStamp now);

	};

}
#endif