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

			utility::math::matrix::Transform3D getTransform() const{
				auto T = utility::math::matrix::Transform3D(utility::math::matrix::Rotation3D(rotation));
				T.translation() = position;
				return T;
			}
		};

		struct Frame {
			std::map<RigidBodyID, RigidBody> rigidBodies;
			std::string toString(){
				std::stringstream os;
				for(auto& x : rigidBodies){
					os << "rigidBodies[" << int(x.first) << "] = \n" << x.second.getTransform() << std::endl;
				}
				os << "=======================";
				return os.str();
			}
		};

		std::map<TimeStamp, Frame> stream;
	private:

		std::string stream_name;

		TimeStamp getTimeStamp(const std::chrono::system_clock::time_point& t){
			return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
		}

		Frame createFrame(arma::mat m);

		TimeStamp streamStart;

	public:
		MocapStream() : stream_name(""){}
		MocapStream(std::string name) : stream_name(name){}

		int size() const {return stream.size();}
		
		std::string name() const {return stream_name;}

		std::string toString(){
			std::stringstream os;
			os << "Mocap Stream: " << name()  << " || Size: " << size() << std::endl;
			for(auto& x : stream){
				os << "stream[" << int(x.first) << "] = \n" << x.second.toString() << std::endl;
			}
			return os.str();
		}

		void markStart(TimeStamp t){
			streamStart = t;
		}

		bool loadMocapData(std::string folder_path, const TimeStamp& start_time, const std::chrono::system_clock::time_point& end_time);

		bool setRigidBodyInFrame(const std::chrono::system_clock::time_point& frame_time, const unsigned int& id, const arma::vec3& position, const arma::mat33& rotation);
		
		bool setRigidBodyInFrame(const TimeStamp& frame_time, const unsigned int& id, const arma::vec3& position, const arma::mat33& rotation);

		Frame getFrame(const std::chrono::system_clock::time_point& start_time);
		Frame getFrame(const TimeStamp& start_time);

		std::map<MocapStream::RigidBodyID, utility::math::matrix::Transform3D> getInvariates(TimeStamp now);

	};


}
#endif