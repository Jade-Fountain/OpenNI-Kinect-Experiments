/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "MocapStream.h"

namespace autocal {

	using utility::math::matrix::Transform3D;

	MocapStream::Frame MocapStream::createFrame(arma::mat m){
		Frame f;
		for(int n = 0; n < m.n_cols; n++){
			arma::vec data = m.col(n);
			
			RigidBody r;
			
			r.position = data.rows(1,3);
			
			int start = 4;
			for(int i = 0; i < 3; i++){
				r.rotation.row(i) = data.rows(start + 3 * i, start + 3 * i + 2).t();
			}

			// std::cout << "data: " <<  data << std::endl;
			// std::cout << "id:" << int(data[0]) << std::endl;
			// std::cout << "position:" << r.position << std::endl;
			// std::cout << "rotation:" << r.rotation << std::endl;
			
			f.rigidBodies[int(data[0])] = r;
		}
		return f;
	}

	MocapStream::Frame MocapStream::getFrame(const std::chrono::system_clock::time_point& t){
		//Get last frame at current time point
		//TODO: optimise?
		return stream.lower_bound(getTimeStamp(t))->second;
	}

	
	bool MocapStream::loadMocapData(std::string folder_path, const TimeStamp& start_time, const std::chrono::system_clock::time_point& end_time){
		std::cout << "Loading data ..." << std::endl;

		TimeStamp min = start_time;
		TimeStamp max = getTimeStamp(end_time);

		bool success = true;

		DIR* dir = opendir(folder_path.c_str());
		std::cout << "File opened ..." << std::endl;
		dirent * file;
		while ((file = readdir(dir)) != NULL){
			
			std::string filename = file->d_name;
			std::cout << "Filename = " << filename << std::endl;
			TimeStamp timestamp;
			try{
				timestamp = std::stoll(filename);
			} catch(std::invalid_argument) {
				std::cout << "Skipping file " << filename << std::endl;
				continue;
			}
			
			if(timestamp < max && timestamp > min){
				arma::mat frame;

				std::stringstream path;
				path << folder_path << "/" << filename;
				success = success && frame.load(path.str(), arma::arma_binary);

				if(success){
					stream[timestamp] = createFrame(frame);
				} else {
					break;
				}
			}

		}
	   	(void)closedir(dir);
		std::cout << "Loading finished " << (success ? "successfully" : "UNSUCCESSFULLY") << std::endl;
		return success;
	}

	bool MocapStream::setRigidBodyInFrame(const std::chrono::system_clock::time_point& frame_time, const unsigned int& id, const arma::vec3& position, const arma::mat33& rotation){
		TimeStamp t = getTimeStamp(frame_time);
		if(stream.count(t) == 0){
			stream[t] = Frame();
		}
		stream[t].rigidBodies[id] = RigidBody({position,rotation});
	}

	bool MocapStream::setRigidBodyInFrame(const TimeStamp& frame_time, const unsigned int& id, const arma::vec3& position, const arma::mat33& rotation){
		TimeStamp t = frame_time;
		if(stream.count(t) == 0){
			stream[t] = Frame();
		}
		stream[t].rigidBodies[id] = RigidBody({position,rotation});
	}

	std::map<MocapStream::RigidBodyID, Transform3D> MocapStream::getInvariates(TimeStamp now){
		std::map<MocapStream::RigidBodyID, Transform3D> invariates;

		if(stream.size() != 0){
			Frame firstFrame = stream.upper_bound(0)->second;
			Frame latestFrame = stream.lower_bound(now)->second;
			for (auto& rb : firstFrame.rigidBodies){
				auto rbID = rb.first;
				auto initialTransform = rb.second.getTransform();
				std::cout << "initialTransform" << initialTransform << std::endl;
				if(latestFrame.rigidBodies.count(rbID)!=0){
					auto latestTransform = latestFrame.rigidBodies[rbID].getTransform();
					std::cout << "latestTransform" << latestTransform << std::endl;

					//TODO generalise to other sensors and invariates
					invariates[rbID] = latestTransform.i() * initialTransform;
					std::cout << "invariates[rbID]" << invariates[rbID] << std::endl;
				}
			}
		}

		return invariates;
	}














}