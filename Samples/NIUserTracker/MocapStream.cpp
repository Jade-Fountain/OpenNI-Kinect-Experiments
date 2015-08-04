/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "MocapStream.h"

namespace autocal {
	MocapStream::Frame MocapStream::createFrame(arma::mat m){
		std::cout << m << std::endl;
		return Frame();
	}

	MocapStream::Frame MocapStream::getFrame(std::chrono::system_clock::time_point start_time){
		return Frame();
	}

	
	bool MocapStream::loadMocapData(std::string folder_path, std::chrono::system_clock::time_point start_time, std::chrono::system_clock::time_point end_time){
		std::cout << "Loading data ..." << std::endl;

		long long int min = getCount(start_time);
		long long int max = getCount(end_time);

		bool success = true;

		DIR* dir = opendir(folder_path.c_str());
		std::cout << "File opened ..." << std::endl;
		
		dirent * file;
		while ((file = readdir(dir)) != NULL){
			
			std::string filename = file->d_name;
			std::cout << "Filename = " << filename << std::endl;
			long long int timestamp;
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

}