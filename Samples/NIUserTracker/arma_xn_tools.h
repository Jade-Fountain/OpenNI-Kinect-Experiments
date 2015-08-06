/*
author Jake Fountain
This code is part of mocap-kinect experiments.
The sensor plant is responsible for fusing multiple measurements*/
#include <armadillo>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>

#ifndef ARMA_XN_TOOLS
#define ARMA_XN_TOOLS

static inline arma::mat33 getArma(const XnMatrix3X3& m){
	return arma::mat33({
		 m.elements[0], m.elements[3], m.elements[6],
 		 m.elements[0+1], m.elements[3+1], m.elements[6+1],
 		 m.elements[0+2], m.elements[3+2], m.elements[6+2],
	}).t();
}

static inline arma::vec3 getArma(const XnPoint3D& v){
	return {v.X,v.Y,v.Z};
}

#endif