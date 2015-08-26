/*
author Jake Fountain
This code is part of mocap-kinect experiments.
The sensor plant is responsible for fusing multiple measurements*/
#include <armadillo>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <XnPropNames.h>
#include <map>

#ifndef ARMA_XN_TOOLS
#define ARMA_XN_TOOLS

inline arma::mat33 getArma(const XnMatrix3X3& m){
	return arma::mat33({
		 m.elements[0], m.elements[3], m.elements[6],
 		 m.elements[0+1], m.elements[3+1], m.elements[6+1],
 		 m.elements[0+2], m.elements[3+2], m.elements[6+2],
	});
}

inline arma::vec3 getArma(const XnPoint3D& v){
	return {v.X,v.Y,v.Z};
}

template<typename A, typename B>
inline std::pair<B,A> flip_pair(const std::pair<A,B> &p)
{
    return std::pair<B,A>(p.second, p.first);
}

template<typename A, typename B>
inline std::multimap<B,A> flip_map(const std::map<A,B> &src)
{
    std::multimap<B,A> dst;
    std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()), 
                   flip_pair<A,B>);
    return dst;
}

#endif