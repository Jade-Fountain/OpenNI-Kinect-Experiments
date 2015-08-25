/*****************************************************************************
*                                                                            *
*  OpenNI 1.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <math.h>
#include <iostream>
#include <fstream>
#include <armadillo>
#include "MocapStream.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "SensorPlant.h"
#include "arma_xn_tools.h"

#include "SceneDrawer.h"

#ifndef USE_GLES
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif
#else
	#include "opengles.h"
#endif

extern xn::UserGenerator g_UserGenerator;
extern xn::DepthGenerator g_DepthGenerator;

extern XnBool g_bDrawBackground;
extern XnBool g_bDrawPixels;
extern XnBool g_bDrawSkeleton;
extern XnBool g_bPrintID;
extern XnBool g_bPrintState;

extern XnBool g_bPrintFrameID;
extern XnBool g_bMarkJoints;

extern autocal::SensorPlant sensorPlant;
extern autocal::TimeStamp kinectFileStartTime;
extern bool streamsStarted;

using utility::math::matrix::Transform3D;
using utility::math::matrix::Rotation3D;


#include <map>
std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;
void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability& /*capability*/, XnUserID id, XnCalibrationStatus calibrationError, void* /*pCookie*/)
{
	m_Errors[id].first = calibrationError;
}
void XN_CALLBACK_TYPE MyPoseInProgress(xn::PoseDetectionCapability& /*capability*/, const XnChar* /*strPose*/, XnUserID id, XnPoseDetectionStatus poseError, void* /*pCookie*/)
{
	m_Errors[id].second = poseError;
}

unsigned int getClosestPowerOfTwo(unsigned int n)
{
	unsigned int m = 2;
	while(m < n) m<<=1;

	return m;
}
GLuint initTexture(void** buf, int& width, int& height)
{
	GLuint texID = 0;
	glGenTextures(1,&texID);

	width = getClosestPowerOfTwo(width);
	height = getClosestPowerOfTwo(height); 
	*buf = new unsigned char[width*height*4];
	glBindTexture(GL_TEXTURE_2D,texID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	return texID;
}

GLfloat texcoords[8];
void DrawRectangle(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	GLfloat verts[8] = {	topLeftX, topLeftY,
		topLeftX, bottomRightY,
		bottomRightX, bottomRightY,
		bottomRightX, topLeftY
	};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	//TODO: Maybe glFinish needed here instead - if there's some bad graphics crap
	glFlush();
}
void DrawTexture(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 0, texcoords);

	DrawRectangle(topLeftX, topLeftY, bottomRightX, bottomRightY);

	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

XnFloat Colors[][3] =
{
	{0,1,1},
	{0,0,1},
	{0,1,0},
	{1,1,0},
	{1,0,0},
	{1,.5,0},
	{.5,1,0},
	{0,.5,1},
	{.5,0,1},
	{1,1,.5},
	{1,1,1}
};
XnUInt32 nColors = 10;
#ifndef USE_GLES
void glPrintString(void *font, char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{
		glutBitmapCharacter(font,*str++);
	}
}
#endif

void drawLine(float x1, float y1, float x2, float y2){
#ifndef USE_GLES
	glVertex3i(x1, y1, 0);
	glVertex3i(x2, y2, 0);
#else
	GLfloat verts[4] = {x1, y1, x2, y2};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_LINES, 0, 2);
	glFlush();
#endif
}

bool DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return true;
	}

	if (!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint1) ||
		!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint2))
	{
		return false;
	}

	XnSkeletonJointPosition joint1, joint2;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return true;
	}

	XnPoint3D pt[2];
	pt[0] = joint1.position;
	pt[1] = joint2.position;

	g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);

	drawLine(pt[0].X, pt[0].Y, pt[1].X, pt[1].Y);

	return true;
}

static const float DEG2RAD = 3.14159/180;
 
void drawCircle(float x, float y, float radius)
{
   glBegin(GL_TRIANGLE_FAN);
 
   for (int i=0; i < 360; i++)
   {
      float degInRad = i*DEG2RAD;
      glVertex2f(x + cos(degInRad)*radius, y + sin(degInRad)*radius);
   }
 
   glEnd();
}

void logFile(std::string str){
	std::fstream file("data.txt", std::fstream::app);
}

void DrawJoint(XnUserID player, XnSkeletonJoint eJoint, autocal::TimeStamp timeSinceStart)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	if (!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint))
	{
		return;
	}

	XnSkeletonJointTransformation joint;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(player, eJoint, joint);

	if (joint.position.fConfidence < 0.5 && joint.orientation.fConfidence < 0.5)
	{
		return;
	}

	XnPoint3D pt = joint.position.position;
	XnMatrix3X3 orientation = joint.orientation.orientation;

	XnPoint3D or_pt_x = {100 * orientation.elements[0] + pt.X, 100 * orientation.elements[3] + pt.Y, 100 * orientation.elements[6] + pt.Z} ;
	XnPoint3D or_pt_y = {100 * orientation.elements[0+1] + pt.X, 100 * orientation.elements[3+1] + pt.Y, 100 * orientation.elements[6+1] + pt.Z} ;
	XnPoint3D or_pt_z = {100 * orientation.elements[0+2] + pt.X, 100 * orientation.elements[3+2] + pt.Y, 100 * orientation.elements[6+2] + pt.Z} ;
	
	g_DepthGenerator.ConvertRealWorldToProjective(1, &or_pt_x, &or_pt_x);
	g_DepthGenerator.ConvertRealWorldToProjective(1, &or_pt_y, &or_pt_y);
	g_DepthGenerator.ConvertRealWorldToProjective(1, &or_pt_z, &or_pt_z);
	g_DepthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);

	// drawCircle(pt.X, pt.Y, 10);
	drawCircle(or_pt_x.X, or_pt_x.Y, 3);
	drawCircle(or_pt_y.X, or_pt_y.Y, 1);
	// drawCircle(or_pt_z.X, or_pt_z.Y, 2);

#ifndef USE_GLES
	glBegin(GL_LINES);
#endif
	drawLine(or_pt_x.X, or_pt_x.Y, pt.X, pt.Y);
	drawLine(or_pt_y.X, or_pt_y.Y, pt.X, pt.Y);
	drawLine(or_pt_z.X, or_pt_z.Y, pt.X, pt.Y);
#ifndef USE_GLES
	glEnd();
#endif

	autocal::TimeStamp timestamp = kinectFileStartTime + timeSinceStart;
	arma::vec3 pt_arma = 0.001 * getArma(pt); //Convert to m
	Rotation3D orientation_arma = getArma(orientation);
	Transform3D pose(orientation_arma);
	pose.translation() = pt_arma;

	bool rigidBodyNotEmpty = arma::all(arma::all(orientation_arma));
	if(rigidBodyNotEmpty){ //Throw out end points of skeleton
		std::stringstream name;
		name << "Skeleton " << int(player);
		sensorPlant.mocapRecording.addMeasurement(name.str(), timestamp, int(eJoint), pose);
	}
}
void DrawSensorMatch(XnUserID player, int sensorID, XnSkeletonJoint eJoint)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	if (!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint))
	{
		return;
	}

	XnSkeletonJointTransformation joint;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(player, eJoint, joint);

	if (joint.position.fConfidence < 0.5 && joint.orientation.fConfidence < 0.5)
	{
		return;
	}

	XnPoint3D pt = joint.position.position;
	XnMatrix3X3 orientation = joint.orientation.orientation;
	
	g_DepthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);

	int markerSize = 10;
	int crossSize = markerSize/2 + markerSize;
	drawCircle(pt.X, pt.Y, markerSize);

#ifndef USE_GLES
	glBegin(GL_LINES);
#endif
	drawLine(pt.X+crossSize, pt.Y+crossSize, pt.X-crossSize, pt.Y-crossSize);
	drawLine(pt.X+crossSize, pt.Y-crossSize, pt.X-crossSize, pt.Y+crossSize);
#ifndef USE_GLES
	glEnd();
#endif
}

void DrawTransform3D(Transform3D pose){

	arma::vec3 p = 1000 * pose.translation();
	// std::cout << "p = " << p.t() << std::endl;
	arma::vec3 x = 1000 * pose.x();
	// std::cout << "x = " << x.t() << std::endl;
	arma::vec3 y = 1000 * pose.y();
	// std::cout << "y = " << y.t() << std::endl;
	arma::vec3 z = 1000 * pose.z();
	// std::cout << "z = " << z.t() << std::endl;
	
	float sphereRadius = 50; //mm

	XnPoint3D position = {float(p[0]),float(p[1]),float(p[2])};
	XnPoint3D sphereEdge = {float(p[0] + sphereRadius),float(p[1]),float(p[2])};
	XnPoint3D x_end_point = {float(0.1 * x[0]+p[0]), float(0.1 * x[1]+p[1]), float(0.1 * x[2]+p[2])};
	XnPoint3D y_end_point = {float(0.1 * y[0]+p[0]), float(0.1 * y[1]+p[1]), float(0.1 * y[2]+p[2])};
	XnPoint3D z_end_point = {float(0.1 * z[0]+p[0]), float(0.1 * z[1]+p[1]), float(0.1 * z[2]+p[2])};

	g_DepthGenerator.ConvertRealWorldToProjective(1, &position, &position);
	g_DepthGenerator.ConvertRealWorldToProjective(1, &sphereEdge, &sphereEdge);
	g_DepthGenerator.ConvertRealWorldToProjective(1, &x_end_point, &x_end_point);
	g_DepthGenerator.ConvertRealWorldToProjective(1, &y_end_point, &y_end_point);
	g_DepthGenerator.ConvertRealWorldToProjective(1, &z_end_point, &z_end_point);

	int sphereScreenRadius = std::round(std::sqrt( (position.X-sphereEdge.X) * (position.X-sphereEdge.X) + (position.Y-sphereEdge.Y) * (position.Y-sphereEdge.Y))); 

	std::map<float, std::pair<XnPoint3D,arma::vec4>> sortedLines;
	sortedLines[-x_end_point.Z] = std::make_pair(x_end_point,arma::vec4({1,0,0,1}));

	sortedLines[-y_end_point.Z] = std::make_pair(y_end_point,arma::vec4({0,1,0,1}));

	sortedLines[-z_end_point.Z] = std::make_pair(z_end_point,arma::vec4({0,0,1,1}));


#ifndef USE_GLES
	glBegin(GL_LINES);
#endif
	for(auto& elem : sortedLines){
		glColor4f(elem.second.second[0],elem.second.second[1],elem.second.second[2],elem.second.second[3]);
		drawLine(elem.second.first.X, elem.second.first.Y, position.X, position.Y);
	}
#ifndef USE_GLES
	glEnd();
#endif
}

const XnChar* GetCalibrationErrorString(XnCalibrationStatus error)
{
	switch (error)
	{
	case XN_CALIBRATION_STATUS_OK:
		return "OK";
	case XN_CALIBRATION_STATUS_NO_USER:
		return "NoUser";
	case XN_CALIBRATION_STATUS_ARM:
		return "Arm";
	case XN_CALIBRATION_STATUS_LEG:
		return "Leg";
	case XN_CALIBRATION_STATUS_HEAD:
		return "Head";
	case XN_CALIBRATION_STATUS_TORSO:
		return "Torso";
	case XN_CALIBRATION_STATUS_TOP_FOV:
		return "Top FOV";
	case XN_CALIBRATION_STATUS_SIDE_FOV:
		return "Side FOV";
	case XN_CALIBRATION_STATUS_POSE:
		return "Pose";
	default:
		return "Unknown";
	}
}
const XnChar* GetPoseErrorString(XnPoseDetectionStatus error)
{
	switch (error)
	{
	case XN_POSE_DETECTION_STATUS_OK:
		return "OK";
	case XN_POSE_DETECTION_STATUS_NO_USER:
		return "NoUser";
	case XN_POSE_DETECTION_STATUS_TOP_FOV:
		return "Top FOV";
	case XN_POSE_DETECTION_STATUS_SIDE_FOV:
		return "Side FOV";
	case XN_POSE_DETECTION_STATUS_ERROR:
		return "General error";
	default:
		return "Unknown";
	}
}


void DrawDepthMap(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd)
{
	static bool bInitialized = false;	
	static GLuint depthTexID;
	static unsigned char* pDepthTexBuf;
	static int texWidth, texHeight;

	float topLeftX;
	float topLeftY;
	float bottomRightY;
	float bottomRightX;
	float texXpos;
	float texYpos;

	if(!bInitialized)
	{
		texWidth =  getClosestPowerOfTwo(dmd.XRes());
		texHeight = getClosestPowerOfTwo(dmd.YRes());

//		printf("Initializing depth texture: width = %d, height = %d\n", texWidth, texHeight);
		depthTexID = initTexture((void**)&pDepthTexBuf,texWidth, texHeight) ;

//		printf("Initialized depth texture: width = %d, height = %d\n", texWidth, texHeight);
		bInitialized = true;

		topLeftX = dmd.XRes();
		topLeftY = 0;
		bottomRightY = dmd.YRes();
		bottomRightX = 0;
		texXpos =(float)dmd.XRes()/texWidth;
		texYpos  =(float)dmd.YRes()/texHeight;

		memset(texcoords, 0, 8*sizeof(float));
		texcoords[0] = texXpos, texcoords[1] = texYpos, texcoords[2] = texXpos, texcoords[7] = texYpos;
	}

	unsigned int nValue = 0;
	unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
	XnUInt16 g_nXRes = dmd.XRes();
	XnUInt16 g_nYRes = dmd.YRes();

	unsigned char* pDestImage = pDepthTexBuf;

	const XnDepthPixel* pDepth = dmd.Data();
	const XnLabel* pLabels = smd.Data();

	static unsigned int nZRes = dmd.ZRes();
	static float* pDepthHist = (float*)malloc(nZRes* sizeof(float));

	// Calculate the accumulative histogram
	memset(pDepthHist, 0, nZRes*sizeof(float));
	for (nY=0; nY<g_nYRes; nY++)
	{
		for (nX=0; nX<g_nXRes; nX++)
		{
			nValue = *pDepth;

			if (nValue != 0)
			{
				pDepthHist[nValue]++;
				nNumberOfPoints++;
			}

			pDepth++;
		}
	}

	for (nIndex=1; nIndex<nZRes; nIndex++)
	{
		pDepthHist[nIndex] += pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex<nZRes; nIndex++)
		{
			pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}

	pDepth = dmd.Data();
	if (g_bDrawPixels)
	{
		XnUInt32 nIndex = 0;
		// Prepare the texture map
		for (nY=0; nY<g_nYRes; nY++)
		{
			for (nX=0; nX < g_nXRes; nX++, nIndex++)
			{

				pDestImage[0] = 0;
				pDestImage[1] = 0;
				pDestImage[2] = 0;
				if (g_bDrawBackground || *pLabels != 0)
				{
					nValue = *pDepth;
					XnLabel label = *pLabels;
					XnUInt32 nColorID = label % nColors;
					if (label == 0)
					{
						nColorID = nColors;
					}

					if (nValue != 0)
					{
						nHistValue = pDepthHist[nValue];

						pDestImage[0] = nHistValue * Colors[nColorID][0]; 
						pDestImage[1] = nHistValue * Colors[nColorID][1];
						pDestImage[2] = nHistValue * Colors[nColorID][2];
					}
				}

				pDepth++;
				pLabels++;
				pDestImage+=3;
			}

			pDestImage += (texWidth - g_nXRes) *3;
		}
	}
	else
	{
		xnOSMemSet(pDepthTexBuf, 0, 3*2*g_nXRes*g_nYRes);
	}

	glBindTexture(GL_TEXTURE_2D, depthTexID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

	// Display the OpenGL texture map
	glColor4f(0.75,0.75,0.75,1);

	glEnable(GL_TEXTURE_2D);
	DrawTexture(dmd.XRes(),dmd.YRes(),0,0);	
	glDisable(GL_TEXTURE_2D);

	char strLabel[50] = "";
	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	g_UserGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
#ifndef USE_GLES
		if (g_bPrintID)
		{
			XnPoint3D com;
			g_UserGenerator.GetCoM(aUsers[i], com);
			g_DepthGenerator.ConvertRealWorldToProjective(1, &com, &com);

			XnUInt32 nDummy = 0;

			xnOSMemSet(strLabel, 0, sizeof(strLabel));
			if (!g_bPrintState)
			{
				// Tracking
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d", aUsers[i]);
			}
			else if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
			{
				// Tracking
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Tracking", aUsers[i]);
			}
			else if (g_UserGenerator.GetSkeletonCap().IsCalibrating(aUsers[i]))
			{
				// Calibrating
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Calibrating [%s]", aUsers[i], GetCalibrationErrorString(m_Errors[aUsers[i]].first));
			}
			else
			{
				// Nothing
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Looking for pose [%s]", aUsers[i], GetPoseErrorString(m_Errors[aUsers[i]].second));
			}


			glColor4f(1-Colors[i%nColors][0], 1-Colors[i%nColors][1], 1-Colors[i%nColors][2], 1);

			glRasterPos2i(com.X, com.Y);
			glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
		}
#endif
		if (g_bDrawSkeleton && g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
			glColor4f(1-Colors[aUsers[i]%nColors][0], 1-Colors[aUsers[i]%nColors][1], 1-Colors[aUsers[i]%nColors][2], 1);

			// Draw Joints
			if (g_bMarkJoints)
			{
				// sleep(1);
				autocal::TimeStamp timestamp = smd.Timestamp();
				// std::cout << "timestamp = " << timestamp << std::endl;
				// Try to draw all joints
				DrawJoint(aUsers[i], XN_SKEL_HEAD, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_NECK, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_TORSO, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_WAIST, timestamp);

				DrawJoint(aUsers[i], XN_SKEL_LEFT_COLLAR, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_SHOULDER, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_ELBOW, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_WRIST, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_HAND, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_FINGERTIP, timestamp);

				DrawJoint(aUsers[i], XN_SKEL_RIGHT_COLLAR, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_SHOULDER, timestamp);
				// std::cout << "XN_SKEL_RIGHT_SHOULDER " << XN_SKEL_RIGHT_SHOULDER << std::endl;
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_ELBOW, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_WRIST, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_HAND, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_FINGERTIP, timestamp);

				DrawJoint(aUsers[i], XN_SKEL_LEFT_HIP, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_KNEE, timestamp);
				// std::cout << "XN_SKEL_LEFT_KNEE " << XN_SKEL_LEFT_KNEE << std::endl;
				DrawJoint(aUsers[i], XN_SKEL_LEFT_ANKLE, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_FOOT, timestamp);

				DrawJoint(aUsers[i], XN_SKEL_RIGHT_HIP, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_KNEE, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_ANKLE, timestamp);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_FOOT, timestamp);

				std::map<int, Transform3D> groundTruth = sensorPlant.getGroundTruth("mocap","Skeleton 1",timestamp + kinectFileStartTime);
				
				for(auto& rb : groundTruth){
					Transform3D T = rb.second;
					//TODO: Figure out why the coordinates dont line up properly
					// T.translation()[0] += -0.38;
					DrawTransform3D(T);

					// //DEBUG
					// if(rb.first == 1) {

					// 	XnSkeletonJointTransformation joint;
					// 	g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(aUsers[i], XN_SKEL_LEFT_KNEE, joint);
					// 	XnPoint3D pt = joint.position.position;
					// 	arma::vec3 true_pos = arma::vec3({pt.X,pt.Y,pt.Z}) * 1e-3;

					// 	std::cout << true_pos - T.translation() << std::endl;
					// } else if(rb.first == 2) {
					// 	std::cout << "RB2 - XN_SKEL_RIGHT_SHOULDER error" << std::endl;

					// 	XnSkeletonJointTransformation joint;
					// 	g_UserGenerator.GetSkeletonCap().GetSkeletonJoint(aUsers[i], XN_SKEL_RIGHT_SHOULDER, joint);
					// 	XnPoint3D pt = joint.position.position;
					// 	arma::vec3 true_pos = arma::vec3({pt.X,pt.Y,pt.Z}) * 1e-3;

					// 	std::cout << true_pos - T.translation() << std::endl;
					// }
				}

				
				if(!streamsStarted){
					sensorPlant.mocapRecording.markStartOfStreams(timestamp + kinectFileStartTime);
					streamsStarted = true;
				}

				//TODO: generalise to multiple skeletons
				auto startCalc = std::chrono::high_resolution_clock::now();
				std::vector<std::pair<int,int>> correlations = sensorPlant.matchStreams("mocap","Skeleton 1",timestamp + kinectFileStartTime);
				auto finishCalc = std::chrono::high_resolution_clock::now();
				double millisecondsDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(finishCalc-startCalc).count() * 1e-6;
				// std::cout << "time = " << millisecondsDuration << std::endl;
				
				// std::vector<std::pair<int,int>> correlations = sensorPlant.getCorrelations("mocap","Skeleton 1",timestamp + kinectFileStartTime);
				// std::vector<std::pair<int,int>> correlations = sensorPlant.getCorrelationsOfInvariants("mocap","Skeleton 1",timestamp + kinectFileStartTime);

				for(auto match : correlations){
					int sensorID = match.first;
					int skeletonID = match.second;
					glColor4f(sensorID%2, (sensorID+1)%2, 0, 1);
					// glColor4f(1-Colors[(sensorID+1+aUsers[i])%nColors][0], 1-Colors[(sensorID+2+aUsers[i])%nColors][1], 1-Colors[(sensorID+1+aUsers[i])%nColors][2], 1);
					DrawSensorMatch(aUsers[i], sensorID, XnSkeletonJoint(skeletonID));
				}				
				glColor4f(1-Colors[aUsers[i]%nColors][0], 1-Colors[aUsers[i]%nColors][1], 1-Colors[aUsers[i]%nColors][2], 1);

			}

#ifndef USE_GLES
			glBegin(GL_LINES);
#endif

			// Draw Limbs
			DrawLimb(aUsers[i], XN_SKEL_HEAD, XN_SKEL_NECK);

			DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
			if (!DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_WRIST))
			{
				DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
			}
			else
			{
				DrawLimb(aUsers[i], XN_SKEL_LEFT_WRIST, XN_SKEL_LEFT_HAND);
				DrawLimb(aUsers[i], XN_SKEL_LEFT_HAND, XN_SKEL_LEFT_FINGERTIP);
			}


			DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
			if (!DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_WRIST))
			{
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
			}
			else
			{
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_WRIST, XN_SKEL_RIGHT_HAND);
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_HAND, XN_SKEL_RIGHT_FINGERTIP);
			}

			DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);

			DrawLimb(aUsers[i], XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);

			DrawLimb(aUsers[i], XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);

			DrawLimb(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP);
#ifndef USE_GLES
			glEnd();
#endif
		}
	}

	if (g_bPrintFrameID)
	{
		static XnChar strFrameID[80];
		xnOSMemSet(strFrameID, 0, 80);
		XnUInt32 nDummy = 0;
		xnOSStrFormat(strFrameID, sizeof(strFrameID), &nDummy, "%d", dmd.FrameID());

		glColor4f(1, 0, 0, 1);

		glRasterPos2i(10, 10);

		glPrintString(GLUT_BITMAP_HELVETICA_18, strFrameID);
	}
}
