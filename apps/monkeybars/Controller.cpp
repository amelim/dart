/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.h"

#include "dart/gui/GLFuncs.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Shape.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"
#include "dart/constraint/WeldJointConstraint.h"
#include <iostream>
#include <stdio.h>

using namespace std;


// ================================================================================================
Controller::Controller(dart::dynamics::Skeleton* _skel, dart::constraint::ConstraintSolver* _constrSolver, double _t) {
  mSkel = _skel;
  mConstraintSolver = _constrSolver;
  mTimestep = _t;
	mJump = false;
barName = "bar3";

  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);
  mTorques.resize(nDof);
  mDefaultPose.resize(nDof);
  mDesiredDofs.resize(nDof);
  
  // Set default pose as the initial pose when the controller is instantiated
  mDefaultPose = mSkel->getPositions();
  mDesiredDofs = mDefaultPose;
  
  mTorques.setZero();

  // Using SPD results in simple spring coefficients
  for (int i = 0; i < 6; i++) {
    mKp(i, i) = 0.0;
    mKd(i, i) = 0.0;
  }
  for (int i = 6; i < 35; i++)
    mKp(i, i) = 400.0;
  for (int i = 6; i < 35; i++)
    mKd(i, i) = 40.0;

  // Make shoulders and elbows loose
  for (int i = 27; i < 35; i++) {
    mKp(i, i) = 20.0;
    mKd(i, i) = 2.0;
  }

  // Make wrists even looser
  for (int i = 35; i < 39; i++) {
    mKp(i, i) = 1.0;
    mKd(i, i) = 0.1;
  }

  mPreOffset = 0.0;
  mLeftHandHold = NULL;
  mRightHandHold = NULL;
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  mTimer = 300;
  mState = "STAND";
  mArch = 0;
}

// ================================================================================================
Controller::~Controller() {
}

// ================================================================================================
Eigen::VectorXd Controller::getTorques() {
  return mTorques;
}

// ================================================================================================
double Controller::getTorque(int _index) {
  return mTorques[_index];
}

// ================================================================================================
void Controller::setDesiredDof(int _index, double _val) {
  mDesiredDofs[_index] = _val;
}

// ================================================================================================
bool releaseCalled = false;
void Controller::computeTorques(int _currentFrame) {

	if(_currentFrame % 100 == 0) {
		Eigen::Vector3d com = mSkel->getWorldCOM();
		Eigen::Vector3d com_dq = mSkel->getWorldCOMVelocity();
		cout << "frame: " << _currentFrame << endl;
		cout << "\tcom: " << com.transpose() << endl;
		cout << "\tqdot: " << com_dq.transpose() << endl;
	}
	if(releaseCalled && mRightHandContact) cout << "frame: " << _currentFrame << ", contact!" << endl;

  mCurrentFrame = _currentFrame;
  mTorques.setZero();
  if (mState == "STAND") {
    stand();
  } else if (mState == "CROUCH") {
    crouch();
  } else if (mState == "JUMP") {
    jump();
  } else if (mState == "REACH") {
    reach();
  } else if (mState == "GRAB") {
    grab();
  } else if (mState == "RELEASE") {
    release();
  } else if (mState == "SWING") {
    swing();
  } else if (mState == "MOVE_LEGS_FORWARD") {
    moveLegsForward();
  } else if (mState == "REACH_RIGHT_HAND") {
    reachRightHand();
  } else if (mState == "REACH_LEFT_HAND") {
    reachLeftHand();
  } else {
    std::cout << "Illegal state: " << mState << std::endl;
  }

  // Just to make sure no illegal torque is used. Do not remove this.
  for (int i = 0; i < 6; i++) {
    mTorques[i] = 0.0;
  }
}

// ================================================================================================
void Controller::checkContactState() {
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  dart::collision::CollisionDetector* cd = mConstraintSolver->getCollisionDetector();
  int nContacts = cd->getNumContacts();
  for (int i = 0; i < nContacts; i++) {
    dart::dynamics::BodyNode* body1 = cd->getContact(i).bodyNode1;
    dart::dynamics::BodyNode* body2 = cd->getContact(i).bodyNode2;
    if (body1 == mSkel->getBodyNode("h_heel_left") || body1 == mSkel->getBodyNode("h_heel_left")
        || body1 == mSkel->getBodyNode("h_heel_right") || body1 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body2;
    if (body2 == mSkel->getBodyNode("h_heel_left") || body2 == mSkel->getBodyNode("h_heel_left")
        || body2 == mSkel->getBodyNode("h_heel_right") || body2 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body1;
  }
}

// ================================================================================================
void Controller::stand() {
  // Change to default standing pose
  mDesiredDofs = mDefaultPose;
  stablePD();
  ankleStrategy();
  mTimer--;

  // Switch to crouch if time is up
  if (mTimer == 0) {
    mState = "CROUCH";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "STAND -> CROUCH" << std::endl;
  }
}

// ================================================================================================
void Controller::crouch() {
  // Change to crouching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.7;
  mDesiredDofs[9] = 0.7;
  mDesiredDofs[14] = -1.1;
  mDesiredDofs[15] = -1.1;
  mDesiredDofs[17] = 0.6;
  mDesiredDofs[19] = 0.6;
  mDesiredDofs[13] = -0.2;

  // After a while, lean forward
  if (mTimer < 200) {
    mDesiredDofs[17] = 1.0;
    mDesiredDofs[19] = 1.0;
  }

  stablePD();
  ankleStrategy();
  mTimer--;

  if (mTimer == 0) {
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "CROUCH -> JUMP" << std::endl;

  }
}

// ================================================================================================
void Controller::jump() {
  // Change to leaping pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.2;
  mDesiredDofs[9] = 0.2;
  mDesiredDofs[14] = -0.2;
  mDesiredDofs[15] = -0.2;
  mDesiredDofs[17] = -0.2;
  mDesiredDofs[19] = -0.2;
  mDesiredDofs[27] = 0.3;
  mDesiredDofs[28] = -1.0;
  mDesiredDofs[30] = 0.3;
  mDesiredDofs[31] = 1.0;
  mDesiredDofs[33] = 0.5;
  mDesiredDofs[34] = 0.5;
  stablePD();

  // Use Jacobian transpose to compute pushing torques
  Eigen::Vector3d vf(-1100.0, -2600, 0.0);
  Eigen::Vector3d offset(0.05, -0.02, 0.0);
  virtualForce(vf, mSkel->getBodyNode("h_heel_left"), offset);
  virtualForce(vf, mSkel->getBodyNode("h_heel_right"), offset);

  checkContactState();
  if (mFootContact == NULL) {
    mState = "REACH";
    std::cout << mCurrentFrame << ": " << "JUMP -> REACH" << std::endl;
  }
}

// ================================================================================================
void Controller::reach() {
  // Change to reaching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.2;
  mDesiredDofs[9] = 0.2;
  mDesiredDofs[14] = -0.2;
  mDesiredDofs[15] = -0.2;
  mDesiredDofs[17] = -0.2;
  mDesiredDofs[19] = -0.2;
  mDesiredDofs[27] = 0.7;
  mDesiredDofs[28] = -2.3;
  mDesiredDofs[30] = 0.7;
  mDesiredDofs[31] = 2.3;
  mDesiredDofs[33] = 0.4;
  mDesiredDofs[34] = 0.4;
  stablePD();

  checkContactState();
  if (mFootContact) { // If feet are in contact again, go back to JUMP and continue to push
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "REACH -> JUMP" << std::endl;
  } else if (mLeftHandContact || mRightHandContact) {
    mState = "GRAB";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "REACH -> GRAB" << std::endl;
  } else {
    mState = "REACH";
  }
}

// ================================================================================================
void Controller::grab() {
  leftHandGrab();
  rightHandGrab();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 0.2;
  mDesiredDofs[9] = 0.2;
  mDesiredDofs[14] = -0.2;
  mDesiredDofs[15] = -0.2;
  mDesiredDofs[17] = -0.2;
  mDesiredDofs[19] = -0.2;
  mDesiredDofs[27] = 0.7;
  mDesiredDofs[28] = -2.3;
  mDesiredDofs[30] = 0.7;
  mDesiredDofs[31] = 2.3;
  mDesiredDofs[33] = 0.4;
  mDesiredDofs[34] = 0.4;
  stablePD();
  mTimer--;

  if (mTimer == 0) {
    mState = "SWING";
    mTimer = 200;
    std::cout << mCurrentFrame << ": " << "GRAB -> SWING" << std::endl;
  }
}  

// ================================================================================================
void Controller::moveLegsForward() {
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[6] = 1.0; 
  mDesiredDofs[9] = 1.0; 
  
  stablePD();
  mTimer--;

	if (mTimer == 0) {
    mState = "SWING";
		mTimer = 1500;
    std::cout << mCurrentFrame << ": " << "MOVE_LEGS_FORWARD -> SWING" << std::endl;
  }
}

// ================================================================================================
void Controller::swing() {

	int counter = 0;
	counter++;
	bool dbg = counter % 25;

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[27] = 1;
  mDesiredDofs[28] = -2.6;
  mDesiredDofs[30] = 1;
  mDesiredDofs[31] = 2.6;
  mDesiredDofs[33] = 0.4;
  mDesiredDofs[34] = 0.4;
  mDesiredDofs[13] = 0.0; 
  
  stablePD();
  mTimer--;

	// Determine if need to jump
  Eigen::Vector3d com = mSkel->getWorldCOM();
	Eigen::Vector3d com_dq = mSkel->getWorldCOMVelocity();
	bool jump = false;
	dart::dynamics::BodyNode* nextBar = mWorld->getSkeleton(barName.c_str())->getBodyNode("box");
	Eigen::Vector3d barLoc = nextBar->getTransform().translation();
	if((com(0) > (barLoc(0) + 0.15)) && (com_dq(0) > 0.1)) jump = true;
	if(dbg) printf("%lf vs. %lf, %lf\n", com(0), barLoc(0), com_dq(0));

	static double lastCOM = com(0); 
	static double lastCOMdq = com_dq(0);
	static bool startCounting = false;
	if(!startCounting) 
		startCounting = (lastCOMdq < 0 && (com_dq(0) > 0));
	static int bla = 0;
	if(startCounting) bla++;
	// Jump or move to the next bar if possible
	if(jump || mJump) {

		// Check if there is a second bar; if not, jump
		if(mWorld->getSkeleton("bar2") == NULL) {
			std::cout << mCurrentFrame << ": " << "SWING-> RELEASE " << std::endl;
			mState = "RELEASE";
			mTimer = 0;
			bla = 0;
			startCounting = false;
		}

		// Otherwise, move to the next bar
		else {
			bla = 0;
			startCounting = false;
			std::cout << mCurrentFrame << ": " << "SWING -> REACH_RIGHT_HAND" << std::endl;
			mState = "REACH_RIGHT_HAND";
		}
	}

	// If not ready to jump, move legs to increase velocity
//  else if (mTimer == 0) {
  // else if (com(0) > (barLoc(0) - 0.1)) {
 // else if ((com(0) > (barLoc(0) - 0.2)) && (com_dq(0) > 0.0)){

	// else if(lastCOM <= com(0) && (com_dq(0) > 0)) {
	else if (bla > 200) {
//		mState = "REACH_LEFT_HAND";
//		std::cout << mCurrentFrame << ": " << "SWING -> REACH_LEFT_HAND" << std::endl;

		bla = 0;
		startCounting = false;
		cout << "hi?" << endl;
		mState = "MOVE_LEGS_FORWARD";
		mTimer = 500;
		std::cout << mCurrentFrame << ": " << "SWING -> MOVE_LEGS_FORWARD" << std::endl;
	}

	lastCOM = com(0);
	lastCOMdq = com_dq(0);
}

// ================================================================================================
void Controller::resetArmGains () {

	// Make shoulders and elbows loose
  for (int i = 27; i < 35; i++) {
    mKp(i, i) = 20.0;
    mKd(i, i) = 2.0;
  }

  // Make wrists even looser
  for (int i = 35; i < 39; i++) {
    mKp(i, i) = 1.0;
    mKd(i, i) = 0.1;
  }
}

// ================================================================================================
void Controller::reachLeftHand() {

	// Let go of the left hand
	static int counter = 0;
	if(counter == 0) leftHandRelease();
	counter++;
	if(counter % 100 == 0) cout << "reachLeftHand counter: " << counter << endl;

	// Move to the new object
	dart::dynamics::BodyNode* nextBar = mWorld->getSkeleton(barName.c_str())->getBodyNode("box");
	Eigen::Vector3d goal = nextBar->getTransform().translation() + Eigen::Vector3d(0.0, 0.0, -0.25);
	Eigen::Vector3d hand = mSkel->getBodyNode("h_hand_left")->getTransform().translation();
	Eigen::VectorXd pose = ik(mSkel->getBodyNode("h_hand_left"), goal);
	size_t leftArmIds [] = {27, 28, 29, 33, 35, 37};
	for(size_t i = 0; i < 6; i++)
		mDesiredDofs[leftArmIds[i]] = pose[leftArmIds[i]];

	// Make shoulders and elbows loose
  for (int i = 0; i < 6; i++) {
		int j = leftArmIds[i];
    mKp(j, j) = 400.0;
    mKd(j, j) = 40.0;
  }

	// Break the left elbow to get the right hand higher
	mDesiredDofs[34] = 2* M_PI / 3.0;
	mKp(34,34) = 400.0;
	mKd(34,34) = 40.0;

	// Attempt to hold the next object and if you can, change state
	if(counter > 500) leftHandGrab();
	if((counter > 500) && (mLeftHandHold != NULL)) {
		counter = 0;
		mKp(33,33) = 20.0;
    mState = "SWING";
    std::cout << mCurrentFrame << ": " << "REACH_LEFT_HAND -> SWING" << std::endl;
		barName = "bar4";
		resetArmGains();

		leftHandRelease();
		Eigen::VectorXd state = mSkel->getState();
		for(size_t i = 0; i < 6; i++)
			state[leftArmIds[i]] = pose[leftArmIds[i]];
		mSkel->setState(state);
		leftHandGrab();
	
	}
	
	stablePD();
}

// ================================================================================================
void Controller::reachRightHand() {
	
	// Release the right hand
	static int counter = 0;
	if(counter == 0) rightHandRelease();
	counter++;

	// Move to the new object
	dart::dynamics::BodyNode* nextBar = mWorld->getSkeleton(barName.c_str())->getBodyNode("box");
	Eigen::Vector3d goal = nextBar->getTransform().translation() + Eigen::Vector3d(0.0, 0.0, 0.25);
	Eigen::Vector3d hand = mSkel->getBodyNode("h_hand_right")->getTransform().translation();
	// goal(2) = hand(2);
	Eigen::VectorXd pose = ik(mSkel->getBodyNode("h_hand_right"), goal);
	size_t rightArmIds [] = {30, 31, 32, 34, 36, 38};
	for(size_t i = 0; i < 6; i++)
		mDesiredDofs[rightArmIds[i]] = pose[rightArmIds[i]];

	// Break the left elbow to get the right hand higher
	mDesiredDofs[33] = M_PI / 3.0;
	mKp(33,33) = 400.0;

	// Attempt to hold the next object and if you can, change state
	if(counter > 500) rightHandGrab();
	if((counter > 500) && (mRightHandHold != NULL)) {
		mKp(33,33) = 20.0;
    mState = "REACH_LEFT_HAND";
    std::cout << mCurrentFrame << ": " << "REACH_RIGHT_HAND -> REACH_LEFT_HAND" << std::endl;
		counter = 0;

		// Cheat and realign the hand - which is not really cheating in real life
		rightHandRelease();
		Eigen::VectorXd state = mSkel->getState();
		for(size_t i = 0; i < 6; i++)
			state[rightArmIds[i]] = pose[rightArmIds[i]];
		mSkel->setState(state);
		rightHandGrab();
		
	}
	
	stablePD();
}

// ================================================================================================
void Controller::release() {
	releaseCalled = true;
  leftHandRelease();
  rightHandRelease();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[13] = -1.5;
  mDesiredDofs[14] = -1.5;
  mDesiredDofs[15] = -1.5;
  stablePD();
}
  
// ================================================================================================
void Controller::stablePD() {
  Eigen::VectorXd q = mSkel->getPositions();
  Eigen::VectorXd dq = mSkel->getVelocities();
  Eigen::VectorXd constrForces = mSkel->getConstraintForces();
  Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  Eigen::VectorXd p = -mKp * (q + dq * mTimestep - mDesiredDofs);
  Eigen::VectorXd d = -mKd * dq;
  Eigen::VectorXd qddot =
      invM * (-mSkel->getCoriolisAndGravityForces() + p + d + constrForces);

  mTorques += p + d - mKd * qddot * mTimestep;
}

// ================================================================================================
void Controller::ankleStrategy() {
  Eigen::Vector3d com = mSkel->getWorldCOM();
  Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getTransform()
                        * Eigen::Vector3d(0.05, 0, 0);
  double offset = com[0] - cop[0];
   if (offset < 0.1 && offset > 0.0) {
    double k1 = 200.0;
    double k2 = 100.0;
    double kd = 10.0;
    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  } else if (offset > -0.2 && offset < -0.05) {
    double k1 = 2000.0;
    double k2 = 100.0;
    double kd = 100.0;
    mTorques[17] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[25] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[19] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[26] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }  
}

// ================================================================================================
void Controller::virtualForce(Eigen::Vector3d _force, dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _offset) {
  Eigen::MatrixXd jacobian = mSkel->getJacobian(_bodyNode, _offset);
  mTorques += jacobian.transpose() * _force;
}

// ================================================================================================
void Controller::leftHandGrab() {  
  if (mLeftHandHold != NULL)
    return;
  checkContactState();
  if (mLeftHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_left");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mLeftHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mLeftHandHold = hold;
}

// ================================================================================================
void Controller::leftHandRelease() {
  if (mLeftHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mLeftHandHold);
  mSkel->getBodyNode("h_hand_left")->setCollidable(true);
  mLeftHandHold = NULL;
}

// ================================================================================================
void Controller::rightHandGrab() {  
  if (mRightHandHold != NULL)
    return;

  checkContactState();
  if (mRightHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_right");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mRightHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mRightHandHold = hold;
}

// ================================================================================================
void Controller::rightHandRelease() {
  if (mRightHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mRightHandHold);
  mSkel->getBodyNode("h_hand_right")->setCollidable(true);
  mRightHandHold = NULL;
}

// ================================================================================================
dart::dynamics::Skeleton*Controller::getSkel() {
  return mSkel;
}

// ================================================================================================
Eigen::VectorXd Controller::getDesiredDofs() {
  return mDesiredDofs;
}

// ================================================================================================
Eigen::MatrixXd Controller::getKp() {
  return mKp;
}

// ================================================================================================
Eigen::MatrixXd Controller::getKd() {
  return mKd;
}

// ================================================================================================
Eigen::VectorXd Controller::ik(dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _target) {
  Eigen::Vector3d offset = _bodyNode->getLocalCOM();
  Eigen::VectorXd oldPose = mSkel->getPositions();
  Eigen::VectorXd newPose;
	size_t rightArmIds [] = {30, 31, 32, 34, 36, 38};
	size_t leftArmIds [] = {27, 28, 29, 33, 35, 37};
	size_t armIds [6];
	bool right = (_bodyNode->getName().compare("h_hand_right") == 0);
	for(size_t i = 0; i < 6; i++) armIds[i] = right ? rightArmIds[i] : leftArmIds[i];
	double improvement = 1000.0;
	double lastNorm = 1000.0;
	// printf("ik for %s\n", right ? "right" : "left");
  for (int i = 0; i < 200; i++) {
    Eigen::Vector3d diff = _bodyNode->getWorldCOM() - _target;
		double norm = diff.norm();
		// printf("\t%lf\n", diff.norm());
    Eigen::MatrixXd jFull= mSkel->getJacobian(_bodyNode, offset);
    // jacobian.block(0, 0, 3, 6).setZero();
		Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(jFull.rows(), jFull.cols());
		for(size_t i = 0; i < 6; i++)
			jacobian.block<3,1>(0,armIds[i]) = jFull.block<3,1>(0, armIds[i]);
    newPose = mSkel->getPositions() - 0.1 * 2 * jacobian.transpose() * diff;
    mSkel->setPositions(newPose); 
    mSkel->computeForwardKinematics(true, false, false); // DART updates all the transformations based on newPose
		if(fabs(norm - lastNorm) < 0.0001) break;
		lastNorm = norm;
  }
  mSkel->setPositions(oldPose);
  mSkel->computeForwardKinematics(true, false, false);
  return newPose;
}  

// ================================================================================================
// 0-5 pelvis
// 6-8 left hip
// 9-11 right hip
// 12,13 abdomen
// 14 left knee
// 15 right knee
// 16 spine
// 17,18 left ankle
// 19,20 right ankle
// 21,22 head
// 23 left scapula
// 24 right scapula
// 25 left toe
// 26 right toe
// 27-29 left shoulder
// 30-32 right shoulder
// 33 left elbow
// 34 right elbow
// 35 left wrist
// 36 right wrist
// ================================================================================================
