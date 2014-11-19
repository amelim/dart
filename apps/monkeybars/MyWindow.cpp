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

#include <iostream>
#include <stdio.h>
#include "MyWindow.h"

#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/gui/GLFuncs.h"
#include "dart/utils/FileInfoWorld.h"

using namespace std;

MyWindow::MyWindow(): SimWindow() {
  mForce = Eigen::Vector3d::Zero();
  mController = NULL;
  mImpulseDuration = 0;

  mTrans[0] = -600.f;
  mZoom = 0.25f;
}

MyWindow::~MyWindow() {
}

void MyWindow::timeStepping() {
  mWorld->getSkeleton(1)->getBodyNode("h_spine")->addExtForce(mForce);

  mController->computeTorques(mWorld->getSimFrames());
  mWorld->getSkeleton("fullbody1")->setForces(mController->getTorques());

  mWorld->step();

  // for perturbation test
  mImpulseDuration--;
  if (mImpulseDuration <= 0) {
    mImpulseDuration = 0;
    mForce.setZero();
  }
}

void MyWindow::drawSkels() {
  glEnable(GL_LIGHTING);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++)
    mWorld->getSkeleton(i)->draw(mRI);

	// Draw the vector to the next bar
	if(mWorld->getSkeleton("bar3") != NULL) {
		dart::dynamics::Skeleton* mSkel = mWorld->getSkeleton("fullbody1");
		dart::dynamics::BodyNode* nextBar = mWorld->getSkeleton("bar3")->getBodyNode("box");
		Eigen::Vector3d goal = nextBar->getTransform().translation();
		Eigen::Vector3d hand = mSkel->getBodyNode("h_hand_right")->getTransform().translation();
		goal(2) = hand(2);
		double length = (goal - hand).norm();
		Eigen::Vector3d dir = (goal - hand).normalized();
		dart::gui::drawArrow3D(hand, dir, length, 0.02); 
	}

}

void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
	static int stepSize = 10;
  switch (_key) {
		case 'j':
			mController->mJump = true;
			break;
		case 'i': {
	dart::dynamics::Skeleton* mSkel = mWorld->getSkeleton("fullbody1");
			dart::dynamics::BodyNode* nextBar = mWorld->getSkeleton("bar3")->getBodyNode("box");
			Eigen::Vector3d goal = nextBar->getTransform().translation();
			Eigen::Vector3d hand = mSkel->getBodyNode("h_hand_right")->getTransform().translation();
			goal(2) = hand(2);
			Eigen::VectorXd pose = mController->ik(mSkel->getBodyNode("h_hand_right"), goal);
		} break;

		case '=': stepSize += 5; printf("step size: %d\n", stepSize); break;
		case '-': stepSize -= 5; printf("step size: %d\n", stepSize); break;
    case ' ':  // use space key to play or stop the motion
      mSimulating = !mSimulating;
      if (mSimulating) {
        mPlay = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case 'p':  // playBack
      mPlay = !mPlay;
      if (mPlay) {
        mSimulating = false;
        glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
      }
      break;
    case '[':  // step backward
			if (!mSimulating) {
				mPlayFrame-=stepSize;
        if (mPlayFrame < 0)
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case ']':  // step forwardward
      if (!mSimulating) {
        mPlayFrame+=stepSize;
        if (mPlayFrame >= mWorld->getRecording()->getNumFrames())
          mPlayFrame = 0;
        glutPostRedisplay();
      }
      break;
    case 'v':  // show or hide markers
      mShowMarkers = !mShowMarkers;
      break;
    default:
      Win3D::keyboard(_key, _x, _y);
  }
  glutPostRedisplay();
}

void MyWindow::setController(Controller* _controller) {
  mController = _controller;
}

