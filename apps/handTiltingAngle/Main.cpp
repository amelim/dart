/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Yunfei Bai <ybai30@mail.gatech.edu>,
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

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/simulation/World.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/urdf/DartLoader.h"

#include "apps/handTiltingAngle/MyWindow.h"

using namespace std;
using namespace Eigen;

using namespace dart;
using namespace dynamics;
using namespace simulation;
using namespace utils;

int main(int argc, char* argv[])
{
  // create and initialize the world
  World *myWorld = new World();
  Vector3d gravity(0.0, -9.81, 0.0);
  myWorld->setGravity(gravity);

  // load a skeleton file
  Skeleton* ground = SkelParser::readSkeleton(DART_DATA_PATH"skel/ground2.skel");
  Skeleton* cube = SkelParser::readSkeleton(DART_DATA_PATH"skel/cube1.skel");

  //
  myWorld->addSkeleton(ground);
  myWorld->addSkeleton(cube);

  //myWorld->getConstraintSolver()->set->mMu = 1.5;
  for (size_t i = 0; i < myWorld->getNumSkeletons(); ++i)
  {
    Skeleton* skel = myWorld->getSkeleton(i);

    for (size_t j = 0; j < skel->getNumBodyNodes(); ++j)
    {
      BodyNode* body = skel->getBodyNode(j);

      body->setFrictionCoeff(1.5);
    }
  }

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(myWorld);
  window.updateAngles();

  VectorXd initPose = ground->getPositions();
  initPose[2] = window.getAngle(0);
  ground->setPositions(initPose);
  ground->computeForwardKinematics(true, true, true);

  initPose.setZero();
  Vector3d localPos(-0.5, 0.2 + 0.5 * 0.05, 0.0);
  Vector3d worldPos = ground->getBodyNode(0)->getTransform() * localPos;
  Isometry3d cubeT = Isometry3d::Identity();
  cubeT.translation() = worldPos;
  cubeT.linear() = math::eulerXYZToMatrix(Vector3d(0.0, 0.0, window.getAngle(0)));
  cube->setPositions(math::logMap(cubeT));
  cube->computeForwardKinematics(true, true, true);

  cout << "space bar: simulation on/off" << endl;
  cout << "'p': playback/stop" << endl;
  cout << "'[' and ']': play one frame backward and forward" << endl;
  cout << "'v': visualization on/off" << endl;
  cout << "'1'--'4': programmed interaction" << endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Boxes");
  glutMainLoop();

  return 0;
}
