/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/Console.h"
#include "dart/math/Geometry.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/Paths.h"

using namespace Eigen;

using namespace dart;
using namespace dynamics;

//==============================================================================
class AccuracyTest : public ::testing::Test
{
public:
  // Spawn a single box and record accuracy for momentum and energy conservation
  void Boxes(double _dt,
             int _iterations,
             int _boxCount,
             bool _gravity,
             bool _collision,
             bool _linear);
};

//==============================================================================
void AccuracyTest::Boxes(double _dt,
                         int _iterations,
                         int _boxCount,
                         bool _gravity,
                         bool _collision,
                         bool _linear)
{
  World* world = new World();

  // get gravity value
  if (_gravity)
    world->setGravity(Vector3d(0.0, 0.0, -9.81));
  else
    world->setGravity(Vector3d::Zero());

  Eigen::Vector3d g = world->getGravity();

  // Box size
  const double dx = 0.1;
  const double dy = 0.4;
  const double dz = 0.9;
  const double mass = 10.0;
  // inertia matrix, recompute if the above change
  const double Ixx = 0.80833333;
  const double Iyy = 0.68333333;
  const double Izz = 0.14166667;
  const Matrix3d I0 = Vector3d(Ixx, Iyy, Izz).asDiagonal();

  // Create box with inertia based on box of uniform density
  Skeleton* boxSkel = new Skeleton();
  BodyNode* box = new BodyNode();
  BoxShape* boxVizShape = new BoxShape(Vector3d(dx, dy, dz));
  BoxShape* boxColShape = new BoxShape(Vector3d(dx, dy, dz));
  box->addVisualizationShape(boxVizShape);
  box->addCollisionShape(boxColShape);
  boxSkel->addBodyNode(box);
  world->addSkeleton(boxSkel);

  // initial linear velocity in global frame
  const Vector3d v0(0.1, 0.4, 0.9);

  // initial angular velocity in global frame
  Vector3d w0;

  // initial energy value
  double E0;

  if (_linear)
  {
    // Use angular velocity with one non-zero component
    // to ensure linear angular trajectory
    w0 << 1.5e-1, 0.0, 0.0;
    E0 = 4.9090937462499999;
  }
  else
  {
    // Since Ixx > Iyy > Izz,
    // angular velocity with large y component
    // will cause gyroscopic tumbling
    w0 << 1e-3, 1.5e0, 1.5e-2;
    E0 = 5.668765966704;
  }

  for (int i = 0; i < _boxCount; ++i)
  {
    Skeleton* skel = world->getSkeleton(i);

    // give models unique names
    skel->setName("model");

    // give models unique positions
    msgs::Set(msgModel.mutable_pose()->mutable_position(),
              math::Vector3(dz*2*i, 0.0, 0.0));

    // Set initial conditions
    link->SetLinearVel(v0);
    link->SetAngularVel(w0);
  }
  ASSERT_EQ(v0, link->GetWorldCoGLinearVel());
  ASSERT_EQ(w0, link->GetWorldAngularVel());
  ASSERT_EQ(I0, link->GetInertial()->GetMOI());
  ASSERT_NEAR(link->GetWorldEnergy(), E0, 1e-6);

  delete world;
}

//==============================================================================
TEST_F(AccuracyTest, testBoxes)
{
  Boxes(0.001, 50, 1, false, false, true);
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


