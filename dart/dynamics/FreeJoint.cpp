/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/FreeJoint.h"

#include <string>

#include "dart/math/Helpers.h"
#include "dart/math/Geometry.h"

namespace dart {
namespace dynamics {

////==============================================================================
//FreeJoint::FreeJoint(const std::string& _name)
//  : MultiDofJoint(_name),
//    mQ(Eigen::Isometry3d::Identity())
//{
//  // Jacobian
//  Eigen::Matrix6d J = Eigen::Matrix6d::Identity();
//  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
//  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
//  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
//  mJacobian.col(3) = math::AdT(mT_ChildBodyToJoint, J.col(3));
//  mJacobian.col(4) = math::AdT(mT_ChildBodyToJoint, J.col(4));
//  mJacobian.col(5) = math::AdT(mT_ChildBodyToJoint, J.col(5));
//  assert(!math::isNan(mJacobian));

//  // Time derivative of Jacobian is always zero
//}

////==============================================================================
//FreeJoint::~FreeJoint()
//{
//}

////==============================================================================
//void FreeJoint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
//{
//  Joint::setTransformFromChildBodyNode(_T);

//  Eigen::Matrix6d J = Eigen::Matrix6d::Identity();

//  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
//  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
//  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
//  mJacobian.col(3) = math::AdT(mT_ChildBodyToJoint, J.col(3));
//  mJacobian.col(4) = math::AdT(mT_ChildBodyToJoint, J.col(4));
//  mJacobian.col(5) = math::AdT(mT_ChildBodyToJoint, J.col(5));

//  assert(!math::isNan(mJacobian));
//}

////==============================================================================
//void FreeJoint::integratePositions(double _dt)
//{
//  mQ = mQ * math::expMap(mVelocities * _dt);

//  mPositions = math::logMap(mQ);
//}

////==============================================================================
//void FreeJoint::updateLocalTransform()
//{
//  mQ = math::expMap(mPositions);

//  mT = mT_ParentBodyToJoint * mQ * mT_ChildBodyToJoint.inverse();

//  assert(math::verifyTransform(mT));
//}

////==============================================================================
//void FreeJoint::updateLocalJacobian()
//{
//  // Do nothing since Jacobian is constant
//}

////==============================================================================
//void FreeJoint::updateLocalJacobianTimeDeriv()
//{
//  // Time derivative of Jacobian is constant
//  assert(mJacobianDeriv == (Eigen::Matrix6d::Zero()));
//}

//==============================================================================
FreeJoint::FreeJoint(const std::string& _name)
  : MultiDofJoint(_name),
    mQ(Eigen::Isometry3d::Identity())
{
  // Jacobian
  Eigen::Matrix6d J = Eigen::Matrix6d::Identity();
  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
  mJacobian.col(3) = math::AdT(mT_ChildBodyToJoint, J.col(3));
  mJacobian.col(4) = math::AdT(mT_ChildBodyToJoint, J.col(4));
  mJacobian.col(5) = math::AdT(mT_ChildBodyToJoint, J.col(5));
  assert(!math::isNan(mJacobian));

  // Time derivative of Jacobian is always zero
}

//==============================================================================
FreeJoint::~FreeJoint()
{
}

//==============================================================================
void FreeJoint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  Joint::setTransformFromChildBodyNode(_T);

  Eigen::Matrix6d J = Eigen::Matrix6d::Identity();

  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
  mJacobian.col(3) = math::AdT(mT_ChildBodyToJoint, J.col(3));
  mJacobian.col(4) = math::AdT(mT_ChildBodyToJoint, J.col(4));
  mJacobian.col(5) = math::AdT(mT_ChildBodyToJoint, J.col(5));

  assert(!math::isNan(mJacobian));
}

//==============================================================================
void FreeJoint::integratePositions(double _dt)
{
//  mQ = mQ * math::expMap(mVelocities * _dt);
  mQ.linear() = mQ.linear() * math::expMapRot(mVelocities.head<3>() * _dt);
  mQ.translation() += mVelocities.tail<3>() * _dt;

  mPositions = math::logMap(mQ);
}

//==============================================================================
void FreeJoint::updateLocalTransform()
{
//  mQ = math::expMap(mPositions);
  mQ.linear() = math::expMapRot(mPositions.head<3>());
  mQ.translation() = mPositions.tail<3>();

  mT = mT_ParentBodyToJoint * mQ * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void FreeJoint::updateLocalJacobian()
{
  Eigen::Vector3d q(mPositions[0], mPositions[1], mPositions[2]);

  Eigen::Matrix3d J = math::expMapJac(q);

  Eigen::Vector6d J0;
  Eigen::Vector6d J1;
  Eigen::Vector6d J2;
  Eigen::Vector6d J3;
  Eigen::Vector6d J4;
  Eigen::Vector6d J5;

  J0 << J(0,0), J(0,1), J(0,2), 0, 0, 0;
  J1 << J(1,0), J(1,1), J(1,2), 0, 0, 0;
  J2 << J(2,0), J(2,1), J(2,2), 0, 0, 0;
  J3 << 0, 0, 0, 1, 0, 0;
  J4 << 0, 0, 0, 0, 1, 0;
  J5 << 0, 0, 0, 0, 0, 1;

  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J0);
  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J1);
  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J2);
  mJacobian.col(3) = math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J3);
  mJacobian.col(4) = math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J4);
  mJacobian.col(5) = math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J5);
}

//==============================================================================
void FreeJoint::updateLocalJacobianTimeDeriv()
{
  Eigen::Vector3d q(mPositions[0], mPositions[1], mPositions[2]);
  Eigen::Vector3d dq(mVelocities[0], mVelocities[1], mVelocities[2]);

  Eigen::Matrix3d dJ = math::expMapJacDot(q, dq);

  Eigen::Vector6d dJ0;
  Eigen::Vector6d dJ1;
  Eigen::Vector6d dJ2;
  Eigen::Vector6d J3;
  Eigen::Vector6d J4;
  Eigen::Vector6d J5;

  dJ0 << dJ(0,0), dJ(0,1), dJ(0,2), 0, 0, 0;
  dJ1 << dJ(1,0), dJ(1,1), dJ(1,2), 0, 0, 0;
  dJ2 << dJ(2,0), dJ(2,1), dJ(2,2), 0, 0, 0;
  J3 << 0, 0, 0, 1, 0, 0;
  J4 << 0, 0, 0, 0, 1, 0;
  J5 << 0, 0, 0, 0, 0, 1;

  mJacobianDeriv.col(0) = math::AdT(mT_ChildBodyToJoint, dJ0);
  mJacobianDeriv.col(1) = math::AdT(mT_ChildBodyToJoint, dJ1);
  mJacobianDeriv.col(2) = math::AdT(mT_ChildBodyToJoint, dJ2);
  mJacobianDeriv.col(3) = -math::ad(mJacobian.leftCols<3>() * dq.head<3>(), math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J3));
  mJacobianDeriv.col(4) = -math::ad(mJacobian.leftCols<3>() * dq.head<3>(), math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J4));
  mJacobianDeriv.col(5) = -math::ad(mJacobian.leftCols<3>() * dq.head<3>(), math::AdT(mT_ChildBodyToJoint * math::expAngular(-q), J5));
}

}  // namespace dynamics
}  // namespace dart
