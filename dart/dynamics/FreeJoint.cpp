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

//==============================================================================
FreeJoint::FreeJoint(const std::string& _name)
  : MultiDofJoint(_name)
{
}

//==============================================================================
FreeJoint::~FreeJoint()
{
}

//==============================================================================
void FreeJoint::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
{
  Joint::setTransformFromChildBodyNode(_T);

//  Eigen::Matrix6d J = Eigen::Matrix6d::Identity();

//  mJacobian.col(0) = math::AdT(mT_ChildBodyToJoint, J.col(0));
//  mJacobian.col(1) = math::AdT(mT_ChildBodyToJoint, J.col(1));
//  mJacobian.col(2) = math::AdT(mT_ChildBodyToJoint, J.col(2));
//  mJacobian.col(3) = math::AdT(mT_ChildBodyToJoint, J.col(3));
//  mJacobian.col(4) = math::AdT(mT_ChildBodyToJoint, J.col(4));
//  mJacobian.col(5) = math::AdT(mT_ChildBodyToJoint, J.col(5));

//  assert(mJacobian.topRightCorner(3, 3) == Eigen::Matrix3d::Zero());

  assert(!math::isNan(mJacobian));
}

//==============================================================================
void FreeJoint::integratePositions(double _dt)
{
//  mQ = mQ * math::expMap(mVelocities * _dt);

//  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//  T.translation() = _dt * mVelocities.tail<3>();
//  T.linear()      = math::expMapRot(_dt * mVelocities.head<3>());
//  mQ = mQ * T;

//  mPositions = math::logMap(mQ);
}

//==============================================================================
void FreeJoint::updateLocalTransform()
{
  const Eigen::Vector3d& r = mPositions.head<3>();
  const Eigen::Vector3d& p = mPositions.tail<3>();

  mT = mT_ParentBodyToJoint
       * Eigen::Translation3d(p)
       * math::expAngular(r)
       * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void FreeJoint::updateLocalJacobian()
{
  const Eigen::Matrix3d& R       = mT_ChildBodyToJoint.linear();
  const Eigen::Vector3d& p       = mT_ChildBodyToJoint.translation();
  const Eigen::Matrix3d& skewP   = math::makeSkewSymmetric(p);
  const Eigen::Vector3d& r       = mPositions.head<3>();
  const Eigen::Matrix3d& invExp  = math::expMapRot(-r);
  const Eigen::Matrix3d& expJ    = math::expMapJac(r);
  //const Eigen::Matrix3d& invExpJ = math::expMapJac(-r);

  mJacobian.topLeftCorner<3, 3>()     = R * invExp * expJ;
  assert(mJacobian.topRightCorner(3, 3) == Eigen::Matrix3d::Zero());
  mJacobian.bottomLeftCorner<3, 3>()  = skewP * R * invExp * expJ;
  mJacobian.bottomRightCorner<3, 3>() = R * invExp;
}

//==============================================================================
void FreeJoint::updateLocalJacobianTimeDeriv()
{
  // Time derivative of Jacobian is constant
  assert(mJacobianDeriv == (Eigen::Matrix6d::Zero()));
}


//==============================================================================
FreeJoint2::FreeJoint2(const std::string& _name)
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
FreeJoint2::~FreeJoint2()
{
}

//==============================================================================
void FreeJoint2::setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
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
void FreeJoint2::integratePositions(double _dt)
{
  mQ = mQ * math::expMap(mVelocities * _dt);

//  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//  T.translation() = _dt * mVelocities.tail<3>();
//  T.linear()      = math::expMapRot(_dt * mVelocities.head<3>());
//  mQ = mQ * T;

  mPositions = math::logMap(mQ);
}

//==============================================================================
void FreeJoint2::updateLocalTransform()
{
  mQ = math::expMap(mPositions);

  mT = mT_ParentBodyToJoint * mQ * mT_ChildBodyToJoint.inverse();

  assert(math::verifyTransform(mT));
}

//==============================================================================
void FreeJoint2::updateLocalJacobian()
{
  // Do nothing since Jacobian is constant
}

//==============================================================================
void FreeJoint2::updateLocalJacobianTimeDeriv()
{
  // Time derivative of Jacobian is constant
  assert(mJacobianDeriv == (Eigen::Matrix6d::Zero()));
}

}  // namespace dynamics
}  // namespace dart
