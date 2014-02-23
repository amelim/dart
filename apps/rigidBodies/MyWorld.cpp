#include "MyWorld.h"
#include "RigidBody.h"
#include "CollisionInterface.h"
#include "kinematics/FileInfoSkel.hpp"
#include "robotics/parser/dart_parser/DartLoader.h"
#include "utils/Paths.h"
#include "dynamics/SkeletonDynamics.h"

using namespace Eigen;

MyWorld::MyWorld() {
    mFrame = 0;

    // Create a collision detector
    mCollisionDetector = new CollisionInterface();

    // Add rigid bodies (this will be replaced by your code) 
    RigidBody *rb1 = new RigidBody(kinematics::Shape::P_BOX, Vector3d(0.1, 0.1, 0.1));
    mCollisionDetector->addRigidBody(rb1); // Put rb1 in collision detector
    mRigidBodies.push_back(rb1);
    
    RigidBody *rb2 = new RigidBody(kinematics::Shape::P_ELLIPSOID, Vector3d(0.1, 0.2, 0.1));
    mCollisionDetector->addRigidBody(rb2); // Put rb2 in collision detector
    rb2->mPosition[0] = 0.1;
    rb2->mColor = Vector4d(0.2, 0.8, 0.2, 1.0); // Blue
    mRigidBodies.push_back(rb2);

    // Load a blender and a blade
    DartLoader dl;
    string blenderFileName(DART_DATA_PATH"urdf/cylinder.urdf");
    mBlender = dl.parseSkeleton(blenderFileName);
    mCollisionDetector->addSkeleton(mBlender); // Put blender in collision detector

    string bladeFileName(DART_DATA_PATH"urdf/blade.urdf");
    mBlade = dl.parseSkeleton(bladeFileName);
    VectorXd pose = mBlade->getPose();
    pose[1] = -0.3;
    mBlade->setPose(pose, true, false);
    mCollisionDetector->addSkeleton(mBlade); // Put blade in collision detector
}

MyWorld::~MyWorld() {
    for (int i = 0; i < mRigidBodies.size(); i++)
        delete mRigidBodies[i];
    mRigidBodies.clear();
    if (mCollisionDetector)
        delete mCollisionDetector;
    if (mBlender)
        delete mBlender;
    if (mBlade)
        delete mBlade;
}

void MyWorld::simulate() {
    // TODO: Replace the following code
    for (int i = 0; i < mRigidBodies.size(); i++)
        mRigidBodies[i]->mPosition[1] -= 0.005;

    // Run collision detector
    mCollisionDetector->checkCollision();

    // Move the blade
    VectorXd pose = mBlade->getPose();
    pose[4] += 0.01;
    if (pose[4] > 2 * 3.14)
        pose[4] = 0.0;
    mBlade->setPose(pose, true, false);
}
