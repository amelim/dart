#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>

namespace kinematics {
    class Skeleton;
}

class RigidBody;
class CollisionInterface;

class MyWorld {
 public:
    MyWorld();

    virtual ~MyWorld();

    int getNumRigidBodies() {
        return mRigidBodies.size();
    }

    RigidBody* getRigidBody(int _index) {
        return mRigidBodies[_index];
    }
   
    // TODO: your simulation code goes here
    void simulate();
   
    kinematics::Skeleton* getBlender() {
        return mBlender;
    }

    kinematics::Skeleton* getBlade() {
        return mBlade;
    }
    
    
    CollisionInterface* getCollisionDetector() {
        return mCollisionDetector;
    }

    int getSimFrames() const { 
        return mFrame; 
    }

 protected:
    int mFrame;
    std::vector<RigidBody*> mRigidBodies;
    CollisionInterface* mCollisionDetector; // Access to collision detection information
    kinematics::Skeleton* mBlender;
    kinematics::Skeleton* mBlade;
};

#endif
