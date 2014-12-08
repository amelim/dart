#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>
#include "dart/dynamics/Skeleton.h"
#include <map>


class MyWorld {
 public:
    MyWorld();
    virtual ~MyWorld();
    dart::dynamics::Skeleton* getSkel() {
        return mSkel;
    }

    void solve();
    void createConstraint(int _index);
    void modifyConstraint(int _index, Eigen::Vector3d _deltaP);
    void removeConstraint(int _index);
		Eigen::MatrixXd getArmJacobian (dart::dynamics::BodyNode* node, Eigen::Vector4d offset);

    Eigen::VectorXd updateGradients();

    dart::dynamics::Skeleton *mSkel;
		std::map <int, Eigen::Vector3d> constraints;
		std::map <int, double> weights;
		bool noBadKnees;
};

#endif
