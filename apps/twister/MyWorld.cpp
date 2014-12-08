#include "MyWorld.h"
#include "dart/utils/Paths.h"
#include "dart/utils/SkelParser.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Marker.h"
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace dart::dynamics;

MyWorld::MyWorld() {
    // Load a skeleton from file 
    mSkel = dart::utils::SkelParser::readSkeleton(DART_DATA_PATH"skel/human.skel");
		noBadKnees = false;
}

MyWorld::~MyWorld() {
    delete mSkel;
}

void MyWorld::solve() {
		if(constraints.empty()) return;
    int numIter = 300;
    double alpha = 0.01;
    int nDof = mSkel->getNumDofs();
    VectorXd gradients(nDof);
    VectorXd newPose(nDof);
    for (int i = 0; i < numIter; i++) {
        gradients = updateGradients();
        newPose = mSkel->getPositions() - alpha * gradients;
				if(noBadKnees) {
					if(newPose(11) > 0.0) newPose(11) = 0.0;
					if(newPose(12) > 0.0) newPose(12) = 0.0;
				}
        mSkel->setPositions(newPose); 
        mSkel->computeForwardKinematics(true, false, false); // DART updates all the transformations based on newPose
    }
}

/* ******************************************************************************************** */
MatrixXd MyWorld::getArmJacobian (BodyNode* node0, Eigen::Vector4d offset) {

	Eigen::MatrixXd mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
	BodyNode* heel = mSkel->getBodyNode("h_heel_left");
	Joint* j_shin_heel = heel->getParentJoint();
	Matrix4d untilTk2 = heel->getParentBodyNode()->getTransform().matrix();
	Matrix4d T2a = j_shin_heel->getTransformFromParentBodyNode().matrix();
	Matrix4d dRdq4 = j_shin_heel->getTransformDerivative(0); 
	Matrix4d Rq5 = j_shin_heel->getTransform(1).matrix();
	Matrix4d Ta3 =j_shin_heel->getTransformFromChildBodyNode().inverse().matrix();
	Vector4d jCol_q4 = untilTk2 * T2a * dRdq4 * Rq5 * Ta3 * offset;
	int colIndex_q4 = j_shin_heel->getIndexInSkeleton(0);
	mJ.col(colIndex_q4) = jCol_q4.head(3); 

	Eigen::Vector4d bottom = offset;

	// With respect to q5 in shin-heel joint
	Matrix4d Rq4 = j_shin_heel->getTransform(0).matrix();
	Matrix4d dRdq5 = j_shin_heel->getTransformDerivative(1);
	Vector4d jCol_q5 = untilTk2 * T2a * Rq4 * dRdq5 * Ta3 * bottom;
	int colIndex_q5 = j_shin_heel->getIndexInSkeleton(1);
	mJ.col(colIndex_q5) = jCol_q5.head(3);
	bottom = T2a * Rq4 * Rq5 * Ta3 * bottom;

	// With respect to q3 in thigh-heel joint
	BodyNode* shin = mSkel->getBodyNode("h_shin_left");
	Joint* j_thigh_shin = shin->getParentJoint();
	Matrix4d untilTh1 = shin->getParentBodyNode()->getTransform().matrix();
	Matrix4d T1k = j_thigh_shin->getTransformFromParentBodyNode().matrix();
	Matrix4d dRdq3 = j_thigh_shin->getTransformDerivative(0); 
	Matrix4d Tk2 = j_thigh_shin->getTransformFromChildBodyNode().inverse().matrix();
	// Vector4d jCol_q3 = untilTh1 * T1k * dRdq3 * Tk2 * T2a * Rq4 * Rq5 * Ta3 * offset;
	Vector4d jCol_q3 = untilTh1 * T1k * dRdq3 * Tk2 * bottom;
	int colIndex_q3 = j_thigh_shin->getIndexInSkeleton(0);
	mJ.col(colIndex_q3) = jCol_q3.head(3); 
	Matrix4d Rq3 = j_thigh_shin->getTransform(0).matrix();
	bottom = T1k * Rq3 * Tk2 * bottom;

	// With respect to q2 in pelvis-thigh joint
	BodyNode* thigh = mSkel->getBodyNode("h_thigh_left");
	Joint* j_pelvis_thigh = thigh->getParentJoint();
	Matrix4d untilPelvis = thigh->getParentBodyNode()->getTransform().matrix();
	Matrix4d T0h = untilPelvis * j_pelvis_thigh->getTransformFromParentBodyNode().matrix();
	Matrix4d Rq0 = j_pelvis_thigh->getTransform(0).matrix();
	Matrix4d Rq1 = j_pelvis_thigh->getTransform(1).matrix();
	Matrix4d dRdq2 = j_pelvis_thigh->getTransformDerivative(2); 
	Matrix4d Th1 = j_pelvis_thigh->getTransformFromChildBodyNode().inverse().matrix();
	Vector4d jCol_q2 = T0h * Rq0 * Rq1 * dRdq2 * Th1 * bottom;
	int colIndex_q2 = j_pelvis_thigh->getIndexInSkeleton(2);
	mJ.col(colIndex_q2) = jCol_q2.head(3); 
	Matrix4d Rq2 = j_pelvis_thigh->getTransform(2).matrix();

	// With respect to q1 in pelvis-thigh joint
	Matrix4d dRdq1 = j_pelvis_thigh->getTransformDerivative(1); 
	Vector4d jCol_q1 = T0h * Rq0 * dRdq1 * Rq2 * Th1 * bottom;
	int colIndex_q1 = j_pelvis_thigh->getIndexInSkeleton(1);
	mJ.col(colIndex_q1) = jCol_q1.head(3); 

	// With respect to q0 in pelvis-thigh joint
	Matrix4d dRdq0 = j_pelvis_thigh->getTransformDerivative(0); 
	Vector4d jCol_q0 = T0h * dRdq0 * Rq1 * Rq2 * Th1 * bottom;
	int colIndex_q0 = j_pelvis_thigh->getIndexInSkeleton(0);
	mJ.col(colIndex_q0) = jCol_q0.head(3); 

	return mJ;
}

/* ******************************************************************************************** *
MatrixXd MyWorld::getArmJacobian (BodyNode* node0, Eigen::Vector4d offset) {

	bool debug = false;

	// Create the Jacobian
	Eigen::MatrixXd mJ = MatrixXd::Zero(3, mSkel->getNumDofs());
	Vector4d bottom = offset;
	BodyNode* node = node0;
	// printf("\n");
	bool useChildAsParent = false;
	Joint* joint; 
	while(true) {

		if(debug) cout << "\tnode name: " << node->getName().c_str() << endl;

		// Abdomen case: start using the child node as the parent node
		if(node->getName().compare("h_abdomen") == 0) {
			useChildAsParent = true;
			parentNode = node;
			node = mSkel->getBodyNode("h_scapula_right");
		}

		// Stop if the node parent becomes the world
		BodyNode* parentNode = node->getParentBodyNode();
		if(parentNode->getName().compare("h_abdomen") == 0) break;
		//if(parentNode == NULL) break;
		//printf("child: '%s', parent: '%s'\n", node->getName().c_str(), parentNode->getName().c_str());

		// Compute the fixed translations regarding this body node
		Joint* joint = node->getParentJoint();
		Matrix4d Tw0 = Matrix4d::Identity();
		if(parentNode) Tw0 = parentNode->getTransform().matrix();
		Matrix4d T0b = joint->getTransformFromParentBodyNode().matrix();
		Matrix4d Tb1 = joint->getTransformFromChildBodyNode().inverse().matrix();

		// For each degree of freedom of the joint, compute a Jacobian column
		size_t numDofs = joint->getNumDofs();
		for(size_t partial_dof_idx = 0; partial_dof_idx < numDofs; partial_dof_idx++) {

			// Compute the multiplication of the dofs
			Matrix4d Rtotal = Matrix4d::Identity();
			for(size_t dof_idx = 0; dof_idx < numDofs; dof_idx++) {
	
				// Get the partial derivative of the key dof and multiply to total
				if(dof_idx == partial_dof_idx) {
					Matrix4d dRdq = joint->getTransformDerivative(dof_idx);
					Rtotal = Rtotal * dRdq;
				}

				// If not key dof, multiply the transform to total
				else {
					Matrix4d Rq = joint->getTransform(dof_idx).matrix();
					Rtotal = Rtotal * Rq;
				}
			}

			// Compute the column
			Vector4d col;
			if(useChildAsParent) col = Tw0 * (T0b * Rtotal * Tb1).inverse() * bottom;
			else col = Tw0 * T0b * Rtotal * Tb1 * bottom;
			int colIndex = joint->getIndexInSkeleton(partial_dof_idx);
			mJ.col(colIndex) = col.head(3); 
		}

		// Compute the multiplication of the rotations
		Matrix4d Rfull = Matrix4d::Identity();
		for(size_t dof_idx = 0; dof_idx < numDofs; dof_idx++) 
			Rfull = Rfull * joint->getTransform(dof_idx).matrix();

		// Update the bottom and the node
		bottom = T0b * Rfull * Tb1 * bottom;
		if(useChildAsParent) {
			parentNode = node;
			node = node->getChildBodyNode(0);
		}
		else node = parentNode;
	}

	return mJ;
}

/* ******************************************************************************************** */
// Current code only works for the left ankle with only one constraint
VectorXd MyWorld::updateGradients() {

		static int counter = 0;
		bool debug = false && (counter++ % 500 == 0);

		// Compute the component of gradient for each constraint
    int nDof = mSkel->getNumDofs();
    VectorXd gradients = VectorXd::Zero(nDof);
		if(debug) cout << "# constraints: " << constraints.size() << endl;
		for(map <int, Vector3d>::iterator it = constraints.begin(); it != constraints.end(); it++) {

			// Get the constraint info
			Vector3d mTarget = it->second;
			int mConstrainedMarker = it->first;
			map <int, double>::iterator it2 = weights.find(it->first);
			if(it2 == weights.end()) weights[it->first] = 0.5;
			double weight = weights[it->first];
			if(debug) cout << "marker #: " << mConstrainedMarker << ", weight: " << weight << ", xx: " << weights[it->first] << endl;

			// Compute the offset
			Vector4d offset;
			offset << mSkel->getMarker(mConstrainedMarker)->getLocalPosition(), 1; 

			// Get the Jacobian
			BodyNode* node = mSkel->getMarker(mConstrainedMarker)->getBodyNode();
			MatrixXd mJ = getArmJacobian(node, offset);
//			cout << "mJ:\n" << mJ << endl;
//			BodyNode* node2 = mSkel->getBodyNode("h_hand_left");
//			MatrixXd mJ2 = getArmJacobian(node2, Eigen::Vector4d(0, 0, 0, 1));
////			cout << "mJ2:\n" << mJ2 << endl;
////			mJ = mJ * mJ2.inverse();
//			mJ = mJ2 * mJ.inverse();

			// Move the entire robot
			// mJ.col(0) = Eigen::Vector3d(1, 0, 0);
			// mJ.col(2) = Eigen::Vector3d(0, 0, 1);

			// Compute gradients
			Eigen::Vector3d mC = mSkel->getMarker(mConstrainedMarker)->getWorldPosition() - mTarget;
			if(debug)
				cout << "\terror: " << mC.transpose() << ", norm: " << mC.norm() << endl;
			gradients += weight * 2 * mJ.transpose() * mC;
		}

		// Stop negative gradients if the knee is less than epsilon
		Eigen::VectorXd state = mSkel->getState();
		double left_knee = state(11);
		double right_knee = state(12);
		debug = (counter++ % 500 == 0);
		if(debug) cout << left_knee << " " << right_knee << endl;
		
		
    return gradients;
}

// Current code only handlse one constraint on the left foot.
void MyWorld::createConstraint(int _index) {
	Vector3d mTarget = mSkel->getMarker(_index)->getWorldPosition();
	constraints.insert(make_pair(_index, mTarget));
	cout << "# constraints: " << constraints.size() << endl;
}

void MyWorld::modifyConstraint(int _index, Vector3d _deltaP) {
	map <int, Vector3d>::iterator it = constraints.find(_index);
	it->second += _deltaP;
}

void MyWorld::removeConstraint(int _index) {
	map <int, Vector3d>::iterator it = constraints.find(_index);
	if(it != constraints.end()) constraints.erase(it);
	cout << "# constraints: " << constraints.size() << endl;
}

