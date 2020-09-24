#ifndef PRIORS_HPP_
#define PRIORS_HPP_

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"

class PlanarRotationPrior: public g2o::BaseUnaryEdge<7, g2o::Sim3, g2o::VertexSim3Expmap >{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PlanarRotationPrior(float height):height_(height) {}

	// overriding comptue error: this cost function will be evaluated when
	// considering this PlanarRotationPrior edge with a node
	void computeError(){

		const g2o::VertexSim3Expmap* v = static_cast < const g2o::VertexSim3Expmap*>(_vertices[0]);

		g2o::Sim3 est(v->estimate());
		g2o::Sim3 prior(_measurement);


		g2o::Sim3 err;

		// compute error.  Rotation of the orb nodes during the optimization should only
		//                 comprize a rotation about the Y-axis
		//err[0] = est[0] - prior[0];		

		err[4] = est[4] - 0.3;
		err[5] = est[5] - height_;
		//err[6] = est[6] - prior[6];
		//std::cerr<<"PRIOR:"<<err[1]<<std::endl;
		
		//err = est.inverse() * prior;

		//err[3] = err[4] = err[5] = 0;
		// convert to EXPMAP
		_error = err.log();	

	}

	virtual bool read(std::istream&){ return false; }

	virtual bool write(std::ostream& os) const { return false; }
private:

	float height_;
};




class NonHolonomicPlanarMotionPrior: public g2o::BaseBinaryEdge<7, g2o::Sim3, g2o::VertexSim3Expmap, g2o::VertexSim3Expmap >{

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	NonHolonomicPlanarMotionPrior() {


		std::cerr<<"PriorConstructore"<<std::endl;
	}

	// overriding comptue error: this cost function will be evaluated when
	// considering this PlanarRotationPrior edge with a node
	void computeError(){

		//std::cerr<<"Error:"<<std::endl;
		const g2o::VertexSim3Expmap* v0 = static_cast < const g2o::VertexSim3Expmap*>(_vertices[0]);
		const g2o::VertexSim3Expmap* v1 = static_cast < const g2o::VertexSim3Expmap*>(_vertices[1]);

		g2o::Sim3 est0(v0->estimate());
		g2o::Sim3 est1(v1->estimate());

		// compute translation vector.  X,Y,Z: 4,5,6
		Eigen::Vector3d t_0_1;
		t_0_1[0] = est1[4] - est0[4];
		t_0_1[1] = est1[5] - est0[5];
		t_0_1[2] = est1[6] - est0[6];

		// normalize translation i.e. make t_0_1 into a unit vector
		t_0_1 = t_0_1 / t_0_1.norm();

		// first: the translation vector should be as close to [0 0 1] as possible.		

		g2o::Sim3 err;

		err[4] = t_0_1[0] - 0;
		err[5] = t_0_1[1] - 0;
		err[6] = t_0_1[2] - 1;

		//std::cerr<<"Error:"<<err<<std::endl;
		// rotation between the two nodes should only change in y direction not in Z and X and 
		// also should not change much.

		// err[0] = est1[0] - est0[0];
		// err[1] = est1[1] - est0[1];
		// err[2] = est1[2] - est0[2];

		// EXPERIMENTAL : scale should be same
		//err[3] = est1[3] - est0[3];

		// convert to EXPMAP
		_error = err.log();	

	}

	virtual bool read(std::istream&){ return false; }

	virtual bool write(std::ostream& os) const { return false; }

};




#endif

