#include <iostream>

#include "g2o/core/block_solver.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "g2o/types/slam3d/se3_ops.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>


#include "Priors.hpp"

/*  IDEA: the cars can be part of static map where sim(3) can make more sense 
 */


// Function to read the data into vector of Eigen Matrix; Everything will be hardcoded here
void readData(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &orbVO, \
			  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &gtVO,  \
			  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &shapePose, \
			  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &lstmMotion){


	// all the files have 13 columnes - framenumber R(rowmajor) tx ty tz
	char* orbVOFile     = "../../data/orb_full3.txt";
	char* gtVOFile      = "../../data/gt_orbfull3.txt";
	char* shapePoseFile = "../../data/shapePose_full3.txt";


	char* motionFile = "../../data/lstmMotion2_full3.txt";
	
	// open files
	FILE *orbVOfp = fopen(orbVOFile, "r");
	FILE *gtVOfp  = fopen(gtVOFile, "r");
	FILE *shapePosefp = fopen(shapePoseFile, "r");
	FILE *lstmMotionfp = fopen(motionFile, "r");


	if(gtVOfp == NULL)
		std::cerr<<"NULL";
	
	int	numFrames = 38;	
	
	// make sure to use iterators in the main code
	for(int i = 0; i<numFrames; ++i){
	
		double val[13];
		Eigen::Matrix4d m;
		
		// ORB VO
		for(int j=0; j<13; j++){						
			fscanf(orbVOfp, "%lf", &val[j]);
		}
				
		m << val[1], val[2], val[3], val[10], \
		     val[4], val[5], val[6], val[11], \
		     val[7], val[8], val[9], val[12], \
		        0.0,    0.0,    0.0,     1.0; 

		//m << 1, 0, 0, val[10], \
		     0, 1, 0, val[11], \
		     0, 0, 1, val[12], \
		     0, 0, 0, 1.0; 

		orbVO.push_back(m);					
		

		//====================================================================================================
		
		// GT VO
		for(int j=0; j<13; j++){						
			fscanf(gtVOfp, "%lf", &val[j]);			
		}
		
		m << val[1], val[2], val[3], val[10], \
		     val[4], val[5], val[6], val[11], \
		     val[7], val[8], val[9], val[12], \
		        0.0,    0.0,    0.0,     1.0; 

		gtVO.push_back(m);
		

		//====================================================================================================
		// pose shape 
		for(int j=0; j<13; j++){						
			fscanf(shapePosefp, "%lf", &val[j]);			
		}
		
		//m << val[1], val[2], val[3], val[10], \
		     val[4], val[5], val[6], 1.7,     \
		     val[7], val[8], val[9], val[12], \
		        0.0,    0.0,    0.0,     1.0; 

		m << 1, 0, 0, val[10], \
		     0, 1, 0, 1.7, \
		     0, 0, 1, val[12], \
		     0, 0, 0, 1.0; 

		shapePose.push_back(m);		

		//====================================================================================================
		// LSTM motion 
		for(int j=0; j<13; j++){						
			fscanf(lstmMotionfp, "%lf", &val[j]);			
		}
		
		//m << val[1], val[2], val[3], val[10], \
		     val[4], val[5], val[6], 1.7,     \
		     val[7], val[8], val[9], val[12], \
		        0.0,    0.0,    0.0,     1.0; 


		m << 1, 0, 0, val[10], \
		     0, 1, 0, 1.7, \
		     0, 0, 1, val[12], \
		     0, 0, 0, 1.0; 

		lstmMotion.push_back(m);		

	} 

	
	
}

// compute V2V transform for a vehicle in frame i using shapepose and LSTM entries of frame i-1; 
// This transform can be used to add edge from V(i-1) -> V(i); And also add V(i) node 
Eigen::Matrix4d computeV2VMotion(int frameNo, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &shapePose, \
											  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &lstmMotion){


	Eigen::Matrix4d T_Cprev_Vprev;		// ShapePose
	Eigen::Matrix4d T_Cprev_Vcurr;		// lstm

	T_Cprev_Vprev = shapePose[frameNo-1];
	T_Cprev_Vcurr = lstmMotion[frameNo-1];

	// returning: T_Vprev_Vcurr
	
	std::cerr<<"v2v["<<frameNo<<"]:"<<T_Cprev_Vprev.inverse() * T_Cprev_Vcurr<<std::endl;

	return T_Cprev_Vprev.inverse() * T_Cprev_Vcurr;

}


// make sure to use consts ...
void addSimNode(int id, g2o::Sim3 pose, bool fixScale, bool fixNode, g2o::SparseOptimizer &optimizer){

	g2o::VertexSim3Expmap *N = new g2o::VertexSim3Expmap();
	
	N->setEstimate(pose);
	N->setFixed(fixNode);
	N->_fix_scale = fixScale;
	N->setId(id);	
	
	optimizer.addVertex(N);
	
}


// make sure to use consts ...
void addSimNode(int id, Eigen::Matrix4d pose, bool fixScale, bool fixNode, float scale, g2o::SparseOptimizer &optimizer){

	g2o::VertexSim3Expmap *N = new g2o::VertexSim3Expmap();
	
	Eigen::Matrix3d R;
	Eigen::Vector3d t;					
				
	R<< pose(0,0), pose(0,1), pose(0,2), \
	    pose(1,0), pose(1,1), pose(1,2), \
	    pose(2,0), pose(2,1), pose(2,2);		    			    
	t<< pose(0,3), pose(1,3), pose(2,3);

	g2o::Sim3 simPose(R, t, scale);	
	
	N->setEstimate(simPose);
	N->setFixed(fixNode);
	N->_fix_scale = fixScale;
	N->setId(id);	
	
	optimizer.addVertex(N);
	
}

// add edge
void addSimEdge(int idFrom, int idTo, Eigen::Matrix4d pose, Eigen::Matrix<double, 7, 7> infoMat, g2o::SparseOptimizer &optimizer){
	
	Eigen::Matrix3d R;
	Eigen::Vector3d t;		

	g2o::EdgeSim3 *E = new g2o::EdgeSim3();

	E->vertices()[0] = optimizer.vertex(idFrom);
	E->vertices()[1] = optimizer.vertex(idTo);			
				
	R<< pose(0,0), pose(0,1), pose(0,2), \
	    pose(1,0), pose(1,1), pose(1,2), \
	    pose(2,0), pose(2,1), pose(2,2);		    			    
	t<< pose(0,3), pose(1,3), pose(2,3);
	
	g2o::Sim3 constraint(R, t, 1.0);
	E->setMeasurement(constraint);
	E->setInformation(infoMat);	//Weight * g2o::EdgeSim3::InformationType::Identity()
	
	optimizer.addEdge(E);
}


void addNonHolonomicPlanarMotionPrior(int idFrom, int idTo, float infoMat, g2o::SparseOptimizer &optimizer){

	NonHolonomicPlanarMotionPrior *E = new NonHolonomicPlanarMotionPrior();

	E->vertices()[0] = optimizer.vertex(idFrom);
	E->vertices()[1] = optimizer.vertex(idTo);

	Eigen::Matrix4d T;
	T.setIdentity();
	T(2,3) = 10;

	g2o::Sim3 dummy(T.block(0,0,3,3), T.block(0,3,3,1), 1.0);
	E->setMeasurement(dummy);
	E->setInformation(infoMat * g2o::EdgeSim3::InformationType::Identity());

	optimizer.addEdge(E);

}


void addRotationAboutYPrior(int idTo, Eigen::Matrix4d T, float infoMat, float h, g2o::SparseOptimizer &optimizer){


	PlanarRotationPrior *E = new PlanarRotationPrior(h);

	E->vertices()[0] = optimizer.vertex(idTo);

	g2o::Sim3 constraint(T.block(0,0,3,3), T.block(0,3,3,1), 1.0);

	E->setMeasurement(constraint);

	E->setInformation(infoMat * g2o::EdgeSim3::InformationType::Identity());

	optimizer.addEdge(E);
}



// add edge - uses nodes transform to compute edge automatically. Does not require to pass transform
void addSimEdgeAuto(int idFrom, int idTo, Eigen::Matrix<double, 7, 7> infoMat, g2o::SparseOptimizer &optimizer){
	
	g2o::EdgeSim3 *E = new g2o::EdgeSim3();
	
	E->vertices()[0] = optimizer.vertex(idFrom);
	E->vertices()[1] = optimizer.vertex(idTo);
	
	E->setMeasurement(dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(idFrom))->estimate().inverse() \
							* dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(idTo))->estimate());
		
	E->setInformation(infoMat);			
	optimizer.addEdge(E);
}



void printFullOptimizedDataToFile(g2o::SparseOptimizer &optimizer){

	Eigen::Matrix<double, 76, 13> nodeTrans;
	nodeTrans.setZero();
	Eigen::Vector3d t;
	Eigen::Matrix3d R;

	//std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > nodeTrans;
	
	for(int i=0; i<38; ++i){	
		
		t = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().translation(); 
		R = (dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().rotation()).toRotationMatrix();
		
		nodeTrans(i,0) = R(0,0);
		nodeTrans(i,1) = R(0,1);
		nodeTrans(i,2) = R(0,2);
		nodeTrans(i,3) = R(1,0);
		nodeTrans(i,4) = R(1,1);
		nodeTrans(i,5) = R(1,2);
		nodeTrans(i,6) = R(2,0);
		nodeTrans(i,7) = R(2,1);
		nodeTrans(i,8) = R(2,2);

		nodeTrans(i,9)  = t[0];
		nodeTrans(i,10) = t[1];
		nodeTrans(i,11) = t[2];

		nodeTrans(i,12) = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().scale();
		

	}

	
	for(int i=0; i<38; ++i){	
		
		t = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+100))->estimate().translation(); 
		R = (dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().rotation()).toRotationMatrix();
		
		nodeTrans(i+38,0) = R(0,0);
		nodeTrans(i+38,1) = R(0,1);
		nodeTrans(i+38,2) = R(0,2);
		nodeTrans(i+38,3) = R(1,0);
		nodeTrans(i+38,4) = R(1,1);
		nodeTrans(i+38,5) = R(1,2);
		nodeTrans(i+38,6) = R(2,0);
		nodeTrans(i+38,7) = R(2,1);
		nodeTrans(i+38,8) = R(2,2);

		nodeTrans(i+38,9)  = t[0];
		nodeTrans(i+38,10) = t[1];
		nodeTrans(i+38,11) = t[2];
		nodeTrans(i+38,12) = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+100))->estimate().scale() ;
	}

	std::ofstream fp("optiTranslations.txt");
	fp << nodeTrans << std::endl;
	
	std::cerr<<"saved optimized translations of every node ... "<<std::endl;
}


void printOptimizedDataToFile(g2o::SparseOptimizer &optimizer){

	Eigen::Matrix<double, 76, 4> nodeTrans;
	nodeTrans.setZero();
	Eigen::Vector3d t;
	
	//std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > nodeTrans;
	
	for(int i=0; i<38; ++i){	
		
		t = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().translation(); 
		
		nodeTrans(i,0) = t[0];
		nodeTrans(i,1) = t[1];
		nodeTrans(i,2) = t[2];
		nodeTrans(i,3) = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().scale();
	}
	
	for(int i=0; i<38; ++i){	
		
		t = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+100))->estimate().translation(); 
		
		nodeTrans(i+38,0) = t[0];
		nodeTrans(i+38,1) = t[1];
		nodeTrans(i+38,2) = t[2];
		nodeTrans(i+38,3) = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+100))->estimate().scale() ;
	}

	std::ofstream fp("optiTranslations.txt");
	fp << nodeTrans << std::endl;
	
	std::cerr<<"saved optimized translations of every node ... "<<std::endl;
}



//void addChainLink();

int main(int argc, char **argv){




	Eigen::Matrix<double, 7, 7> CAM2CAM_INFO;
	CAM2CAM_INFO.setIdentity();

	//CAM2CAM_INFO = CAM2CAM_INFO * 100000;
	CAM2CAM_INFO(0,0) = CAM2CAM_INFO(2,2) = CAM2CAM_INFO(1,1) = 100000;
	CAM2CAM_INFO(3,3) = CAM2CAM_INFO(4,4) = CAM2CAM_INFO(5,5) = 100000;
	CAM2CAM_INFO(6,6) = 1;

	Eigen::Matrix<double, 7, 7> VEH2VEH_INFO;
	VEH2VEH_INFO.setIdentity();
	VEH2VEH_INFO = 100* VEH2VEH_INFO;
	/*VEH2VEH_INFO(0,0) = VEH2VEH_INFO(1,1) = VEH2VEH_INFO(2,2) = 10;
	VEH2VEH_INFO(3,3) = VEH2VEH_INFO(4,4) = VEH2VEH_INFO(5,5) = 1000000;
	VEH2VEH_INFO(6,6) = 1000000;*/

	Eigen::Matrix<double, 7, 7> CAM2VEH_INFO;
	CAM2VEH_INFO.setIdentity();
	CAM2VEH_INFO = CAM2VEH_INFO * 1;

	

	// Eigen::Matrix<double, 7, 7> CAM2VEH_INFO;
	// CAM2VEH_INFO.setIdentity();
	// CAM2VEH_INFO = CAM2VEH_INFO * 100000000000000;

	//    CAM2VEH_INFO(0,0) = CAM2VEH_INFO(1,1) = CAM2VEH_INFO(2,2) = 1000;
	// CAM2VEH_INFO(3,3) = CAM2VEH_INFO(4,4) = CAM2VEH_INFO(5,5) = 1000;
	// CAM2VEH_INFO(6,6) = 1000;

	int numOfFrames = 38;
	
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > orbVO;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > gtVO;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > shapePose;	
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > lstmMotion;	
	
	// read the files in to the respective vectors
	readData(orbVO, gtVO, shapePose, lstmMotion);
	
	
	// validate data read by printing it.
	/*for(int i = 0; i<numOfFrames; ++i){		
		Eigen::Matrix4d temp(orbVO[i]); 
		std::cerr<<i+13<<","<<temp(0,0)<<","<<temp(0,1)<<","<<temp(0,2)<<","<<temp(1,0)<<","<<temp(1,1)<<","<<temp(1,2)<<","<<temp(2,0)<<","<<temp(2,1)<<","<<temp(2,2)<<","<<temp(0,3)<<","<<temp(1,3)<<","<<temp(2,3)<<std::endl;		
	}*/

	// initializing optimizer
	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(true);
	
	g2o::BlockSolverX::LinearSolverType * linearSolver;
	
	
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	
	g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);	


	Eigen::Matrix<int, 76, 76> adjMat;
	adjMat.setZero();
	
	// using 100 as offset for vehicle node ids
	for(int i=0; i<numOfFrames; ++i){
		
		//If first frame connect only ego Car and Vehicle
		if(i == 0){												

				// add C0 : T_W_C0
				addSimNode(i, orbVO[i], true, true, 1, optimizer);

				// add V0 : Est: T_W_C0 * T_C0_V0 {C(i)*P(i)}
				addSimNode(i+100, orbVO[i]*shapePose[i], true, true, 1, optimizer);

				// add edge C0->V0 : Meas: T_C0_V0 {P(i)}
				addSimEdge(i, i+100, shapePose[i], CAM2VEH_INFO, optimizer);

				// ADD PRIOR
				//addRotationAboutYPrior(i, orbVO[i], 100, 0.4, optimizer);
				//addRotationAboutYPrior(i+100, shapePose[i], 10000, 1.7, optimizer);
				//addNonHolonomicPlanarMotionPrior(i, i+1, 1000000000, optimizer);


				// ===================MAKE ADJACENCY MATRIX FOR DEBUGGING GRAPH STRUCTURE ======================	

				// Make adjacency matrix to test structure. 
				// Later have a function which iterates over graph to generate the adj. matrix
				//adjMat(i, i+1) = 1.0;		//C1->C2
				//adjMat(i, i+numOfFrames) = 1.0;		//C1->V1
				//adjMat(i+1, i+numOfFrames+1) = 1.0;	//C2->V2
				//adjMat(i+numOfFrames, i+numOfFrames+1) = 1.0;  //V1->V2 
				
				// ==============================================================================================								
		}

		else{


				// add C1  : T_W_C1
				addSimNode(i, orbVO[i], false, false, 1, optimizer);

				// add edge C1->C2 : T_C1_C2 = inv(T_W_C1) * T_W_C2 = inv(C(0))*C(1);
				addSimEdge(i, i+1, orbVO[i-1].inverse()*orbVO[i], CAM2CAM_INFO, optimizer);


				
				// get V0->V1

					Eigen::Matrix4d T_Vprev_Vcurr;
					T_Vprev_Vcurr = computeV2VMotion(i, shapePose,lstmMotion);

					// get V0 from in world. And convert to Matrix4d from Sim3
					Eigen::Matrix4d T_W_Vprev_Mat;

					g2o::Sim3 T_W_Vprev;
					T_W_Vprev = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+100-1))->estimate();											
					Eigen::Matrix3d R_W_Vprev;
					Eigen::Vector3d t_W_Vprev;					
					R_W_Vprev = T_W_Vprev.rotation().toRotationMatrix();
					t_W_Vprev = T_W_Vprev.translation();
					T_W_Vprev_Mat.setIdentity();
					T_W_Vprev_Mat.block(0,0,3,3) = R_W_Vprev;
					T_W_Vprev_Mat.block(0,3,3,1) = t_W_Vprev;	


				// add V1 using LSTM
				addSimNode(i+100, T_W_Vprev_Mat * T_Vprev_Vcurr, true, false, 1, optimizer);

				//std::cerr<<"v["<<i<<"]:"<<T_W_Vprev_Mat * T_Vprev_Vcurr<<std::endl;				

				// add V1 - using ORB
				//addSimNode(i+100, orbVO[i]*shapePose[i], true, false, 1, optimizer);

				// add edge V0->V1
				addSimEdge(i+100-1, i+100, T_Vprev_Vcurr, VEH2VEH_INFO, optimizer);

				// add edge C1->V1
				addSimEdge(i, i+100, shapePose[i], CAM2VEH_INFO, optimizer);

				// add edge C0->V1
				addSimEdge(i-1, i+100, lstmMotion[i-1], CAM2VEH_INFO, optimizer);

								
				// ADD PRIOR
				
				//addNonHolonomicPlanarMotionPrior(i-1, i, 100000, optimizer);
				//addRotationAboutYPrior(i, orbVO[i], 100, 0.4, optimizer);	
				//addRotationAboutYPrior(i+100, shapePose[i], 10000, 1.7, optimizer);	

				// ===================MAKE ADJACENCY MATRIX FOR DEBUGGING GRAPH STRUCTURE ======================	

				// Make adjacency matrix to test structure. 
				// Later have a function which iterates over graph to generate the adj. matrix
				//adjMat(i, i+1) = 1.0;
				//adjMat(i+1, i+numOfFrames+1) = 1.0;
				//adjMat(i+numOfFrames, i+numOfFrames+1) = 1.0;  //V1->V2 	

				//adjMat(i+numOfFrames, i+numOfFrames+1) = 1.0;  //C1->V2 	

				// ==============================================================================================																
		}
		
	}
	
	//std::cerr<<"\n saved before opti. :"<<optimizer.save("testSave_old.g2o")<<std::endl;	
	//std::ofstream file("adjMatTest.txt");	
	//file<<(adjMat)<<"\n";


	

	
	optimizer.initializeOptimization();
	optimizer.optimize(20);
	
	
	//std::cerr<<"\n saved after opti.:"<<optimizer.save("testSave_opti.g2o")<<std::endl;

	std::cerr<<"saved optimized translations of every node ... "<<std::endl;
	printOptimizedDataToFile(optimizer);
	

	
	
	
	
	
	
	std::cout<<"test g2o...";
	
	return 0;
}

