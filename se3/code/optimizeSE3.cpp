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


void processData_(char* fname, int numFrames, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &data){

	FILE *dataFP = fopen(fname, "r");

	if(dataFP == NULL)
		std::cerr<<"FILE INVALID"<<std::endl;

	for(int i = 0; i<numFrames; ++i){
	
		double val[12];
		Eigen::Matrix4d m;
		
		// 12 cols - 9 for ROT and 3 for TRANS.
		for(int j=0; j<12; j++){						
			fscanf(dataFP, "%lf", &val[j]);
		}
				
		m << val[0], val[3], val[6], val[9],  \
		     val[1], val[4], val[7], val[10], \
		     val[2], val[5], val[8], val[11], \
		        0.0,    0.0,    0.0,     1.0; 

		//std::cerr<<m<<std::endl;        

		data.push_back(m);		
	}

}


void processData(char* fname, int numFrames, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &data){

	FILE *dataFP = fopen(fname, "r");

	if(dataFP == NULL)
		std::cerr<<"FILE INVALID"<<std::endl;

	for(int i = 0; i<numFrames; ++i){
	
		double val[12];
		Eigen::Matrix4d m;
		
		// 12 cols - 9 for ROT and 3 for TRANS.
		for(int j=0; j<12; j++){						
			fscanf(dataFP, "%lf", &val[j]);
		}
				
		m << val[0], val[1], val[2], val[9],  \
		     val[3], val[4], val[5], val[10], \
		     val[6], val[7], val[8], val[11], \
		        0.0,    0.0,    0.0,     1.0; 

		//std::cerr<<m<<std::endl;        

		data.push_back(m);		
	}

}

// make vehicle to vehicle motion transform using pose-shape predictions and orbscaled.
// The output of this function is VERIFIED - OK
void makeV2VMotion(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& shapePoseHat, \
	               const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& orbScaled, \
	               std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& vehicleMotion){


	for(int i = 0; i<orbScaled.size()-1; ++i){


		Eigen::Matrix4d v2v;

		v2v = shapePoseHat[i].inverse() * orbScaled[i].inverse() * orbScaled[i+1] * shapePoseHat[i+1];

		std::cerr<<v2v(0,0)<<","<<v2v(0,1)<<","<<v2v(0,2)<<"," \
		         <<v2v(1,0)<<","<<v2v(1,1)<<","<<v2v(1,2)<<"," \
		         <<v2v(2,0)<<","<<v2v(2,1)<<","<<v2v(2,2)<<","\		         
		         <<v2v(0,3)<<","<<v2v(1,3)<<","<<v2v(2,3) \
		         << std::endl;


		vehicleMotion.push_back(v2v);
	}
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

	//std::cerr<<R<<"\n"<<t<<"\n";

	g2o::Sim3 simPose(R, t, scale);	
	
	N->setEstimate(simPose);
	N->setFixed(fixNode);
	N->_fix_scale = fixScale;
	N->setId(id);	
	N->setMarginalized(false); 
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


void printFullOptimizedDataToFile(char* fname, const int numOfFrames, g2o::SparseOptimizer &optimizer){
	
	std::vector<std::vector<double> > optimizedData;
	
	Eigen::Vector3d t_;
	Eigen::Matrix3d R_;

	//std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > nodeTrans;
	
	for(int i=0; i<numOfFrames; ++i){	
		
		t_ = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().translation(); 
		R_ = (dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().rotation()).toRotationMatrix();

		Eigen::Vector3d t;
		Eigen::Matrix3d R;
		
		R = R_.inverse();
		t = -R_.inverse()*t_;

		std::vector<double> nodeTrans;

		nodeTrans.push_back(R(0,0));
		nodeTrans.push_back(R(0,1));
		nodeTrans.push_back(R(0,2));
		nodeTrans.push_back(R(1,0));
		nodeTrans.push_back(R(1,1));
		nodeTrans.push_back(R(1,2));
		nodeTrans.push_back(R(2,0));
		nodeTrans.push_back(R(2,1));
		nodeTrans.push_back(R(2,2));

		nodeTrans.push_back(t[0]);
		nodeTrans.push_back(t[1]);
		nodeTrans.push_back(t[2]);

		nodeTrans.push_back(dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().scale());
		
		optimizedData.push_back(nodeTrans);



	}

	
	for(int i=0; i<numOfFrames; ++i){	
		
		t_ = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+numOfFrames+10))->estimate().translation(); 
		R_ = (dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+numOfFrames+10))->estimate().rotation()).toRotationMatrix();

		Eigen::Vector3d t;
		Eigen::Matrix3d R;
		
		R = R_.inverse();
		t = -R_.inverse()*t_;

		std::vector<double> nodeTrans;

		nodeTrans.push_back(R(0,0));
		nodeTrans.push_back(R(0,1));
		nodeTrans.push_back(R(0,2));
		nodeTrans.push_back(R(1,0));
		nodeTrans.push_back(R(1,1));
		nodeTrans.push_back(R(1,2));
		nodeTrans.push_back(R(2,0));
		nodeTrans.push_back(R(2,1));
		nodeTrans.push_back(R(2,2));

		nodeTrans.push_back(t[0]);
		nodeTrans.push_back(t[1]);
		nodeTrans.push_back(t[2]);

		nodeTrans.push_back(dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+numOfFrames+10))->estimate().scale());
		
		optimizedData.push_back(nodeTrans);

	}	


	std::ofstream fp(fname);
	
	for(int i = 0; i<optimizedData.size(); ++i){

		for(int j = 0; j<13; ++j){

			//std::cerr<<(optimizedData[i])[j]<<" ";
			fp << (optimizedData[i])[j] <<" ";		
		}
		fp << "\n";
	}

	fp.close();

	// fp << nodeTrans << std::endl;
	
	std::cerr<<"saved optimized translations of every node ... "<<std::endl;
}


// void printOptimizedDataToFile(g2o::SparseOptimizer &optimizer){

// 	Eigen::Matrix<double, 76, 4> nodeTrans;
// 	nodeTrans.setZero();
// 	Eigen::Vector3d t;
	
// 	//std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > nodeTrans;
	
// 	for(int i=0; i<38; ++i){	
		
// 		t = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().translation(); 
		
// 		nodeTrans(i,0) = t[0];
// 		nodeTrans(i,1) = t[1];
// 		nodeTrans(i,2) = t[2];
// 		nodeTrans(i,3) = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i))->estimate().scale();
// 	}
	
// 	for(int i=0; i<38; ++i){	
		
// 		t = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+100))->estimate().translation(); 
		
// 		nodeTrans(i+38,0) = t[0];
// 		nodeTrans(i+38,1) = t[1];
// 		nodeTrans(i+38,2) = t[2];
// 		nodeTrans(i+38,3) = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(i+100))->estimate().scale() ;
// 	}

// 	std::ofstream fp("optiTranslations.txt");
// 	fp << nodeTrans << std::endl;
	
// 	std::cerr<<"saved optimized translations of every node ... "<<std::endl;
// }


int main(int argc, char **argv){

	const int numOfFrames = 154	;
	
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> orbVOScaled;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> shapePoseHat;	
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> shapePose;
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> vehicleMotion;	
	std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> orbVO;	

	// information matrices
	Eigen::Matrix<double, 7, 7> CAM2CAM_INFO;
	CAM2CAM_INFO.setIdentity();
	Eigen::Matrix<double, 7, 7> CAM2VEH_INFO;
	CAM2VEH_INFO.setIdentity();
	Eigen::Matrix<double, 7, 7> VEH2VEH_INFO;
	VEH2VEH_INFO.setIdentity();


	// process data

	
	processData_("../data/gt_0.txt",    numOfFrames, orbVO);   // orbVOScaled
	//processData_("../../data/gt_0_scaled.txt",    numOfFrames, orbVO);   // orbVOScaled
	processData("../data/shapePose_0.txt", numOfFrames, shapePoseHat);  // shapePoseHat
	processData("../data/shapePose_0.txt",    numOfFrames, shapePose);     // shapePose

	std::cerr<<"processed"<<std::endl;

	// make vehicle motion from orbscaled and shape pose predictions
	makeV2VMotion(shapePoseHat, orbVO, vehicleMotion);


	// initializing optimizer
	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(true);
	
	g2o::BlockSolverX::LinearSolverType * linearSolver;
	
	
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	
	g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);	

	Eigen::Matrix4d lc;
	lc.setIdentity();
	// Eigen::Matrix<int, 150, 150> adjMat;
	// adjMat.setZero();
	
	// using 100 as offset for vehicle node ids
	for(int i=0; i<numOfFrames; ++i){
		
		//If first frame connect only ego Car and Vehicle
		if(i == 0){												

				//std::cerr<<orbVOScaled[i]<<std::endl;

				// add C0 
				addSimNode(i,   orbVO[i].inverse(),   true, true, 1.0, optimizer);
				
				// add V0 : Est: T_W_C0 * T_C0_V0 {C(i)*P(i)}
				addSimNode(i+numOfFrames+10, shapePose[i].inverse()*orbVO[i].inverse(), true, true, 1, optimizer);

				// add edge C0->V0 : Meas: T_C0_V0 {P(i)}
				addSimEdge(i, i+numOfFrames+10, shapePose[i].inverse(), CAM2VEH_INFO, optimizer);

				// ===================MAKE ADJACENCY MATRIX FOR DEBUGGING GRAPH STRUCTURE ======================	

				//Make adjacency matrix to test structure. 
				//Later have a function which iterates over graph to generate the adj. matrix

				// adjMat(i, i+75) = 1;		//C0->V0
				
				// ==============================================================================================								
		}

		else{


				//std::cerr<<orbVOScaled[i]<<std::endl;

				// add C1
				addSimNode(i, orbVO[i].inverse(), true, false, 1.0, optimizer);
				
				// add V1 :
				addSimNode(i+numOfFrames+10, shapePose[i].inverse()*orbVO[i].inverse(), true, false, 1, optimizer);

				// add edge C1->V1 
				addSimEdge(i, i+numOfFrames+10, shapePose[i].inverse(), CAM2VEH_INFO, optimizer);

				// C0-Ci
				addSimEdge(i-1, i, orbVO[i].inverse()*orbVO[i-1], CAM2CAM_INFO, optimizer);
			

				// add edge: V0->V1
				addSimEdge((i-1)+numOfFrames+10, i+numOfFrames+10, vehicleMotion[i-1].inverse(), VEH2VEH_INFO	, optimizer);

				// ===================MAKE ADJACENCY MATRIX FOR DEBUGGING GRAPH STRUCTURE ======================	

				// Make adjacency matrix to test structure. 
				// // Later have a function which iterates over graph to generate the adj. matrix
				// adjMat(i-1, i) = 1.0;  // C0->C1		
				// adjMat(i, i+75) = 1.0;  //C1->V1 	
				// adjMat((i-1)+75, i+75) = 1.0;  //V0->V1

				// ==============================================================================================																
		}
		
	}
	

	

	// add edge: C0->C154
	// addSimEdge(0, 153, orbVO[153], CAM2CAM_INFO*10000000, optimizer);

	// addSimEdge(0, 50, orbVO[50], CAM2CAM_INFO*10000000, optimizer);

	// addSimEdge(0, 100, orbVO[100], CAM2CAM_INFO*10000000, optimizer);

	// std::cerr<<"\n saved before opti. :"<<optimizer.save("testSave_old.g2o")<<std::endl;	
	// std::ofstream file("adjMatTest.txt");	
	// file<<(adjMat)<<"\n";


	

	printFullOptimizedDataToFile("beforeOpti.txt", numOfFrames, optimizer);
	optimizer.initializeOptimization();
	optimizer.optimize(100);
	printFullOptimizedDataToFile("afterOpti.txt", numOfFrames, optimizer);
	
	//std::cerr<<"\n saved after opti.:"<<optimizer.save("testSave_opti.g2o")<<std::endl;

	std::cerr<<"saved optimized translations of every node ... "<<std::endl;
	
	
	
	std::cout<<"test g2o..." << std::endl ;
	
	return 0;
}

