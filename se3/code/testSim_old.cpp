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


#include "PlanarRotationPrior.hpp"

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


	char* motionFile = "../../data/lstmMotion_full3.txt";
	
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
		        0.0,    0.0,    0.0,     1.0; 


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

		m << 1, 0, 0, val[10], \
		     0, 1, 0, val[11], \
		     0, 0, 1, val[12], \
		        0.0,    0.0,    0.0,     1.0; 


		gtVO.push_back(m);
		

		//====================================================================================================
		// pose shape 
		for(int j=0; j<13; j++){						
			fscanf(shapePosefp, "%lf", &val[j]);			
		}
		

		//m << 1, 0, 0, val[10], \
		     0, 1, 0, val[11], \
		     0, 0, 1, val[12], \
		        0.0,    0.0,    0.0,     1.0; 

		m << val[1], val[2], val[3], val[10], \
		     val[4], val[5], val[6], val[11], \
		     val[7], val[8], val[9], val[12], \
		        0.0,    0.0,    0.0,     1.0; 

		shapePose.push_back(m);		

		//std::cerr<<"readData:\n"<<m<<std::endl;

		//====================================================================================================
		// pose shape 
		for(int j=0; j<13; j++){						
			fscanf(lstmMotionfp, "%lf", &val[j]);			
		}
		

		m << 1, 0, 0, val[10], \
		     0, 1, 0, val[11], \
		     0, 0, 1, val[12], \
		        0.0,    0.0,    0.0,     1.0; 

		//m << val[1], val[2], val[3], val[10], \
		     val[4], val[5], val[6], val[11], \
		     val[7], val[8], val[9], val[12], \
		        0.0,    0.0,    0.0,     1.0; 

		lstmMotion.push_back(m);		

		//std::cerr<<"readData:\n"<<m<<std::endl;		

	} 

	
	
}

// make sure to use consts ...
void addSimNode(int id, Eigen::Matrix4d pose, bool fixScale, bool fixNode,  float scale, g2o::SparseOptimizer &optimizer){

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



void addRotationAboutYPrior(int idTo, Eigen::Matrix4d T, float infoMat, g2o::SparseOptimizer &optimizer){


	PlanarRotationPrior *E = new PlanarRotationPrior();

	E->vertices()[0] = optimizer.vertex(idTo);

	g2o::Sim3 constraint(T.block(0,0,3,3), T.block(0,3,3,1), 1.0);

	E->setMeasurement(constraint);

	E->setInformation(infoMat * g2o::EdgeSim3::InformationType::Identity());

	optimizer.addEdge(E);
}


// add edge - have one function with proper infoMat type and constraint type.
void addSimEdgeNew(int idFrom, int idTo, Eigen::Matrix4d T, Eigen::Matrix<double, 7, 7> infoMat, g2o::SparseOptimizer &optimizer){
	
	g2o::EdgeSim3 *E = new g2o::EdgeSim3();
	
	E->vertices()[0] = optimizer.vertex(idFrom);
	E->vertices()[1] = optimizer.vertex(idTo);
	

	g2o::Sim3 constraint(T.block(0,0,3,3), T.block(0,3,3,1), 1.0);
	
	E->setMeasurement(constraint);
		
	E->setInformation(infoMat);	//Weight * g2o::EdgeSim3::InformationType::Identity()
	

	
	optimizer.addEdge(E);
}



// add edge - have one function with proper infoMat type and constraint type.
void addSimEdge(int idFrom, int idTo, Eigen::Matrix<double, 7, 7> infoMat, g2o::SparseOptimizer &optimizer){
	
	g2o::EdgeSim3 *E = new g2o::EdgeSim3();
	
	E->vertices()[0] = optimizer.vertex(idFrom);
	E->vertices()[1] = optimizer.vertex(idTo);
	
	E->setMeasurement(dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(idFrom))->estimate().inverse() \
							* dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(idTo))->estimate());
		
	E->setInformation(infoMat);	
	

	
	optimizer.addEdge(E);
}



void addMotionModelSimEdge(int vIdFrom, int vIdTo, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &gtVO, \
												   std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &shapePose, \
												   Eigen::Matrix<double, 7, 7> infoMat, g2o::SparseOptimizer &optimizer){
	
	Eigen::Matrix4d motion;											   
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
		
			
	g2o::EdgeSim3 *E = new g2o::EdgeSim3();				
	
	
	motion = shapePose[vIdFrom].inverse() *  gtVO[vIdFrom].inverse() * gtVO[vIdTo] * shapePose[vIdTo];
	
//	motion = shapePose[vIdFrom].inverse() *  gtVO[vIdFrom].inverse() * gtVO[vIdTo] * shapePose[vIdTo];
	
	
//	motion = (shapePose[vIdFrom].inverse() * gtVO[vIdFrom-100]) *  			\
					(gtVO[vIdFrom-100].inverse() * gtVO[vIdTo-100]) *  		\
							(gtVO[vIdTo-100].inverse() * shapePose[vIdTo]) ;
	
	
	E->vertices()[0] = optimizer.vertex(vIdFrom+100);
	E->vertices()[1] = optimizer.vertex(vIdTo+100);

	R << motion(0,0), motion(0,1), motion(0,2), \
		 motion(1,0), motion(1,1), motion(1,2), \
		 motion(2,0), motion(2,1), motion(2,2);		 
	t << motion(0,3), motion(1,3), motion(2,3);		  
	

	g2o::Sim3 motionConstraint(R, t, 1.0);				
	
	E->setMeasurement(motionConstraint);
	E->setInformation(infoMat);	

	optimizer.addEdge(E);

}



void addMotionModelSimEdgeNewData(int vIdFrom, int vIdTo,\
								  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &lstmMotion, \
								  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &shapePose, \
								  Eigen::Matrix<double, 7, 7> infoMat, g2o::SparseOptimizer &optimizer){
	
	Eigen::Matrix4d invPS;											   
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
		

	invPS = shapePose[vIdFrom].inverse();
		
	R.setIdentity();	
			
	g2o::EdgeSim3 *E = new g2o::EdgeSim3();				
	

	t = invPS.block(0,3,3,1)+ lstmMotion[vIdFrom].block(0,3,3,1) ;	
		
	
	E->vertices()[0] = optimizer.vertex(vIdFrom+100);
	E->vertices()[1] = optimizer.vertex(vIdTo+100);
	
	std::cerr<<"T:"<<t<<std::endl;

	g2o::Sim3 motionConstraint(R, t, 1.0);				
	
	E->setMeasurement(motionConstraint);
	E->setInformation(infoMat);// * g2o::EdgeSim3::InformationType::Identity());	

	optimizer.addEdge(E);

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
	CAM2CAM_INFO(0,0) = CAM2CAM_INFO(2,2) = 10000;
	CAM2CAM_INFO(1,1) = 10000;
	CAM2CAM_INFO(3,3) = CAM2CAM_INFO(4,4) = CAM2CAM_INFO(5,5) = 0.0000001;
	CAM2CAM_INFO(6,6) = 0.0000001;
	
	Eigen::Matrix<double, 7, 7> VEH2VEH_INFO;
	VEH2VEH_INFO.setIdentity();
	VEH2VEH_INFO = 1000000* VEH2VEH_INFO;
	/*VEH2VEH_INFO(0,0) = VEH2VEH_INFO(1,1) = VEH2VEH_INFO(2,2) = 10;
	VEH2VEH_INFO(3,3) = VEH2VEH_INFO(4,4) = VEH2VEH_INFO(5,5) = 1000000;
	VEH2VEH_INFO(6,6) = 1000000;*/

	Eigen::Matrix<double, 7, 7> CAM2VEH_INFO;
	CAM2VEH_INFO.setIdentity();
    CAM2VEH_INFO = CAM2VEH_INFO * 1000;
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


	// save the adjacency matrix to verify the graph structure in matlab.
	//FILE *fp = fopen("adjacency.txt", "w");
	
	Eigen::Matrix<int, 76, 76> adjMat;
	adjMat.setZero();
	
	// using 100 as offset for vehicle node ids
	for(int i=0; i<numOfFrames-1; ++i){
		
		//If first frame connect only ego Car and Vehicle
		if(i == 0){												

				
				// add nodes
				addSimNode(i,   orbVO[i],   true, true, 1.0, optimizer);				// Add C1
				addSimNode(i+1, orbVO[i+1], false, false, 1.0, optimizer);				// Add C2
				
				// converting the vehicle pose in global frame before adding it to the graph.
				// As the orb is already in global pose, we can just multiply with the vehicle pose
				

				//addSimNode(i  +100, gtVO[i]*shapePose[i],     true, true, 1.0, optimizer);   	// Add V1
				//addSimNode(i+1+100, gtVO[i+1]*shapePose[i+1], true, false, 1.0, optimizer);   	// Add V2 	

				addSimNode(i  +100, gtVO[i]*shapePose[i],     true, true, 1.0, optimizer);   	// Add V1
				addSimNode(i+1+100, gtVO[i+1]*shapePose[i+1], true, false, 1.0, optimizer);   	// Add V2 	

				std::cerr<<"V"<<i<<":"<<shapePose[i]<<std::endl;		
				std::cerr<<"V"<<i+1<<":"<<shapePose[i+1]<<std::endl;		
				
				// make links as: C1 -> V1    ; C1->C2   ; V2      -> V1    ;  C2  -> V2
				//                i  -> i+100 ; i -> i+1 ; i+1+100 -> i+100 ;  i+1 -> i+1+100												

				addSimEdge(i,       i+1,     CAM2CAM_INFO, optimizer);									   // Add edge C1->C2


				//addSimEdge(i,       i+100,   1.0, optimizer);									   // Add edge C1->V1			
				//addSimEdge(i+1,     i+100+1, 1.0, optimizer);									  		 // Add edge C2->V2
				
				Eigen::Matrix4d sp;
				sp = shapePose[i]	;
				sp.block(0,0,3,3).setIdentity();
				addSimEdgeNew(i,       i+100,   sp, CAM2VEH_INFO, optimizer);									   // Add edge C1->V1			
				sp = shapePose[i+1]	;
				sp.block(0,0,3,3).setIdentity();
				addSimEdgeNew(i+1,     i+100+1, sp, CAM2VEH_INFO, optimizer);									  		 // Add edge C2->V2




				// ADD PRIOR
				//addRotationAboutYPrior(i, orbVO[i], 1000, optimizer);

				// // ADD PRIOR
				// addRotationAboutYPrior(i+100, shapePose[i], 10000000000, optimizer);
				
				// The tricky part.
				//addMotionModelSimEdge(i+1, i, gtVO, shapePose, VEH2VEH_INFO, optimizer);
				addMotionModelSimEdgeNewData( i, i+1, lstmMotion , shapePose , VEH2VEH_INFO, optimizer);
				
				// Make adjacency matrix to test structure. 
				// Later have a function which iterates over graph to generate the adj. matrix
				adjMat(i, i+1) = 1.0;		//C1->C2
				adjMat(i, i+numOfFrames) = 1.0;		//C1->V1
				adjMat(i+1, i+numOfFrames+1) = 1.0;	//C2->V2
				adjMat(i+numOfFrames, i+numOfFrames+1) = 1.0;  //V1->V2 
												
		}
		else{
				
				addSimNode(i+1, orbVO[i+1], false, false, 1.0, optimizer);				// Add C2
				addSimNode(i+100+1, gtVO[i+1]*shapePose[i+1], true, false, 1.0, optimizer);		// Add V2
				

				std::cerr<<"V"<<i<<":"<<shapePose[i+1]<<std::endl;		


				addSimEdge(i,   i+1,     CAM2CAM_INFO, optimizer);								// Add C1->C2							 
//				addSimEdge(i+1, i+100+1, 1.0, optimizer);								// Add C2->V2	 
				
				Eigen::Matrix4d sp;
				sp = shapePose[i+1]	;
				sp.block(0,0,3,3).setIdentity();
				addSimEdgeNew(i+1,     i+100+1, sp, CAM2VEH_INFO, optimizer);									  		 // Add edge C2->V2

				// ADD PRIOR
				//addRotationAboutYPrior(i, orbVO[i], 1000, optimizer);
				// // ADD PRIOR
				// addRotationAboutYPrior(i+100, shapePose[i], 100000000000, optimizer);

				// The tricky part.
				//addMotionModelSimEdge(i+1, i, gtVO, shapePose, VEH2VEH_INFO, optimizer);
				addMotionModelSimEdgeNewData( i, i+1, lstmMotion, shapePose, VEH2VEH_INFO, optimizer);
				//addMotionModelSimEdgeNewData(i, i+1, gtVO, lstmMotion, 100000.0, optimizer);
				
				// Make adjacency matrix to test structure. 
				// Later have a function which iterates over graph to generate the adj. matrix
				adjMat(i, i+1) = 1.0;
				adjMat(i+1, i+numOfFrames+1) = 1.0;
				adjMat(i+numOfFrames, i+numOfFrames+1) = 1.0;  //V1->V2 									
		}
		
	}
	
	std::cerr<<"\n saved before opti. :"<<optimizer.save("testSave_old.g2o")<<std::endl;
	
	std::ofstream file("adjMatTest.txt");
	
	file<<(adjMat)<<"\n";


	//test by adding one extra node and edge from grnd truth




	// Eigen::Matrix3d Rgtln;
	// Eigen::Vector3d tgtln;
 //    Eigen::Matrix4d T;
 //    T.setIdentity();


	// Rgtln << 0.9998, -0.0078522, 0.018295, 0.0079395, 0.99996 ,-0.0047012 ,-0.018257 ,0.0048455, 0.99982; ;
	
	// tgtln << -0.23905, -1.2909, 57.386;
	
	// T.block(0,0,3,3) = Rgtln;
	// T.block(0,3,3,1) = tgtln;

	// // add a support node


	// addSimNode(1000, T, true, true, 1.0, optimizer);


	// g2o::Sim3 support(Rgtln, tgtln, 1);
	// g2o::EdgeSim3 *E = new g2o::EdgeSim3();
	
	// E->vertices()[0] = optimizer.vertex(0);
	// E->vertices()[1] = optimizer.vertex(37);
	
	// E->setMeasurement(support);
	// E->setInformation(10000 * g2o::EdgeSim3::InformationType::Identity());//infoMatWeight * g2o::EdgeSim3::InformationType::Identity());	
	// optimizer.addEdge(E);



	// Rgtln <<1,0,0,0,1,0,0,0,1;	
	// tgtln <<  0,0,0;

	// g2o::Sim3 support2(Rgtln, tgtln, 1);
	// g2o::EdgeSim3 *E2 = new g2o::EdgeSim3();
	
	// E2->vertices()[0] = optimizer.vertex(37);
	// E2->vertices()[1] = optimizer.vertex(1000);
	
	// E2->setMeasurement(support2);
	// E2->setInformation(1000 * g2o::EdgeSim3::InformationType::Identity());//infoMatWeight * g2o::EdgeSim3::InformationType::Identity());	
	// optimizer.addEdge(E2);




	
	optimizer.initializeOptimization();
	optimizer.optimize(50);
	
	
	std::cerr<<"\n saved after opti.:"<<optimizer.save("testSave_opti.g2o")<<std::endl;

	
	printOptimizedDataToFile(optimizer);
	

	
	
	
	
	
	
	std::cout<<"test g2o...";
	
	return 0;
}

