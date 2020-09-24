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

/*  IDEA: the cars can be part of static map where sim(3) can make more sense 
 */

//std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &motion 
// Function to read the data into vector of Eigen Matrix; Everything will be hardcoded here
void readData(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &orbVO, \
			  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &gtVO,  \
			  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &shapePose \
			  ){


	// all the files have 13 columnes - framenumber R(rowmajor) tx ty tz
	char* orbVOFile     = "../../data/orb_full3.txt";
	char* gtVOFile      = "../../data/gt_full3.txt";
	char* shapePoseFile = "../../data/shapePose_full3.txt";//shapePose_full3.txt";
	//char* motionFile    = "../../data/motion_full3.txt";
	
	// open files
	FILE *orbVOfp = fopen(orbVOFile, "r");
	FILE *gtVOfp  = fopen(gtVOFile, "r");
	FILE *shapePosefp = fopen(shapePoseFile, "r");
	
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
		orbVO.push_back(m);					
		
		
		// GT VO
		for(int j=0; j<13; j++){						
			fscanf(gtVOfp, "%lf", &val[j]);			
		}						
		
		//m << 1, 0, 0, val[10], \
		     0, 1, 0, val[11], \
		     0, 0, 1, val[12], \
		        0.0,    0.0,    0.0,     1.0;

		m << val[1], val[2], val[3], val[10], \
		     val[4], val[5], val[6], val[11], \
		     val[7], val[8], val[9], val[12], \
		        0.0,    0.0,    0.0,     1.0; 
		gtVO.push_back(m);
		
		// pose shape 
		for(int j=0; j<13; j++){						
			fscanf(shapePosefp, "%lf", &val[j]);			
		}		
		        
		m << val[1], val[2], val[3], val[10], \
		     val[4], val[5], val[6], val[11], \
		     val[7], val[8], val[9], val[12], \
		        0.0,    0.0,    0.0,     1.0; 
		m << 1, 0, 0, val[10], \
		     0, 1, 0, val[11], \
		     0, 0, 1, val[12], \
		        0.0,    0.0,    0.0,     1.0;
		shapePose.push_back(m);				        		
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
	
	g2o::Sim3 simPose(R, t, scale);	
	
	N->setEstimate(simPose);
	N->setFixed(fixNode);
	N->_fix_scale = fixScale;
	N->setId(id);	
	
	optimizer.addVertex(N);
	
}




// add edge - have one function with proper infoMat type and constraint type.
void addSimEdge(int idFrom, int idTo, Eigen::Matrix<double, 7, 7> infoMat, double hth, g2o::SparseOptimizer &optimizer){


	g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;                
	
	g2o::EdgeSim3 *E = new g2o::EdgeSim3();
	
	E->vertices()[0] = optimizer.vertex(idFrom);
	E->vertices()[1] = optimizer.vertex(idTo);
	
	E->setMeasurement((dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(idFrom))->estimate().inverse() \
							* dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(idTo))->estimate()));
		
	std::cerr<<"test\n"	<<(dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(idFrom))->estimate().inverse() \
							* dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(idTo))->estimate());

	E->setInformation(infoMat);//infoMatWeight * g2o::EdgeSim3::InformationType::Identity());	
	
	E->setRobustKernel(rk);
    rk->setDelta(hth);
	
	optimizer.addEdge(E);
}



void addMotionModelSimEdge(int vIdFrom, int vIdTo, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &gtVO, \
												   std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > &shapePose, \
												   Eigen::Matrix<double, 7, 7> infoMat, double hth,\
												   g2o::SparseOptimizer &optimizer){
	
	Eigen::Matrix4d motion;											   
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
		
	g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;                
			
	g2o::EdgeSim3 *E = new g2o::EdgeSim3();				
	
	
	Eigen::Matrix3d rr;
	Eigen::Matrix<double, 3, 1> tt;


	// Eigen::Matrix4d Tco_11, Tco_12;
	// Tco_11 = shapePose[vIdFrom];	
	// Tco_12 = gtVO[vIdFrom].inverse() * gtVO[vIdTo] * shapePose[vIdTo];
	//motion = Tco_11.inverse() * Tco_12;
	
	g2o::Sim3 vfrom = dynamic_cast<g2o::BaseVertex<7, g2o::Sim3>* >(optimizer.vertex(vIdFrom+100))->estimate();


	motion =  (shapePose[vIdFrom].inverse() *  gtVO[vIdFrom].inverse() * gtVO[vIdTo] * shapePose[vIdTo]);
	
	R << motion(0,0), motion(0,1), motion(0,2), \
		 motion(1,0), motion(1,1), motion(1,2), \
		 motion(2,0), motion(2,1), motion(2,2);		 
	//t << 0, 0, 0;		  	 
	t << motion(0,3), motion(1,3), motion(2,3);		  
	
	g2o::Sim3 motionSim(R, t, 1.0);				

	// g2o::Sim3 motionConstraint;
	// motionConstraint = vfrom*motionSim;


//	motion = (shapePose[vIdFrom].inverse() * gtVO[vIdFrom-100]) *  			\
					(gtVO[vIdFrom-100].inverse() * gtVO[vIdTo-100]) *  		\
							(gtVO[vIdTo-100].inverse() * shapePose[vIdTo]) ;
	
	
	E->vertices()[0] = optimizer.vertex(vIdFrom+100);
	E->vertices()[1] = optimizer.vertex(vIdTo+100);


	
	//g2o::Sim3 motionConstraint(R, t, 1.0);				
	
	E->setMeasurement(motionSim);
	E->setInformation(infoMat);//infoMatWeight * g2o::EdgeSim3::InformationType::Identity());	
	
	E->setRobustKernel(rk);
    rk->setDelta(hth);

	optimizer.addEdge(E);


	//std::cerr<<"["<<vIdFrom<<","<<vIdTo<<"]\n"<<t<<std::endl;


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
	
	// read the files in to the respective vectors
	readData(orbVO, gtVO, shapePose);
	
	
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
				addSimNode(i,   orbVO[i],   true, true,   1.0, optimizer);				// Add C1
				addSimNode(i+1, orbVO[i+1], false, false, 1.0, optimizer);				// Add C2
				
				// converting the vehicle pose in global frame before adding it to the graph.
				// As the orb is already in global pose, we can just multiply with the vehicle pose
				addSimNode(i  +100, orbVO[i]*shapePose[i],     true, false, 1.0, optimizer);   	    // Add V1
				addSimNode(i+1+100, orbVO[i+1]*shapePose[i+1], false, false, 1.0, optimizer);   	// Add V2 			
				
				// make links as: C1 -> V1    ; C1->C2   ; V2      -> V1    ;  C2  -> V2
				//                i  -> i+100 ; i -> i+1 ; i+1+100 -> i+100 ;  i+1 -> i+1+100												
				addSimEdge(i,       i+1,     CAM2CAM_INFO, 0.1, optimizer);									   // Add edge C1->C2
				addSimEdge(i,       i+100,   CAM2VEH_INFO, 0.001, optimizer);									   // Add edge C1->V1
				addSimEdge(i+1,     i+100+1, CAM2VEH_INFO, 0.001, optimizer);									  		 // Add edge C2->V2
				//return 0;
				
				// The tricky part.
				addMotionModelSimEdge(i+1, i, gtVO, shapePose, VEH2VEH_INFO, 1.1, optimizer);
				
				// Make adjacency matrix to test structure. 
				// Later have a function which iterates over graph to generate the adj. matrix
				adjMat(i, i+1) = 1.0;		//C1->C2
				adjMat(i, i+numOfFrames) = 1.0;		//C1->V1
				adjMat(i+1, i+numOfFrames+1) = 1.0;	//C2->V2
				adjMat(i+numOfFrames, i+numOfFrames+1) = 1.0;  //V1->V2 
												
		}
		else{
				
				addSimNode(i+1, orbVO[i+1], false, false, 1.0, optimizer);				// Add C2
				addSimNode(i+100+1, orbVO[i+1]*shapePose[i+1], false, false, 1.0, optimizer);		// Add V2
				
				addSimEdge(i,   i+1,     CAM2CAM_INFO, 0.1, optimizer);								// Add C1->C2							 
				addSimEdge(i+1, i+100+1, CAM2VEH_INFO, 0.001, optimizer);								// Add C2->V2	 
				
				// The tricky part.
				addMotionModelSimEdge(i+1, i, gtVO, shapePose, VEH2VEH_INFO, 1.1, optimizer);
				
				// Make adjacency matrix to test structure. 
				// Later have a function which iterates over graph to generate the adj. matrix
				adjMat(i, i+1) = 1.0;
				adjMat(i+1, i+numOfFrames+1) = 1.0;
				adjMat(i+numOfFrames, i+numOfFrames+1) = 1.0;  //V1->V2 									
		}
		
	}




	//test by adding one extra node and edge from grnd truth

	// Eigen::Matrix3d Rgtln;
	// Rgtln.setIdentity();

	// Eigen::Vector3d tgtln;
	// tgtln << 0.25227, -1.0847, 57.635;
	
	// Eigen::Matrix<double, 7, 7> SUPPORT_INFO;
	// SUPPORT_INFO.setIdentity();
	// SUPPORT_INFO = 1.0*SUPPORT_INFO;

	// g2o::Sim3 support(Rgtln, tgtln, 1);
	// g2o::EdgeSim3 *E = new g2o::EdgeSim3();
	
	// E->vertices()[0] = optimizer.vertex(0);
	// E->vertices()[1] = optimizer.vertex(37);
	
	// E->setMeasurement(support);
	// E->setInformation(SUPPORT_INFO);//infoMatWeight * g2o::EdgeSim3::InformationType::Identity());	
	//optimizer.addEdge(E);




	

	std::cerr<<"\n saved before opti. :"<<optimizer.save("testSave_old.g2o")<<std::endl;
	
	std::ofstream file("adjMatTest.txt");
	
	file<<(adjMat)<<"\n";
	
	optimizer.initializeOptimization();
	optimizer.optimize(50);
	
	
	std::cerr<<"\n saved after opti.:"<<optimizer.save("testSave_opti.g2o")<<std::endl;

	
	printOptimizedDataToFile(optimizer);
	

	
	
	
	/*
	// Eigen matrices for rotaion and translation
	Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Identity();	
	Eigen::Matrix<double, 3, 1> t;
	
	// Vertices and edges of type SIM(3) in minimal representation i.e. in manifold 
	// representation. 
	
	g2o::VertexSim3Expmap * C1 = new g2o::VertexSim3Expmap();
	g2o::VertexSim3Expmap * C2 = new g2o::VertexSim3Expmap();	
	g2o::VertexSim3Expmap * V1 = new g2o::VertexSim3Expmap();
	g2o::VertexSim3Expmap * V2 = new g2o::VertexSim3Expmap();
	
	g2o::VertexSim3Expmap * V3 = new g2o::VertexSim3Expmap();	
	g2o::VertexSim3Expmap * C3 = new g2o::VertexSim3Expmap();	
	
	g2o::EdgeSim3 * CC12 = new g2o::EdgeSim3();	
	g2o::EdgeSim3 * CV11 = new g2o::EdgeSim3();
	g2o::EdgeSim3 * CV22 = new g2o::EdgeSim3();	
	g2o::EdgeSim3 * VV21 = new g2o::EdgeSim3();
	g2o::EdgeSim3 * VV23 = new g2o::EdgeSim3();			
	g2o::EdgeSim3 * CC23 = new g2o::EdgeSim3();
	g2o::EdgeSim3 * CV33 = new g2o::EdgeSim3();

	// initializing optimizer
	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(true);
	
	g2o::BlockSolverX::LinearSolverType * linearSolver;
	
	
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
	
	g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
	g2o::OptimizationAlgorithmLevenberg * solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver);	


	// making vertex C1 -------------------------------------------------------------------------
	t<<0, 0, 0;
	g2o::Sim3 pC1(R, t, 1.0);					
	C1->setEstimate(pC1);
	C1->setFixed(true);
	C1->_fix_scale = true;
	C1->setId(0);	
	optimizer.addVertex(C1);

	// making vertex C2 -------------------------------------------------------------------------
	t<<0.0, 0, 0.5;
	g2o::Sim3 pC2(R, t, 0.5);		// should be 1 but after loop we propagate scale		
	C2->setEstimate(pC2);
	//C2->_fix_scale = true;
	//C2->setFixed(true);
	C2->setId(1);	
	optimizer.addVertex(C2);

	// making vertex V2 -------------------------------------------------------------------------
	
	t<<1.0, 0, 0.0;
	g2o::Sim3 pV2(R, t, 1.0); // shapepose on C2-V2
	V2->setEstimate(pC2*pV2);
	//V2->setFixed(true);
	//V2->_fix_scale = true;
	V2->setId(2);	
	optimizer.addVertex(V2);

	// making vertex V1-------------------------------------------------------------------------
	t<<0.0, 0, 1.0;
	g2o::Sim3 lstmV1V2(R, t, 1.0);					
	V1->setEstimate(V2->estimate()*lstmV1V2.inverse());
	//V1->setFixed(true);
	//V1->_fix_scale = true;
	V1->setId(3);	
	optimizer.addVertex(V1);


	// EDGES...


	// C1 -> C2
	
	CC12->setMeasurement(C1->estimate().inverse() * C2->estimate());
	CC12->setInformation(0.01 * g2o::EdgeSim3::InformationType::Identity());
	CC12->vertices()[0] = optimizer.vertex(0);
	CC12->vertices()[1] = optimizer.vertex(1);
	optimizer.addEdge(CC12);

	// C2 -> V2
	CV22->setMeasurement(C2->estimate().inverse() * V2->estimate());
	CV22->setInformation(10* g2o::EdgeSim3::InformationType::Identity());
	CV22->vertices()[0] = optimizer.vertex(1);
	CV22->vertices()[1] = optimizer.vertex(2);
	optimizer.addEdge(CV22);

	// V2 -> V1
	
	VV21->setMeasurement(V2->estimate().inverse() * V1->estimate());
	VV21->setInformation(10* g2o::EdgeSim3::InformationType::Identity());
	VV21->vertices()[0] = optimizer.vertex(2);
	VV21->vertices()[1] = optimizer.vertex(3);
	optimizer.addEdge(VV21);

	// V1 -> C2 - LOOP CLOSING
	t<<-1.0, 0.0, 0.0;
	g2o::Sim3 c(R, t, 1.0);		
	CV11->setMeasurement(c);
	CV11->setInformation(10* g2o::EdgeSim3::InformationType::Identity());
	CV11->vertices()[0] = optimizer.vertex(3);
	CV11->vertices()[1] = optimizer.vertex(0);
	optimizer.addEdge(CV11);


		
	optimizer.initializeOptimization();
	optimizer.optimize(1000);	


	std::cout<<"after optimization:"<<std::endl;
	
	std::cout<<(C2->estimate())<<std::endl;
	std::cout<<"scale C2: "<<(C2->estimate()).scale()<<std::endl;
	
	
	std::cout<<(V1->estimate())<<std::endl;
	std::cout<<"scale V1: "<<(V1->estimate()).scale()<<std::endl;
	
	std::cout<<(V2->estimate())<<std::endl;
	std::cout<<"scale V2: "<<(V2->estimate()).scale()<<std::endl;
	
	*/
	
	
	std::cout<<"test g2o...";
	
	return 0;
}



/* EXAMPLE: SE(3) EXPMAP

	// making vertex no. 1
	g2o::SE3Quat p1, p2, p3, p12, p13;
	p1 = g2o::SE3Quat(R,t1);
	t1 << 0., 0., 0.;		
	v1->setEstimate(p1);
	v1->setFixed(true);
	v1->setId(1);
	optimizer.addVertex(v1);
	
	// making vertex no. 2

	p2 = g2o::SE3Quat(R,t1);
	v2->setEstimate(p2);
	v2->setFixed(0);
	v2->setId(2);
	optimizer.addVertex(v2);
	
	
	
	
	
	
	t2 << 0., 0., 5.0;	
	e1->setInformation(g2o::EdgeSE3Expmap::InformationType::Identity());		
	e1->setMeasurement(g2o::SE3Quat(R,t2));		
	e1->vertices()[0] = optimizer.vertex(1);	
	e1->vertices()[1] = optimizer.vertex(2);		
	optimizer.addEdge(e1);
	
	
	
	optimizer.initializeOptimization();
	optimizer.optimize(20);	


	std::cout<<"after optimization:"<<std::endl;
	
	std::cout<<v2->estimate();	
	
	std::cout<<"test g2o...";



*/

/*// making vertex C1
	t<<0, 0, 0;
	g2o::Sim3 pC1(R, t, 1.0);				
	C1->setEstimate(pC1);
	C1->setFixed(true);
	C1->_fix_scale = true;
	C1->setId(0);	
	optimizer.addVertex(C1);

	// making vertex C2
	t<<0, 0, 0.8;
	g2o::Sim3 pC2(R, t, 1.0);				
	C2->setEstimate(pC2);
	C2->_fix_scale = true;
	//C2->setFixed(true);
	C2->setId(1);	
	optimizer.addVertex(C2);

	// making vertex C3
	t<<0, 0, 0.2;
	g2o::Sim3 pC3(R, t, 9.8);				
	//C3->_fix_scale = true;
	//C3->setFixed(true);
	C3->setEstimate(pC3);	
	C3->setId(2);	
	optimizer.addVertex(C3);
	
	// making vertex V1
	t<<1.0, 0, 0;
	g2o::Sim3 pV1(R, t, 1.0);				
	V1->setEstimate(pV1);
	V1->setFixed(true);
	V1->_fix_scale = true;
	V1->setId(3);	
	optimizer.addVertex(V1);

	// making vertex V2
	t<<1.0, 0, 1.0;
	g2o::Sim3 pV2(R, t, 1.0);				
	V2->setEstimate(pV2);
	V2->setFixed(true);
	V2->_fix_scale = true;
	V2->setId(4);	
	optimizer.addVertex(V2);

	// making vertex V3
	t<<1.0, 0, 2.0;
	g2o::Sim3 pV3(R, t, 1.0);				
	V3->setEstimate(pV3);
	V3->setFixed(true);
	V3->_fix_scale = true;
	V3->setId(5);	
	optimizer.addVertex(V3);



	// EDGES...

	// C1 -> C2
	CC12->setMeasurement((pC1.inverse()*pC2).inverse());
	CC12->setInformation(g2o::EdgeSim3::InformationType::Identity());
	CC12->vertices()[0] = optimizer.vertex(0);
	CC12->vertices()[1] = optimizer.vertex(1);
	optimizer.addEdge(CC12);

	// C2 -> C3
	CC23->setMeasurement((pC2.inverse()*pC3));
	CC23->setInformation(g2o::EdgeSim3::InformationType::Identity());
	CC23->vertices()[0] = optimizer.vertex(1);
	CC23->vertices()[1] = optimizer.vertex(2);
	optimizer.addEdge(CC23);

	// C1 -> V1
	t << 1, 0, 0;
	g2o::Sim3 c(R, t, 1.0);
	CV11->setMeasurement(c.inverse());
	CV11->setInformation(g2o::EdgeSim3::InformationType::Identity());
	CV11->vertices()[0] = optimizer.vertex(0);
	CV11->vertices()[1] = optimizer.vertex(3);
	optimizer.addEdge(CV11);

	// C2 -> V2
	CV22->setMeasurement(c.inverse());
	CV22->setInformation(g2o::EdgeSim3::InformationType::Identity());
	CV22->vertices()[0] = optimizer.vertex(1);
	CV22->vertices()[1] = optimizer.vertex(4);
	optimizer.addEdge(CV22);
/*
	// C3 -> V3
	CV33->setMeasurement(c);
	CV33->setInformation(g2o::EdgeSim3::InformationType::Identity());
	CV33->vertices()[0] = optimizer.vertex(2);
	CV33->vertices()[1] = optimizer.vertex(5);
	optimizer.addEdge(CV33);
	
	std::cerr<<"Test: "<<CV33->measurement()<<std::endl;

	// V1 -> V2
	VV12->setMeasurement((pV1.inverse()*pV2).inverse());
	VV12->setInformation(g2o::EdgeSim3::InformationType::Identity());
	VV12->vertices()[0] = optimizer.vertex(3);
	VV12->vertices()[1] = optimizer.vertex(4);
	optimizer.addEdge(VV12);
/*
	// V2 -> V3
	VV23->setMeasurement((pV2.inverse()*pV3));
	VV23->setInformation(g2o::EdgeSim3::InformationType::Identity());
	VV23->vertices()[0] = optimizer.vertex(4);
	VV23->vertices()[1] = optimizer.vertex(5);
	optimizer.addEdge(VV23);

	// C2 -> C3
	CC23->setMeasurement((pC2.inverse()*pC3));
	CC23->setInformation(g2o::EdgeSim3::InformationType::Identity());
	CC23->vertices()[0] = optimizer.vertex(1);
	CC23->vertices()[1] = optimizer.vertex(2);
	optimizer.addEdge(CC23);
*/

