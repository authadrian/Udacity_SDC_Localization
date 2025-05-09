// Udacity SDC C3 Localization
// Dec 21 2020
// Aaron Brown

using namespace std;
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>

enum Registration{ Off, Icp, Ndt};
Registration matching = Off;

Pose pose(Point(0,0,0), Rotate(0,0,0));
Pose savedPose = pose;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{
	if (event.getKeySym() == "Right" && event.keyDown()){
		matching = Off;
		pose.position.x += 0.1;
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		matching = Off;
		pose.position.x -= 0.1;
  	}
  	else if (event.getKeySym() == "Up" && event.keyDown()){
		matching = Off;
		pose.position.y += 0.1;
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		matching = Off;
		pose.position.y -= 0.1;
  	}
	else if (event.getKeySym() == "k" && event.keyDown()){
		matching = Off;
		pose.rotation.yaw += 0.1;
		while( pose.rotation.yaw > 2*pi)
			pose.rotation.yaw -= 2*pi; 
  	}
	else if (event.getKeySym() == "l" && event.keyDown()){
		matching = Off;
		pose.rotation.yaw -= 0.1;
		while( pose.rotation.yaw < 0)
			pose.rotation.yaw += 2*pi; 
  	}
	else if(event.getKeySym() == "n" && event.keyDown()){
		matching = Ndt;
	}
	else if(event.getKeySym() == "i" && event.keyDown()){
		matching = Icp;
	}
	else if(event.getKeySym() == "space" && event.keyDown()){
		matching = Off;
		pose = savedPose;
	}
	else if(event.getKeySym() == "x" && event.keyDown()){
		matching = Off;
		savedPose = pose;
	}

}

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	// Transform the source to the startingPose
	// Create transformation matrix from the startingPose
	Eigen::Matrix4d initTransform = transform3D(
	  startingPose.rotation.yaw,
	  startingPose.rotation.pitch,
	  startingPose.rotation.roll,
	  startingPose.position.x,
	  startingPose.position.y,
	  startingPose.position.z
  );
  
  // Create a transformed source point cloud
  PointCloudT::Ptr transformSource(new PointCloudT);
  pcl::transformPointCloud(*source, *transformSource, initTransform);
  
  // Timer for testing correct iterations count later
  pcl::console::TicToc time;
  time.tic();

  // Create the PCL ICP object and set its parameters
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  
  // Set ICP parameters
  icp.setMaximumIterations(iterations);
  icp.setInputSource(transformSource);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(2);  // Max distance between corresponding points
  
  // Optional parameters for fine-tuning
  // icp.setTransformationEpsilon(0.001);  // Transformation threshold for convergence
  // icp.setEuclideanFitnessEpsilon(0.05); // Fitness score threshold for convergence
  
  // Perform ICP alignment
  PointCloudT::Ptr aligned_cloud(new PointCloudT);
  icp.align(*aligned_cloud);
  // Time taken for ICP to run
  //std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
  
  // Check convergence and return appropriate transformation
  if (icp.hasConverged()) {
	  // Get the final transformation and adjust by initial transform
	  Eigen::Matrix4d icp_transform = icp.getFinalTransformation().cast<double>();
	  transformation_matrix = icp_transform * initTransform;
	  // Print Fitness Score for testing correct iteration count. Lower Fitness the better.
	  //std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
	  return transformation_matrix;
  } else {
	  // If ICP did not converge, log message and return identity matrix
	  std::cout << "WARNING: ICP did not converge" << std::endl;
	  return transformation_matrix;  // Return the identity matrix as initialized
  }
}

Eigen::Matrix4d NDT(pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt, PointCloudT::Ptr source, Pose startingPose, int iterations){

  	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

	// TODO: Implement the PCL NDT function and return the correct transformation matrix
	
	// Timer for testing correct iterations count later
	pcl::console::TicToc time;
	time.tic ();
	
	// Set maximum iterations
  	ndt.setMaximumIterations(iterations);
  	
  	// Set source point cloud
  	ndt.setInputSource(source);
  	
  	// Create initial transformation matrix from starting pose
  	Eigen::Matrix4d init_guess = transform3D(
  	    startingPose.rotation.yaw, 
  	    startingPose.rotation.pitch, 
  	    startingPose.rotation.roll, 
  	    startingPose.position.x, 
  	    startingPose.position.y, 
  	    startingPose.position.z
  	);
  	
  	// Create output point cloud
  	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  	
  	// Align the source to the target using our initial guess
  	ndt.align(*output_cloud, init_guess.cast<float>());
  	
  	// Get the final transformation
  	transformation_matrix = ndt.getFinalTransformation().cast<double>();
  	
  	return transformation_matrix;
}

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

void loadScans(vector<PointCloudT::Ptr>& scans, int num){
	for(int index = 0; index < num; index++){
		// Load scan
		PointCloudT::Ptr scanCloud(new PointCloudT);
  		pcl::io::loadPCDFile("scan"+to_string(index+1)+".pcd", *scanCloud);
  		scans.push_back(scanCloud);
	}
}

struct Tester{

	Pose pose;
	bool init = true;
	int cycles = 0;
	pcl::console::TicToc timer;

	//thresholds
	double distThresh = 1e-3;
	double angleThresh = 1e-3;

	vector<double> distHistory;
	vector<double> angleHistory;

	void Reset(){
		cout << "Total time: " << timer.toc () << " ms, Total cycles: " << cycles << endl;
		init = true;
		cycles = 0;
		distHistory.clear();
		angleHistory.clear();
	}

	double angleMag( double angle){

		return abs(fmod(angle+pi, 2*pi) - pi);
	}

	bool Displacement( Pose p){

		if(init){
			timer.tic();
			pose = p;
			init = false;
			return true;
		}

		Pose movement = p - pose;
		double tdist = sqrt(movement.position.x * movement.position.x + movement.position.y * movement.position.y + movement.position.z * movement.position.z);
		double adist = max( max( angleMag(movement.rotation.yaw), angleMag(movement.rotation.pitch)), angleMag(movement.rotation.roll) );

		if(tdist > distThresh || adist > angleThresh){
			distHistory.push_back(tdist);
			angleHistory.push_back(adist);
			pose = p;

			cycles++;
			return true;
		}
		else
			return false;
	
	}

};

int main(){

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
	viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);

	// Load map and display it
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	// True pose for the input scan
	vector<Pose> truePose ={Pose(Point(2.62296,0.0384164,0), Rotate(6.10189e-06,0,0)), Pose(Point(4.91308,0.0732088,0), Rotate(3.16001e-05,0,0))};
	drawCar(truePose[0], 0,  Color(1,0,0), 0.7, viewer);

	// Load input scan
	vector<PointCloudT::Ptr> scans;
	loadScans(scans, 1);

	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

	// Create voxel filter for input scan and save to cloudFiltered
	pcl::VoxelGrid<PointT> voxel;
	voxel.setInputCloud(scans[0]);
	// Set leaf size (resolution of voxel grid)
	float filterRes = 0.5;
	voxel.setLeafSize(filterRes, filterRes, filterRes);
	voxel.filter(*cloudFiltered);
	std::cout << "Filtered cloud contains " << cloudFiltered->size() 
          << " data points from original " << scans[0]->size() << std::endl;

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	//TODO: Set resolution and point cloud target (map) for ndt
	// Setting minimum transformation difference for termination condition
	ndt.setTransformationEpsilon(0.0001);
	// Setting maximum step size for More-Thuente line search
	ndt.setStepSize(1.0);
	// Setting Resolution of NDT grid structure (VoxelGridCovariance)
	ndt.setResolution(1.0);
	// Set the target point cloud (map)
	ndt.setInputTarget(mapCloud);

	PointCloudT::Ptr transformed_scan (new PointCloudT);
	Tester tester;

	while (!viewer->wasStopped())
  	{
		Eigen::Matrix4d transform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);

		if( matching != Off){
			if( matching == Ndt)
				transform = NDT(ndt, cloudFiltered, pose, 10); //TODO: change the number of iterations to positive number
			else if( matching == Icp)
				transform = ICP(mapCloud, cloudFiltered, pose, 10);
			pose = getPose(transform);
			if( !tester.Displacement(pose) ){
				if(matching == Ndt)
					cout << " Done testing NDT" << endl;
				else if(matching == Icp)
					cout << " Done testing ICP" << endl;
				tester.Reset();
				double pose_error = sqrt( (truePose[0].position.x - pose.position.x) * (truePose[0].position.x - pose.position.x) + (truePose[0].position.y - pose.position.y) * (truePose[0].position.y - pose.position.y) );
				cout << "pose error: " << pose_error << endl;
				matching = Off;
			}
		}
		
  		pcl::transformPointCloud (*cloudFiltered, *transformed_scan, transform);
		viewer->removePointCloud("scan");
		renderPointCloud(viewer, transformed_scan, "scan", Color(1,0,0)	);

		viewer->removeShape("box1");
		viewer->removeShape("boxFill1");
		drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
		
  		viewer->spinOnce ();
  	}

	return 0;
}
