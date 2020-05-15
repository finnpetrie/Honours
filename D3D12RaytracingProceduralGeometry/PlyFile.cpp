/*
 * PlyFile.cpp
 *
 *  Created on: 11/01/2019
 *      Author: finn
 */
#include "stdafx.h"

#include "PlyFile.h"



PlyFile::PlyFile(std::vector<Vertex_Ply> vertices) : points_(vertices){
	std::cout << "init via vertex" << std::endl;

}

PlyFile::PlyFile(std::string pathToPly){
	this->read(pathToPly);
}
//default init

PlyFile::PlyFile(){
}

std::vector<Vertex_Ply> PlyFile::getPoints(){
	return points_;
}

int vertexHandler(p_ply_argument argument) {
	long elemIx, vertIx;
	std::vector<Vertex_Ply>* points;
	ply_get_argument_element(argument, NULL, &vertIx);
	ply_get_argument_user_data(argument, (void**)(&points), &elemIx);

	switch (elemIx) {
	case 0:
		(*points)[vertIx].location(0) = ply_get_argument_value(argument);
		break;
	case 1:
		(*points)[vertIx].location(1) = ply_get_argument_value(argument);
		break;
	case 2:
		(*points)[vertIx].location(2) = ply_get_argument_value(argument);
		break;
	case 3:
		(*points)[vertIx].colour(0) = ply_get_argument_value(argument);
		break;
	case 4:
		(*points)[vertIx].colour(1) = ply_get_argument_value(argument);
		break;
	case 5:
		(*points)[vertIx].colour(2) = ply_get_argument_value(argument);
		break;
	case 6:
		(*points)[vertIx].normal(0) = ply_get_argument_value(argument);
		break;
	case 7:
		(*points)[vertIx].normal(1) = ply_get_argument_value(argument);
		break;
	case 8:
		(*points)[vertIx].normal(2) = ply_get_argument_value(argument);
		break;
	default:
		std::cerr << "Unrecognised element type " << elemIx << std::endl;
		return 0;
	}
	return 1;
}


void PlyFile::translateCloud(Eigen::Vector3d trans){
	Eigen::Vector3d cent = centroid();
	translateToOrigin(cent);
	for(int i = 0; i < size(); i++){
		points_[i].location += trans;
	}
	translateToOrigin(-cent);
}


Eigen::Matrix3d PlyFile::covariance(){

	//compute means for each axis.
	double cv[3][3];
	Eigen::Matrix3d cov;

	double means[3] = {0,0,0};

	for(int i =0 ; i < size(); i++){
		means[0] += points_[i].location.x();
		means[1] += points_[i].location.y();
		means[2] += points_[i].location.z();
	}
	means[0] /= size();
	means[1] /= size();
	means[2] /= size();

	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			cv[i][j] = 0;
			for(int k = 0; k < points_.size(); k++){
				cv[i][j] += (points_[k].location[i] - means[i]) *
										(points_[k].location[j] - means[j]);
			}
			cv[i][j] /= size() - 1;

		}
	}
	cov << cv[0][0], cv[0][1], cv[0][2], cv[1][0], cv[1][1], cv[1][2], cv[2][0], cv[2][1], cv[2][2];
	return cov;
}


void PlyFile::rotateCloud(Eigen::Matrix3d rotation){
	Eigen::Vector3d rotated;
	for(int i = 0 ; i < size(); i++){
		 rotated = rotation*points_[i].location;
		points_[i].location = rotated;
	}
}

Eigen::Vector3d PlyFile::centroid(){
	Eigen::Vector3d centroid(0, 0, 0);

	for(int i = 0; i < size(); i++){
		centroid += points_[i].location;
	}
	centroid /= size();
	return centroid;
}

void PlyFile::translateToOrigin(Eigen::Vector3d centroid){
	for(int i =0 ; i < size(); i++){
		points_[i].location -= centroid;
	}
}


void PlyFile::rotateOrigin(Eigen::Matrix3d rotation){
	Eigen::Vector3d cent = centroid();
	//translate to origin
	translateToOrigin(cent);
	//write("TranslatedCentroid.ply");
	//rotate around origin
	rotateCloud(rotation);
	//translate back
	translateToOrigin(-cent);
}

Vertex_Ply PlyFile::getRandomPoint(){
	std::cout<<"Getting random" <<std::endl;
	Vertex_Ply v = points_[rand()%points_.size()];
	v.print();
	std::cout<<"Got random"<<std::endl;
	return v;
}

Eigen::Vector3d PlyFile::closestPoint(Vertex_Ply x){
	double closestDistance = std::numeric_limits<double>::infinity();
	int index = -1;
	for(int i = 0; i < size(); i++){
		if(x.EuclideanDistance(points_[i]) < closestDistance){
			closestDistance = x.EuclideanDistance(points_[i]);
			index = i;
		}
	}

	return points_[index].location;

}
void PlyFile::order(){
	//order according to the first element
	Vertex_Ply first = points_[0];
	//find the next point -- i.e., the closest point
	for(int i = 0 ; i < size(); i++){


	}
}

void PlyFile::augment(PlyFile aug){
	for(int i = 0; i < aug.size(); i++){
		push_back(aug.points_[i]);
	}
}


bool PlyFile::read(const std::string& filename){
	p_ply ply = ply_open(filename.c_str(),NULL, 0, NULL);
	if(!ply){
		std::cerr << "Failed to open PLY file" << std::endl;
		return false;
	}

	if(!ply_read_header(ply)){
		std::cerr << "Failed to open PLY header" << std::endl;
		return false;

	}

	long nVertexs = ply_set_read_cb(ply, "vertex", "x", vertexHandler, &points_, 0);
	ply_set_read_cb(ply, "vertex", "y", vertexHandler, &points_, 1);
	ply_set_read_cb(ply, "vertex", "z", vertexHandler, &points_, 2);
	ply_set_read_cb(ply, "vertex", "r", vertexHandler, &points_, 3);
	ply_set_read_cb(ply, "vertex", "g", vertexHandler, &points_, 4);
	ply_set_read_cb(ply, "vertex", "b", vertexHandler, &points_, 5);
	ply_set_read_cb(ply, "vertex", "red", vertexHandler, &points_, 3);
	ply_set_read_cb(ply, "vertex", "blue", vertexHandler, &points_, 4);
	ply_set_read_cb(ply, "vertex", "green", vertexHandler, &points_, 5);
	ply_set_read_cb(ply, "vertex", "nx", vertexHandler, &points_, 6);
	ply_set_read_cb(ply, "vertex", "ny", vertexHandler, &points_, 7);
	ply_set_read_cb(ply, "vertex", "nz", vertexHandler, &points_, 8);
	points_.resize(nVertexs);

	ply_read(ply);

	return true;
}

void PlyFile::sortAlongAxis(int axis, int low, int high){
	if (low < high){
		//std::cout << low;
		int p = partition(axis, low, high);
		sortAlongAxis(axis, low, p);
		sortAlongAxis(axis, p + 1, high);
	}

}


int PlyFile::partition(int axis, int low, int high){
	Eigen::Vector3d pivot = points_[low].location;

	int leftwall = low;
	for(int i = low + 1; i < high; i++){
		if(points_[i].location(axis) < pivot(axis)){
			swap(i, leftwall+1);
			leftwall++;
		}
	}
	swap(low, leftwall);
	return leftwall;



}

void PlyFile::swap(int i, int j){
	Vertex_Ply v = points_[i];
	points_[i] = points_[j];

	points_[j] = v;
}
bool PlyFile::write(const std::string& filename){

	std::ofstream fout(filename);
	    if (!fout) {
	        std::cerr << "Failed to open PLY file " << filename << " for writing" << std::endl;
	        return false;
	    }

	    fout << "ply\n"
	    << "format ascii 1.0\n"
	    << "element vertex " << size() << "\n"
	    << "property float x\n"
	    << "property float y\n"
	    << "property float z\n"
	    << "property float nx\n"
	    << "property float ny\n"
	    << "property float nz\n"
	    << "property uchar red\n"
	    << "property uchar green\n"
	    << "property uchar blue\n"
	    << "end_header\n";

	    for (size_t ix = 0; ix < size(); ++ix) {
	        fout << points_[ix].location(0) << " " << points_[ix].location(1)<< " " << points_[ix].location(2) << " " << points_[ix].normal(0) << " " << points_[ix].normal(1) << " " << points_[ix].normal(2) << " "  << points_[ix].colour(0) << " " << points_[ix].colour(1) << " "<< points_[ix].colour(2) << "\n";
	      // std::cout << "Points: " << points_[ix].location << " normals : " << points_[ix].normal << " " << 255 <<  " " << 0 << " " << 0 << "\n";
	    }

	    fout.close();

	    return true;

}

bool PlyFile::writeBlue(const std::string& filename){

	std::ofstream fout(filename);
	    if (!fout) {
	        std::cerr << "Failed to open PLY file " << filename << " for writing" << std::endl;
	        return false;
	    }

	    fout << "ply\n"
	    << "format ascii 1.0\n"
	    << "element vertex " << size() << "\n"
	    << "property float x\n"
	    << "property float y\n"
	    << "property float z\n"
	    << "property float nx\n"
	    << "property float ny\n"
	    << "property float nz\n"
	    << "property uchar red\n"
	    << "property uchar green\n"
	    << "property uchar blue\n"
	    << "end_header\n";

	    for (size_t ix = 0; ix < size(); ++ix) {
	        fout << points_[ix].location << " " << points_[ix].normal << " " << 0 <<  " " << 0 << " " << 255 << "\n";
	    }
	    fout.close();
	    return true;

}

bool PlyFile::writeRed(const std::string& filename){

	std::ofstream fout(filename);
	    if (!fout) {
	        std::cerr << "Failed to open PLY file " << filename << " for writing" << std::endl;
	        return false;
	    }

	    fout << "ply\n"
	    << "format ascii 1.0\n"
	    << "element vertex " << size() << "\n"
	    << "property float x\n"
	    << "property float y\n"
	    << "property float z\n"
	    << "property float nx\n"
	    << "property float ny\n"
	    << "property float nz\n"
	    << "property uchar red\n"
	    << "property uchar green\n"
	    << "property uchar blue\n"
	    << "end_header\n";

	    for (size_t ix = 0; ix < size(); ++ix) {
	        fout << points_[ix].location << " " << points_[ix].normal << " " << 255 <<  " " << 0 << " " << 0 << "\n";
	    }
	    fout.close();
	    return true;
}


int PlyFile::size(){
	return points_.size();
}


double PlyFile::curvatureAtPoint(int index){
	int previousIndex = index -1;
	int afterIndex = index + 1;

	if(index == 0){
		previousIndex = points_.size();
	}
	else if(index == points_.size()){
		afterIndex = 0;
	}

	Eigen::Vector3d previous = points_[index].location - points_[previousIndex].location;
	Eigen::Vector3d next = points_[index].location - points_[afterIndex].location;

	double previousMag = previous.norm();
	double nextMag = next.norm();

	double cos = previous.dot(next)/(previousMag*nextMag);
	return acos(cos);


}


void PlyFile::reColour(int r, int g, int b){
	for(int i = 0; i < points_.size(); i++){
		Eigen::Vector3i newColour(r,g,b);
		points_[i].colour = newColour;

	}
}

void PlyFile::rotateAboutPoint(Eigen::Matrix3d rotation, Eigen::Vector3d point){
	translateToOrigin(point);
	rotateCloud(rotation);
	translateToOrigin(-point);
}

void PlyFile::rotateAxis(int axis, double amount){
	Eigen::Matrix3d cov = covariance();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covEigens(cov);
	std::cout << "The eigenvalues of A are: \n" << covEigens.eigenvalues() << std::endl;
	std::cout << "Here's a matrix whose columns are eigenvectors of A \n"
	        << "corresponding to these eigenvalues:\n"
	        << covEigens.eigenvectors() << std::endl;

	std::cout << "\n\n\"" << std::endl;
	std::cout << cov << std::endl;
	std::cout << "The second eigenvector of the 3x3 covariance matrix is:"
	     << std::endl << covEigens.eigenvectors().col(2) << std::endl;

	 Eigen::Matrix3d m;
	 m = Eigen::AngleAxisd(amount*M_PI, covEigens.eigenvectors().col(axis));

	 rotateOrigin(m);
}

void PlyFile::rotateAxisAboutPoint(int axis, double amount, Eigen::Vector3d point){
	Eigen::Matrix3d cov = covariance();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covEigens(cov);
	//std::cout << "The eigenvalues of A are: \n" << covEigens.eigenvalues() << std::endl;
	//std::cout << "Here's a matrix whose columns are eigenvectors of A \n"
	//        << "corresponding to these eigenvalues:\n"
	//        << covEigens.eigenvectors() << std::endl;

	////std::cout << "\n\n\"" << std::endl;
	//std::cout << cov << std::endl;
	//std::cout << "The second eigenvector of the 3x3 covariance matrix is:"
	   //  << std::endl << covEigens.eigenvectors().col(2) << std::endl;

	 Eigen::Matrix3d m;
	 m = Eigen::AngleAxisd(amount*M_PI, covEigens.eigenvectors().col(axis));

	 rotateAboutPoint(m, point);
}


Eigen::Matrix3d PlyFile::changeOfBasis(Eigen::Matrix3d toBasis, Eigen::Matrix3d fromBasis){
		Eigen::Matrix3d fromBasisInv = fromBasis.inverse();
		Eigen::Matrix3d changeOfBasis = toBasis*fromBasisInv;

		return changeOfBasis;
}


void PlyFile::representUnderChangeBasis(Eigen::Matrix3d toBasis, Eigen::Matrix3d fromBasis, Eigen::Vector3d centroid){
	Eigen::Matrix3d change = changeOfBasis(toBasis, fromBasis);

	for(int i = 0 ; i < size(); i++){
		Eigen::Vector3d point = points_[i].location - centroid;
		Eigen::Vector3d changePoint = change*point;

		updateLocation(changePoint, i);
	}

}


std::vector<Vertex_Ply> PlyFile::collectPositiveVertices(int axis){
	std::vector<Vertex_Ply> positives;

	for(int i =0 ; i < size(); i++){
		if(points_[i].location(axis) >= 0){
			positives.push_back(points_[i]);
		}
	}

	return positives;
}

std::vector<Vertex_Ply> PlyFile::collectNegativeVertices(int axis){
	std::vector<Vertex_Ply> negatives;

	for(int i =0 ; i < size(); i++){
		if(points_[i].location(axis) < 0){
			negatives.push_back(points_[i]);
		}
	}

	return negatives;
}

void PlyFile::orientateAroundYAxis(){
	//compute covariance.
	Eigen::Matrix3d cov = covariance();
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> covEigens(cov);
	Eigen::Matrix3d currentBasis = covEigens.eigenvectors();

	Eigen::Matrix3d identity;
	//swap Y and Z
	identity << 1, 0, 0,
				0, 0, 1,
				0, 1 ,0;

	representUnderChangeBasis(identity, currentBasis, centroid());


}


PlyFile PlyFile::colourThreshold(Eigen::Vector3i colour, double threshold, std::string filename ){
	PlyFile colourFiltered;
	PlyFile otherColours;
	int count = 0;
	//d::cout << "in here\n";
	for(int i = 0; i < points_.size(); i++){
		//d::cout << "Within the for loo\n";
		Eigen::Vector3i colourDifference(0,0,0);
	//td::cout << "Our point : \n" << points_[i].colour << std::endl;
		colourDifference = (points_[i].colour - colour);
	//td::cout << colourDifference << std::endl;
        float distance = (fabs(colourDifference[0]/255.0) + fabs(colourDifference[1]/255.0) + fabs(colourDifference[2]/255.0)) * 100.0 /3.0;
		//d::cout << "Our distance:\n" << distance;
		if(distance < threshold){
			//d::cout << "distance: \n" << distance << "\n" << "threshold: \n" << threshold << std::endl;
			colourFiltered.push_back(points_[i]);


			count++;
		}else{
			otherColours.push_back(points_[i]);
		}
	}

	std::cout << "Writing new ply filtered by colour\n";
	colourFiltered.write(filename + ".ply");
	std::cout << "Writing filtered plane" << std::endl;
	otherColours.write(filename + "plane.ply");
	return colourFiltered;
}




Vertex_Ply PlyFile::getPointAt(int i){
assert(i < points_.size());
	return points_[i];
}

void PlyFile::print(){
	for(auto& element: points_){
		element.print();

	}
}


void PlyFile::clear(){
	this->points_.clear();

}

void PlyFile::updateLocation(Eigen::Vector3d update, int index){
	points_[index].location = update;
}





