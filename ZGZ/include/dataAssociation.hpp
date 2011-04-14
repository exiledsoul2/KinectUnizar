/*
 * dataAssociation.hpp
 *
 *  Created on: Apr 8, 2011
 *      Author: yasir
 */

#ifndef DATAASSOCIATION_HPP_
#define DATAASSOCIATION_HPP_

#include <ZGZ.hpp>

using namespace Eigen;
using namespace cv;

/*
template <typename T, dataAssocation_t method>
class dataAssociator{
private:
	dataAssociation_t _method;
public:

	dataAssociator(){
		_method = method;
	}

	dataAssociation_t getAssociationMethod()
	{
		return _method;
	}


	 *
	 * @param prediction for NN,JCBB contains prediction of points for FM, NED contains
	 * @param observations
	 * @return

	T Hypothesis(MatrixXf& prediction, MatrixXf& observations)
	{

	}


};

typedef dataAssociator< Eigen::Matrix<float,Dynamic,2>, dataAssociation_t::NN > NN;
typedef dataAssociator< Eigen::Matrix<float,Dynamic,2>, dataAssociation_t::JCBB > JCBB;
typedef dataAssociator< Eigen::Matrix<float,Dynamic,1>, dataAssociation_t::FM > FundamentalMatrix;
typedef dataAssociator< Eigen::Matrix<float,Dynamic,1>, dataAssociation_t::NED > NED;

*/


//template <dataAssociation_t(NED)>
std::vector<int> Hypothesis(MatrixXf& Errors, int Nbins) //!< Error is a 2Nx1 Vector of residuals Errors
{

	int rows = Errors.rows();
	int cols = Errors.cols();

	std::vector<int> H(rows/2,0);

	MatrixXf ErrorOrientations = MatrixXf(rows/2,cols);

	Vector2f E ;

	std::vector<int> V(Nbins,0);


	for(int i=0;i<rows;i+=2)
	{
		E << Errors(i,0),Errors(i+1,0);
		if(E.norm()> 0)
			E.normalize();
		else
		{
			E<< 0 , 1e-15;
		}
		double orient = atan2(E(1),E(0));
		int index = int((orient+PI)/(2*PI)*(Nbins-1));
		index = (index>(Nbins-1))? Nbins-1:index;
		std::cout<<i<<" : "<<E(0)<<","<<E(1)<<"->"<<index<<" "<<"("<<orient<<")"<<std::endl;
		V[index]++;
		ErrorOrientations(i/2)=index;
	}

	/*
	std::cout<<Mat(V)<<std::endl;
	std::cout<<ErrorOrientations<<std::endl;
	*/
	double maxValue;
	Point maxLoc;

	minMaxLoc(Mat(V),NULL,&maxValue,NULL,&maxLoc);

	//std::cout<<"Mat at "<<maxLoc.x<<","<<maxLoc.y<<std::endl;

	for(int i=0 ; i<rows/2 ; i++)
	{
		int loc = ErrorOrientations(i)-maxLoc.y;
		if(loc == 0 )
			H[i]=(1);
		else
			H[i]=(0);
	}

	return H;

}

#endif /* DATAASSOCIATION_HPP_ */
