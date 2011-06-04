/*
 * kdtree.hpp
 *
 *  Created on: May 4, 2011
 *      Author: yasir
 */

#ifndef KDTREE_HPP_
#define KDTREE_HPP_

#include <ZGZ.hpp>
/*
 * kdtree.hpp
 *
 *  Created on: May 4, 2011
 *      Author: yasir
 */
/**
 * KD tree implementation for 3d Points
 */


template <typename node> //!< Type of node ( Eigen Vector3f or any struct that provide [] operator
class KDtree{
private:
	KDtree * head;
	node data;
	KDtree * left;
	KDtree * right;
	int depth;
	static int _dims;

	bool insert(KDtree*& head, node N, int depth, float thresh, node& NN){
		if(head == NULL)
		{
			head = new KDtree();
			head -> data = N;
			head -> depth = depth;
			NN = N;
			return false;
		}
		else
		{
			if(distance(head->data,N) < thresh)
			{
				//std::cerr<<" thresh check "<<std::endl;
				NN = head->data;
				return true;
			}
			if(isSmaller(head->data,N,depth))
			{
				return insert(head->left,N,depth+1,thresh,NN);
			}
			else
			{
				return insert(head->right,N,depth+1,thresh,NN);
			}
		}
	}


	void show(KDtree* head)
	{
		if(head==NULL)
			return;
		else
		{
			show(head->left);
			std::cout<< "["<<head->depth<<"] [";
			std::cout<<head->data[0]<<" , ";
			std::cout<<head->data[1]<<" , ";
			std::cout<<head->data[2]<<" ]"<<std::endl;
			show(head->right);
		}
	}
	/*This implementation is just looking for the first node taht we may find that is less the
	 * the required distance "d" given. We are not looking the best Nearest neighbour
	 * */
	/*
	bool NNd(node& N,KDTree*& head,float d, int depth, float& best, KDTree*& bestNode)
	{
		if(head->left == NULL && head->right == NULL)
		{
			// Leaf node
			float dist = distance(N,head->data);
			bestNode = head;
			best = dist;
			if(dist < d){
				return true;
			}
			else return false;
		}
		else // keep going till we find the leaf node
		{
			if(head->data[depth%_dims] < N[depth%_dims])
			{
				return NNd(N,head->left,d,depth+1,best,bestNode);
			}
			else
			{
				return NNd(N,head->left,d,depth+1,best,bestNode);
			}
		}
		float dist = distance(N,head->data);
		if(dist < best)
		{
			best = dist;
		}

		if(head->left!=NULL){
			float r = head->left->data[depth%_dims]-N[depth%_dims];
			r = r*r;
			if(r<best)
			{
				return NNd(N,head->left,d,depth+1,best,bestNode);
			}
		}
		if(head->left!=NULL){
			float r = head->right->data[depth%_dims]-N[depth%_dims];
			r = r*r;
			if(r<best)
			{
				return NNd(N,head->right,d,depth+1,best,bestNode);
			}
		}
		return false;

	}
	*/

	float distance(Eigen::Vector3f& N1, Eigen::Vector3f& N2)
	{
		return (N1-N2).squaredNorm();
	}


	float distance(cv::Point3d N1, cv::Point3d N2)
	{
		return (N1.x - N2.x)*(N1.x - N2.x)+(N1.x - N2.x)*(N1.x - N2.x)+(N1.x - N2.x)*(N1.x - N2.x);
	}


	bool isSmaller( cv::Point3d& data, cv::Point3d& N, int depth)
	{

		switch (depth%3){
			case 0:	return N.x< data.x; break;
			case 1: return N.y< data.y; break;
			case 2: return N.z< data.z; break;
		}
		return false;
	}


	bool isSmaller( Eigen::Vector3f & data, Eigen::Vector3f & N, int depth)
	{
		return N[depth%3]<data[depth%3];
	}
public:
	KDtree(): head(NULL), left(NULL),right(NULL) , depth(0) {}
	void show()
	{
		show(head);
	}

	bool insert(node N, float thresh, node& NN)
	{
		return insert(head,N,0,thresh,NN);
	}

	/**
	 * Nearest neighbour within the distance d
	 * @param d : distance (squared)
	 */
	/*
	float NNd(node N,float d)
	{
		float best = 1e5;
		KDTree* bestNode;
		//bool NNd(node& N,KDTree*& head,float d, int depth, int &best, KDTree*& bestNode)
		if(NNd(N,head,d,0,best,bestNode))
			return best;
		else
			return -1;
	}
	*/


};


template <typename node>
int KDtree< node >::_dims = 3;

#define pKDTree KDtree< cv::Point3d >
#define vKDTree KDtree< Eigen::Vector3f >


#endif /* KDTREE_HPP_ */
