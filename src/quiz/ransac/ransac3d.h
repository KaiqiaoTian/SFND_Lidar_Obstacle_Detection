#ifndef _RANSAC3D_H_
#define _RANSAC3D_H_

#include<pcl/common/common.h>
#include<unordered_set>


template<typename PointT>
class RANSAC3D{
public:

static std::unordered_set<int> ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	while(maxIterations--)
	{

		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, y1, x2, y2, x3, y3, z1, z2, z3;

		auto itr = inliers.begin();  //pointer
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float d = -(a*x1+b*y1+c*z1);

			for(int index = 0; index < cloud->points.size(); index++)
			{
				
				if(inliers.count(index)>0)
				continue;

				PointT point = cloud -> points[index];
				float x0 = point.x;
				float y0 = point.y;
				float z0 = point.z;

				float d = fabs(a*x0 + b*y0 + c*z0 + d)/sqrt(a*a+b*b+c*c);

				if(d <= distanceTol)
					inliers.insert(index);

			}

			if(inliers.size()>inliersResult.size())
			{
					inliersResult = inliers;
			}


	}

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}





};
#endif //_RANSAC3D_H_