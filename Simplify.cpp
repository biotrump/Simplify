/**
 * @url https://github.com/mourner/simplify-js
 */
#include <stdio.h>
#include <stdlib.h>

#include <vector>
#include <algorithm>
using namespace std;

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
using namespace cv;
#endif

/**
 * p1[0] = x, p1[1] = y
 * p2[0] = x, p2[1] = y
 */
static float getSquareDistance(float p1[], float p2[]) {
	float dx = p1[0] - p2[0];
	float dy = p1[1] - p2[1];
	return dx * dx + dy * dy;
}


// square distance from a point to a segment
static float getSquareSegmentDistance(float p[], float p1[],
		float p2[]) {
	float x = p1[0], y = p1[1];

	float dx = p2[0] - x, dy = p2[1] - y;

	float t;

	if (dx != 0 || dy != 0 ) {
		t = ((p[0] - x) * dx + (p[1] - y) * dy) 
				/ (dx * dx + dy * dy);

		if (t > 1) {
			x = p2[0];
			y = p2[1];

		} else if (t > 0) {
			x += dx * t;
			y += dy * t;
		}
	}

	dx = p[0] - x;
	dy = p[1] - y;

	return dx * dx + dy * dy ;
}

// distance-based simplification
// len : the total points. A point is a (x,y) pair
static std::vector<float> simplifyRadialDistance(float (*points)[2], int len,
		float sqTolerance) {
	std::vector<float> newPoints;
	if(len==0)
		return newPoints;

	float *point=NULL;//point (x,y)
	float *prevPoint = points[0];

	printf("newPoints.push:%f,%f\n", prevPoint[0], prevPoint[1]);
	newPoints.push_back(prevPoint[0]);//push x
	newPoints.push_back(prevPoint[1]);//push y

	for (int i = 1; i < len; i++) {
		point = points[i];//point_i
		//printf("%s:%d (%f,%f)\n",__func__, i, point[0], point[1]);
		if (getSquareDistance(point, prevPoint) > sqTolerance) {
			newPoints.push_back(point[0]);//x
			newPoints.push_back(point[1]);//y
			prevPoint = point;
			printf("newPoints.push:%zu (%f,%f)\n",newPoints.size()/2, point[0], point[1]);
		}
	}
	printf("point=%p, (x,y)=(%f,%f)\n",point, point[0], point[1]);
	printf("prevPoint=%p, (x,y)=(%f,%f)\n",prevPoint, prevPoint[0], prevPoint[1]);
	if(point){
		if (prevPoint[0] != point[0] ||
			prevPoint[1] != point[1]) {
			newPoints.push_back(point[0]);
			newPoints.push_back(point[1]);
		}
	}
	return newPoints;
}

// simplification using optimized Douglas-Peucker algorithm with recursion
// elimination
// len : the elements of points[len][]
static std::vector<float> simplifyDouglasPeucker(std::vector<float> &input ,
		float sqTolerance) {
	float (*points)[2] = (float (*)[2])&input[0];
	int len=input.size()/2;	//pair(x,y) is a point
	int markers[len];

	int first = 0;
	int last = len - 1;

	float maxSqDist;
	float sqDist;
	int index = 0;

	std::vector<int> firstStack;
	std::vector<int> lastStack;
	std::vector<float> newPoints;

	markers[first] = markers[last] = 1;

	while (last ) {
		maxSqDist = 0;

		for (int i = first + 1; i < last; i++) {
			sqDist = getSquareSegmentDistance(points[i], points[first],
					points[last]);

			if (sqDist > maxSqDist) {
				index = i;
				maxSqDist = sqDist;
			}
		}

		if (maxSqDist > sqTolerance) {
			markers[index] = 1;

			firstStack.push_back(first);
			lastStack.push_back(index);

			firstStack.push_back(index);
			lastStack.push_back(last);
		}

		if (firstStack.size() >1 ){
			first = firstStack.back();
			firstStack.pop_back();
		}else
			first = 0;

		if (lastStack.size() >1){
			last = lastStack.back();
			lastStack.pop_back();
		}else
			last = 0;
	}

	for (int i = 0; i < len; i++) {
		if (markers[i]){
			newPoints.push_back(points[i][0]);
			newPoints.push_back(points[i][1]);
		}
	}

	return newPoints;
}

/**
 * points : points[len][2]  for 2D points
 * len :total points, a point is a (x,y) pair
 */
std::vector<float> simplify(float (*points)[2], int len, float tolerance, int highestQuality) 
{
	float sqTolerance = tolerance * tolerance;
	std::vector<float> outPoints, tmpPoints;
	if(len <=0)
		return outPoints;	//empty
	else if (len == 1){
		//only 1 point
		outPoints.push_back(points[0][0]);//push x
		outPoints.push_back(points[0][1]);//push y
		return outPoints;
	}
		
	if (!highestQuality)//no, high quality, simplify the points
		tmpPoints = simplifyRadialDistance(points, len, sqTolerance);
	printf(">%s:tmpPoints size=%zu\n",__func__, tmpPoints.size());
	outPoints = simplifyDouglasPeucker(tmpPoints, sqTolerance);
	printf("<%s:outPoints size=%zu\n",__func__, outPoints.size());
	return outPoints;
}

extern "C"{
/**
 * @param[IN] points : points[len][2]  for 2D points
 * @param[IN] len :total points, a point is a (x,y) pair
 * @param[IN/OUT] output: actually it's output[len][2].
 * @return : the new simplified points
 */
int c_simplify(float (*points)[2], int len, float *output, float tolerance, int highestQuality) 
{
	std::vector<float> out= simplify(points, len, tolerance, highestQuality);
	if(out.empty())
		return 0;
	std::copy(out.begin(), out.end(), output);
	return out.size();
}

}

int main(int argc, char **argv)
{
	float points[][2] = {
    {224.55,250.15},{226.91,244.19},{233.31,241.45},{234.98,236.06},
    {244.21,232.76},{262.59,215.31},{267.76,213.81},{273.57,201.84},
    {273.12,192.16},{277.62,189.03},{280.36,181.41},{286.51,177.74},
    {292.41,159.37},{296.91,155.64},{314.95,151.37},{319.75,145.16},
    {330.33,137.57},{341.48,139.96},{369.98,137.89},{387.39,142.51},
    {391.28,139.39},{409.52,141.14},{414.82,139.75},{427.72,127.30},
    {439.60,119.74},{474.93,107.87},{486.51,106.75},{489.20,109.45},
    {493.79,108.63},{504.74,119.66},{512.96,122.35},{518.63,120.89},
    {524.09,126.88},{529.57,127.86},{534.21,140.93},{539.27,147.24},
    {567.69,148.91},{575.25,157.26},{580.62,158.15},{601.53,156.85},
    {617.74,159.86},{622.00,167.04},{629.55,194.60},{638.90,195.61},
    {641.26,200.81},{651.77,204.56},{671.55,222.55},{683.68,217.45},
    {695.25,219.15},{700.64,217.98},{703.12,214.36},{712.26,215.87},
    {721.49,212.81},{727.81,213.36},{729.98,208.73},{735.32,208.20},
    {739.94,204.77},{769.98,208.42},{779.60,216.87},{784.20,218.16},
    {800.24,214.62},{810.53,219.73},{817.19,226.82},{820.77,236.17},
    {827.23,236.16},{829.89,239.89},{851.00,248.94},{859.88,255.49},
    {865.21,268.53},{857.95,280.30},{865.48,291.45},{866.81,298.66},
    {864.68,302.71},{867.79,306.17},{859.87,311.37},{860.08,314.35},
    {858.29,314.94},{858.10,327.60},{854.54,335.40},{860.92,343.00},
    {856.43,350.15},{851.42,352.96},{849.84,359.59},{854.56,365.53},
    {849.74,370.38},{844.09,371.89},{844.75,380.44},{841.52,383.67},
    {839.57,390.40},{845.59,399.05},{848.40,407.55},{843.71,411.30},
    {844.09,419.88},{839.51,432.76},{841.33,441.04},{847.62,449.22},
    {847.16,458.44},{851.38,462.79},{853.97,471.15},{866.36,480.77},
	{224.55,250.15}
	};

	float simplified[][2] = {
    {224.55,250.15},{267.76,213.81},{296.91,155.64},{330.33,137.57},
    {409.52,141.14},{439.60,119.74},{486.51,106.75},{529.57,127.86},
    {539.27,147.24},{617.74,159.86},{629.55,194.60},{671.55,222.55},
    {727.81,213.36},{739.94,204.77},{769.98,208.42},{779.60,216.87},
    {800.24,214.62},{820.77,236.17},{859.88,255.49},{865.21,268.53},
    {857.95,280.30},{867.79,306.17},{859.87,311.37},{854.54,335.40},
    {860.92,343.00},{849.84,359.59},{854.56,365.53},{844.09,371.89},
    {839.57,390.40},{848.40,407.55},{839.51,432.76},{853.97,471.15},
    {866.36,480.77}
	};

	int len=0;
	float tolerance=15.0f;
	int highestQuality=0;

	if(argc > 1){
		printf("argv[1]:%s\n", argv[1]);
		sscanf(argv[1],"%f", &tolerance);
		printf("tolerance=%f\n", tolerance);
	}
	std::vector<float> simPoints = simplify(points, len=0, tolerance, highestQuality) ;
	printf("simplify: len=%d, return points=%zu\n", len, simPoints.size());
	
	simPoints = simplify(points, len=1, tolerance, highestQuality) ;
	printf("simplify: len=%d, return points=%zu, (%f, %f)\n", len, simPoints.size()/2, simPoints[0], simPoints[1]);

	simPoints = simplify(points, len=2, tolerance, highestQuality) ;
	printf("simplify: len=%d, return points=%zu, (%f, %f)\n", len, simPoints.size()/2, simPoints[0], simPoints[1]);

	len=sizeof(points)/(2*sizeof(float));
	simPoints = simplify(points, len, tolerance, highestQuality) ;
	printf("simplify: len=%d, return points=%zu, (%f, %f)\n", len, simPoints.size()/2, simPoints[0], simPoints[1]);

	float output[len][2];
	int olen = c_simplify(points, len, (float *)output, tolerance, highestQuality) ;
	
	int win_height=768, win_width=1024;
	cv::Mat mainWin( win_height, win_width ,CV_8UC3, Scalar(0,0,0));
	for(int i = 0 ; i < len-1 ; i++){
		int x0 = points[i][0];
		int y0 = points[i][1];
		int x1 = points[i+1][0];
		int y1 = points[i+1][1];

		cv::circle(mainWin, cv::Point(x0, y0), 1, Scalar(0,255,0), -1);
		cv::line(mainWin, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,255,0), 1) ;
	}

	for(int i=0; i < simPoints.size()-2; i+=2){
		printf("%d:(x,y)=(%f,%f)\n", i/2, simPoints[i], simPoints[i+1]);
		int x0= simPoints[i];
		int y0= simPoints[i+1];
		int x1= simPoints[i+2];
		int y1= simPoints[i+2+1];

		cv::circle(mainWin, cv::Point(x0, y0), 1, Scalar(0,0, 255), -1);
		cv::line(mainWin, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0,0, 255), 1) ;
	}

	len=sizeof(simplified)/(2*sizeof(float));
	for(int i = 0 ; i < len-1 ; i++){
		int x0 = simplified[i][0];
		int y0 = simplified[i][1];
		int x1 = simplified[i+1][0];
		int y1 = simplified[i+1][1];

		cv::circle(mainWin, cv::Point(x0, y0), 1, Scalar(255,0,0), -1);
		cv::line(mainWin, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(255,0,0), 1) ;
	}

	cv::imshow("simplify", mainWin);
    cv::waitKey(0);

}