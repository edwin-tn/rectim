#pragma once
/* a class to rectify oblique image into vertical using collinearity transformation                    */
/* Source: Introduction to modern photogrammetry by Edward M. Mikhail pp87-88, 2001                    */
/* Two types of vertical images can be created:                                                        */
/* 1. a new vertical image with an equal resolution but different pixel size with the tilted image, or */
/* 2. a new vertical image with an equal pixel size but different resolution with the tilted image     */
/* Created by Martinus Edwin Tjahjadi, January 2021                                                    */

#include <math.h>
#include<corecrt_math_defines.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <vector>
#include <queue>
#include "InputData.h"
#include "rotations.h"

using namespace std;
using namespace cv;

class collinearRect
{
public:
	collinearRect();
	~collinearRect();
	collinearRect(const double& omega, const double& phi, const double& kappa, const string& imagefile); //rotations are in degrees

	/*rectify image 4 corner points to become vertical image's 4 corner points*/
	void rectifyPoints();
	/*warp vertical image */
	void warpVerticalImage();

	rotations getRotation() { return opk; };

private:
	//Input corner points
	vector<Point2d> srccorners;		//4 corners points of source image (oblique image) in metric unit (mm) - toplelt origin
	vector<Point2d> pxsrccorners;	//4 corners points of source image (oblique image) in pixels - topleft origin
	vector<Point2d> austsrccorners;	//4 corners points of source image (oblique image) in Australis system (mm) - middle/centre origin
	vector<Point2d> pxaustsrccorners;	//4 corners points of source image (oblique image) in Australis system (pixel) top left origin

	//Output corner points
	vector<Point2d> dstcorners;		//4 corners points of destination image (oblique image) in metric unit (mm) - toplelt origin
	vector<Point2d> pxdstcorners;	//4 corners points of destination image (oblique image) in pixels - topleft origin
	vector<Point2d> austdstcorners;	//4 corners points of destination image (oblique image) in Australis system (mm) - middle/centre origin
	vector<Point2d> pxaustdstcorners;	//4 corners points of destination image (oblique image) in Australis system (pixel) top left origin

	vector<Point2d> pxdstequal;	//4 corners points of destination image for equal pixel size solution

	rotations  opk;	//known rotation angles: omega phi kappa
	string imagepathfile;	//location and image file name
	//bool bLowobliqueimage;	//true if low oblique image

	/*Homography from source (tilted) image to destination (vertical) image*/
	Mat Hpx;	//Homography calculated using image corners in pixels, for equal resolutions
	Mat Hunit;	//Homography calculated using image corners in unit mm, for equal resolutions
	Mat Hsize;	//Homography calculated using image corners in pixels, for equal pixel zize different resolutions
	Size widthheight;	//size of a new equal pixel size image

	Matx33d setRots_opk_o2i();	//compose a rotation matrix omega-phi-kappa from object to camera system
	void printCorners(const vector<Point2d> &pts);
	void printCorners(const vector<Point3d> &pts);
	Point3d imagetocameracoord(Point2d pt);
	
	/*new image dimension must have an equal pixel size or an equal resolutions, not both */
	vector<Point2d> createImageDim_pixelsizeEq(int& cols, int& rows);	//return new image's width and height
	//Size2d createImageDim_resolutionEq();	//return new image's pixel size int x and y 
	Point2d unittopixel(Point3d pt3);	//convert aust mm to pixel
	Point2d unittopixel(Point2d pt2);	//convert aust mm to pixel
	Point2d pixeltounit(Point2d pt2);	//convert aust pixel to unit
	Point2d pixeltounit(size_t col, size_t row);	//convert aust pixel to unit

	/*Homography matrix from source to destination image plan */
	Mat getPerspectiveTransformbetweentwoimages(vector<Point2d>& src, vector<Point2d>& dst);	//use cv::getPerspectiveTransform
	Mat getHomographybetweentwoimages(vector<Point2d>& src, vector<Point2d>& dst);				//use cv::findHomography

	void decomposeH_PolarTransform(const Mat H, Mat3d& Rs, Vec3d& ts, string str);
	double degreesToRadians(const double angleDegrees) { return (angleDegrees * M_PI / 180.0); }
	double radiansToDegrees(const double angleRadians) { return (angleRadians * 180.0 / M_PI); }
	//get wpk in radians
	void wpk_in_rads(const cv::Matx33d& R, double& omega, double& phi, double& kappa);


};

collinearRect::collinearRect() {}

collinearRect::~collinearRect() {}

collinearRect::collinearRect(const double& degomega, const double& degphi, const double& degkappa, const string& imagefile)
{
	opk.setOmega(degomega);	opk.setPhi(degphi);	opk.setKappa(degkappa);
	opk.rotAts();
	
	//set inputs
	int corners = 4;
	srccorners.clear();	pxsrccorners.clear();	austsrccorners.clear();		pxaustsrccorners.clear();
	dstcorners.clear();	pxdstcorners.clear();	austdstcorners.clear();		pxaustdstcorners.clear();
	Hpx.empty();	Hunit.empty();	Hsize.empty();
	pxdstequal.clear();
	widthheight.empty();
	imagepathfile = imagefile;
	//bLowobliqueimage = lowobq;


	for (size_t i{}; i < corners; ++i)
	{
		/* sequences: top-left(tl), top-right(tr), bottom-right(br), bottom-left(bl) in pixels(px) */
		Point2d pxsrc = rectim::pxCorners[i];
		pxsrccorners.push_back(pxsrc);

		/*Coordinate Origin: Top Left image coordinate origin  in mm*/
		Point2d src = rectim::mmCorners[i];
		srccorners.push_back(src);

		/*Australis Coords system: Middle image coordinate origin  in mm*/
		Point2d aust = rectim::mmAust[i];
		austsrccorners.push_back(aust);

		Point2d pxAust = rectim::mmAustCheck[i];
		pxaustsrccorners.push_back(pxAust);
	}

	//print corners
	cout << " corners in mm (srccorners):\n";
	printCorners(srccorners);

	cout << " corners in pixel (pxsrccorners):\n";
	printCorners(pxsrccorners);

	cout << " corners in australis system (mm) (austsrccorners):\n";
	printCorners(austsrccorners);

	cout << " corners in australis system (pixel) (pxaustsrccorners):\n";
	printCorners(pxaustsrccorners);

}

void collinearRect::printCorners(const vector<cv::Point2d> &pts)
{
	size_t counts = pts.size();
	for (size_t i{}; i < counts; ++i)
	{
		cout << pts[i] << "\t";
	}

	cout << endl << endl;
}

void collinearRect::printCorners(const vector<cv::Point3d> &pts)
{
	size_t counts = pts.size();
	for (size_t i{}; i < counts; ++i)
	{
		cout << pts[i] << "\t";
	}

	cout << endl << endl;
}


/*rectify image 4 corner points to become vertical image's 4 corner points*/
void collinearRect::rectifyPoints()
{
	// set a rotation matrix object to camera
	Matx33d rot = opk.rotationOmegaPhiKappa_object2camera();

	//assume that the vertical image has equal focal length with tilted image and
	//coordinates system are accroding to the vertical image axes
		
	//convert to camera coordinate system
	vector<Point3d> pts;	
	for (size_t i{}; i < 4; ++i) {
		Point3d pt = imagetocameracoord(this->austsrccorners[i]);
		pts.push_back(pt);
	}

	//for (size_t i{}; i < 4; ++i)
	//{
	//	Point3d pt = imagetocameracoord(srccorners[i]);
	//	srcpts.push_back(pt);
	//	//pt = imagetocameracoord(austsrccorners[i]);
	//	//austpts.push_back(pt);
	//}
	//cout << "corners of source image in camera coords (topleft in mm):\n";
	//printCorners(srcpts);
	//printCorners(austpts);

	//transform camera space to object space system
	vector<Point3d> dst;
	//vector<Point2d> pxpts;
	for (size_t i{}; i < 4; ++i)
	{
		//transformation from vertical to tillted image
		//assuming XL=YL=ZL=0
		Point3d uvw = rot * pts[i];
		double scale = -rectim::focal / uvw.z;
		uvw = uvw * scale;
		dst.push_back(uvw);
		Point2d temp;
		temp.x = uvw.x;
		temp.y = uvw.y;
		austdstcorners.push_back(temp);

		// convert to pixel
		Point2d pt2 = unittopixel(dst[i]);
		pxaustdstcorners.push_back(pt2);

	//	//save result
	//	Point2d pt2;
	//	pt2.x = uvw.x;	pt2.y = uvw.y;
	//	dstcorners.push_back(pt2);
	}
	cout << "3D corners of destination image in camera coords (aust in mm):\n";
	printCorners(dst);
	cout << "2D corners of destination image in camera coords (aust in mm) (austdstcorners):\n";
	printCorners(austdstcorners);
	cout << "corners of destination image in  (aust in pixel) (pxaustdstcorners):\n";
	printCorners(pxaustdstcorners);

	//determine new dimension of new image
	//for equal pixel size
	//cv::Size wh;
	pxdstequal = createImageDim_pixelsizeEq(widthheight.width, widthheight.height);
	cout << "new corners in pixels for equal pixel size (pxdstequal):\n";
	printCorners(pxdstequal);
	cout << "size of new equal pixel size image dimension, width: " << widthheight.width << "\t and height: " << widthheight.height << "\t pixels" << endl;
	
	//Hsize = getPerspectiveTransformbetweentwoimages(pxsrccorners, pxdstequal);
	Hsize = getPerspectiveTransformbetweentwoimages(pxsrccorners, pxdstequal);
	cout << "\nH for equal pixel size Hsize:\n" << Hsize << endl;

	//Compare getPerspectiveTransform and findHomograpgy for pixels
	//Hpx = getPerspectiveTransformbetweentwoimages(pxaustdstcorners, pxaustsrccorners);
	Hpx = getPerspectiveTransformbetweentwoimages(pxaustsrccorners, pxaustdstcorners);
	cout << "\nHpx from getPerspectiveTransform in pixel:\n" << Hpx << endl;

	//Compare getPerspectiveTransform and findHomograpgy for mm
	Hunit = getPerspectiveTransformbetweentwoimages(austsrccorners, austdstcorners);
	//Hunit = getPerspectiveTransformbetweentwoimages(austdstcorners, austsrccorners);
	cout << "\nHunit from getPerspectiveTransform in mm:\n" << Hunit << endl;

	//Now, Decompose Homography to calculate Rotations and Translation between Tilted Image (origin or soure) 
	//into the vertical image (destination)
	// 1. Decomposing Homography of Hpx
	//cout << "\n\t\t\t Decomposing Homography of Hpxperspective: \n";
	cv::Mat3d Rs1;			cv::Vec3d ts1;
	decomposeH_PolarTransform(Hpx, Rs1, ts1, "\n\t\t\t Decomposing Homography of equal resolution Hpx: \n");

	// 2. Decomposing Homography of Hunit
	cv::Mat3d Rs2;			cv::Vec3d ts2;
	decomposeH_PolarTransform(Hunit, Rs2, ts2, "\n\t\t\t Decomposing Homography of equal resolution Hunit: \n");

	// 3. Decomposing Homography of Hsize
	cv::Mat3d Rs3;			cv::Vec3d ts3;
	decomposeH_PolarTransform(Hsize, Rs3, ts3, "\n\t\t\t Decomposing Homography of equal pixel size Hsize: \n");

}

inline void collinearRect::warpVerticalImage()
{
	//rectify  the vertical image using Homogrphic matrices
	Mat img = imread(imagepathfile, IMREAD_COLOR);
	if (img.empty())                      // Check for invalid input
	{
		std::cout << "Could not open or find the image" << std::endl;
		return;
	}

	//perform direct and indirect(inverse) projective rectification of low oblique image
	Mat dstpx, dstunit, dstunitinverse, dstsize;
	//Mat m = Mat::zeros(3797, 4951, CV_8UC3);
	Mat m = Mat::zeros(widthheight, CV_8SC3);
	//cv::Size sz(4951, 3797);
	warpPerspective(img, dstpx, Hpx, img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
	warpPerspective(img, dstunit, Hunit, img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
	warpPerspective(img, dstunitinverse, Hunit, img.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar());
	warpPerspective(img, dstsize, Hsize, m.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

	// Display source Image
	cv::namedWindow("Source Image", cv::WINDOW_KEEPRATIO);
	cv::imshow("Source Image", img);                // Show our image inside it.

	/* Display vertical images: */
	cv::namedWindow("Vertical Image with Hpx", cv::WINDOW_KEEPRATIO);
	cv::imshow("Vertical Image with Hpx", dstpx);                // Show our image inside it.

	cv::namedWindow("Vertical Image with Hunit", cv::WINDOW_KEEPRATIO);
	cv::imshow("Vertical Image with Hunit", dstunit);                // Show our image inside it.

	cv::namedWindow("Vertical Image with Hunit inverse", cv::WINDOW_KEEPRATIO);
	cv::imshow("Vertical Image with Hunit inverse", dstunitinverse);                // Show our image inside it.

	cv::namedWindow("Vertical Image with Hsize", cv::WINDOW_KEEPRATIO);
	cv::imshow("Vertical Image with Hsize", dstsize);                // Show our image inside it.

	/* Save all images */
	imwrite("Image Hpx.jpg", dstpx);
	imwrite("Image Hunit.jpg", dstunit);
	imwrite("Image HunitInverse.jpg", dstunitinverse);
	imwrite("Image Hsize.jpg", dstsize);

}

Point3d collinearRect::imagetocameracoord(Point2d pt)
{
	Point3d pt3;
	pt3.x = pt.x;	pt3.y = pt.y;	pt3.z = -rectim::focal;
	return pt3;
}

//with equal pixel size, the dimension (resolutions) of a new image might differ
vector<Point2d> collinearRect::createImageDim_pixelsizeEq(int& cols, int& rows)
{
	//Do it in aust coords system
	Size dim; dim.empty();
	double xmin, xmax, ymin, ymax;
	vector<Point2d> corners;	corners.clear();

	xmin = std::min(min(austdstcorners[0].x, austdstcorners[1].x), min(austdstcorners[2].x, austdstcorners[3].x));
	xmax = std::max(max(austdstcorners[0].x, austdstcorners[1].x), max(austdstcorners[2].x, austdstcorners[3].x));
	ymin = std::min(min(austdstcorners[0].y, austdstcorners[1].y), min(austdstcorners[2].y, austdstcorners[3].y));
	ymax = std::max(max(austdstcorners[0].y, austdstcorners[1].y), max(austdstcorners[2].y, austdstcorners[3].y));
	double newidth = std::abs(xmax - xmin);
	double neheight = std::abs(ymax - ymin);
	cols = (int) ((newidth / rectim::pixelsizeHorizontal) + 0.5);
	rows = (int) ((neheight / rectim::pixelsizeVertical) + 0.5);

	//determine corners coordinates of new image
	Point2d topleft, topright, bottomright, bottomleft;
	topleft.x = 0.0;				topleft.y = 0.0;
	topright.x = cols - 1.0;		topright.y = 0.0;
	bottomright.x = cols - 1.0;	bottomright.y = rows - 1.0;
	bottomleft.x = 0.0;				bottomleft.y = rows - 1.0;
	corners.push_back(topleft);
	corners.push_back(topright);
	corners.push_back(bottomright);
	corners.push_back(bottomleft);

	return corners;
}

//Size2d collinearRect::createImageDim_resolutionEq()
//{
//	Size2d dim; dim.empty();
//	vector<double> vecx, vecy;
//	double xmin, xmax, ymin, ymax;
//	for (size_t i{}; i < 4; ++i)
//	{
//		vecx.push_back(dstcorners[i].x);
//		vecy.push_back(dstcorners[i].y);
//	}
//
//	xmin = std::min(min(dstcorners[0].x, dstcorners[1].x), min(dstcorners[2].x, dstcorners[3].x));
//	xmax = std::max(max(dstcorners[0].x, dstcorners[1].x), max(dstcorners[2].x, dstcorners[3].x));
//	ymin = std::min(min(dstcorners[0].y, dstcorners[1].y), min(dstcorners[2].y, dstcorners[3].y));
//	ymax = std::max(max(dstcorners[0].y, dstcorners[1].y), max(dstcorners[2].y, dstcorners[3].y));
//	double newidth = std::abs(xmax - xmin);
//	double neheight = std::abs(ymax - ymin);
//	dim.width = newidth / rectim::pixelWidth;
//	dim.height = neheight / rectim::pixelHeight;
//
//	return dim;
//
//}

inline Point2d collinearRect::unittopixel(Point3d pt3)
{
	Point2d pt;
	pt.x = pt3.x / rectim::pixelsizeHorizontal + rectim::halfpixelWidth;	//col
	pt.y = rectim::halfpixelHeight - (pt3.y / rectim::pixelsizeVertical);	//row

	return pt;
}

inline Point2d collinearRect::unittopixel(Point2d pt2)
{
	Point2d pt;
	pt.x = pt2.x / rectim::pixelsizeHorizontal + rectim::halfpixelWidth;	//col
	pt.y = rectim::halfpixelHeight - (pt2.y / rectim::pixelsizeVertical);	//row

	return pt;
}

inline Point2d collinearRect::pixeltounit(Point2d pt2)
{
	Point2d pt;
	pt.x = (pt2.x - rectim::halfpixelWidth) * rectim::pixelsizeHorizontal;	//col to x
	pt.y = (rectim::halfpixelHeight - pt2.y) * rectim::pixelsizeVertical;	//row to y

	return pt;
}

inline Point2d collinearRect::pixeltounit(size_t col, size_t row)
{
	Point2d pt;
	pt.x = (col - rectim::halfpixelWidth) * rectim::pixelsizeHorizontal;	//col to x
	pt.y = (rectim::halfpixelHeight - row) * rectim::pixelsizeVertical;	//row to y

	return pt;
}

inline Mat collinearRect::getPerspectiveTransformbetweentwoimages(vector<Point2d>& src, vector<Point2d>& dst)
{
	Point2f srcQuad[] = {
		Point2f((float) src[0].x, (float) src[0].y),
		Point2f((float) src[1].x, (float) src[1].y),
		Point2f((float) src[2].x, (float) src[2].y),
		Point2f((float) src[3].x, (float) src[3].y)
	};

	Point2f dstQuad[] = {
		Point2f((float) dst[0].x, (float) dst[0].y),
		Point2f((float) dst[1].x, (float) dst[1].y),
		Point2f((float) dst[2].x, (float) dst[2].y),
		Point2f((float) dst[3].x, (float) dst[3].y)
	};

	Mat m = getPerspectiveTransform(srcQuad, dstQuad);
	
	return m;
}

inline Mat collinearRect::getHomographybetweentwoimages(vector<Point2d>& src, vector<Point2d>& dst)
{
	Point2f srcQuad[] = {
		Point2f((float)src[0].x, (float)src[0].y),
		Point2f((float)src[1].x, (float)src[1].y),
		Point2f((float)src[2].x, (float)src[2].y),
		Point2f((float)src[3].x, (float)src[3].y)
	};

	Point2f dstQuad[] = {
		Point2f((float)dst[0].x, (float)dst[0].y),
		Point2f((float)dst[1].x, (float)dst[1].y),
		Point2f((float)dst[2].x, (float)dst[2].y),
		Point2f((float)dst[3].x, (float)dst[3].y)
	};

	Mat m = findHomography(src, dst);

	return Mat();
}

void collinearRect::decomposeH_PolarTransform(const Mat H, Mat3d& Rs, Vec3d& ts, string str)
{
	//using namespace std;
	//using namespace cv;

	//cout << "\n\t\t ============== Polar Decomposition in Function decomposeH_PolarTransform() ============\n";
	cout << str;

	double mag = H.at<double>(0, 0)*H.at<double>(0, 0) + H.at<double>(1, 0)*H.at<double>(1, 0) + H.at<double>(2, 0)*H.at<double>(2, 0);

	//normalize to ensure that ||c1|| = 1
	double norm = sqrt(mag);
	//double norm = cv::sqrt(_H.at<double>(0, 0)*_H.at<double>(0, 0) +
	//				       _H.at<double>(1, 0)*_H.at<double>(1, 0) +
	//				       _H.at<double>(2, 0)*_H.at<double>(2, 0));

	cout << "magnitude = " << mag << "\t\tnorm  = " << norm << endl;

	Mat _H;
	if (norm != 1.00)
		_H = H / norm;
	else
		_H = H;

	cout << "H before normalization: \n" << H << endl;
	cout << "H AFTER NORMALIZATION: \n" << _H << endl;

	Mat c1 = _H.col(0);
	//cout << "c1: \n" << c1 << endl;

	Mat c2 = _H.col(1);
	//cout << "c2: \n" << c2 << endl;

	Mat c3 = c1.cross(c2);
	//cout << "c3: \n" << c3 << endl;

	Mat T = _H.col(2);
	cout << "Translation: \n" << T << endl;

	//Normalized translation
	double w = T.at<double>(2);
	T /= w;
	cout << "Normalized Translation: \n" << T << endl;

	Mat R(3, 3, CV_64F);

	for (int i = 0; i < 3; i++)
	{
		R.at<double>(i, 0) = c1.at<double>(i, 0);
		R.at<double>(i, 1) = c2.at<double>(i, 0);
		R.at<double>(i, 2) = c3.at<double>(i, 0);
	}

	cout << "R before polar decomposition: \n" << R << "  \ndet = " << cv::determinant(R) << endl;

	Mat W, U, Vt;
	SVDecomp(R, W, U, Vt);
	R = U * Vt;
	cout << "R after polar decomposition: \n" << R << "  \ndet = " << cv::determinant(R) << endl;
	Rs = R;
	ts = T;

	//compute angles
	Mat rvec;
	Rodrigues(R, rvec);
	cout << "Rotation angles: \n" << rvec << endl;

	double o, p, k;
	wpk_in_rads(R, o, p, k);
	cout << "\n omega phi kappa in rads: \t" << o << "\t" << p << "\t" << k << endl;
	cout << "\n omega phi kappa in degs: \t" << radiansToDegrees(o) << "\t" << radiansToDegrees(p) << "\t" << radiansToDegrees(k) << endl;
	cout << "\n Rodriguez in degs: \t" << radiansToDegrees(rvec.at<double>(0)) << "\t" << radiansToDegrees(rvec.at<double>(1)) << "\t" << radiansToDegrees(rvec.at<double>(2)) << endl;

	//current tilt
	double tilt = radiansToDegrees(acos((R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2) - 1.0) / 2.0));
	cout << "\nCurrent tilt of image: " << tilt << "\tdegrees" << endl;
}

//get wpk in radians
inline void collinearRect::wpk_in_rads(const cv::Matx33d & R, double & omega, double & phi, double & kappa)
{
	phi = asin(R(2, 0));	//phi in radians
	omega = atan2(-R(2, 1), R(2, 2));	//omega in radians
	kappa = atan2(-R(1, 0), R(0, 0));	//kappa in radians

}




