#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

namespace rectim {
/* Almost vertical image (small tilt angel < 3 deg */
//dji_0091.jpg is a low oblique photo which is almost vertical (nadir) image, and
constexpr auto OMEGA1 = 0.1;	//rotation around X-axes, in degrees
constexpr auto PHI1 = 0.003;	//rotation around Y-axes, in degrees
constexpr auto KAPPA1 = 1.6;	//rotation around Z-axes, in degrees
constexpr auto lowObliqueImage("../mydata/DJI_0091.JPG");	//area 1: low oblique

/* Oblique image (tilt angel < 5 deg */
//dji_0970.jpg (area 3) is a high oblique image which is off-nadir image.
constexpr auto OMEGA2 = 0;		//rotation around X-axes, in degrees
constexpr auto PHI2 = 0;		//rotation around Y-axes, in degrees
constexpr auto KAPPA2 = 0.1;	//rotation around Z-axes, in degrees
constexpr auto lowObliqueImage2("../mydata/DJI_0970.JPG");	//area 3: High oblique

/* Almost vertical image (small tilt angel < 3 deg */
//dji_0701.jpg (area 4) is a low oblique photo which is almost vertical (nadir) image, and
constexpr auto OMEGA3 = 0.1;	//rotation around X-axes, in degrees
constexpr auto PHI3 = 0.1;		//rotation around Y-axes, in degrees
constexpr auto KAPPA3 = 0.6;	//rotation around Z-axes, in degrees
constexpr auto HighObliqueImage2("../mydata/DJI_0701.JPG");	//area 3: High oblique

/*camera perspective center coordinates*/
constexpr auto XL = 0.00;
constexpr auto YL = 0.00;
constexpr auto ZL = 0.00;

/* Camera spesification for both images - DJI Phantom 4 pro*/
constexpr auto focal = 8.8;			//Focal length in mm
constexpr auto ccdWidth = 13.20;	//sensor size width in mm
constexpr auto ccdHeight = 8.8;		//sensor size width in mm
constexpr auto pixelWidth = 4864;	//resolution width in pixel
constexpr auto pixelHeight = 3648;	//resolution height in pixel

constexpr auto pixelsizeHorizontal = ccdWidth / pixelWidth;	//a pixel size in cols
constexpr auto pixelsizeVertical = ccdHeight / pixelHeight;	// a pixel size in rows

/*Australis coordinate system*/
constexpr auto halfpixelWidth = (pixelWidth / 2.0) - 0.5;	//centre x
constexpr auto halfpixelHeight = (pixelHeight / 2.0) - 0.5;	//centre y

/*four image corner points coordinates: Top Left image coordinate origin  in pixels */
/* sequences: top-left(tl), top-right(tr), bottom-right(br), bottom-left(bl) in pixels(px) */
Point2f pxCorners[] = {
	Point2f(0.0f, 0.0f),							// 1. Top left, Origin
	Point2f(pixelWidth - 1.0f, 0.0f),				// 2. Top right
	Point2f(pixelWidth - 1.0f, pixelHeight - 1.0f),	// 3. Bottom right
	Point2f(0.0f ,pixelHeight - 1.0f)				// 4. Bottom left
};

/*four image corner points coordinates:  */
/*Coordinate Origin: Top Left image coordinate origin  in mm*/
/* sequences: top-left(tl), top-right(tr), bottom-right(br), bottom-left(bl) in pixels(px) */
Point2f mmCorners[] = {
	Point2f(0.0f, 0.0f),	// 1. Top left, Origin
	Point2f(13.2f, 0.0f),	// 2. Top right
	Point2f(13.2f, 8.8f),	// 3. Bottom right
	Point2f(0.0f, 8.8f)		// 4. Bottom left
};

/*four image corner points coordinates:  */
/*Australis Coords system: Middle image coordinate origin  in mm*/
/* sequences: top-left(tl), top-right(tr), bottom-right(br), bottom-left(bl) in pixels(px) */
Point2d mmAust[] = {
	Point2d((pxCorners[0].x - halfpixelWidth) * pixelsizeHorizontal, (halfpixelHeight - pxCorners[0].y) * pixelsizeVertical),	// 1. Top left
	Point2d((pxCorners[1].x - halfpixelWidth) * pixelsizeHorizontal, (halfpixelHeight - pxCorners[1].y) * pixelsizeVertical),	// 2. Top right
	Point2d((pxCorners[2].x - halfpixelWidth) * pixelsizeHorizontal, (halfpixelHeight - pxCorners[2].y) * pixelsizeVertical),	// 3. Bottom right
	Point2d((pxCorners[3].x - halfpixelWidth) * pixelsizeHorizontal, (halfpixelHeight - pxCorners[3].y) * pixelsizeVertical)	// 4. Bottom left
};

/*check Australis coordinate system, in pixels*/
Point2d mmAustCheck[] = {
	Point2d((mmAust[0].x / pixelsizeHorizontal) + halfpixelWidth, halfpixelHeight - (mmAust[0].y / pixelsizeVertical)),	// 1. Top left
	Point2d((mmAust[1].x / pixelsizeHorizontal) + halfpixelWidth, halfpixelHeight - (mmAust[1].y / pixelsizeVertical)),	// 2. Top right
	Point2d((mmAust[2].x / pixelsizeHorizontal) + halfpixelWidth, halfpixelHeight - (mmAust[2].y / pixelsizeVertical)),	// 3. Bottom right
	Point2d((mmAust[3].x / pixelsizeHorizontal) + halfpixelWidth, halfpixelHeight - (mmAust[3].y / pixelsizeVertical))	// 4. Bottom left
};

}//namespace rectim


class data
{
public:

}; 