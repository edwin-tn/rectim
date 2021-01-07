// rectim.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>

#include "InputData.h"
#include "rotations.h"
#include "collinearRect.h"

using namespace std;

int main()
{
	collinearRect rectifylowoblique(rectim::OMEGA1, rectim::PHI1, rectim::KAPPA1, rectim::lowObliqueImage);
	double tilt = rectifylowoblique.getRotation().getTilt();
	double swing = rectifylowoblique.getRotation().getSwing();
	cout << "\n =========================    Image tilt : " << tilt << "\tand swing:\t" << swing << "\tdegrees  ================================" << endl;

	if (tilt > 5.0 )
	{
		cout << "\nSource image has tilt and swing more than 5 degrees. Progam will quit.................................\n";
		return EXIT_FAILURE;
	}
	else {
		rectifylowoblique.rectifyPoints();
		rectifylowoblique.warpVerticalImage();
	}



	//collinearRect rectifyhighoblique(rectim::OMEGA2, rectim::PHI2, rectim::KAPPA2, rectim::lowObliqueImage2);
	//double tilt = rectifyhighoblique.getRotation().getTilt();
	//double swing = rectifyhighoblique.getRotation().getSwing();
	//cout << "\n =========================    Image tilt : " << tilt << "\tand swing:\t" << swing << "\tdegrees  ================================" << endl;

	//if (tilt > 5.0)
	//{
	//	cout << "\nSource image has tilt and/or swing more than 5 degrees. Progam will quit.................................\n";
	//	return EXIT_FAILURE;
	//}
	//else {
	//	rectifyhighoblique.rectifyPoints();
	//	rectifyhighoblique.warpVerticalImage();
	//}

	
	std::cout << "Hello World!\n";
	((cv::waitKey() & 255) /*== 27*/);  // Wait for a keystroke in the window
	return 0;

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
