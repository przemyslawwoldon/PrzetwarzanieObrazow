#include "stdafx.h"
#include <chrono>
#include "opencv2\opencv.hpp"

#define TARGET_FPS 30
#define MSE_THRESHOLD 5

using namespace std;

void DilateErode(cv::Mat thresholdedImage)
{
	////morphological opening (removes small objects from the foreground)
	erode(thresholdedImage, thresholdedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	dilate(thresholdedImage, thresholdedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	//morphological closing (removes small holes from the foreground)
	dilate(thresholdedImage, thresholdedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	erode(thresholdedImage, thresholdedImage, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

}

//Discard all the elements from the list, for which the contour is smaller than {X} points
//We are only interested in big objects
unsigned int minPointsInContour = 50;

float calulateRadiusMSE(int posX, int posY, vector<cv::Point> contour) {
	float sum = 0;
	int n = 0;
	for (cv::Point point : contour)
	{
		sum += std::pow((point.x - posX), 2) + std::pow((point.y - posY), 2);
		n++;
	}

	printf("MSE RAdius: %.2f\n", std::sqrt(sum / n));
	return  std::sqrt(sum / n);
}

float calculateMSEValue(int posX, int posY, vector<cv::Point> contour, float radiusMSE) {
	float mseSum = 0;
	int n = 0;

	for (cv::Point point : contour)
	{
		mseSum += pow(radiusMSE - sqrt(pow((point.x - posX), 2) + pow((point.y - posY), 2)), 2);
		n++;
	}
	mseSum = std::sqrt(mseSum / n);
	printf("MSE threshold: %.3f\n", mseSum);

	return mseSum;
}


//Could be two functions, but wanted to use std::pair and std::tie to get results
std::pair <std::vector<std::vector<cv::Point>>, std::vector<cv::Vec3i>> getContoursAndCircles(cv::Mat imageToSearchContoursIn)
{

	//******************************************************************************************************************************************************
	//Finding circles - one way
	std::vector<std::vector<cv::Point>> contouredElementsOnScreen;
	std::vector<cv::Vec3i> circlesFoundUsingContours;


	//cv::findContours(canny_out, conturedElementsOnScreen, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_NONE);
	cv::findContours(imageToSearchContoursIn, contouredElementsOnScreen, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_NONE);

	//Discarding using C++11 std::remove_if and lambda expressions
	contouredElementsOnScreen.erase(std::remove_if(contouredElementsOnScreen.begin(),
		contouredElementsOnScreen.end(),
		[=](const std::vector<cv::Point>& contour)
	{
		if (contour.size() < minPointsInContour) return true;
		else { return false; }
	}),
		contouredElementsOnScreen.end());

	//Iterate over what's left (only big contours) and try to find all the circles

	for (auto contour : contouredElementsOnScreen)
	{
		cv::Moments tempMoments = cv::moments(contour);
		//Moments
		//double dM02 = tempMoments.m02;
		//double dM20 = tempMoments.m20;
		//Normalized moments
		double dN02 = tempMoments.nu02;
		double dN20 = tempMoments.nu20;

		double area = tempMoments.m00;
		double radius = std::sqrt(area / 3.141592);
		printf("Norm radius: %.2f\n", radius);

		double maxMoment = std::max(dN02, dN20);
		double minMoment = std::min(dN02, dN20);

		int posY = (int)(tempMoments.m01 / area);
		int posX = (int)(tempMoments.m10 / area);

		float radiusMSE = calulateRadiusMSE(posX, posY, contour);

		float err = (radius - radiusMSE) * 100 / radius;
		err = std::pow(err, 2);
		printf("Blad: %.2f\n", err);

		float mseValue = calculateMSEValue(posX, posY, contour, radiusMSE);

		if ((maxMoment / minMoment) > 0.75 && (maxMoment / minMoment) < 1.25 && mseValue < 3)
		{
			std::cout << "Found circle" << std::endl;

			if (posX > 0 && posY > 0)
			{
				circlesFoundUsingContours.push_back(cv::Vec3i(posX, posY, (int)radius));
			}
		}

	}

	return{ contouredElementsOnScreen, circlesFoundUsingContours };
}


cv::Point findCenterOfBlobUsingMoments(cv::Mat imageToSearchBlobIn)
{

	cv::Point blobCenter = { -1,-1 };
	//Calculate the moments of the thresholded image
	cv::Moments oMoments = cv::moments(imageToSearchBlobIn);

	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	// if the area of the found object is not big enough, then it's just noise
	if (dArea > 10000)
	{
		//calculate the position of the blob
		int posX = (int)(dM10 / dArea);
		int posY = (int)(dM01 / dArea);

		if (posX >= 0 && posY >= 0)
		{
			blobCenter = cv::Point{ posX, posY };
		}
	}

	return blobCenter;
}


cv::Vec2i calculateCameraMovement(cv::Point	screenCenter, cv::Point blobCenter, int originalImageColumns, int originalImageRows, int minRange = -20, int maxRange = -20)
{
	cv::Vec2i movementVectorScaled(0, 0);
	cv::Vec2i movementVectorOnScreen(0, 0);

	movementVectorOnScreen[0] = blobCenter.x - screenCenter.x;
	movementVectorOnScreen[1] = blobCenter.y - screenCenter.y;

	std::cout << "Vector coords: (" << movementVectorOnScreen[0] << ", " << movementVectorOnScreen[1] << ") " << std::endl;

	//Scaling camera movement vector from image coordinates to new range default (-20,20) -> servo movement (degrees)
	//int NewValue = ( ( (OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin;

	movementVectorScaled[0] = (((movementVectorOnScreen[0] - (-originalImageColumns / 2)) * (maxRange - (minRange))) / ((originalImageColumns / 2) - (-originalImageColumns / 2))) + (minRange);
	movementVectorScaled[1] = (((movementVectorOnScreen[1] - (-originalImageRows / 2)) * (maxRange - (minRange))) / ((originalImageRows / 2) - (-originalImageRows / 2))) + (minRange);


	return movementVectorScaled;

}


int main()
{

	cv::VideoCapture cap(0); //capture the video from webcam

	if (!cap.isOpened())  // if not success, exit program
	{
		std::cout << "Cannot access the camera with given index" << std::endl;
		return -1;
	}

	cv::namedWindow("Control Sliders", CV_WINDOW_AUTOSIZE);
	//create a window called "Control"

	////RED
	int iLowH = 170;
	int iHighH = 179;

	int iLowS = 150;
	int iHighS = 255;

	int iLowV = 60;
	int iHighV = 255;

	//BLUE
	//    int iLowH = 93;
	//    int iHighH = 120;

	//    int iLowS = 196;
	//    int iHighS = 255;

	//    int iLowV = 121;
	//    int iHighV = 255;


	//Create trackbars in "Control" window
	cv::createTrackbar("LowH", "Control Sliders", &iLowH, 179); //Hue (0 - 179)
	cv::createTrackbar("HighH", "Control Sliders", &iHighH, 179);

	cv::createTrackbar("LowS", "Control Sliders", &iLowS, 255); //Saturation (0 - 255)
	cv::createTrackbar("HighS", "Control Sliders", &iHighS, 255);

	cv::createTrackbar("LowV", "Control Sliders", &iLowV, 255);//Value (0 - 255)
	cv::createTrackbar("HighV", "Control Sliders", &iHighV, 255);

	bool dilate_erode = true, hough_circles = true, canny = true;

	while (true)
	{
		auto time_start = std::chrono::high_resolution_clock::now();
		cv::Mat imgOriginal;

		// read a new frame from video
		bool bSuccess = cap.read(imgOriginal);

		if (!bSuccess) //if not success, break loop
		{
			std::cout << "Could not read frame from the video stream" << std::endl;
			break;
		}


		//Scale down one step for faster processing
		cv::pyrDown(imgOriginal, imgOriginal);


		//Convert the captured frame from BGR to HSV
		cv::Mat imgHSV;
		cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

		//Convert the captured frame to greyscale
		cv::Mat imgGray;
		cv::cvtColor(imgOriginal, imgGray, cv::COLOR_BGR2GRAY);



		//Threshold the image
		cv::Mat imgThresholded;
		inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);

		//Close the gaps in image
		if (dilate_erode)
		{
			DilateErode(imgThresholded);
		}


		//Get edges using canny algorithm
		cv::Mat canny_out;
		cv::Canny(imgThresholded, canny_out, 100, 200);


		//Find the contours of the image
		std::vector<std::vector<cv::Point>> contouredElementsOnScreen;
		std::vector<cv::Vec3i> circlesFoundUsingContours;

		if (canny)
		{
			std::tie(contouredElementsOnScreen, circlesFoundUsingContours) = getContoursAndCircles(canny_out);
		}
		else
		{
			std::tie(contouredElementsOnScreen, circlesFoundUsingContours) = getContoursAndCircles(imgThresholded);
		}

		//Draw the results on screen:
		//contours
		cv::drawContours(imgGray, contouredElementsOnScreen, -1, cv::Scalar{ 255,0,0 }, 2);
		//circles
		for (auto circle : circlesFoundUsingContours)
		{
			cv::circle(imgGray, cv::Point{ (int)circle[0], (int)circle[1] }, (int)circle[2], cv::Scalar{ 255,0,0 }, 5);
		}




		//******************************************************************************************************************************************************
		//FINDING Objects of interest - another way, using blobs of single color and (optionally) Hough circles algorithm

		cv::Point blobCenter = findCenterOfBlobUsingMoments(imgThresholded);


		if (blobCenter.x > 0 && blobCenter.y > 0)
		{

			//Draw a red line from the middle of the screen to ball position
			cv::Point screenCenter(imgOriginal.cols / 2, imgOriginal.rows / 2);
			cv::line(imgOriginal, screenCenter, blobCenter, cv::Scalar(0, 0, 255), 5);

			//Create a vector for storing information about how much the camera has to move in each direction;
			cv::Vec2i scaledVector = calculateCameraMovement(screenCenter, blobCenter, imgOriginal.cols, imgOriginal.rows);
		}



		//show the thresholded image
		cv::imshow("Thresholded Image", imgThresholded);
		//show B&W image
		cv::imshow("Black&White", imgGray);
		//show Edges
		cv::imshow("Canny", canny_out);
		//show the original image with annotations
		cv::imshow("Original", imgOriginal);



		//timing the code execution
		auto time_end = std::chrono::high_resolution_clock::now();
		auto time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
		auto willWaitFor = std::max(1, (1000 / TARGET_FPS) - (int)time_passed);


		std::cout << "Processing time was: " << (int)time_passed << "ms. Will wait for: " << willWaitFor << "ms" << std::endl;


		//Wait for key presses
		int key = cvWaitKey(willWaitFor);
		key = (key == 255) ? -1 : key;

		//handling key presses - enable/disable functionality
		switch (key)
		{
		case 27:
		{
			std::cout << "esc key is pressed by user" << std::endl;
			cv::destroyAllWindows();
			return 0;
		}
		case 'd':
		{
			dilate_erode = !dilate_erode;
			break;
		}
		case 'c':
		{
			canny = !canny;
			break;
		}
		case 'h':
		{
			hough_circles = !hough_circles;
		}
		}

	}


	return 0;

}