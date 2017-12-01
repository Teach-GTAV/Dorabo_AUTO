/*  Morphological Functions used in Image Processing
 *  Developed by Anubhav Rohatgi 
 *  Date :: 08/08/2014
 *
 *
 */

#pragma once

#ifndef MORPHOLOGY_H
#define MORPHOLOGY_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace ai {
	

	/*!
				\fn    imclearborder(cv::Mat& bwimg)
				\brief imclearboder suppresses structures that are lighter than their surroundings and that are connected to the image border. 8 neighborhood
				\param bwimg The input binary image(frame) - CV_8UC1 .
    */	
	void imclearborder(cv::Mat& bwimg);



	/*!
				\fn    imClearBorder(cv::Mat& bwimg, int neighborhood)
				\brief imclearboder suppresses structures that are lighter than their surroundings and that are connected to the image border.
				       This implementation is better than the above implementation imclearborder(). It uses floodfill and takes the border which maynot be just 1 pixel in size.
				\param bwimg The input binary image(frame) - CV_8UC1 .
				\param neighborhood either 4 or 8 neighborhood
    */	
	void imClearBorder(cv::Mat& bwimg, int neighborhood = 8);



	/*!
				\fn    bwareaopen(cv::Mat& img, double size)
				\brief removes from a binary image all connected components (objects) that have fewer than size pixels, producing another binary image.
				\param img The input binary image(frame) - CV_8UC1 .
				\param size minimal number of pixels
    */		 
	void bwareaopen(cv::Mat& img, double size);




	/*!
				\fn    unsharpMask(cv::Mat& im)
				\brief Sharpens the image
				\param im The input grayscale image(frame) - CV_8UC1 .				
    */	
	void unsharpMask(cv::Mat& im);



	/*!
				\fn		thinningIteration(cv::Mat& im, int iter)
				\brief  Perform one thinning iteration. This function is not called directly. It is called via skeletonization function.
				\param  im    Binary image with range = 0-1 
				\param	iter  0=even, 1=odd 
	*/
	void thinningIteration(cv::Mat& im, int iter);



	/*!
				\fn		skeletonizaton(cv::Mat& im)
				\brief  Perform one thinning/skeletonization on binary image. Calls the thinningIteration fucntion.
				\param  im    Binary image with range = 0-255				
	*/
	void skeletonizaton(cv::Mat& im);

    cv::Mat thinImage(const cv::Mat & src, int maxIterations);

} //end of namespace ai


#endif
