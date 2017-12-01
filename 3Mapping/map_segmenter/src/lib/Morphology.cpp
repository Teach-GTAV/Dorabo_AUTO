/*  Morphological Functions used in Image Processing
 *  Developed by Anubhav Rohatgi 
 *  Date :: 08/08/2014
 *
 *
 */

#include "Morphology.h"


namespace ai {


//8 neighbourhood implementation on a binary black & white image 
void imclearborder(cv::Mat& bwimg)
{
	CV_Assert(bwimg.channels() == 1 && bwimg.type() == CV_8U);

	int width = bwimg.cols;
	int height = bwimg.rows;

	//running from left to right and top to bottom
	for( int row = 0; row < height; row++)
	{
		for( int col = 0; col < width; col++)
		{
			int pixel = bwimg.at<uchar>(row,col); //getting the intensity at pixel(x,y)

			if(pixel == 255)
			{
				//check if the pixel is on the border
				if( col == 0 || row == 0|| col+1 == width || row+1 == height || col ==1 || row ==1 )
				{
					pixel = 50; //set the flag
					bwimg.at<uchar>(row,col) = pixel;
				}

				//check if the pixel is adjacent to border - left upper left and upwards
				else if( bwimg.at<uchar>(row,col-1) == 50 || bwimg.at<uchar>(row-1,col-1) == 50 || bwimg.at<uchar>(row-1,col) == 50)
				{
					pixel = 50;
					bwimg.at<uchar>(row,col) = pixel;
				}
			}
		}
	}


	for(int row = height-1; row > 0; row--)
	{ 
		//running from right to left, bottom up
		for(int col = width-1;col > 0; col--)
		{
			int pixel = bwimg.at<uchar>(row,col); //get intensity of every pixel
			
			if(pixel == 255)
			{
				//check if it is the border
				if(col == 0 || row == 0 || col+1 == width || row+1 == height || col ==1 || row ==1)
				{
					continue; //skip border
				}

				//check if it is adjacent of border - right, bottom-right and bottom
				else if(bwimg.at<uchar>(row,col+1) == 50 || bwimg.at<uchar>(row+1,col+1) == 50 || bwimg.at<uchar>(row+1,col) == 50)
				{
					pixel= 50; //set flag
					bwimg.at<uchar>(row,col) = pixel;
				}
			}
		}
	}


	for(int row = 0; row < height; row++)
	{
		//running from left to right, top to bottom
		for(int col = 0;col < width; col++)
		{
			int pixel = bwimg.at<uchar>(row,col); //get intensity of every pixel
			if(pixel == 50)
			{ 
				//check flag
				pixel = 0; //set it black
				bwimg.at<uchar>(row,col) = pixel;
			}
		}
	}
}




void imClearBorder(cv::Mat& bwimg, int neighborhood)
{
	int newMaskVal = 255;
	int ffillMode = 1;
	
	int lo = ffillMode == 0 ? 0 : 20;
	int up = ffillMode == 0 ? 0 : 20;
	int flags = neighborhood + (newMaskVal << 8) +(ffillMode == 1 ? CV_FLOODFILL_FIXED_RANGE : 0);


	cv::Mat bordermask = bwimg.clone();
	cv::rectangle(bordermask,cv::Rect(3,3,bordermask.cols -5, bordermask.rows -5),cv::Scalar::all(0),-1);
	std::vector<cv::Point> borderPixelsLoc;

	//generate map of white border pixels
	for( int row = 0; row < bordermask.rows; row++)
	{
		for( int col = 0; col < bordermask.cols; col++)
		{
			int pixel = bordermask.at<uchar>(row,col); //getting the intensity at pixel(x,y)
			if(pixel == 255)
				borderPixelsLoc.push_back(cv::Point(col,row));
		}
	}

	//flood fill iterations
	for(int i =0; i < borderPixelsLoc.size(); i++)
	{
			cv::Rect ccomp;
			
			//rechecks if the pixel has turned black, if so then the floodfill is not called on that pixel
			if(bwimg.at<uchar>(borderPixelsLoc[i]) == 255)
				cv::floodFill(bwimg, borderPixelsLoc[i], cv::Scalar::all(0), &ccomp, cv::Scalar(lo, lo, lo),cv::Scalar(up, up, up), flags);
			else
				continue;
	}
}



void bwareaopen(cv::Mat& img, double size)
{
	CV_Assert(img.channels() == 1 && img.type() == CV_8U);

	cv::Mat imgClone = img.clone();

    std::vector<std::vector<cv::Point> > contours;
	cv::findContours(imgClone,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

	for(size_t i = 0; i < contours.size(); i++)
	{
		double area = cv::contourArea(contours[i]);

		if(area > 0 && area <= size)
			cv::drawContours(img,contours, i, cv::Scalar::all(0),-1);
	}

	imgClone.release();
	contours.clear();

}



void unsharpMask(cv::Mat& im) 
{
    cv::Mat tmp;
    cv::GaussianBlur(im, tmp, cv::Size(5,5), 5);
    cv::addWeighted(im, 1.5, tmp, -0.5, 0, im);
}



void thinningIteration(cv::Mat& im, int iter)
{
    cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

    for (int i = 1; i < im.rows-1; i++)
    {
        for (int j = 1; j < im.cols-1; j++)
        {
            uchar p2 = im.at<uchar>(i-1, j);
            uchar p3 = im.at<uchar>(i-1, j+1);
            uchar p4 = im.at<uchar>(i, j+1);
            uchar p5 = im.at<uchar>(i+1, j+1);
            uchar p6 = im.at<uchar>(i+1, j);
            uchar p7 = im.at<uchar>(i+1, j-1);
            uchar p8 = im.at<uchar>(i, j-1);
            uchar p9 = im.at<uchar>(i-1, j-1);

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + 
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) + 
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i,j) = 1;
        }
    }

    im &= ~marker;
}



void skeletonizaton(cv::Mat& im)
{
    im /= 255;

    cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(im, 0);
        thinningIteration(im, 1);
        cv::absdiff(im, prev, diff);
        im.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    im *= 255;
}

cv::Mat thinImage(const cv::Mat & src, int maxIterations)
{
    assert(src.type() == CV_8UC1);
    cv::Mat dst;
    int width  = src.cols;
    int height = src.rows;
    src.copyTo(dst);
    int count = 0;  //记录迭代次数
    while (true)
    {
        count++;
        if (maxIterations != -1 && count > maxIterations) //限制次数并且迭代次数到达
            break;
        std::vector<uchar *> mFlag; //用于标记需要删除的点
        //对点标记
        for (int i = 0; i < height ;++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p+j);
                    }
                }
            }
        }

        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }

        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }

        //对点标记
        for (int i = 0; i < height; ++i)
        {
            uchar * p = dst.ptr<uchar>(i);
            for (int j = 0; j < width; ++j)
            {
                //如果满足四个条件，进行标记
                //  p9 p2 p3
                //  p8 p1 p4
                //  p7 p6 p5
                uchar p1 = p[j];
                if (p1 != 1) continue;
                uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
                uchar p8 = (j == 0) ? 0 : *(p + j - 1);
                uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
                uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
                uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
                uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
                uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
                uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

                if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
                {
                    int ap = 0;
                    if (p2 == 0 && p3 == 1) ++ap;
                    if (p3 == 0 && p4 == 1) ++ap;
                    if (p4 == 0 && p5 == 1) ++ap;
                    if (p5 == 0 && p6 == 1) ++ap;
                    if (p6 == 0 && p7 == 1) ++ap;
                    if (p7 == 0 && p8 == 1) ++ap;
                    if (p8 == 0 && p9 == 1) ++ap;
                    if (p9 == 0 && p2 == 1) ++ap;

                    if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
                    {
                        //标记
                        mFlag.push_back(p+j);
                    }
                }
            }
        }

        //将标记的点删除
        for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
        {
            **i = 0;
        }

        //直到没有点满足，算法结束
        if (mFlag.empty())
        {
            break;
        }
        else
        {
            mFlag.clear();//将mFlag清空
        }
    }
    return dst;
}


}//end of namespace ai
