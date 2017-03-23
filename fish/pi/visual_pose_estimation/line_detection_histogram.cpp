#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include "histogram.h"


// cv::inRange(im, cv::Scalar(97, 40, 0), cv::Scalar(125, 255, 140), ret);

int index(int i, int j, int k, int n_hbins, int n_sbins, int n_vbins) {
	return i*(n_sbins*n_vbins) + j*(n_vbins) + k;
}

cv::Scalar calc_lower(int i, int j, int k, int n_hbins, int n_sbins, int n_vbins) {
	cv::Scalar ret((int)(i*(180.0/n_hbins)), (int)(j*(256.0/n_sbins)), (int)(k*(256.0/n_vbins)));
	return ret;
}

cv::Scalar calc_upper(int i, int j, int k, int n_hbins, int n_sbins, int n_vbins) {
	cv::Scalar ret((int)((i+1)*(180.0/n_hbins))-1, (int)((j+1)*(256.0/n_sbins))-1, (int)((k+1)*(256.0/n_vbins))-1);
	return ret;
}

cv::Mat threshold_hsv(cv::Mat im, int* histogram, int n_hbins, int n_sbins, int n_vbins, int count_thresh) {
	cv::Mat ret(im.size(), CV_8UC1, cv::Scalar(0,0,0));
	std::cout << "looping\n";
	for (int i = 0; i < n_hbins; i++) {
		for (int j = 0; j < n_sbins; j++) {
			for (int k = 0; k < n_vbins; k++) {
				if (i == 0 && j == 0 && k == n_vbins-1) {
					continue;
				}
				if (histogram[index(i, j, k, n_hbins, n_sbins, n_vbins)] > count_thresh) {
					cv::Scalar lower, upper;
					lower = calc_lower(i, j, k, n_hbins, n_sbins, n_vbins);
					upper = calc_upper(i, j, k, n_hbins, n_sbins, n_vbins);
					cv::Mat mask;
					cv::inRange(im, lower, upper, mask);
					cv::bitwise_or(ret, mask, ret);
				}
			}
		}
	}
	return ret;
}

void find_threshold(std::string directory, std::string filename, int* histogram, int n_hbins, int n_sbins, int n_vbins, int count_thresh, int i, int j, int k) {
	cv::Mat im, hsv, bw;
	im = cv::imread(directory+filename, CV_LOAD_IMAGE_COLOR);
	cv::cvtColor(im, hsv, CV_BGR2HSV);
	bw = threshold_hsv(hsv, histogram, n_hbins, n_sbins, n_vbins, count_thresh);
	cv::imwrite("/Users/shomberg/Pictures/bw_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k) + ".png", bw);
}

void process_image(std::string directory, std::string filename, int* histogram, int n_hbins, int n_sbins, int n_vbins, int count_thresh) {
	cv::Mat im, im_crop, bw, smoothed, edges;
	int thresh = 40;
	std::cout << "reading image\n";
	im = cv::imread(directory+filename, CV_LOAD_IMAGE_COLOR);
	std::cout << "cropping\n";
	cv::Rect roi(0,(int)(.6*im.size().height),im.size().width,(int)(.4*im.size().height));
	im_crop = im(roi);
	std::cout << "converting\n";
	cv::cvtColor(im_crop, im_crop, CV_BGR2HSV);
	// cv::cvtColor(im, im, CV_BGR2HSV);
	std::cout << "thresholding\n";
	bw = threshold_hsv(im_crop, histogram, n_hbins, n_sbins, n_vbins, count_thresh);
	// bw = threshold_hsv(im, histogram, n_hbins, n_sbins, n_vbins, count_thresh);
	cv::imwrite("/Users/shomberg/Pictures/pool_lanes_bw.png", bw);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(20,20));
	cv::dilate(bw, smoothed, element);
	cv::imwrite("/Users/shomberg/Pictures/pool_lanes_smoothed.png", smoothed);
	cv::Mat extended(smoothed.size()+cv::Size(2,2), smoothed.type());
	cv::copyMakeBorder(smoothed, extended, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
	cv::imwrite("/Users/shomberg/Pictures/pool_lanes_extended.png", extended);
	// cv::Canny(im_crop, edges, thresh, thresh*3);
	// cv::imwrite("/Users/shomberg/Pictures/pool_lanes_edges.png", edges);
	cv::Canny(extended, edges, thresh, thresh*3);
	cv::imwrite("/Users/shomberg/Pictures/pool_lanes_bw_edges.png", edges);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	int max_contour_idx;
	double max_area = -1;
	for (int i = 0; i < contours.size(); i++) {
		// cv::Mat contour_frame(extended.size(), extended.type(), cv::Scalar(0,0,0));
		// cv::drawContours(contour_frame, contours, i, cv::Scalar(255,255,255));
		// cv::imwrite("/Users/shomberg/Pictures/pool_lanes_max_contour_" + std::to_string(i) + ".png", contour_frame);
		double this_area = cv::contourArea(contours[i]);
		std::cout << "idx " << i << " area " << this_area << "\n";
		if (this_area > max_area) {
			max_contour_idx = i;
			max_area = this_area;
			std::cout << "new max contour: " << i << "\n";
		}
	}
	cv::Mat contour_frame(extended.size(), extended.type(), cv::Scalar(0,0,0));
	cv::drawContours(contour_frame, contours, -1, cv::Scalar(255,255,255));
	cv::imwrite("/Users/shomberg/Pictures/pool_lanes_contours.png", contour_frame);
	cv::Mat max_contour_frame(extended.size(), extended.type(), cv::Scalar(0,0,0));
	cv::drawContours(max_contour_frame, contours, max_contour_idx, cv::Scalar(255,255,255));
	cv::imwrite("/Users/shomberg/Pictures/pool_lanes_max_contour.png", max_contour_frame);
	std::vector<cv::Point> polygon;
	cv::approxPolyDP(contours[max_contour_idx], polygon, 6, true);
	cv::Mat polygon_frame(extended.size(), extended.type(), cv::Scalar(0,0,0));
	cv::polylines(polygon_frame, polygon, true, cv::Scalar(255,255,255));
	cv::imwrite("/Users/shomberg/Pictures/pool_lanes_polygon.png", polygon_frame);

	int top_l, top_r, bot_l, bot_r;
	int init = 3*max_contour_frame.size().height + max_contour_frame.size().width;
	int top_l_val = init, top_r_val = init, bot_l_val = init, bot_r_val = init;
	for (int i = 0; i < polygon.size(); i++) {
		cv::Point p = polygon[i];
		int top_l_val_this = 3*p.y + p.x;
		int top_r_val_this = 3*p.y - p.x;
		int bot_l_val_this = -3*p.y + p.x;
		int bot_r_val_this = -3*p.y - p.x;
		std::cout << polygon[i].x << ", " << polygon[i].y << " " << top_l_val_this << " " << top_r_val_this << " " << bot_l_val_this << " " << bot_r_val_this << "\n";
		if (top_l_val_this < top_l_val) {
			top_l = i;
			top_l_val = top_l_val_this;
		}
		if (top_r_val_this < top_r_val) {
			top_r = i;
			top_r_val = top_r_val_this;
		}
		if (bot_l_val_this < bot_l_val) {
			bot_l = i;
			bot_l_val = bot_l_val_this;
		}
		if (bot_r_val_this < bot_r_val) {
			bot_r = i;
			bot_r_val = bot_r_val_this;
		}
	}

	std::cout << "top right top left bot left bot right: " << top_r << " " << top_l << " " << bot_l << " " << bot_r << "\n";
}



int main(int argc, char *argv[]) {
	std::string file(argv[1]);
	// int n_hbins = 10, n_sbins = 8, n_vbins = 4;
	// int* histogram = (int*)calloc(n_hbins*n_sbins*n_vbins, sizeof(int));
	std::cout << "processing\n";
	// histogram[index(6,0,0,n_hbins,n_sbins,n_vbins)] = 100;
	process_image("/Users/shomberg/Pictures/", file, (int*)HISTOGRAM, N_HBINS, N_SBINS, N_VBINS, THRESH);
}