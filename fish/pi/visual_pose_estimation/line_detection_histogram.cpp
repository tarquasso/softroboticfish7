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
					std::cout << "ijk " << std::to_string(i) << std::to_string(j) << std::to_string(k) << '\n';
					cv::Scalar lower, upper;
					lower = calc_lower(i, j, k, n_hbins, n_sbins, n_vbins);
					upper = calc_upper(i, j, k, n_hbins, n_sbins, n_vbins);
					cv::Mat mask;
					cv::inRange(im, lower, upper, mask);
					std::cout << "im type: " << im.type() << " mask " << mask.type() << "\n";
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
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10,10));
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
	cv::approxPolyDP(contours[max_contour_idx], polygon, 4, true);
	std::vector<cv::Point> corners;
	for (int i = 0; i < polygon.size(); i++) {
		int pre_idx = (i-1+polygon.size())%polygon.size();
		int post_idx = (i+1)%polygon.size();
		double theta_pre = atan2(polygon[i].y-polygon[pre_idx].y, polygon[i].x-polygon[pre_idx].x);
		double theta_post = atan2(polygon[post_idx].y-polygon[i].y, polygon[post_idx].x-polygon[i].x);
		double diff = fmod(theta_post - theta_pre + 3*M_PI, 2*M_PI) - M_PI;
		std::cout << polygon[i].x << ", " << polygon[i].y << "\n";
		std::cout << theta_post << " - " << theta_pre << " = " << diff << "\n";
		if (std::abs(diff) > M_PI/6) {
			corners.push_back(polygon[i]);
		}
	}
	cv::Mat polygon_frame(extended.size(), extended.type(), cv::Scalar(0,0,0));
	cv::polylines(polygon_frame, polygon, true, cv::Scalar(255,255,255));
	cv::imwrite("/Users/shomberg/Pictures/pool_lanes_polygon.png", polygon_frame);
	cv::Mat corners_frame(extended.size(), extended.type(), cv::Scalar(0,0,0));
	cv::polylines(corners_frame, corners, true, cv::Scalar(255,255,255));
	cv::imwrite("/Users/shomberg/Pictures/pool_lanes_corners.png", corners_frame);

	int long1_idx = -1, long2_idx = -1;
	double long1_length = -1, long2_length = -1;

	for (int i = 0; i < corners.size(); i++) {
		int next_idx = (i+1)%corners.size();
		cv::Point diff = corners[next_idx] - corners[i];
		double len = diff.ddot(diff);
		if (len > long1_length) {
			long2_idx = long1_idx;
			long2_length = long1_length;
			long1_idx = i;
			long1_length = len;
		} else if (len > long2_length) {
			long2_idx = i;
			long2_length = len;
		}
	}

	int long1_idx_n = (long1_idx+1)%corners.size(), long2_idx_n = (long2_idx+1)%corners.size();
	int top1, top2, bot1, bot2;
	if (corners[long1_idx].y < corners[long1_idx_n].y) {
		top1 = long1_idx;
		bot1 = long1_idx_n;
	} else {
		top1 = long1_idx_n;
		bot1 = long1_idx;
	}
	if (corners[long2_idx].y < corners[long2_idx_n].y) {
		top2 = long2_idx;
		bot2 = long2_idx_n;
	} else {
		top2 = long2_idx_n;
		bot2 = long2_idx;
	}
	int top_l, top_r, bot_l, bot_r;
	if (corners[top1].x < corners[top2].x) {
		top_l = top1;
		top_r = top2;
		bot_l = bot1;
		bot_r = bot2;
	} else {
		top_l = top2;
		top_r = top1;
		bot_l = bot2;
		bot_r = bot1;
	}

	std::cout << "top left bot left top right bot right: " << top_l << " " << bot_l << " " << top_r << " " << bot_r << "\n";
}



int main(int argc, char *argv[]) {
	std::string file(argv[1]);
	// int n_hbins = 10, n_sbins = 8, n_vbins = 4;
	// int* histogram = (int*)calloc(n_hbins*n_sbins*n_vbins, sizeof(int));
	std::cout << "processing\n";
	// histogram[index(6,0,0,n_hbins,n_sbins,n_vbins)] = 100;
	process_image("/Users/shomberg/Pictures/", file, (int*)HISTOGRAM, N_HBINS, N_SBINS, N_VBINS, 1000);
}