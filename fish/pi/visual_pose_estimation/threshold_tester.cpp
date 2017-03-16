#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <string>

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
	cv::Mat ret;
	for (int i = 0; i < n_hbins; i++) {
		for (int j = 0; j < n_sbins; j++) {
			for (int k = 0; k < n_vbins; k++) {
				if (histogram[index(i, j, k, n_hbins, n_sbins, n_vbins)] > count_thresh) {
					cv::Scalar lower, upper;
					lower = calc_lower(i, j, k, n_hbins, n_sbins, n_vbins);
					upper = calc_upper(i, j, k, n_hbins, n_sbins, n_vbins);
					cv::inRange(im, lower, upper, ret);
					return ret;
				}
			}
		}
	}
	return im;
}

void find_threshold(std::string directory, std::string filename, int* histogram, int n_hbins, int n_sbins, int n_vbins, int count_thresh, int i, int j, int k) {
	cv::Mat im, hsv, bw;
	im = cv::imread(directory+filename, CV_LOAD_IMAGE_COLOR);
	cv::cvtColor(im, hsv, CV_BGR2HSV);
	bw = threshold_hsv(hsv, histogram, n_hbins, n_sbins, n_vbins, count_thresh);
	cv::imwrite("/Users/shomberg/Pictures/bw_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(k) + ".png", bw);
}

int main(int argc, char *argv[]) {
	std::string file(argv[1]);
	int n_hbins = 10, n_sbins = 8, n_vbins = 4;
	for (int i = 0; i < n_hbins; i++) {
		for (int j = 0; j < n_sbins; j++) {
			for (int k = 0; k < n_vbins; k++) {
				int* histogram = (int*)calloc(n_hbins*n_sbins*n_vbins, sizeof(int));
				histogram[index(i,j,k,n_hbins,n_sbins,n_vbins)] = 100;
				find_threshold("/Users/shomberg/Pictures/", file, histogram, n_hbins, n_sbins, n_vbins, 50, i, j, k);
				free(histogram);
			}
		}
	}
}