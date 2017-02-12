#pragma once

#include "Vectorfield.h"
#include "Vertex2Pixel.h"

//#define HAVE_OPENCV

#ifdef HAVE_OPENCV

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

template <typename T>
inline void normalizeV(T &x, T &y) {
	T d = x*x + y*y;
	if (d > 0) {
		d = sqrt(d);
		x /= d; y /= d;
	}
}
template <typename T>
void normalizeVF(vector<T>& vx, vector<T>& vy)
{	
	for (int i = 0; i < vx.size(); i++) {
		normalizeV(vx[i], vy[i]);
	}	
}


#define TERM_CRITERIA_SINGULAR 0x01
#define TERM_CRITERIA_BOUNDARY 0x02
#define TERM_CRITERIA_MAX_ITER 0x04

struct trace_streamline_termination_criteria {
	int type;
	int max_iter = 0;
	trace_streamline_termination_criteria(int iter = 10, int t = TERM_CRITERIA_SINGULAR | TERM_CRITERIA_BOUNDARY | TERM_CRITERIA_MAX_ITER)
	{
		type = t;  max_iter = iter;
	}

};

template <typename T>
int lic_streamline(const unsigned char* noise, const T *vx, const  T *vy,
	const T x0, const T y0, const  int width, const int height,
	trace_streamline_termination_criteria tc = trace_streamline_termination_criteria(),
	const T EPSILON =1e-10)
{
	double x, y;
	int ix, iy;
	int noise_color = 0;
	
	T vx0, vy0;
	int num = 0, idx = 0;
	for (int i = 0; i < 2; i++) {
		int j = 0;
		x = x0; y = y0;
		do
		{
			ix = x + 0.5; iy = y + 0.5;
			if ((tc.type&TERM_CRITERIA_BOUNDARY) && (ix < 0 || iy < 0 || ix >= width || iy >= height))
				break;

			idx = iy * width + ix;
			noise_color += noise[idx]; j++;

			vx0 = vx[idx];
			vy0 = vy[idx];
			if (tc.type&TERM_CRITERIA_SINGULAR &&fabs(vx0) < EPSILON&& fabs(vy0) < EPSILON)
				break;
			if (i == 1) {
				vx0 = -vx0; vy0 = -vy0;
			}
			x += vx0;
			y += vy0;
			if (tc.type&TERM_CRITERIA_MAX_ITER) {
				if (j > tc.max_iter) break;
			}
		} while (1);
		num += j;
	}
	if (num != 0)
		return noise_color / num;
	else
		return noise[((int)y0) * width + (int)x0];
}

template <typename T>
int lic_streamline(const unsigned char* noise, const vector<T>& vx, const vector<T>& vy, 
	const T x0, const T y0, const  int width, const int height,
	trace_streamline_termination_criteria tc = trace_streamline_termination_criteria(),
	const T EPSILON =1e-10)
{
	return lic_streamline(noise, &(vx[0]), &(vy[0]), x0, y0, width, height, tc, EPSILON);
}


//=============generate image or texture=======================
#include <cstdlib>
#include <cmath>
#include <ctime>
inline void gen_noise_img(unsigned char *data, int width, int height)
{
	srand(time(NULL));
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
			data[y * width + x] = rand() % 256;
}

// generate texture using line integral converlution from the vector field
template <typename T>
inline void gen_lic(unsigned char * output, const unsigned char * noise, 
	const T *vx, const T *vy, const int width, const int height)
{
	int y, x;

	trace_streamline_termination_criteria tc(10);// , TERM_CRITERIA_SINGULAR | TERM_CRITERIA_MAX_ITER);
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++)	{
			int sum_val = lic_streamline(noise, vx, vy, (T)x, (T)y, width, height, tc);
			output[y*width + x] = sum_val;
		}
	}
}


inline void gray2rgb(unsigned char * rgb, const unsigned char * gray, 
	const int width, const int height) {
	const unsigned char *p_gray = gray;
	unsigned char *p_rgb = rgb;
	for (int y = 0; y< height; y++)
		for (int x = 0; x < width; x++, p_gray++) {
			*p_rgb++ = *p_gray;
			*p_rgb++ = *p_gray;
			*p_rgb++ = *p_gray;
		}
}

template <typename T>
void VectorField2Image(vector<float> &vx, vector<float> &vy,
	vector<unsigned char> &glTex_img,const VectorField<T> &VF,const int img_width, const int img_height)
{
	
#if 0
	int width, height;
	VectorField2grid(VF, vx, vy, width, height);
#else
	//render_to_frameBuffer(VF, vx, vy, img_width, img_height);
	renderOffScreen(VF, vx, vy, img_width, img_height);
#endif
	normalizeVF(vx, vy);

	int size = img_width*img_height;
	vector<unsigned char> noise_img(size);
	vector<unsigned char> lic_img(size);
	glTex_img.resize(size * 3);

	gen_noise_img(&(noise_img[0]), img_width, img_height);

	gen_lic(&(lic_img[0]), &(noise_img[0]), &(vx[0]), &(vy[0]), img_width, img_height);
	gray2rgb(&(glTex_img[0]), &(lic_img[0]), img_width, img_height);

#ifdef HAVE_OPENCV
#define SHOW_IMAGE(x) {cv::imshow(#x, x); cv::waitKey(10);}
	cv::Mat noiseImg(img_height, img_width, CV_8UC1, &(noise_img[0]));
	SHOW_IMAGE(noiseImg);
	cv::Mat licImg(img_height, img_width, CV_8UC1, &(lic_img[0]));
	SHOW_IMAGE(licImg);
	cv::Mat txtImg(img_height, img_width, CV_8UC3, &(glTex_img[0]));
	SHOW_IMAGE(txtImg);
#endif	
}

/*
template <typename T>
void VectorField2Image(const VectorField<T> &VF, vector<unsigned char> &glTex_img
	,const int img_width, const int img_height)
{
	vector<float> vx, vy;	
#if 0
	int width, height;
	VectorField2grid(VF,vx, vy, width, height);
#else
	//render_to_frameBuffer(VF, vx, vy, img_width, img_height);
	renderOffScreen(VF,vx,vy, img_width, img_height);
#endif
	normalizeVF(vx,vy);

	int size = img_width*img_height;
	vector<unsigned char> noise_img (size);
	vector<unsigned char> lic_img(size);
	glTex_img.resize(size*3);	

	gen_noise_img(&(noise_img[0]), img_width, img_height);

	gen_lic(&(lic_img[0]), &(noise_img[0]), &(vx[0]), &(vy[0]), img_width, img_height);
	gray2rgb(&(glTex_img[0]), &(lic_img[0]), img_width, img_height);

#ifdef HAVE_OPENCV
#define SHOW_IMAGE(x) {cv::imshow(#x, x); cv::waitKey(10);}
	cv::Mat noiseImg(img_height, img_width, CV_8UC1, &(noise_img[0]));
	SHOW_IMAGE(noiseImg);
	cv::Mat licImg(img_height, img_width, CV_8UC1, &(lic_img[0]));
	SHOW_IMAGE(licImg);
	cv::Mat txtImg(img_height, img_width, CV_8UC3, &(glTex_img[0]));
	SHOW_IMAGE(txtImg);
#endif

	if (record_vx_vy) {//debug ,sss
		vx_vec.push_back(vx); vy_vec.push_back(vy);
	}
}
*/

//-------------- draw arrow ------------
inline void putpixel(unsigned char * outimg, const int x, const int y, const unsigned char color, const int width) {
	int idx = y*width + x;
	unsigned char *p = outimg + idx;
	p[0] = color;
}
inline void putpixel(unsigned char * outimg, const int x, const int y, const unsigned char color[], const int width) {
	int idx = y*width + x;
	unsigned char *p = outimg + 3 * idx;
	p[0] = color[0];  p[1] = color[1];  p[2] = color[2];
}

inline void draw_line_rgb_DDA(unsigned char * outimg, const int x1, const int y1, const int x2, const int y2,
	const unsigned char color[], const int width) {
	float dX, dY, iSteps;
	float xInc, yInc, iCount, x, y;

	dX = x1 - x2;
	dY = y1 - y2;

	if (fabs(dX) > fabs(dY))
	{
		iSteps = fabs(dX);
	}
	else
	{
		iSteps = fabs(dY);
	}

	xInc = dX / iSteps;
	yInc = dY / iSteps;

	x = x1;
	y = y1;

	putpixel(outimg, (int)x, (int)y, color, width);

	for (iCount = 1; iCount <= iSteps; iCount++) {
		putpixel(outimg, floor(x), floor(y), color, width);

		x -= xInc;
		y -= yInc;
	}
	putpixel(outimg, x, y, color, width);
	return;
}

inline void draw_arrow(unsigned char * outimg, const int x1, const int y1, const int x2, const int y2,
	const unsigned char color[], const int width, const int height)
{
	float delta = 0.2, D[2] = { x2 - x1, y2 - y1 }, D2[2] = { delta*D[0], delta*D[1] },
		A[2] = { x2 - D2[0], y2 - D2[1] }, D3[2] = { -D2[1] / 2,D2[0] / 2 };

	int x3 = A[0] + D3[0], y3 = A[1] + D3[1];
	int x4 = A[0] - D3[0], y4 = A[1] - D3[1];

	draw_line_rgb_DDA(outimg, x1, y1, x2, y2, color, width);
	if (x3 < 1 || x3 >= width || y3 < 1 || y3 >= height) return;
	draw_line_rgb_DDA(outimg, x3, y3, x2, y2, color, width);
	if (x4 < 1 || x4 >= width || y4 < 1 || y4 >= height) return;
	draw_line_rgb_DDA(outimg, x4, y4, x2, y2, color, width);
}

#include <iostream>
inline void test_draw_arrow_rgb(unsigned char * outimg, const float *vx, const  float *vy,
	const  int width, const int height, const int step = 30)
{
	unsigned char color[3] = { 255, 0, 0 }, scale = 1.0f;

	//	std::cout << width << "," << height << "\n";
	//	draw_line_rgb_DDA(outimg, 10, 10, 100, 100, color, width);

	//	return;

	int idx = 0;
	float arrow_length_scale = 1.0f, max_v = 0;

	for (int y = 0; y < height; y += step) {
		for (int x = 0; x < width; x += step) {
			idx = y*width + x;
			if (fabs(vx[idx])>max_v)max_v = fabs(vx[idx]);
			if (fabs(vy[idx])>max_v)max_v = fabs(vy[idx]);
		}
	}
	arrow_length_scale = 0.8*step / max_v;

	for (int y = 0; y < height; y += step) {
		for (int x = 0; x < width; x += step) {
			idx = y*width + x;
			int x2 = x + arrow_length_scale * vx[idx], y2 = y + arrow_length_scale * vy[idx];
			//		std::cout << x << "," << y << ",  " << x2 << "," << y2 << "\n";

			if (x2 < 0 || x2 >= width || y2 < 0 || y2 >= height) continue;

			draw_arrow(outimg, x, y, x2, y2, color, width, height);
		}
		//break;
	}
}


//=====================draw singularity================
extern void draw_line_rgb_DDA(unsigned char * outimg, const int x1, const int y1, const int x2, const int y2,
	const unsigned char color[], const int width);



