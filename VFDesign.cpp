#include <cstdlib>
#include <cmath>
#include <ctime>
#include "gl/glut.h"


#include <cstdlib>
#include <cmath>
#include <ctime>
#include "gl/glut.h"

//#define HAVE_OPENCV

#ifdef HAVE_OPENCV

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

int img_width= 512, img_height=512;

unsigned char *noise_img = 0;   //[img_height][img_width]
unsigned char *lic_img = 0;    //[img_height][img_width]
unsigned char *glTex_img = 0;  //[img_height][img_width][3];

#define EPSILON 1e-10


void gen_noise_img(unsigned char *data, int width, int height)
{
	srand(time(NULL));
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
			data[y * width + x] = rand() % 256;  
}


#include <vector>
#include <glm/glm.hpp>

float length2(glm::vec2 v) {
	return v[0] * v[0] + v[1] * v[1];
}

class VFDesign_Element{
protected:
	glm::vec2 p;
public:
	VFDesign_Element(glm::vec2 p0) :p(p0) { }	
	virtual glm::vec2  operator()(glm::vec2  p) = 0;
};

class VFDesign_SingularElement:VFDesign_Element
 {
	int type; float k,d;
 public:
	VFDesign_SingularElement(glm::vec2 p0, float d, float k, int type):type(type),k(k),d(d), VFDesign_Element(p0){
		
	}
	virtual glm::vec2  operator()(glm::vec2  p) {
		glm::vec2  p0p = p - this->p;		
		if (p[0] == 0 && p[1] == 0 || p[0] == 240 && p[1] == 240)
			int  sss = 0;
//		float x = -d*length2(p0p), y = exp(x);
		glm::vec2  V = k*exp (-d*length2(p0p))*p0p;
		if (type == 1)   //Sink
			V = -V;
		else if (type == 2)  //sandle
			V[1] = -V[1];
		
		else if (type == 3) { //counter-clockwisecenter
			float t = V[0];
			V[0] = -V[1];
			V[1] = t;
		}
		else if (type == 4) { //clockwisecenter
			float t = V[1];
			V[1] = -V[0];
			V[0] = t;
		}	
		return V;		
	};
};
class VFDesign_ResularElement :VFDesign_Element
{
	float  d; 
	glm::vec2 v;
public:
	VFDesign_ResularElement(glm::vec2 p0, float d, glm::vec2 v0) :d(d), v(v0), VFDesign_Element(p0) {
		
	}
	virtual glm::vec2  operator()(glm::vec2  p) {
		glm::vec2  p0p = p - this->p;
		glm::vec2  V = exp(-d*length2(p0p))*v;
		return V;
	};	
};

void gen_vector_field_ByElements(float *vx, float *vy, int width, int height, std::vector<VFDesign_Element*> eles,bool normlizeF = false)
{
	int size = eles.size();
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int idx = y * width + x;
			glm::vec2 p(x, y);
			glm::vec2 V(0,0);
			for (auto ele : eles) {
				glm::vec2 t = (*ele)(p);
				V += (*ele)(p);
			}
			if (normlizeF)
				V = glm::normalize(V);// V /= size;
			vx[idx] = V[0];  vy[idx] = V[1];			
		}
	}
}

void test_gen_vector_field_ByElements(float *vx, float *vy, int width, int height) {
	std::vector<VFDesign_Element*> eles;
        glm::vec2 p0(width /4, height / 2);

	float d = 10./(width*height), k = 1;  
//	float d = 0.0001 , k = 1.;
	int type = 0; VFDesign_SingularElement *singular = 0;
#if 1		
	singular = new VFDesign_SingularElement(p0, d, k, type);
	eles.push_back((VFDesign_Element*)singular);
#endif

#if 1	
	type = 1;
	p0[0] = 3 * width / 4; p0[1] = 1 * height / 2; //k = 0.5;
	singular = new VFDesign_SingularElement(p0, d, k, type);
	eles.push_back((VFDesign_Element*)singular);
#endif	
#if 1	
	type = 3;
	p0[0] = 1 * width / 2; p0[1] = 3 * height / 4; //k = 0.5;
	singular = new VFDesign_SingularElement(p0, d, k, type);
	eles.push_back((VFDesign_Element*)singular);
#endif	
#if 1	
	glm::vec2 v0(1,0);
	VFDesign_ResularElement *regular = new VFDesign_ResularElement(p0, d, v0);
	eles.push_back((VFDesign_Element*)regular);
#endif
	gen_vector_field_ByElements(vx, vy, width, height, eles,true);	
}


#define TERM_CRITERIA_SINGULAR 0x01
#define TERM_CRITERIA_BOUNDARY 0x02
#define TERM_CRITERIA_MAX_ITER 0x04

struct trace_streamline_termination_criteria{
	int type ;
	int max_iter = 0;
	trace_streamline_termination_criteria(int iter = 10,int t = TERM_CRITERIA_SINGULAR | TERM_CRITERIA_BOUNDARY | TERM_CRITERIA_MAX_ITER)
	{	
		type = t;  max_iter = iter;		
	}
	
};

int lic_streamline(const unsigned char* noise, const float *vx, const  float *vy,
	const double x0, const double y0, const  int width, const int height,
	trace_streamline_termination_criteria tc  = trace_streamline_termination_criteria() )
{
	double x, y;
	int ix ,iy;
	int noise_color = 0;
	if (x0 == 0 && y0 == 247)
		int sss = 0;

	float vx0, vy0;
	int num = 0,idx=0;
	for (int i = 0; i < 2; i++) {
		int j = 0;	
		x = x0; y = y0;
		do
		{
			ix = x +0.5; iy = y+0.5;
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
			x +=  vx0 ;
			y +=  vy0 ;
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


// generate texture using line integral converlution from the vector field
void gen_lic(unsigned char * output, const unsigned char * noise, const float *vx, const float *vy, const int width, const int height)
{
	int y, x;

	trace_streamline_termination_criteria tc(10);// , TERM_CRITERIA_SINGULAR | TERM_CRITERIA_MAX_ITER);
	for (y = 0; y < height; y++)
	{
		for (x = 0; x < width; x++)		
		{
			int sum_val = lic_streamline(noise, vx, vy, x, y, width, height,tc);			
			output[y*width + x] = sum_val ;
		}
	}
}

void gray2rgb(unsigned char * rgb, const unsigned char * gray, const int width, const int height) {
	const unsigned char *p_gray = gray;
	unsigned char *p_rgb = rgb;
	for(int y = 0 ; y< height;y++)
		for (int x = 0; x < width; x++, p_gray++) {
			*p_rgb++ = *p_gray;
			*p_rgb++ = *p_gray;
			*p_rgb++ = *p_gray;
		}

}

//----------------forward streamline on a 1-channle image---------------------
void draw_streamline_gray(unsigned char * outimg, const float *vx, const  float *vy,
	const float x0, const float y0, const  int width, const int height,
	const unsigned char color = 255, const float scale = 1.0f)
{
	int ix, iy, idx;
	double x = x0, y = y0;
	do
	{
		ix = x + 0.5;
		iy = y + 0.5;
		if (ix<0 || iy<0 || ix >= width || iy >= height) break;

		idx = iy*width + ix;	
		outimg[idx] = color; //		putpixel(outimg, x, y, color, width); 

		x += scale*vx[idx];
		y += scale*vy[idx];

	} while (1);
}
//----------------forward streamline on a 3-channle image---------------------
void draw_streamline_rgb(unsigned char * outimg, const float *vx, const  float *vy,
	const float x0, const float y0, const  int width, const int height,
	const  unsigned char color[ ] , const float scale = 1.0f)
{
	int ix, iy, idx;
	double x = x0, y = y0;
	do
	{
		ix = x + 0.5;
		iy = y + 0.5;
		if (ix<0 || iy<0 || ix >= width || iy >= height) break;

		idx = iy*width + ix;
#if 1
		unsigned char *p = outimg + 3 * idx;
		p[0] = color[0];		p[1] = color[1];		p[2] = color[2];
#else
		putpixel(outimg, ix, iy, color, width);
#endif
		x += scale*vx[idx];
		y += scale*vy[idx];


	} while (1);
}

void test_draw_streamline_rgb(unsigned char * outimg, const float *vx, const  float *vy,
	const  int width, const int height)
{
	unsigned char color[3] = {255,0,0}, scale = 1.0f;

	int idx=0,step = 30;
	for (int y = 0; y < height; y+=step) {
		for (int x = 0; x < width; x += step) {
			idx = y*width + x;
			unsigned char *p = outimg + 3 * idx;
			if (p[0] == 255) continue;
			draw_streamline_rgb(outimg, vx, vy, x, y, width, height, color);
		}
	}
}



//-------------- draw arrow ------------
inline void putpixel(unsigned char * outimg, const int x, const int y, const unsigned char color, const int width){
	int idx = y*width + x;
	unsigned char *p = outimg + idx;
	p[0] = color;
}
inline void putpixel(unsigned char * outimg, const int x, const int y, const unsigned char color[], const int width){
	int idx = y*width + x;
	unsigned char *p = outimg + 3 * idx;
	p[0] = color[0];  p[1] = color[1];  p[2] = color[2];
}

void draw_line_rgb_DDA(unsigned char * outimg, const int x1, const int y1, const int x2, const int y2, 
	const unsigned char color[],const int width){
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
	
	for (iCount = 1; iCount <= iSteps; iCount++)	{
		putpixel(outimg, floor(x), floor(y), color, width);
	
		x -= xInc;
		y -= yInc;
	}
	putpixel(outimg, x, y, color, width);
	return;
}

void draw_arrow(unsigned char * outimg, const int x1, const int y1, const int x2, const int y2,
	const unsigned char color[], const int width,const int height)
{
	float delta = 0.2, D[2] = { x2 - x1, y2 - y1 }, D2[2] = { delta*D[0], delta*D[1] },
		A[2] = { x2 - D2[0], y2 - D2[1] }, D3[2] = {-D2[1]/2,D2[0]/2};

	int x3 = A[0] + D3[0], y3 = A[1] + D3[1]; 
	int x4 = A[0] - D3[0], y4 = A[1] - D3[1];



	draw_line_rgb_DDA(outimg, x1, y1, x2, y2, color, width);
	if (x3 < 1 || x3 >= width || y3 < 1 || y3 >= height) return;
	draw_line_rgb_DDA(outimg, x3, y3, x2, y2, color, width);
	if (x4 < 1 || x4 >= width || y4 < 1 || y4 >= height) return;
	draw_line_rgb_DDA(outimg, x4, y4, x2, y2, color, width);
}

#include <iostream>
void test_draw_arrow_rgb(unsigned char * outimg, const float *vx, const  float *vy,
	const  int width, const int height,const int step = 30)
{
	unsigned char color[3] = { 255, 0, 0 }, scale = 1.0f;

//	std::cout << width << "," << height << "\n";
//	draw_line_rgb_DDA(outimg, 10, 10, 100, 100, color, width);

//	return;

	int idx = 0;
	float arrow_length_scale = 1.0f,max_v=0;

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
			
			draw_arrow(outimg,  x, y, x2,y2, color,width,height);
		}
		//break;
	}
}


void init_vector_field() {
	int size = img_height*img_width;
	glTex_img = new unsigned char[size*3];

	unsigned char* lic_img = new unsigned char[size];
	noise_img = new unsigned char[size];
	float *vx = new float[size];
	float *vy = new float[size];
	gen_noise_img(noise_img, img_width, img_height);
	test_gen_vector_field_ByElements(vx, vy, img_width, img_height); //generate vector field from a sparse set of feature elements
	gen_lic(lic_img, noise_img, vx, vy, img_width, img_height);
	gray2rgb(glTex_img, lic_img, img_width, img_height);
//	test_draw_streamline_rgb(glTex_img, vx, vy, img_width, img_height);//draw streamlines on the vector field
	test_draw_arrow_rgb(glTex_img, vx, vy, img_width, img_height); //draw arrows on the vector field

#ifdef HAVE_OPENCV
#define SHOW_IMAGE(x) {cv::imshow(#x, x); cv::waitKey();}
	cv::Mat noiseImg(img_height, img_width, CV_8UC1, noise_img);
	cv::Mat licImg(img_height, img_width, CV_8UC1, lic_img);
	SHOW_IMAGE(licImg);
#endif

}


//----------------------------------------------------
static inline void initGL(void)
{	
	glViewport(0, 0, (GLsizei)img_width, (GLsizei)img_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluOrtho2D(0, 1, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT);
	glTexParameteri(GL_TEXTURE_2D,
		GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D,
		GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D,
		GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,
		GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV,
		GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_FLAT);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glClear(GL_COLOR_BUFFER_BIT);
}



static inline void display(void)
{
	glViewport(0, 0, (GLsizei)img_width, (GLsizei)img_height);
#if 0
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, 1, 0, 1);
#endif

	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClear(GL_COLOR_BUFFER_BIT);



	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_FLAT);

   // Display LIC image using texture mapping
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height, 0,
		GL_RGB, GL_UNSIGNED_BYTE, glTex_img); 

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0, 0.0); glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0, 1.0); glVertex2f(0.0, 1.0);
	glTexCoord2f(1.0, 0.0); glVertex2f(1.0, 0.0);
	glTexCoord2f(1.0, 1.0); glVertex2f(1.0, 1.0);
	glEnd();
	glDisable(GL_TEXTURE_2D);

	glutSwapBuffers();
	glFlush();
}


int main_VFDesign(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(img_width, img_height);
	glutCreateWindow("VFDesign");
	glutDisplayFunc(display);
	glutIdleFunc(display);
	initGL();

	init_vector_field();

	glutMainLoop();
	return 0;
}

