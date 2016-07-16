#include "gl/glut.h"
#include <cstdlib>
#include <cmath>
#include <ctime>

//#define HAVE_OPENCV

#ifdef HAVE_OPENCV

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

int img_width= 512, img_height=512;

unsigned char *noise_img = 0;   //[img_height][img_width]
unsigned char *lic_img = 0;     //[img_height][img_width]
unsigned char *glTex_img = 0;   //[img_height][img_width][3];

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
//		float x = -d*length2(p0p), y = exp(x);
		glm::vec2  V = k*exp (-d*length2(p0p))*p0p;
		if (type == 1)
			V = -V;
		else if (type == 2)
			V[1] = -V[1];
		
		else if (type == 3) {
			float t = V[0];
			V[0] = -V[1];
			V[1] = V[0];
		}
		else if (type == 4) {
			float t = V[1];
			V[1] = -V[0];
			V[0] = V[1];
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

void gen_vector_field_ByElements(float *vx, float *vy, int width, int height, std::vector<VFDesign_Element*> eles)
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
			V = glm::normalize(V);// V /= size;
			vx[idx] = V[0];  vy[idx] = V[1];			
		}
	}
}

void test_gen_vector_field_ByElements(float *vx, float *vy, int width, int height) {
	std::vector<VFDesign_Element*> eles;

	glm::vec2 p0(width / 2, height / 2);
	float d = 1/(width*height), k = 1;
	VFDesign_SingularElement *singular = new VFDesign_SingularElement(p0, d, k, 0);
	eles.push_back((VFDesign_Element*)singular);

	p0[0] = 3 * width / 4; p0[1] = 1 * height / 4; //k = 0.5;
	singular = new VFDesign_SingularElement(p0, d, k, 4);
	eles.push_back((VFDesign_Element*)singular);
	
	
	glm::vec2 v0(1,0);
	VFDesign_ResularElement *regular = new VFDesign_ResularElement(p0, d, v0);
	
//	eles.push_back((VFDesign_Element*)regular);
	gen_vector_field_ByElements(vx, vy, width, height, eles);	
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
		outimg[idx] = color;

		x += scale*vx[idx];
		y += scale*vy[idx];

	} while (1);
}
//----------------forward streamline on a 3-channle image---------------------
void draw_streamline_rgb(unsigned char * outimg, const float *vx, const  float *vy,
	const float x0, const float y0, const  int width, const int height,
	const  unsigned char color[3] , const float scale = 1.0f)
{
	int ix, iy, idx;
	double x = x0, y = y0;
	do
	{
		ix = x + 0.5;
		iy = y + 0.5;
		if (ix<0 || iy<0 || ix >= width || iy >= height) break;

		idx = iy*width + ix;
		unsigned char *p = outimg + 3 * idx;
		p[0] = color[0];
		p[1] = color[1];
		p[2] = color[2];

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
		for (int x = 0; x < height; x+=step) {
			idx = y*width + x;
			unsigned char *p = outimg + 3 * idx;
			if (p[0] == 255) continue;
			draw_streamline_rgb(outimg, vx, vy, x, y, width, height, color);
		}
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
	test_draw_streamline_rgb(glTex_img, vx, vy, img_width, img_height);//draw streamlines on the vector field

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



int main(int argc, char** argv)
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
