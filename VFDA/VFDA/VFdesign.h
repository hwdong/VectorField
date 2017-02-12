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

int img_width = 512, img_height = 512;

unsigned char *noise_img = 0;   //[img_height][img_width]
unsigned char *lic_img = 0;    //[img_height][img_width]
unsigned char *glTex_img = 0;  //[img_height][img_width][3];

#define EPSILON 1e-10





#include <vector>
#include <glm/glm.hpp>

float length2(glm::vec2 v) {
	return v[0] * v[0] + v[1] * v[1];
}

class VFDesign_Element {
protected:
	glm::vec2 p;
public:
	VFDesign_Element(glm::vec2 p0) :p(p0) { }
	virtual glm::vec2  operator()(glm::vec2  p) = 0;
};

class VFDesign_SingularElement :VFDesign_Element
{
	int type; float k, d;
public:
	VFDesign_SingularElement(glm::vec2 p0, float d, float k, int type) :type(type), k(k), d(d), VFDesign_Element(p0) {

	}
	virtual glm::vec2  operator()(glm::vec2  p) {
		glm::vec2  p0p = p - this->p;

		//		float x = -d*length2(p0p), y = exp(x);
		glm::vec2  V = k*exp(-d*length2(p0p))*p0p;
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

void gen_vector_field_ByElements(float *vx, float *vy, int width, int height, std::vector<VFDesign_Element*> eles, bool normlizeF = false)
{
	int size = eles.size();
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int idx = y * width + x;
			glm::vec2 p(x, y);
			glm::vec2 V(0, 0);
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

	glm::vec2 p0(width / 4, height / 2);

	float d = 10. / (width*height), k = 1;
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
	glm::vec2 v0(1, 0);
	VFDesign_ResularElement *regular = new VFDesign_ResularElement(p0, d, v0);
	eles.push_back((VFDesign_Element*)regular);
#endif
	gen_vector_field_ByElements(vx, vy, width, height, eles);
}

void copyVF(const float *vx, const float *vy, float *&dst_vx, float *&dst_vy, int width, int height)
{
	if (dst_vx) delete[] dst_vx; if (dst_vy) delete[] dst_vy;
	int size = width*height;
	dst_vx = new float[size]; dst_vy = new float[size];
	for (int i = 0; i < size; i++) {
		dst_vx[i] = vx[i];
		dst_vy[i] = vy[i];
	}

}
void normalizeVF(float *vx, float *vy, int width, int height)
{
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int idx = y * width + x;
			glm::vec2 V(vx[idx], vy[idx]);
			V = glm::normalize(V);
			vx[idx] = V[0];  vy[idx] = V[1];
		}
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
	const  unsigned char color[], const float scale = 1.0f)
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
	unsigned char color[3] = { 255,0,0 }, scale = 1.0f;

	int idx = 0, step = 30;
	for (int y = 0; y < height; y += step) {
		for (int x = 0; x < width; x += step) {
			idx = y*width + x;
			unsigned char *p = outimg + 3 * idx;
			if (p[0] == 255) continue;
			draw_streamline_rgb(outimg, vx, vy, x, y, width, height, color);
		}
	}
}



#if 0
void init_vector_field() {
	int size = img_height*img_width;
	glTex_img = new unsigned char[size * 3];

	unsigned char* lic_img = new unsigned char[size];
	noise_img = new unsigned char[size];
	float *vx = new float[size];
	float *vy = new float[size];
	gen_noise_img(noise_img, img_width, img_height);
	test_gen_vector_field_ByElements(vx, vy, img_width, img_height); //generate vector field from a sparse set of feature elements

																	 //gen_lic(lic_img, noise_img, vx, vy, img_width, img_height);
	float *des_vx = 0, *dst_vy = 0;
	copyVF(vx, vy, des_vx, dst_vy, img_width, img_height);
	normalizeVF(des_vx, dst_vy, img_width, img_height);
	gen_lic(lic_img, noise_img, des_vx, dst_vy, img_width, img_height);


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

#endif

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
	
	void draw2d(); draw2d();

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

	void mouseButton(int button, int state, int x, int y);	glutMouseFunc(mouseButton);
	void mouseMove(int x, int y);   glutMotionFunc(mouseMove);
	void processSpecialKeys(int key, int x, int y);	glutSpecialFunc(processSpecialKeys);

	initGL();

	init_vector_field();

	glutMainLoop();
	return 0;
}


#include "Mesh.h"
int main(int argc, char** argv) {
	return main_VFDesign(argc,argv);
//	void test_gen_vector_field_ByElements();  test_gen_vector_field_ByElements(); return 0;
	void test_TraceInATriangle(); test_TraceInATriangle(); return 0;
	void test_grid2mesh(); test_grid2mesh();
	void test_GetBarycentricFacters(); test_GetBarycentricFacters();
	
	return 0;
}


//-----------------draw curves---------------
//http://stackoverflow.com/questions/19884182/moving-a-drawing-around-in-opengl-with-mouse
//http://www.opengl-tutorial.org/beginners-tutorials/tutorial-6-keyboard-and-mouse/
//http://stackoverflow.com/questions/9123648/opengl-2d-mouse-click-location

std::vector<float> xs, ys;
static bool drawing = false;
void draw2d() {
	glPointSize(4);
	glColor3f(1.0, 0., 0.);
	glBegin(GL_POINTS);
	for (int i = 0; i < xs.size(); i++) {
		glVertex2f(xs[i], ys[i]);
   }
	glEnd();

	void test_isInDraw(); test_isInDraw();
}
void mouseButton(int button, int state, int x, int y)
{	
	if (state == GLUT_DOWN&&button == GLUT_LEFT_BUTTON)	{	
		drawing = true; xs.clear(); ys.clear();
	}
	else if (state == GLUT_UP&&button == GLUT_LEFT_BUTTON)
		drawing = false;	
	else 	return;

}

void mouseMove(int x, int y) {
	GLint viewport[4];
	GLdouble modelview[16], projection[16];
	GLfloat wx = x, wy, wz;
	GLdouble ox = 0.0, oy = 0.0, oz = 0.0;

	glGetIntegerv(GL_VIEWPORT, viewport);
	y = viewport[3] - y;
	wy = y;
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &wz);
	gluUnProject(wx, wy, wz, modelview, projection, viewport, &ox, &oy, &oz);
	xs.push_back(ox); ys.push_back(oy);
	std::cout << xs.size() << "\n";
	glutPostRedisplay();
}

template<typename T>
inline bool  isIntersection(const T p[2], const T A[2], const T B[2]) {
	if (p[1] >= B[1] || p[1] < A[1]) return false;
	T x = (p[1] - A[1]) / (B[1] - A[1])*(B[0] - A[0]) + A[0];
	return x >= p[0];
}

template<typename T>
bool isIn(T p[2], const std::vector<T> xs, const std::vector<T> &ys) {
	int num_intersection = 0;
	int n = xs.size();
	T A[2] = { xs[n - 1], ys[n - 1] };
	for (int i = 0; i < n; i++) {
		T B[2] = { xs[i], ys[i] };
		//if the scanline of p intersect with AB
		bool ret;
		if (B[1] >= A[1])
			ret = isIntersection(p, A, B);
		else 
			ret = isIntersection(p, B, A);
		if (ret) num_intersection++;
		A[0] = B[0]; A[1] = B[1];
	}
	return num_intersection % 2 == 1;
}

template<typename T>
void isIn(Eigen::Matrix<T, -1, -1> &V, vector<int> &in_vids,
	const std::vector<T> xs, const std::vector<T> &ys)
{
	T p[2];
	for (int i = 0; i < V.rows(); i++) {		
		p[0] = V(i, 0); p[1] = V(i, 1);
		if (isIn(p, xs, ys))
			in_vids.push_back(i);
	}
}

std::vector<float> tx, ty;

template<typename T>
void test_isIn(const std::vector<T> xs,
	const std::vector<T> &ys, 	
	std::vector<T> &tx, std::vector<T> &ty,
	const int width=512, const int height=512 )
{
	tx.clear(); ty.clear();
	int n = xs.size();
	std::vector<T> x(n), y(n);
	T minx = width, miny= height, maxx=0, maxy=0;
	for (int i = 0; i < n; i++) {
		x[i] = xs[i] * width; y[i]  = ys[i]*height;
		if (x[i] < minx) minx = x[i];
		else if (x[i] > maxx) maxx = x[i];

		if (y[i] < miny) miny = y[i];
		else if (y[i] > maxy) maxy = y[i];
	}
	if (n == 0) return;

	int L = minx, R = maxx + 0.5, B = miny, Top = maxy + 0.5;
	T p[2];
	for(int j= B ; j<Top;j++)
		for (int i = L; i < R; i++) {
			p[0] = i; p[1] = j;
			if (isIn(p, x, y)) 
			{
				tx.push_back( T(i)/width);
				ty.push_back( T(j)/height);
			}
		}
	
}
void test_isInDraw() {
	if (tx.size() == 0) return;
	glPointSize(6);
	glColor3f(0.0, 0., 1.);
	glBegin(GL_POINTS);
	for (int i = 0; i < tx.size(); i++) {
		glVertex2f(tx[i], ty[i]);
	}
	glEnd();
}


void processSpecialKeys(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_F1:
		test_isIn(xs,ys,tx,ty);	
		glutPostRedisplay();
		break;
	}
}