#include <cstdlib>
#include <cmath>
#include <ctime>
#include "gl/glut.h"

#include "Visualize.h"
#include "Design.h"

//#define HAVE_OPENCV

#ifdef HAVE_OPENCV

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif


//int img_width = 64, img_height = 64;

int img_width = 512, img_height = 512;

vector<unsigned char> glTex_img;

/*
unsigned char *noise_img = 0;   //[img_height][img_width]
unsigned char *lic_img = 0;    //[img_height][img_width]
unsigned char *glTex_img = 0;  //[img_height][img_width][3];

#define EPSILON 1e-10
*/

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

VectorField<float> *vField = 0;
void updated_vector_field() {
	VectorField2Image(*vField, glTex_img);
//	return ;
	std::vector<Singularity<float>> singularities;
	VF_Analysis(*vField, singularities);
	test_draw_singularities_rgb(&(glTex_img[0]), singularities, img_width);
}

std::vector<VFDesign_Element<float>*> eles;

void init_vector_field() {
#if 0
	test_gen_vector_field_ByElements(vField, img_width, img_height); 
	update_vector_field();
	return;
#else
	typedef float T;

	Eigen::MatrixXf V;	Eigen::MatrixXi F;
	grid2mesh(img_width, img_height, V, F);
	Eigen::Matrix<T, -1, -1> VF;
//	vField = new VectorField<T>(V, F, VF); 
	vField = new VectorField<T>(V, F, VF, "VTT.txt");

	
	readDesignElements(eles, "element.txt");
	std::cout << "has read design elements\n";

	gen_vector_field_ByElements(*vField, eles);
	std::cout << "has generated vector field\n";
	updated_vector_field();
	std::cout << "has updated vector field\n";
#endif
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
		GL_RGB, GL_UNSIGNED_BYTE, &(glTex_img[0]));

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
	const int win_width=512, win_height=512;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(win_width, win_height);
	glutCreateWindow("VFDesign");
	glutDisplayFunc(display);
	glutIdleFunc(display); 

	void mouseButton(int button, int state, int x, int y);	glutMouseFunc(mouseButton);
	void mouseMove(int x, int y);   glutMotionFunc(mouseMove);
	void processSpecialKeys(int key, int x, int y);	glutSpecialFunc(processSpecialKeys);
	void processNormalKeys(unsigned char key, int x, int y); glutKeyboardFunc(processNormalKeys);

	initGL();

	init_vector_field();

	glutMainLoop();
	return 0;
}


#include "Mesh.h"
int main(int argc, char** argv) {
	return main_VFDesign(argc,argv);
//		void test_gen_vector_field_ByElements(VectorField<double> *&vField); 	test_gen_vector_field_ByElements(vField); return 0;
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

   glPointSize(4);
   glColor3f(0.5, 1., 0.);
   glBegin(GL_POINTS);
   for (int i = 0; i < eles.size(); i++) {
	   glVertex2f(eles[i]->p(0)/img_width, eles[i]->p(1)/img_height);
   }
   glEnd();
    
//	void test_isInDraw(); test_isInDraw();
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

void coords_viewport2windows( int x,  int y, GLdouble &ox,
	GLdouble &oy, GLdouble &oz ) 
{
	GLint viewport[4];
	GLdouble modelview[16], projection[16];
	GLfloat wx = x, wy, wz;
//	GLdouble ox = 0.0, oy = 0.0, oz = 0.0;

	glGetIntegerv(GL_VIEWPORT, viewport);
	y = viewport[3] - y;
	wy = y;
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &wz);
	gluUnProject(wx, wy, wz, modelview, projection, viewport, &ox, &oy, &oz);
	xs.push_back(ox); ys.push_back(oy);
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
template<typename T>
void isIn(Eigen::Matrix<T, -1, -1> &V, vector<int> &in_vids,
	const std::vector<T> xs, const std::vector<T> &ys,
	const int width , const int height )
{
	int n = xs.size();
	std::vector<T> x(n), y(n);
	for (int i = 0; i < n; i++) {
		x[i] = xs[i] * width; y[i] = ys[i] * height;
	}	
	isIn(V, in_vids, x, y);
}

template<typename T>
void test_isIn(unsigned char * rgbimg,
	Eigen::Matrix<T, -1, -1> &V, 
	const std::vector<T> &xs, const std::vector<T> &ys,
	const int width = 512, const int height = 512)
{
	int n = xs.size();
	std::vector<T> x(n), y(n);
	for (int i = 0; i < n; i++) {
		x[i] = xs[i] * width; y[i] = ys[i] * height;
	}
	vector<int> in_vids;
	isIn(V, in_vids, x, y);

	unsigned char color[3] = { 0, 0,255 };
	
	int vid,idx;
	for (int i = 0; i < in_vids.size(); i++) {
		vid = in_vids[i];

		idx = V(vid,1)*width + V(vid, 0);
		unsigned char *p = rgbimg + 3 * idx;
		p[0] = color[0];  p[1] = color[1];  p[2] = color[2];
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


#include "Smooth.h"
void processSpecialKeys(int key, int x, int y) {
	vector<int> in_vids;
	switch (key) {
	case GLUT_KEY_F1:
	//	test_isIn(xs,ys,tx,ty);	
		test_isIn(&(glTex_img[0]), vField->V,xs, ys,img_width,img_height);
		glutPostRedisplay();
		break;
	case GLUT_KEY_F2:
		isIn(vField->V, in_vids, xs, ys, img_width, img_height);
		smooth(*vField, in_vids);
		updated_vector_field();
		glutPostRedisplay();
		break;
	case GLUT_KEY_F3:
		gen_vector_field_ByElements(*vField, eles);
		updated_vector_field();
		glutPostRedisplay();
		break;
	}
}
void processNormalKeys(unsigned char key, int x, int y) {
	float d = 0.0004, k = 1.;
	int type=0;
	Eigen::Matrix<float, 1, -1> pos(1, 2), vec(1, 2);
	vec(0) = 1; vec(1) = 0;

	GLdouble ox, oy, oz;
	coords_viewport2windows(x, y, ox, oy, oz);

	pos(0) = ox*img_width; pos(1) = oy*img_height;

	if (key >= '1'&&key<='5') {
		type = key - '0';
		VFDesign_SingularElement<float>* singular = new VFDesign_SingularElement<float>
			(pos, d, type, k);
		eles.push_back((VFDesign_Element<float>*)singular);
	}
	else {
		type = 0;
		if (key == '6') pos(0) = -1;
		VFDesign_ResularElement<float> *resular = new VFDesign_ResularElement<float>(pos, d, type, vec);
		eles.push_back((VFDesign_Element<float>*)resular);
	}
	std::cout <<type<<"   "<<pos(0)<<", "<<pos(1)<< "\n";

	glutPostRedisplay();
}
