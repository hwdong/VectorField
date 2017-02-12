#include <cstdlib>
#include <cmath>
#include <ctime>
#include "VectorFieldDesignData.h"
#include "gl/freeglut.h"
#include "gl_util.h"

const int image_width = 512, image_height = 512;
const int viewport_width = 512, viewport_height = 512;
//extern  VectorFieldDesignData<float> *vector_field_data;
VectorFieldDesignData<float> *vector_field_data = 
     new VectorFieldDesignData<float>(image_width, image_height, "VTT.txt");
const int DRAW_POINT_SIZE = 4;

std::vector<float> color_table = {
	1,0,0,  //red
	0,1,0,  //green
	0,0,1,  //blue
	1,1,0, //yellow
	1,0,1, //Cyan
	0,1,1,  //Magenta
	1,1,1,  //white
	0,0,0  //black
};

static std::vector<float> xs, ys;
static bool drawing = false;
static int mouse_x, mouse_y;
static bool selecting = false;

struct sketching_data {
	vector<vector<std::vector<float>>> xsss, ysss;
	vector<std::vector<float>> xss, yss;
};

std::vector<sketching_data> sketching_datas(1);
int  current_sketch_data_idx=0;

//----------------------------------------------------
static inline void initGL(void)
{
	void read_error_image(const char *file); read_error_image("0.txt");
#if USE_OPENGL_GLEW
	// Init GLEW
	glewExperimental = GL_TRUE;
	glewInit();
	if (glewIsSupported("GL_VERSION_3_3"))
		printf("Ready for OpenGL 3.3\n");
	else {
		printf("OpenGL 3.3 not supported\n");
		;// exit(1);
	}
#endif

//	vector_field_data->generate_noise_image();
	glViewport(0, 0, (GLsizei)viewport_width, (GLsizei)viewport_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0); //gluOrtho2D(0, 1, 0, 1);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
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
	glDisable(GL_STENCIL_TEST);

}

static inline void resizeGL(int width, int height)
{
	height = height ? height : 1;
	glViewport(0, 0, (GLint)width, (GLint)height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
#if 0
	gluPerspective(45.0f, (GLfloat)width / (GLfloat)height, 0.1f, 100.0f);
#else	
	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0); //gluOrtho2D(0, 1, 0, 1);
#endif
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void paintGL()
{
	//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClear(GL_COLOR_BUFFER_BIT);
//	glLoadIdentity();

#if 0
	glTranslatef(-1.5f, 0.0f, -6.0f);

	glBegin(GL_TRIANGLES);
	glVertex3f(0.0f, 1.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, 0.0f);
	glEnd();

	glTranslatef(3.0f, 0.0f, 0.0f);

	glBegin(GL_QUADS);
	glVertex3f(-1.0f, 1.0f, 0.0f);
	glVertex3f(1.0f, 1.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, 0.0f);
	glEnd();
#else 
	/*
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_FLAT);

	// Display LIC image using texture mapping
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, vector_field_data->image_width, vector_field_data->image_height, 0,
		GL_RGB, GL_UNSIGNED_BYTE, &(vector_field_data->glTex_img[0]));

	glBegin(GL_QUAD_STRIP);
	glTexCoord2f(0.0, 0.0); glVertex2f(0.0, 0.0);
	glTexCoord2f(0.0, 1.0); glVertex2f(0.0, 1.0);
	glTexCoord2f(1.0, 0.0); glVertex2f(1.0, 0.0);
	glTexCoord2f(1.0, 1.0); glVertex2f(1.0, 1.0);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	*/
#endif
	void draw(); draw();

	glutSwapBuffers();
	glFlush();
	
}

void update() {
	glutPostRedisplay();
}

int main_glut(int argc, char** argv)
{
	const int win_width = 512, win_height = 512;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(win_width, win_height);
	glutCreateWindow("VFDesign");
	glutDisplayFunc(paintGL);
	glutIdleFunc(paintGL);
	glutReshapeFunc(resizeGL);

	void mouseButton(int button, int state, int x, int y);	glutMouseFunc(mouseButton);
	void mouseMove(int x, int y);   glutMotionFunc(mouseMove);
	void passiveMouseMove(int x, int y);  glutPassiveMotionFunc(passiveMouseMove);
	void processSpecialKeys(int key, int x, int y);	glutSpecialFunc(processSpecialKeys);
	void processNormalKeys(unsigned char key, int x, int y); glutKeyboardFunc(processNormalKeys);

	initGL();

//	init_vector_field();

	glutMainLoop();
	return 0;
}

void mouseButton(int button, int state, int x, int y)
{
	if (selecting) {
		vector_field_data->select(x,y);
		return;
	}
	if (state == GLUT_DOWN&&button == GLUT_LEFT_BUTTON) {		
		drawing = true;		
		xs.clear(); 	ys.clear();
	}
	else if (state == GLUT_UP&&button == GLUT_LEFT_BUTTON) {
		if (xs.size() > 4) {			
			vector_field_data->add_sketch(xs, ys);
	//		sketching_datas[current_sketch_data_idx].xss.push_back(xs); 
	//		sketching_datas[current_sketch_data_idx].yss.push_back(ys);
			update();
		}
		drawing = false;
	}		
}

void mouseMove(int x, int y) {
	mouse_x = x; mouse_y = y;
	if (drawing) {		
		GLdouble ox, oy, oz;
		viewport2object(x, y, ox, oy, oz);
		xs.push_back(ox); ys.push_back(oy);
		update();
	}
}
void passiveMouseMove(int x, int y) {
	mouse_x = x; mouse_y = y;
}
void processNormalKeys(unsigned char key_, int x, int y) {
	int key = key_;
//	if (key<'0' || key>'7') {		processSpecialKey(e);	return;	}

	if (key == 127) {
		vector_field_data->del();	update(); return;
	}
	else if (key == 'a') {
		vector_field_data->add_interpolate_curve();
		return;
	}
	GLdouble ox, oy, oz;
	viewport2object(x, y, ox, oy, oz);

	int type = 0;
	Eigen::Matrix<float, 1, -1> pos(1, 2), vec(1, 2);
	vec(0) = 1; vec(1) = 0;
#if 0
	float d = 0.0004, k = 1.;
	pos(0) = ox*vector_field_data->image_width; pos(1) = oy*vector_field_data->image_height;
#else
	float d = 100, k = 512.;
	pos(0) = ox; pos(1) = oy;
#endif
	//	pos(0) = 0.5; pos(1) = 0.5;
	if (key >= '1'&&key <= '5') {
		type = key - '0';
		vector_field_data->add_singularity_element(pos, type, k, d);
	}
	else {
		type = 0;
		if (key == '6')
			vec(0) = -1;
		vector_field_data->add_regularity_element(pos, vec, type, d);
	}
	update();
}

void processSpecialKeys(int key, int x, int y) {
	/*
	if (key == GLUT_KEY_F1) {
		log_comp_designTriStrip_read(xs, ys);
		vector_field_data->add_sketch(xs, ys);
	}
	else if (key == GLUT_KEY_F2) {
		
	}
	else if (key == GLUT_KEY_F3) {
	
	}
	else if (key == GLUT_KEY_F4) {
		//save sketches
		sketching_datas[current_sketch_data_idx].xsss.push_back(
			sketching_datas[current_sketch_data_idx].xss);
		sketching_datas[current_sketch_data_idx].ysss.push_back(
			sketching_datas[current_sketch_data_idx].yss);
		sketching_datas[current_sketch_data_idx].xss.clear(); 
		sketching_datas[current_sketch_data_idx].yss.clear(); // xs.clear(); ys.clear();
		vector_field_data->clear();
		//	vector_field_data->updated_vector_field();
	}
	else if (key == GLUT_KEY_F5) {
		vector_field_data->add_sketch(xs, ys);
		sketching_datas[current_sketch_data_idx].xss.push_back(xs); 
		sketching_datas[current_sketch_data_idx].yss.push_back(ys);
	}*/
	if (key == GLUT_KEY_F8) {
		vector_field_data->interpolate(); 	
		void compute_error(VectorFieldDesignData<float> *vField);
		compute_error(vector_field_data);
	}
	if (key == GLUT_KEY_F9) {
		vector_field_data->show_design = !vector_field_data->show_design;
	}
	else if (key == GLUT_KEY_F10) {
		vector_field_data->clone_push();
		
	}
	else if (key == GLUT_KEY_F11) {
		selecting = !selecting;
		vector_field_data->show_analysis(!selecting);
	}
	else if (key == GLUT_KEY_F12) {
		vector_field_data->show_mesh = !vector_field_data->show_mesh;
	}
	else if (key == GLUT_KEY_F7) {
		
	}
	
	update();
	
}

//---------------Draw data------------

static inline void draw_sketch2d() {
	glPointSize(DRAW_POINT_SIZE);
	glColor3f(1.0, 0., 0.);
	glBegin(GL_POINTS);
	for (int i = 0; i < xs.size(); i++) {
		glVertex2f(xs[i], ys[i]);
	}
	glEnd();
}
std::vector<unsigned char> error_image;

void draw() {
	if (error_image.size() >0) {
		

		glEnable(GL_TEXTURE_2D);
		glShadeModel(GL_FLAT);

		// Display LIC image using texture mapping
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0,
			GL_RGB, GL_UNSIGNED_BYTE, &(error_image[0]));

		glBegin(GL_QUAD_STRIP);
		glTexCoord2f(0.0, 0.0); glVertex2f(0.0, 0.0);
		glTexCoord2f(0.0, 1.0); glVertex2f(0.0, 1.0);
		glTexCoord2f(1.0, 0.0); glVertex2f(1.0, 0.0);
		glTexCoord2f(1.0, 1.0); glVertex2f(1.0, 1.0);
		glEnd();
		glDisable(GL_TEXTURE_2D);

		static int num = 0; num++; if (num == 10) num = 0;
		std::string s = std::to_string(num);
		s += ".txt";
		void read_error_image(const char *file);		read_error_image(s.c_str());
		return;
	}
	vector_field_data->draw_vectorfield();
	if(drawing)
		draw_sketch2d();//current sketching curve
}


void compute_error(VectorFieldDesignData<float> *vField) {
	
	int n = vField->interpolated_SingleframeDesignDatas.size() - 1;
	vector<float> errors(n);
	for (int i = 0; i < n; i++) {
		std::vector<float> vx1 = vField->interpolated_SingleframeDesignDatas[i].vx,
			vy1 = vField->interpolated_SingleframeDesignDatas[i].vy,
			vx2 = vField->interpolated_SingleframeDesignDatas[i + 1].vx,
			vy2 = vField->interpolated_SingleframeDesignDatas[i + 1].vy;

		std::string s = std::to_string(i);
		s += ".txt";
		float error = 0;
		float max_error = 0;

		int size = 512 * 512;
		ofstream oF(s);
		oF << "512 512\n";
		for(int y = 0 ; y<512;y++)
			for (int x = 0; x < 512; x++) {
				int idx = y * 512 + x;
				
				float x1 = vx1[idx], y1 = vy1[idx],
					x2 = vx2[idx], y2 = vy2[idx];
				float err = sqrt( (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
				error += err;
				oF << err << " ";			
			}
		oF.close();
		error /= size;
		errors.push_back(error);
		std::cout << error<<"\n";
	}
}


void read_error_image(const char *file) {
	ifstream iF(file);
	if (!iF) return;
	int w, h;
	iF >> w >> h;
	int s = w*h * 3;
	vector<float> data(s);
	float err, max_error = 0;
	for (int i = 0; i < s; i++) {
		iF >> err; data[i] = err;
		if (err > max_error) max_error = err;
	}
	for (int i = 0; i < s; i++) {
		data[i] /= max_error;
		data[i] *= 255.;
	}
	error_image.resize(w*h * 3);
	for(int y = 0 ; y<h;y++)
		for (int x = 0; x < w; x++) {
			int idx = y*w + x;
			error_image[idx * 3] = data[idx];
			error_image[idx * 3+1] = data[idx];
			error_image[idx * 3+1] = data[idx];
		}	
	int sss = 0;
}