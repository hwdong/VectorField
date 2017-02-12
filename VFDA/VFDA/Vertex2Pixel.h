#pragma once

#define USE_OPENGL_GLEW 0

#if USE_OPENGL_GLEW
#include "gl/glew.h"
#endif


#include "VectorField.h"
#ifdef USE_QT
#include "qgl.h"
#else
#include "gl/freeglut.h"
#endif



template <typename T>
void drawVectorField2RGB(const VectorField<T> &vField,
	const T vx_rang, const T vy_rang, const  T vx_min, const  T vy_min)
{
	glPolygonMode(GL_FRONT, GL_FILL);
	float rgb[3];
	//   Do we need glLoadIdentity() here?  
	// transform_fun();

	// How about we first draw the mesh based on the x component  
	int nfaces = vField.F.rows();
//	glBegin(GL_TRIANGLES);
	for (int f = 0; f<nfaces; f++) {
		//Eigen::Matrix<T, 1, -1> face = vField.F.row(f);
		int nvertices = vField.F.cols();
			glBegin(GL_POLYGON);
		for (int j = 0; j<nvertices; j++)
		{
			int vert = vField.F(f, j);

			rgb[0] = (vField.VF(vert, 0) - vx_min) / vx_rang;      // red channel:  vx 
			rgb[1] = (vField.VF(vert, 1) - vy_min) / vy_rang;      // green channel: vy
			rgb[2] = 1.;                               // blue channel is free now

			glColor3fv(rgb);
			glVertex2f(vField.V(vert, 0), vField.V(vert, 1));
		//	glVertex3f(vField.V(vert, 0), vField.V(vert, 1),0);
		}
			glEnd();
	}
//	glEnd();

	glFlush();
}

inline void transform_fun(const double zoom_factor=1., const double basis_zoom = 1.1,
	const double trans_x = 0, const double trans_y = 0)
{
	glTranslatef(trans_x, trans_y, 0);
	glTranslatef(0.5, 0.5, 0);
	glScalef(zoom_factor, zoom_factor, zoom_factor);
	glScalef(basis_zoom, basis_zoom, basis_zoom);
	//glScalef(1.5, 1.5, 1.5);
	glTranslatef(-.5, -.5, 0);
}

template<typename T>
void render_to_frameBuffer(const VectorField<T> &vField, 
	vector<float>& vx, vector<float>& vy,
	const int win_width,const int win_height)
{	
	float rgb[3];

//	obtain_maxmin_xy();
	Eigen::Matrix<T,-1,-1> m = vField.V.colwise().minCoeff();
	Eigen::Matrix<T,-1,-1> M = vField.V.colwise().maxCoeff();
	T vx_rang = M(0) - m(0);
	T vy_rang = M(1) - m(1);
	T vx_min = m(0), vy_min = m(1);

	//   OpenGL initialization    
	glClearColor(1, 1, 1, 1);
#ifdef RENDER_FRAMEBUFFER_WITH_DEPTH
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#else
	glClear(GL_COLOR_BUFFER_BIT);
#endif

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);

	glDrawBuffer(GL_BACK);

	//  transformation of the underlying mesh   

	glPushMatrix();

	//   Do we need glLoadIdentity() here?  
	transform_fun();

	// How about we first draw the mesh based on the x component  
	int nfaces = vField.F.rows();
	for(int f = 0 ; f<nfaces;f++){		
		//Eigen::Matrix<T, 1, -1> face = vField.F.row(f);
		int nvertices = vField.F.cols();
		glBegin(GL_POLYGON);
		for (int j = 0; j<nvertices; j++)
		{
			int vert = vField.F(f, j);

			rgb[0] = (vField.VF(vert, 0) - vx_min) / vx_rang;      // red channel:  vx 
			rgb[1] = (vField.VF(vert, 1) - vy_min) / vy_rang;      // green channel: vy
			rgb[2] = 1.;                               // blue channel is free now

			glColor3fv(rgb);
			glVertex2f(vField.V(vert, 0), vField.V(vert, 1));
		}
		glEnd();
	}
	glPopMatrix();

	int size = win_width*win_height;
	if (vx.size() != size) vx.resize(size);
	if (vy.size() != size) vy.resize(size);

	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, win_width, win_height, GL_RED, GL_FLOAT, &(vx[0]));
	glReadPixels(0, 0, win_width, win_height, GL_GREEN, GL_FLOAT, &(vy[0]));
	for (int i = 0; i < size; i++) {
       vx[i] = vx[i]* vx_rang + vx_min;
	   vy[i] = vy[i] *vy_rang + vy_min;
	}
}


#ifdef USE_QT

#include <QtCore>
#include <QtGui>
#include <QtWidgets>
//https://dangelog.wordpress.com/2013/02/10/using-fbos-instead-of-pbuffers-in-qt-5-2/

//#define RENDER_FRAMEBUFFER_WITH_DEPTH
template<typename T>
QImage renderOffScreen(
	const VectorField<T> &vField,
	const T vx_rang, const T vy_rang, const  T vx_min, const  T vy_min,
	const int image_width , const int image_height ,
	const GLfloat *modelview = 0, const GLfloat *projection = 0)
{
	
	float rgb[3];
	// SETUP
	QSurfaceFormat format;
	// following should be commeted, it may due to old opengl driver?
	//	format.setMajorVersion(3);
	//	format.setMinorVersion(3);

	QWindow window;
	window.setSurfaceType(QWindow::OpenGLSurface);
	window.setFormat(format);
	window.create();

	QOpenGLContext context;
	context.setFormat(format);
	if (!context.create())
		qFatal("Cannot create the requested OpenGL context!");
	context.makeCurrent(&window);

	QSize drawRectSize(image_width, image_height);

	QOpenGLFramebufferObjectFormat fboFormat;
	//	fboFormat.setSamples(16);
#ifdef RENDER_FRAMEBUFFER_WITH_DEPTH
	//	fboFormat.setAttachment(QOpenGLFramebufferObject::Depth);
	fboFormat.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
#endif	

	QOpenGLFramebufferObject fbo(drawRectSize, fboFormat);
	fbo.bind();

	// OPENGL CODE
	glViewport(0, 0, image_width, image_height);

#ifdef RENDER_FRAMEBUFFER_WITH_DEPTH
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
#endif

	glShadeModel(GL_FLAT);

	glClearColor(0.0, 0.0, 0.0, 0.0);
	//	glClearColor(1, 1, 1, 1);
#ifdef RENDER_FRAMEBUFFER_WITH_DEPTH
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#else
	glClear(GL_COLOR_BUFFER_BIT);
#endif

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
//	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (modelview)
		glLoadMatrixf(modelview);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (projection)
		glLoadMatrixf(projection);
	else
		glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
	//	gluPerspective(fov, aspect, 0.01, 1.0f);

	glColor3f(1.0, 0.0, 0.0);

//	GLfloat zoom_factor = 1.1;
//	glScalef(zoom_factor, zoom_factor, zoom_factor);//???

	float pos_v[4];
	int nfaces = vField.F.rows();
	glBegin(GL_TRIANGLES);
	for (int f = 0; f<nfaces; f++) {
		//Eigen::Matrix<T, 1, -1> face = vField.F.row(f);
		int nvertices = vField.F.cols();
	//	glBegin(GL_POLYGON);
		for (int j = 0; j<nvertices; j++)
		{
			int vert = vField.F(f, j);

			rgb[0] = (vField.VF(vert, 0) - vx_min) / vx_rang;      // red channel:  vx 
			rgb[1] = (vField.VF(vert, 1) - vy_min) / vy_rang;      // green channel: vy
			rgb[2] = 1.;                               // blue channel is free now

			glColor3fv(rgb);
			glVertex2f(vField.V(vert, 0), vField.V(vert, 1));
	//		pos_v[0] = vField.V(vert, 0); pos_v[1] = vField.V(vert, 1);
    //      pos_v[2] = vField.VF(vert, 0); pos_v[3] = vField.VF(vert, 1);
		}
		//glEnd();
	}
	glEnd();


	glFlush();

	//	QImage result = fbo.toImage();  result.save("screen.png", "PNG");
	fbo.release();
	//	return result;
	return fbo.toImage();
}


template<typename T>
void renderOffScreen(const VectorField<T> &vField,
	vector<float>& vx, vector<float>& vy,
	const int image_width, const int image_height,
	const GLfloat *modelview = 0, const GLfloat *projection = 0)
{
	Eigen::Matrix<T, -1, -1> m = vField.VF.colwise().minCoeff();
	Eigen::Matrix<T, -1, -1> M = vField.VF.colwise().maxCoeff();
	T vx_rang = M(0) - m(0);
	T vy_rang = M(1) - m(1);
	T vx_min = m(0), vy_min = m(1);

	QImage image = renderOffScreen(vField, vx_rang, vy_rang,vx_min,vy_min,
		image_width, image_height, modelview, projection);
	image.save("screen.png", "PNG");
	int size = image_width*image_height;
	if (vx.size() != size) vx.resize(size);
	if (vy.size() != size) vy.resize(size);

	int idx = 0;
	GLfloat byte2float_coeff = 1.f/255.0;
//	for (int row =0;row< image_height ; row++)
	for (int row = image_height - 1; row >= 0; row--)
	{
		QRgb *rowData = (QRgb *)image.scanLine(row);

		for (int col = 0; col<image_width; col++,idx++)	{	
			QRgb pixelData = rowData[col];

		//	int idx = row*image_width + col;
			//	normalize(normVec);		
			vx[idx] = qRed(pixelData)*byte2float_coeff* vx_rang + vx_min;
			vy[idx] = qGreen(pixelData)*byte2float_coeff* vy_rang + vy_min;
		}	
	}
}

#else

#if USE_OPENGL_GLEW
//http://www.lighthouse3d.com/tutorials/opengl_framebuffer_objects/ 
template<typename T>
void renderOffScreenFBO(
	vector<float>& vx, vector<float>& vy,
	const VectorField<T> &vField,
	const T vx_rang, const T vy_rang, const  T vx_min, const  T vy_min,
	const int image_width, const int image_height,
	const GLfloat *modelview = 0, const GLfloat *projection = 0)
{
	int w= image_width, h= image_height;
	const int xres = image_width; // crashes if > 843
	const int yres = image_height;

#if 0
	GLuint fbo = prepareFBO(image_width, image_height, 1);

	//render scene
	// bind a framebuffer object
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);

	glClearColor(0.4f, 0.4f, 0.8f, 1.0f);
	// Set Drawing buffers
	GLuint attachments[1] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, attachments);

//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClear(GL_COLOR_BUFFER_BIT);//

	//set the viewport and projection matrix accordingly
	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (modelview)
		glLoadMatrixf(modelview);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (projection)
		glLoadMatrixf(projection);
	else
		glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	// crashes regardless of whether I draw anything
	drawVectorField2RGB(vField, vx_rang, vy_rang, vx_min, vy_min);


#else
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// create VBOs and FBOs for drawing out geometry
	const int NUM_COLOR_ATTACHMENTS = 1;
	GLuint fbo, depthBufferId;
	GLuint colorBufferIds[NUM_COLOR_ATTACHMENTS];
	glGenFramebuffersEXT(1,  &fbo);
	glBindFramebufferEXT(GL_FRAMEBUFFER, fbo);
	GLuint vboId;
	glGenBuffers(1, &vboId);

	// Create depth renderbuffer
	glGenRenderbuffers(1,&depthBufferId);
	glBindRenderbuffer(GL_RENDERBUFFER, depthBufferId);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, xres, yres);
	glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthBufferId);

	// Create color renderbuffers
	glGenRenderbuffers(NUM_COLOR_ATTACHMENTS, colorBufferIds);
	for (int i = 0; i < NUM_COLOR_ATTACHMENTS; i++) {
		glBindRenderbuffer(GL_RENDERBUFFER, colorBufferIds[i]);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA32F, xres, yres);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, GL_RENDERBUFFER, colorBufferIds[i]);
	}
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

	switch (glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT)) {
	case GL_FRAMEBUFFER_COMPLETE_EXT:
		break;
	case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:
		std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT" << std::endl;
		break;
	case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:
		std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT" << std::endl;
		break;
	case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
		std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT" << std::endl;
		break;
	case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
		std::cerr << "GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT" << std::endl;
		break;
	case GL_FRAMEBUFFER_UNSUPPORTED_EXT:
		std::cerr << "GL_FRAMEBUFFER_UNSUPPORTED_EXT" << std::endl;
		break;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	glViewport(0, 0, w, h);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (modelview)
		glLoadMatrixf(modelview);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (projection)
		glLoadMatrixf(projection);
	else
		glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
	// crashes regardless of whether I draw anything
	drawVectorField2RGB(vField,vx_rang, vy_rang, vx_min, vy_min);

	// read back buffer
	//
	

#endif

	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
//	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glReadBuffer(GL_COLOR_ATTACHMENT0_EXT);

	vector<GLfloat> data(xres*yres * 3);
//	glReadPixels(0, 0, xres, yres, GL_RGB, GL_FLOAT, &(data[0])); // GL_UNSIGNED_BYTE doesn't crash and gives correct (but clamped results)



	int size = image_width*image_height;
	if (vx.size() != size) vx.resize(size);
	if (vy.size() != size) vy.resize(size);
	glReadPixels(0, 0, image_width, image_height, GL_RED, GL_FLOAT, &(vx[0]));
	glReadPixels(0, 0, image_width, image_height, GL_GREEN, GL_FLOAT, &(vy[0]));

#if 0
	for (int y = 0; y< image_height; y++)
		for (int x = 0; x < image_width; x++) {
			int y0 = image_height - 1 - y;
			int idx0 = y0*image_width + x;
			int idx = y*image_width + x;
			vx[idx] = vx[idx0] * vx_rang + vx_min;
			vy[idx] = vy[idx0] * vy_rang + vy_min;
		}
#else
	for (int i = 0; i < size; i++) {
		//	vx[i] = vxx[i]; vy[i] = vyy[i];
		vx[i] = vx[i] * vx_rang + vx_min;
		vy[i] = vy[i] * vy_rang + vy_min;
	}
#endif

	// get back to the default framebuffer
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
}

#endif

template<typename T>
void renderOffScreen(
	vector<float>& vx, vector<float>& vy,
	const VectorField<T> &vField,
	const T vx_rang, const T vy_rang, const  T vx_min, const  T vy_min,
	const int image_width, const int image_height,
	const GLfloat *modelview = 0, const GLfloat *projection = 0)
{
	float rgb[3];

	// OPENGL CODE
	glViewport(0, 0, image_width, image_height);

#ifdef RENDER_FRAMEBUFFER_WITH_DEPTH
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
#endif

   //  OpenGL initialization   
	glClearColor(1, 1, 1, 1);
#ifdef RENDER_FRAMEBUFFER_WITH_DEPTH
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#else
	glClear(GL_COLOR_BUFFER_BIT);
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#endif

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);
//	glEnable(GL_COLOR_MATERIAL); //Qt should commet this line?
	glShadeModel(GL_SMOOTH);

	glDrawBuffer(GL_BACK);

//	GLint viewport[4];	glGetIntegerv(GL_VIEWPORT, viewport);


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (modelview)
		glLoadMatrixf(modelview);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (projection)
		glLoadMatrixf(projection);
	else
		glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

	
	glDrawBuffer(GL_BACK);


	//  transformation of the underlying mesh   
	//glPushMatrix();

	//   Do we need glLoadIdentity() here?  
	// transform_fun();

	// How about we first draw the mesh based on the x component  
	
	drawVectorField2RGB(vField, vx_rang, vy_rang, vx_min, vy_min);

	//glPopMatrix();
	int size = image_width*image_height;
	if (vx.size() != size) vx.resize(size);
	if (vy.size() != size) vy.resize(size);

	std::vector<GLfloat> vxx, vyy;
	vxx.resize(size); vyy.resize(size);

	
	glReadBuffer(GL_BACK);
//	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	
	glReadPixels(0, 0, image_width, image_height, GL_RED, GL_FLOAT, &(vx[0]));
	glReadPixels(0, 0, image_width, image_height, GL_GREEN, GL_FLOAT, &(vy[0]));

#if 0
	for(int y = 0 ; y< image_height;y++)
		for (int x = 0; x < image_width; x++) {
			int y0 = image_height -1 - y;
			int idx0 = y0*image_width + x;
			int idx = y*image_width + x;
			vx[idx] = vx[idx0] * vx_rang + vx_min;
			vy[idx] = vy[idx0] * vy_rang + vy_min;
		}
#else
	for (int i = 0; i < size; i++) {
	//	vx[i] = vxx[i]; vy[i] = vyy[i];
		vx[i] = vx[i] * vx_rang + vx_min;
		vy[i] = vy[i] * vy_rang + vy_min;
	}
#endif

}

template<typename T>
void renderOffScreen(const VectorField<T> &vField,
	vector<float>& vx, vector<float>& vy,
	const int image_width, const int image_height,
	const GLfloat *modelview = 0, const GLfloat *projection = 0)
{
	Eigen::Matrix<T, -1, -1> m = vField.VF.colwise().minCoeff();
	Eigen::Matrix<T, -1, -1> M = vField.VF.colwise().maxCoeff();
	T vx_rang = M(0) - m(0);
	T vy_rang = M(1) - m(1);
	T vx_min = m(0), vy_min = m(1);

#if USE_OPENGL_GLEW
	renderOffScreenFBO(vx, vy, vField, vx_rang, vy_rang, vx_min, vy_min,
		image_width, image_height, modelview, projection);
#else
	renderOffScreen(vx,vy,vField, vx_rang, vy_rang, vx_min, vy_min,
		image_width, image_height, modelview, projection);
#endif

}

#endif


/*
Output the current vector field (stored in the vec variable of each vertex)
in the pixel-based format.
filename:  as it says
size:  can be the power of 2, upto 512 (the IBFV window size)
The output format is
(row col x y vx vy) / perline
*/

inline bool output_cur_VF_pixel(char *filename, vector<float>& vx, vector<float>& vy,
	const int win_width, const int win_height, const int size)
{
	int nstep = win_width / size;  // we subsample the obtained vector field texture

	if (nstep < 1 || nstep >= win_width) return false;

	FILE *fp = fopen(filename, "w");
	if (fp == NULL) return false;

	double dx = 1. / win_width;
	double dy = 1. / win_height;

	double x_coord = -0.5;
	double y_coord = 0.5;

	int count_r = 0;
	for (int i = win_height - 1; i >= 0; i -= nstep)
	{
		y_coord = 0.5 - (win_height - 1 - i)*dy;

		int count_c = 0;
		for (int j = 0; j<win_width; j += nstep)
		{
			x_coord = -0.5 + j*dx;
			int idx = i*win_width + j;

			//	normalize(normVec);

			//  write to the output file  
			fprintf(fp, "%d %d %f %f %f %f\n", count_r, count_c, x_coord, y_coord,
				vx[idx], vy[idx]);

			count_c++;
		}
		count_r++;
	}

	fclose(fp);

	return true;
}



