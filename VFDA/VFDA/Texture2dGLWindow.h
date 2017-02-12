#pragma once
#include "QGLWindow.h"
#include "GL/glu.h"
#include "Visualize.h"

class Texture2dGLWindow : public QGLWindow
{
public:
	Texture2dGLWindow(QWidget *parent = 0, char *name = 0) 
		: QGLWindow(0, parent, name)
	{
	}

protected:
	void initializeGL()
	{
		generate_noise_image();
#if 0
		glShadeModel(GL_SMOOTH);

		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
		glClearDepth(1.0f);

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);

		glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
#else
	//	glViewport(0, 0, (GLsizei)img_width, (GLsizei)img_height);
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

#endif
	}

	void resizeGL(int width, int height)
	{
		height = height ? height : 1;

		glViewport(0, 0, (GLint)width, (GLint)height);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
#if 0
		gluPerspective(45.0f, (GLfloat)width / (GLfloat)height, 0.1f, 100.0f);
#else	
		gluOrtho2D(0, 1, 0, 1);
#endif
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}

	void paintGL()
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();

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
#endif
	}

	void mousePressEvent(QMouseEvent *event) {}

private:
	std::vector<unsigned char> noise_img, glTex_img;
	int img_width=512, img_height = 512;
	void generate_noise_image() {
		const int size = img_width*img_height;
		noise_img.resize(size);
		glTex_img.resize(size*3);		

		gen_noise_img(&(noise_img[0]), img_width, img_height);
		
		gray2rgb(&(glTex_img[0]), &(noise_img[0]), img_width, img_height);
	}
};
