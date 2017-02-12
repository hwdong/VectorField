#include <QtCore>
#include <QtGui>
#include <QtWidgets>
//https://dangelog.wordpress.com/2013/02/10/using-fbos-instead-of-pbuffers-in-qt-5-2/

#define RENDER_FRAMEBUFFER_WITH_DEPTH
QImage renderOffScreen(const int width=512,const int height=512,
	const GLfloat *modelview=0,const GLfloat *projection=0)
{
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

	QSize drawRectSize(width, height);

#if 0
	QOpenGLFramebufferObjectFormat fboFormat;
	fboFormat.setSamples(16);
	fboFormat.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);

	QOpenGLFramebufferObject fbo(drawRectSize, fboFormat);
	fbo.bind();
	
	// following QOpenGLPaintDevice can't work on my machine!
	QOpenGLPaintDevice device(drawRectSize);
	QPainter painter(&device);   //	not workable: QPainter painter;	painter.begin(&device);
	painter.setRenderHints(QPainter::Antialiasing | QPainter::HighQualityAntialiasing);
		
	const QRect drawRect(0, 0, width,height);
	painter.fillRect(drawRect, Qt::blue);

//	painter.drawTiledPixmap(drawRect, QPixmap(":/qt-project.org/qmessagebox/images/qtlogo-64.png"));

	painter.setPen(QPen(Qt::green, 5));
	painter.setBrush(Qt::red);
	painter.drawEllipse(0, 100, 400, 200);
	painter.drawEllipse(100, 0, 200, 400);

	painter.setPen(QPen(Qt::white, 0));
	QFont font;
	font.setPointSize(24);
	painter.setFont(font);
	painter.drawText(drawRect, "Hello FBO", QTextOption(Qt::AlignCenter));	
	painter.end();
	fbo.release();
	return fbo.toImage();
#else

	QOpenGLFramebufferObjectFormat fboFormat;
//	fboFormat.setSamples(16);
#ifdef RENDER_FRAMEBUFFER_WITH_DEPTH
//	fboFormat.setAttachment(QOpenGLFramebufferObject::Depth);
	fboFormat.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
#endif	

	QOpenGLFramebufferObject fbo(drawRectSize, fboFormat);
	fbo.bind();

	// OPENGL CODE
	glViewport(0, 0, width,height);

#ifdef RENDER_FRAMEBUFFER_WITH_DEPTH
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
#endif

	glShadeModel(GL_FLAT);

	glClearColor(0.0, 0.0, 0.0, 0.0);
#ifdef RENDER_FRAMEBUFFER_WITH_DEPTH
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
#else
	glClear(GL_COLOR_BUFFER_BIT);
#endif

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if(modelview)
		glLoadMatrixf(modelview);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if(projection)
		glLoadMatrixf(projection);
	else 
		glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);
//	gluPerspective(fov, aspect, 0.01, 1.0f);

	glColor3f(1.0, 0.0, 0.0);
	glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

	glBegin(GL_POLYGON);
	glVertex3f(0.25, 0.25, 0.0);
	glVertex3f(0.75, 0.25, 0.0);
	glVertex3f(0.75, 0.75, 0.0);
	glVertex3f(0.25, 0.75, 0.0);
	glEnd();

	glFlush();

//	QImage result = fbo.toImage();  result.save("screen.png", "PNG");
	fbo.release();
//	return result;
	return fbo.toImage();
#endif
}

