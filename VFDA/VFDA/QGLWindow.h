#pragma once
//https://github.com/smucs/QtGL/blob/master/QGLWindow.h
//http://www.digitalfanatics.org/projects/qt_tutorial/chapter14.html

#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QtGui>
#include <qtimer.h>
#include <qgl.h>

class QGLWindow : public QGLWidget
{
	Q_OBJECT //must be included for signals and slots

public:
	QGLWindow(int timerInterval = 0, QWidget *parent = 0, char *name = 0)
	{
		if (timerInterval == 0)
			m_timer = 0;
		else
		{
			m_timer = new QTimer(this);
			connect(m_timer, SIGNAL(timeout()), this, SLOT(timeOutSlot()));
			m_timer->start(timerInterval);
		}
	}
	virtual ~QGLWindow() {};

protected:
	void initializeGL()=0;
	void resizeGL(int w, int h)=0;
	void paintGL()=0;
	void mousePressEvent(QMouseEvent *event) {};
	void mouseMoveEvent(QMouseEvent *event) {};
	void keyPressEvent(QKeyEvent *e) {	
			switch (e->key())			{
			case Qt::Key_Escape:
				close();
			}		
	}
	virtual void timeOut() {};

protected slots:
virtual void timeOutSlot() { timeOut(); }

private:
	QTimer *m_timer;
};
