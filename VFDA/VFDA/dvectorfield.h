#ifndef DVECTORFIELD_H
#define DVECTORFIELD_H

#include <QtWidgets/QMainWindow>
#include "ui_dvectorfield.h"

class DVectorField : public QMainWindow
{
	Q_OBJECT

public:
	DVectorField(QWidget *parent = 0);
	~DVectorField();

private:
	Ui::DVectorFieldClass ui;
};

#endif // DVECTORFIELD_H
