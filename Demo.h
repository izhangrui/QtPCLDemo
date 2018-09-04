#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_Demo.h"



#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
#include "vtkConeSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkRenderWindow.h"
#include "vtkCamera.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "Gzz.h"

#include <QMessageBox>
#include <QFileDialog>  
#include <QDebug>  
#include <QTime>  
#include <QDir>  
#include <QFile>

class Demo : public QMainWindow
{
	Q_OBJECT

public:
	Demo(QWidget *parent = Q_NULLPTR);
	void show_cloud();

private:
	Ui::DemoClass ui;
private slots:
	void open_file();
	void m_radar1();
	void m_servo();
};
