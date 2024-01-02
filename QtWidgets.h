#pragma once
#define WIN32_LEAN_AND_MEAN
#include <QtWidgets/QMainWindow>
//#include<Windows.h>
#include "ui_QtWidgets.h"
#include "ExcuseMode3.h"
 

class QtWidgets : public QMainWindow
{
    Q_OBJECT

public:
    QtWidgets(QWidget *parent = Q_NULLPTR);
    

private:
    Ui::QtWidgetsClass ui;

private slots:
    void on_StartWork_released();
    void on_RobTestButton_released();
    void on_CameraTestButton_released();
    void on_WorkMode1_released();
    void on_F51_released();
    void on_F52_released();  ///测试用  按下这个可以控制暂停或继续
    void on_F53_released();
    void on_WorkMode2_released();
    void on_WorkMode3_released();


    void TYStatus(char* str);
    void RobState(char* str);
    void ExcuseIt();
    void ShowColor();
    void ShowBinary();
    void ShowWorkState(char* str);
    void ShowLayerNumbers(QString str);
    void ShowRobx(QString str);
    void ShowRoby(QString str);
    void ShowRobz(QString str);
    void ShowPieceCount(QString str);
};
