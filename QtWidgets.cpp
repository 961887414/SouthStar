#include "QtWidgets.h"

RobotControl robcontrol;
TYcamera camera;
chooseblack Deal1;
complexIm Deal2;
ExcuseMode1 Excuse1;
ExecuseMode2 Execuse2;
ExcuseMode3 Excuse3;

QtWidgets::QtWidgets(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    connect(&camera, SIGNAL(TYstatus(char*)), this, SLOT(TYStatus(char*)));
    connect(&robcontrol, SIGNAL(Robstate(char*)), this, SLOT(RobState(char*)));

    connect(&Excuse1, SIGNAL(StartExcuse()), this, SLOT(ExcuseIt()));
    connect(&Excuse1, SIGNAL(Robstate(char*)), this, SLOT(RobState(char*)));
    connect(&Excuse1, SIGNAL(DisplayColor()), this, SLOT(ShowColor()));
    connect(&Excuse1, SIGNAL(DisplayBinary()), this, SLOT(ShowBinary()));
    connect(&Excuse1, SIGNAL(WorkState(char* )), this, SLOT(ShowWorkState(char*)));
    connect(&Excuse1, SIGNAL(DisplayLayerNumbers(QString )), this, SLOT(ShowLayerNumbers(QString )));
    connect(&Excuse1, SIGNAL(DisplayRobX(QString)), this, SLOT(ShowRobx(QString)));
    connect(&Excuse1, SIGNAL(DisplayRobY(QString)), this, SLOT(ShowRoby(QString)));
    connect(&Excuse1, SIGNAL(DisplayRobZ(QString)), this, SLOT(ShowRobz(QString)));
    connect(&Excuse1, SIGNAL(DisplayPieceCount(QString)), this, SLOT(ShowPieceCount(QString)));

    connect(&Execuse2, SIGNAL(StartExcuse()), this, SLOT(ExcuseIt()));
    connect(&Execuse2, SIGNAL(Robstate(char*)), this, SLOT(RobState(char*)));
    connect(&Execuse2, SIGNAL(DisplayColor()), this, SLOT(ShowColor()));
    connect(&Execuse2, SIGNAL(DisplayBinary()), this, SLOT(ShowBinary()));
    connect(&Execuse2, SIGNAL(WorkState(char*)), this, SLOT(ShowWorkState(char*)));
    connect(&Execuse2, SIGNAL(DisplayLayerNumbers(QString)), this, SLOT(ShowLayerNumbers(QString)));
    connect(&Execuse2, SIGNAL(DisplayRobX(QString)), this, SLOT(ShowRobx(QString)));
    connect(&Execuse2, SIGNAL(DisplayRobY(QString)), this, SLOT(ShowRoby(QString)));
    connect(&Execuse2, SIGNAL(DisplayRobZ(QString)), this, SLOT(ShowRobz(QString)));
    connect(&Execuse2, SIGNAL(DisplayPieceCount(QString)), this, SLOT(ShowPieceCount(QString)));

    connect(&Excuse3, SIGNAL(StartExcuse()), this, SLOT(ExcuseIt()));
    connect(&Excuse3, SIGNAL(Robstate(char*)), this, SLOT(RobState(char*)));
    connect(&Excuse3, SIGNAL(DisplayColor()), this, SLOT(ShowColor()));
    connect(&Excuse3, SIGNAL(DisplayBinary()), this, SLOT(ShowBinary()));
    connect(&Excuse3, SIGNAL(WorkState(char*)), this, SLOT(ShowWorkState(char*)));
    connect(&Excuse3, SIGNAL(DisplayLayerNumbers(QString)), this, SLOT(ShowLayerNumbers(QString)));
    connect(&Excuse3, SIGNAL(DisplayRobX(QString)), this, SLOT(ShowRobx(QString)));
    connect(&Excuse3, SIGNAL(DisplayRobY(QString)), this, SLOT(ShowRoby(QString)));
    connect(&Excuse3, SIGNAL(DisplayRobZ(QString)), this, SLOT(ShowRobz(QString)));
    connect(&Excuse3, SIGNAL(DisplayPieceCount(QString)), this, SLOT(ShowPieceCount(QString)));
}

void QtWidgets::on_StartWork_released()
{

    Sleep(200);
    int x0, y0, x1, y1;
    x0 = 284, y0 = 182, x1 = 680, y1 = 592;
    float NeedZ, NneedZ0;
    NneedZ0 = 1630;
    RobotControl z;
    z.InitRobot();
    TYcamera ty;

    cv::Mat imageB;
    cv::Mat imageDepth;
    cv::Mat imageA = ty.getTYcolor(imageDepth);
    cv::Rect tu(x0, y0, x1, y1);
    complexIm DealIm;
    std::vector<cv::Mat> imageT; std::vector<cv::Point2f> centers; std::vector<float> radius;
    std::vector<int> robX, robY, robZ;
    std::vector<int> RbW;
    std::vector<int> RbWt;
    DealIm.getDepth(imageDepth, NeedZ, tu);
    DealIm.complexim(imageA, imageB, cv::Size(100, 100), imageT, tu, centers, radius, NeedZ, NneedZ0);
    DealIm.getRobData(NeedZ, NneedZ0, centers, radius, robX, robY, robZ, tu, RbW, RbWt);
    for (int i = 0; i < centers.size(); i++)
    {
        while (1)
        {
            int redstatus = z.RobAddressStatus(0, 301, 0);
            Sleep(400);
            redstatus = z.RobAddressStatus(0, 301, 0);
            if (redstatus == 0)
                break;
        }

        if (robZ[i] >= 900000 && robX[i] >= -850000)
        {
            z.RobotStepTop2(6001, robX[i], robY[i], robZ[i], 0, 0, RbW[i], RbWt[i]);
            z.startCode();
            Sleep(5000);
        }

        if (robZ[i] >= 900000 && robX[i] < -850000)
        {
            z.RobotStep1(6001, robX[i], robY[i], robZ[i], 0, 0, RbW[i], RbWt[i]);
            z.startCode();
            Sleep(5000);
        }

        if (robZ[i] < 900000)
        {
            z.RobotStepTop2(6001, robX[i], robY[i], robZ[i], 0, 0, RbW[i], RbWt[i]);
            z.startCode();
            Sleep(5000);
        }
    }

    float needzp;
    cv::Mat imageDepthP;
    cv::Mat imageP = ty.getTYcolor(imageDepthP);
    DealIm.getDepth(imageDepthP, needzp, tu);
    needzp = -needzp + 2000 + 100;
    z.RobotStep2(6009, -1201000, 5000, needzp * 1000, 0, 0, 90000, 0);
    z.startCode();
}

void QtWidgets::on_RobTestButton_released()
{
    robcontrol.start();  
    //int t = robcontrol.InitRobot();
}

void  QtWidgets::on_CameraTestButton_released()
{
    camera.start();
}

void QtWidgets::on_WorkMode1_released()
{
    if (Excuse1.WorkProcess)
    {
        ui.WorkMode1->setText(QString::fromLocal8Bit("工作中"));
        ui.WorkMode1->setEnabled(0);
        Excuse1.start();
    }
    else
    {
        ui.F51->setText(QString::fromLocal8Bit("暂停（1）"));
        ui.F51->setEnabled(1);
        Excuse1.WorkProcess = true;
        ui.WorkMode1->setEnabled(0);

    }
       
}

void QtWidgets::on_WorkMode2_released()
{
    if (Execuse2.WorkProcess)
    {
        ui.WorkMode2->setText(QString::fromLocal8Bit("工作中"));
        ui.WorkMode2->setEnabled(0);
        Execuse2.start();
    }
    else
    {
        ui.F52->setText(QString::fromLocal8Bit("暂停（1）"));
        ui.F52->setEnabled(1);
        Execuse2.WorkProcess = true;
        ui.WorkMode2->setEnabled(0);

    }
}

void QtWidgets::on_WorkMode3_released()
{
    if (Excuse3.WorkProcess)
    {
        ui.WorkMode3->setText(QString::fromLocal8Bit("工作中"));
        ui.WorkMode3->setEnabled(0);
        Excuse3.start();
    }
    else
    {
        ui.F53->setText(QString::fromLocal8Bit("暂停（1）"));
        ui.F53->setEnabled(1);
        Excuse3.WorkProcess = true;
        ui.WorkMode3->setEnabled(0);

    }
}

void QtWidgets::on_F51_released()
{
    Excuse1.WorkProcess = false;
    ui.F51->setEnabled(0);
    ui.F51->setText(QString::fromLocal8Bit("暂停中"));
    ui.WorkMode1->setText(QString::fromLocal8Bit("点击继续码垛"));
    ui.WorkMode1->setEnabled(1);
}

void QtWidgets::on_F52_released()
{
    Execuse2.WorkProcess = false;
    ui.F52->setEnabled(0);
    ui.F52->setText(QString::fromLocal8Bit("暂停中"));
    ui.WorkMode2->setText(QString::fromLocal8Bit("点击继续码垛"));
    ui.WorkMode2->setEnabled(1);
}

void QtWidgets::on_F53_released()
{
    Excuse3.WorkProcess = false;
    ui.F53->setEnabled(0);
    ui.F53->setText(QString::fromLocal8Bit("暂停中"));
    ui.WorkMode3->setText(QString::fromLocal8Bit("点击继续码垛"));
    ui.WorkMode3->setEnabled(1);
}


void QtWidgets::TYStatus(char* str)
{

    ui.CameraTestLine->setText(QString::fromLocal8Bit(str));
}

void QtWidgets::RobState(char* str)
{
    ui.RobTestLine->setText(QString::fromLocal8Bit(str));

}

void QtWidgets::ExcuseIt()
{
    Excuse1.excusemode1(camera,robcontrol,Deal1,Deal2);
}

void QtWidgets::ShowColor()
{
    cv::Mat im = Excuse1.ROIColorImage.clone();
   /* cv::resize(im, im, cv::Size(331, 201));
    cv::cvtColor(im, im, CV_BGR2RGB);
    QImage qimage1 = QImage((const unsigned char*)(im.data), im.cols, im.rows, im.step, QImage::Format_RGB888);
    ui.ColorLabel->setPixmap(QPixmap::fromImage(qimage1));*/

}

void QtWidgets::ShowBinary()
{
    cv::Mat im = Excuse1.ROIDepthImage.clone();
    /*cv::resize(im, im, cv::Size(331, 201));
    QImage qimage1 = QImage((const unsigned char*)(im.data), im.cols, im.rows, im.step, QImage::Format_Indexed8);
    ui.BinaryLabel->setPixmap(QPixmap::fromImage(qimage1));*/

}

void QtWidgets::ShowWorkState(char* str)
{
    ui.WorkStatus->setText(QString::fromLocal8Bit(str));
}

void QtWidgets::ShowLayerNumbers(QString str)
{
    ui.WorkCountLine->setText(str);
}

void QtWidgets::ShowRobx(QString str)
{
    ui.RobNeedX->setText(str);
}

void QtWidgets::ShowRoby(QString str)
{
    ui.RobNeedY->setText(str);
}

void QtWidgets::ShowRobz(QString str)
{
    ui.RobNeedZ->setText(str);
}

void QtWidgets::ShowPieceCount(QString str)
{
    ui.WorkPieceLine->setText(str);
}