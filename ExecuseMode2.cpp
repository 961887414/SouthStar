#include "ExecuseMode2.h"

cv::Rect MU2(190, 15, 1000, 925);
cv::Rect DepthTu2(500, 400, 100, 100);

void ExecuseMode2::execusemode2(TYcamera& a, RobotControl& b, chooseblack& c, complexIm& d)
{
    tu = MU2;

    int LayerNumbers = 0;
    WorkProcess = true;

    emit WorkState("���ӻ�������...");
    b.InitRobot();

    //ͼ�����Լ�������ִ�в��趼�ڴ�ѭ����
    while (1)
    {
        while (!WorkProcess);    //��ͣ��

        //�ͷ�һЩ�ڴ�
        ColorImage.release(); DepthImage.release(); ROIColorImage.release(); ROIDepthImage.release();

        std::vector<int>().swap(Robx); std::vector<int>().swap(Roby); std::vector<int>().swap(Robz);
        std::vector<int>().swap(Robwt1); std::vector<int>().swap(Robwt2);
        std::vector<float>().swap(depthData); std::vector<float>().swap(Radiuses);
        std::vector<cv::Mat>().swap(BinaryIms); std::vector<cv::Point2f>().swap(imcenters);

        emit WorkState("���������...");

        /*��ֽƤ�ĳ���*/

        float NowLayerZ;  //�������������ȡ����ֽƤʱ�����˵�Zֵ  1296
        float needzp;
        cv::Mat imageDepthP;       //rbz=1036   1060  imz=1290

        cv::Mat imageP = a.getTYcolor(imageDepthP);
        // cv::Mat imageP = a.getGainImage(imageDepthP);

        d.getDepth(imageDepthP, FirstPaperZ, DepthTu2);
        if (FirstPaperZ < MaxWorkZ)
        {
            needzp = -FirstPaperZ + 2400;  //1104
            //needzp = needzp + 100;
         /*   b.RobotStep2(6009, -1308000, 1000, needzp * 1000, 0, 0, 90000, 0);
            b.startCode();
            Sleep(2000);*/
        }


        else
        {
            emit WorkState("������������ֹͣ����");
            break;
        }


        while (!WorkProcess); //��ͣ��  


      /*  while (1)
        {
            while (!WorkProcess);
            //��ͣ�� 

            int StepStatus1 = b.RobAddressStatus(0, 874, 3);
            int StepStatus2 = b.RobAddressStatus(0, 875, 3);
            if (!(StepStatus1 || StepStatus2))
            {
                Sleep(300);
                float NowLayerz = (float)b.RobAddressStatus(0, 7100, 1); //��ȡ����ֽƤ��Zֵ
                NowLayerZ = NowLayerz * 0.00001;
                break;
            }
        }*/


        cv::Mat ColorIm = a.getTYcolor(DepthImage);
        

        cv::Mat color1 = ColorIm.clone();
        cv::Mat depth1 = DepthImage.clone();  //depth1����������������ͼ
        cv::Mat outputIm;
        ColorImage = ColorIm.clone();

        float NeedZ;
        //NeedZ = -NowLayerZ + 2400;
        NeedZ = FirstPaperZ + 220;   //����һ���������Ĵ��Zֵ
        LayerNumbers++;
        QString Qlayers = QString::number(LayerNumbers);
        emit DisplayLayerNumbers("Now Number of Layers is " + Qlayers);

        if (NeedZ < MaxWorkZ+200)
        {

           // MainProcess(); Delay(50); MainProcess(); DWrite1R(0, 7001, 1); Delay(200);

            int tx = (int)((NeedZ0 / NeedZ) * tu.width - tu.width); int ty = (int)((NeedZ0 / NeedZ) * tu.height - tu.height);
            cv::Rect R(tu.tl().x - tx / 2, tu.tl().y - ty / 2, tu.width + tx, tu.height + ty);
            tDepth = R;
            

            int xl = R.tl().x *2; int yl = R.tl().y *2; int xt = R.width *2; int yt = R.height *2;
            cv::Rect T(xl, yl, xt, yt);
            tColor = T;
            

            cv::Mat rOIColorImage = color1(tColor);
            cv::Mat rOIDepthImage = depth1(tDepth);
            ROIColorImage = rOIColorImage.clone();
            ROIDepthImage = rOIDepthImage.clone();

            while (!WorkProcess);   //��ͣ��

            emit DisplayColor();
            emit DisplayBinary();

            emit WorkState("����ͼ����...");

            cv::Mat depth1x = d.DepthWay(rOIDepthImage, FirstPaperZ);
            d.DealDepthim(BinaryIms,depth1x,tu, imcenters, Radiuses, NeedZ, NeedZ0);


            while (!WorkProcess);   //��ͣ��

            int CountPiece = imcenters.size();
            QString PIECECOUNT = QString::number(CountPiece);
            emit DisplayPieceCount(PIECECOUNT);


            d.getRobDataDepth(NowLayerZ, NeedZ, NeedZ0, imcenters, Radiuses, Robx, Roby, Robz, R, Robwt1, Robwt2);

            for (int i = 0; i < imcenters.size(); i++)
            {
                emit WorkState("�����˵ȴ�����...");

                while (!WorkProcess);   //��ͣ��

                while (1)
                {
                    int redstatus = b.RobAddressStatus(0, 301, 0);
                    Sleep(400);
                    redstatus = b.RobAddressStatus(0, 301, 0);
                    if (redstatus == 0)
                        break;
                }

                emit WorkState("�����������У�");

                while (!WorkProcess);   //��ͣ��

                if (Robz[i] >= 900000 && Robx[i] >= -850000)
                {
                    b.RobotStepTop2(6001, Robx[i], Roby[i], Robz[i], 0, 0, Robwt1[i], Robwt2[i]);
                    b.startCode();
                    Sleep(2000);
                }

                if (Robz[i] >= 900000 && Robx[i] < -850000)
                {
                    b.RobotStep1(6001, Robx[i], Roby[i], Robz[i], 0, 0, Robwt1[i], Robwt2[i]);
                    b.startCode();
                    Sleep(2000);
                }

                if (Robz[i] < 900000)
                {
                    b.RobotStepTop2(6001, Robx[i], Roby[i], Robz[i], 0, 0, Robwt1[i], Robwt2[i]);
                    b.startCode();
                    Sleep(2000);
                }

                QString ROBX = QString::number(Robx[i] / 1000);
                QString ROBY = QString::number(Roby[i] / 1000);
                QString ROBZ = QString::number(Robz[i] / 1000);
                emit DisplayRobX(ROBX);
                emit DisplayRobY(ROBY);
                emit DisplayRobZ(ROBZ);


                while (!WorkProcess);

                //��ͣ��

                while (1)
                {
                    while (!WorkProcess);

                    //��ͣ��  
                    int StepStatus1 = b.RobAddressStatus(0, 874, 3);
                    int StepStatus2 = b.RobAddressStatus(0, 875, 3);
                    if (!(StepStatus1 || StepStatus2))
                    {
                        Sleep(300);

                        /*if (b.RobAddressStatus(0, 7001, 1))
                        {
                            abort();
                        }*/

                        break;
                    }
                }
            }

            while (!WorkProcess);   //��ͣ��


        }

        else
        {
            emit WorkState("������������ֹͣ����");
            break;
        }

    }



}

void ExecuseMode2::NewExecusemode(TYcamera& a, RobotControl& b, chooseblack& c, complexIm& d)
{
    tu = MU2;

    int LayerNumbers = 0;
    WorkProcess = true;

    emit WorkState("���ӻ�������...");
    b.InitRobot();

    //ͼ�����Լ�������ִ�в��趼�ڴ�ѭ����
    while (1)
    {
        while (!WorkProcess);    //��ͣ��

        //�ͷ�һЩ�ڴ�
        ColorImage.release(); DepthImage.release(); ROIColorImage.release(); ROIDepthImage.release();

        std::vector<int>().swap(Robx); std::vector<int>().swap(Roby); std::vector<int>().swap(Robz);
        std::vector<int>().swap(Robwt1); std::vector<int>().swap(Robwt2);
        std::vector<float>().swap(depthData); std::vector<float>().swap(Radiuses); std::vector<cv::Point2f>().swap(robcenters);
        std::vector<cv::Mat>().swap(BinaryIms); std::vector<cv::Point2f>().swap(imcenters);

        emit WorkState("���������...");

        /*��ֽƤ�ĳ���*/

        float NowLayerZ;  //�������������ȡ����ֽƤʱ�����˵�Zֵ  1296
        float needzp;
        cv::Mat imageDepthP;       //rbz=1036   1060  imz=1290

        cv::Mat imageP = a.getTYcolor(imageDepthP);
        // cv::Mat imageP = a.getGainImage(imageDepthP);

        d.getDepth(imageDepthP, FirstPaperZ, DepthTu2);

        while(FirstPaperZ==0){ d.getDepth(imageDepthP, FirstPaperZ, DepthTu2); }

        if (FirstPaperZ < MaxWorkZ)
        {
            needzp = -FirstPaperZ + 2400;  //1104
            //needzp = needzp + 100;
            b.RobotStep2(6009, -1308000, 1000, needzp * 1000, 0, 0, 90000, 0);
            b.startCode();
            Sleep(2000);
        }


        else
        {
            emit WorkState("������������ֹͣ����");
            break;
        }


        while (!WorkProcess); //��ͣ��  


       while (1)
        {
            while (!WorkProcess);
            //��ͣ��

            int StepStatus1 = b.RobAddressStatus(0, 874, 3);
            int StepStatus2 = b.RobAddressStatus(0, 875, 3);
            if (!(StepStatus1 || StepStatus2))
            {
                Sleep(300);
                float NowLayerz = (float)b.RobAddressStatus(0, 7100, 1); //��ȡ����ֽƤ��Zֵ
                NowLayerZ = NowLayerz * 0.00001;
                break;
            }
        }


       float NeedZ;
       //NeedZ = -NowLayerZ + 2400;
       NeedZ = FirstPaperZ + 220;   //����һ���������Ĵ��Zֵ

        cv::Mat ColorIm = a.getTYcolor(DepthImage);


        cv::Mat color1 = ColorIm.clone();
        cv::Mat depthn = DepthImage.clone();  //depth1����������������ͼ

       // cv::Mat depth1 = depthn.clone();


        cv::Mat depth1n = d.RegsDepth(depthn, NeedZ);
        cv::Mat depth1 = depth1n.clone();

        cv::Mat outputIm;
        ColorImage = ColorIm.clone();

        
        LayerNumbers++;
        QString Qlayers = QString::number(LayerNumbers);
        emit DisplayLayerNumbers("Now Number of Layers is " + Qlayers);

        if (NeedZ < MaxWorkZ + 200)
        {

             MainProcess(); Delay(50); MainProcess(); DWrite1R(0, 7001, 1); Delay(200);

            float highestZ = 1450;
            
            int tx = (int)((highestZ / NeedZ) * tu.width - tu.width); int ty = (int)(abs(highestZ - NeedZ))/5.06 ;
            cv::Rect R(1,ty,1030,950-2*ty*0.7);
            
            //R��ֵ���²���
            tDepth = R;


           /* int xl = R.tl().x * 2; int yl = R.tl().y * 2; int xt = R.width * 2; int yt = R.height * 2;
            cv::Rect T(xl, yl, xt, yt);
            tColor = T;*/


            cv::Mat rOIColorImage = color1(tColor);
            cv::Mat rOIDepthImage = depth1(tDepth);
            ROIColorImage = rOIColorImage.clone();
            ROIDepthImage = rOIDepthImage.clone();

            cv::imwrite("depthS.jpg", rOIDepthImage);
            while (!WorkProcess);   //��ͣ��

            emit DisplayColor();
            emit DisplayBinary();

            emit WorkState("����ͼ����...");

            
            cv::Mat depth1x = d.DepthWay(rOIDepthImage, FirstPaperZ);

            c.RemoveSmallRegion2(depth1x, depth1x, 100, 0, 1);
            c.RemoveSmallRegion2(depth1x, depth1x,100,1,1 );
            cv::imwrite("depthSs.jpg", depth1x);

            cv::Rect nowRect;

            cv::Mat depth2x = d.getdepRect(depth1x,nowRect);

            float imx = nowRect.tl().x + tDepth.tl().x;
            float imy = nowRect.tl().y + tDepth.tl().y;

            //d.NewDealDepthim(BinaryIms, depth2x,  imcenters, Radiuses, NeedZ, NeedZ0);

            d.DealDepthimWithoutDilate(BinaryIms, depth2x, imcenters, Radiuses, NeedZ, NeedZ0);

            for (int i = 0; i < imcenters.size(); i++)
            {
                imcenters[i].x += imx; 
                imcenters[i].y += imy;
            }

         /*   for (int i = 0; i < imcenters.size(); i++)
            {
                Robz.push_back(((int)NowLayerZ - 230) * 1000);
            }*/

            d.NewGetRob(imcenters,robcenters,NeedZ,Robx, Roby, Robz,  Robwt1, Robwt2,NowLayerZ);
            while (!WorkProcess);   //��ͣ��

            int CountPiece = imcenters.size();
            QString PIECECOUNT = QString::number(CountPiece);
            emit DisplayPieceCount(PIECECOUNT);



            for (int i = 0; i < imcenters.size(); i++)
            {
                emit WorkState("�����˵ȴ�����...");

                while (!WorkProcess);   //��ͣ��

                while (1)
                {
                    int redstatus = b.RobAddressStatus(0, 301, 0);
                    Sleep(400);
                    redstatus = b.RobAddressStatus(0, 301, 0);
                    if (redstatus == 0)
                        break;
                }

                emit WorkState("�����������У�");

                while (!WorkProcess);   //��ͣ��

                if (Robz[i] >= 900000 && Robx[i] >= -850000)
                {
                    b.RobotStepTop2(6001, Robx[i], Roby[i], Robz[i], 0, 0, Robwt1[i], Robwt2[i]);
                    b.startCode();
                    Sleep(2000);
                }

                if (Robz[i] >= 900000 && Robx[i] < -850000)
                {
                    b.RobotStep1(6001, Robx[i], Roby[i], Robz[i], 0, 0, Robwt1[i], Robwt2[i]);
                    b.startCode();
                    Sleep(2000);
                }

                if (Robz[i] < 900000)
                {
                    b.RobotStepTop2(6001, Robx[i], Roby[i], Robz[i], 0, 0, Robwt1[i], Robwt2[i]);
                    b.startCode();
                    Sleep(2000);
                }

                QString ROBX = QString::number(Robx[i] / 1000);
                QString ROBY = QString::number(Roby[i] / 1000);
                QString ROBZ = QString::number(Robz[i] / 1000);
                emit DisplayRobX(ROBX);
                emit DisplayRobY(ROBY);
                emit DisplayRobZ(ROBZ);


                while (!WorkProcess);

                //��ͣ��

                while (1)
                {
                    while (!WorkProcess);

                    //��ͣ��  
                    int StepStatus1 = b.RobAddressStatus(0, 874, 3);
                    int StepStatus2 = b.RobAddressStatus(0, 875, 3);
                    if (!(StepStatus1 || StepStatus2))
                    {
                        Sleep(300);

                        if (b.RobAddressStatus(0, 7001, 1))
                        {
                            b.startCode();
                            Sleep(1000);
                            continue;
                        }

                        else
                            break;
                    }
                }
            }

            while (!WorkProcess);   //��ͣ��


        }

        else
        {
            emit WorkState("������������ֹͣ����");
            break;
        }

    }



}

void ExecuseMode2::run()
{
    NewExecusemode(a, b, c, d);
}
