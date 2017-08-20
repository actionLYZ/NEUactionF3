#include "c0.h"

extern POSITION_T Position_t;
extern int g_plan;
float angleP,distantP,pid1,pid2;//angle,co_X,co_Y,
int	yiquan,line,SPE=0; 

/*======================================================================================
函数定义	  ：		将小球相对于摄像头的角度转换成相对于陀螺仪的角度
函数参数	  ：		diatance     小球距离摄像头的距离
                  angle        小球相对于摄像头的角度
函数返回值  ：	  aimAngle     小球相对于陀螺仪的角度(单位：度)
=======================================================================================*/
float AngCamera2Gyro(float distance,float angle)
{
	float ThirdSide=0,rad=0,aimAngle=0;
	rad=ANGTORAD(180-angle);
	
	//余弦定理求第三边
	ThirdSide=sqrt(CAMERATOGYRO*CAMERATOGYRO+distance*distance-2*distance*CAMERATOGYRO*cos(rad));
	
	//正弦定理求目标角度(弧度)
	aimAngle=asin(distance*sin(rad)/ThirdSide);
    return RADTOANG(aimAngle);
}
//为了保证能循环
void  GoOn (void)
{
	yiquan=0;line=1;
}
//一点点加速(米每秒)
int SlowSpeedUp(int topspeed)
{
		SPE=SPE+20;
		if(SPE>=topspeed)
		{
				SPE=topspeed;
		}		
    return SPE;		
}
//脉冲
int SlowSpeedUp2(int topspeed)
{
		SPE=SPE+300;
		if(SPE>=topspeed)
		{
				SPE=topspeed;
		}		
    return SPE;		
}
float PidAngle(float exAngle,float actAngle)
{
		static float error=0,error_old=0,kp,kd=0,adAngle=0;
	  kp=angleP;
	  error =exAngle -actAngle ;

		if(error>180)
		{
			error=error -360;
		}
		else if(error<-180)
		{
			error=360+error;
		}
	  adAngle =kp*error+kd*(error -error_old);
		error_old=error;
	  return adAngle ;
}

float PidCoordinate(float ex,float act)
{
		static float error=0,error_old=0,kp,kd=0,ad=0;
	  kp=distantP;
	  error =ex -act ;
	  ad =kp*error+kd*(error -error_old);
		error_old=error;
	  return ad ;
}

void ClLineAngle(float lineAngle,int speed)
{
	 VelCrl(CAN1, 1,(speed*COUNTS_PER_ROUND )/(WHEEL_DIAMETER*PI)+PidAngle(lineAngle,Position_t.angle));
	 VelCrl(CAN1, 2, -(speed*COUNTS_PER_ROUND )/(WHEEL_DIAMETER*PI)+PidAngle(lineAngle,Position_t.angle));
                     //查看PID调节量
	 pid2=PidAngle(lineAngle,Position_t.angle);
}

void ClLine(float aimX,float aimY,float lineAngle,int speed)
{
		static double distant=0,k=0,degree=0,impulse=0;
	  if(fabs(lineAngle)<=0.0001)
		{
			  distant =aimX -Position_t.X ;
		}
		else if(lineAngle>=179.9||lineAngle<=-179.9)
		{
        distant =Position_t.X -aimX ;
		}
		else
		{
				degree=ANGTORAD(lineAngle+90);
			  k=tan(degree);
				distant=(k*Position_t.X-Position_t.Y-k*aimX+aimY)/(sqrt(1+k*k));
				if(lineAngle<0&&lineAngle>=-180)
				{
            distant=-distant;
				}
		}
    impulse = (speed*COUNTS_PER_ROUND)/(WHEEL_DIAMETER*PI);
   	VelCrl(CAN1, 1,impulse+PidCoordinate(0,distant)+PidAngle(lineAngle,Position_t.angle));
		VelCrl(CAN1, 2,-impulse+PidCoordinate(0,distant)+PidAngle(lineAngle,Position_t.angle));
		pid1=PidCoordinate(0,distant);                        //查看PID调节量
		pid2=PidAngle(lineAngle,Position_t.angle);
}
void ClLine2(float aimX,float aimY,float lineAngle,int speed)
{
		static double distant=0,k=0,degree=0,impulse=0;
	  if(fabs(lineAngle)<=0.0001)
		{
			  distant =aimX -Position_t.X ;
		}
		else if(lineAngle>=179.9||lineAngle<=-179.9)
		{
        distant =Position_t.X -aimX ;
		}
		else
		{
				degree=ANGTORAD(lineAngle+90);
			  k=tan(degree);
				distant=(k*Position_t.X-Position_t.Y-k*aimX+aimY)/(sqrt(1+k*k));
				if(lineAngle<0&&lineAngle>=-180)
				{
            distant=-distant;
				}
		}
    impulse = (speed*COUNTS_PER_ROUND)/(WHEEL_DIAMETER*PI);
   	VelCrl(CAN1, 1,impulse-PidCoordinate(0,distant)+PidAngle(lineAngle,Position_t.angle));
		VelCrl(CAN1, 2,-impulse-PidCoordinate(0,distant)+PidAngle(lineAngle,Position_t.angle));
		pid1=PidCoordinate(0,distant);                        //查看PID调节量
		pid2=PidAngle(lineAngle,Position_t.angle);
}

//顺时针的正方形闭环
void ShunClSquare(int speed,float lineLong,float beginX,float beginY)
{	  
		if(line==1)
		{
			 ClLine(beginX,0,0,speed);
		}
		if(line==1&&Position_t.Y>(beginY+lineLong-AD_HIGH_SP))
		{  
			 line=2;
		}
		if(line==2)
		{
			 ClLine(0,(beginY+lineLong),-90,speed);
		}

		if(line==2&&Position_t.X>(beginX+lineLong-AD_HIGH_SP))
		{
			 line=3;
		}
		if(line==3)
		{
			 ClLine((beginX+lineLong),0,180,speed);
		}

		if(line==3&&Position_t.Y<(beginY+AD_HIGH_SP))
		{
			 line=4;yiquan=1;
		}
		if(line==4)
		{
			 ClLine(0,beginY,90,speed);
		}
}


//逆时针的正方形闭环
void NiClSquare(int speed,float lineLong,float beginX,float beginY)
{	  
		if(line==1)
		{
			 ClLine(beginX,0,0,speed);
		}
		if(line==1&&Position_t.Y>(beginY+lineLong-AD_HIGH_SP))
		{  
			 line=2;
		}
		if(line==2)
		{
			 ClLine(0,(beginY+lineLong),90,speed);
		}

		if(line==2&&Position_t.X<(beginX-lineLong+AD_HIGH_SP))
		{
			 line=3;
		}
		if(line==3)
		{
			 ClLine((beginX-lineLong),0,180,speed);
		}

		if(line==3&&Position_t.X<(beginY+AD_HIGH_SP))
		{
			 line=4;yiquan=1;
		}
		if(line==4)
		{
			 ClLine(0,beginY,-90,speed);
		}
}
int Mas(int number1,int number2,int number3)
{
	int mas;
	if(number1==0&&number2==0&&number3==0)
	{
		mas=0;
	}
	else
	{
	  mas=(number1>number2)? number1: number2;
	  mas=(mas>number3)? mas: number3;
	  if(mas==number1)
	  {
			mas=1;
	  }
	  if(mas==number2)
		{
			mas=2;
		}
		if(mas==number3)
		{
			mas=3;
		}
  }
	return mas;	
}
int Mas2(int number1,int number2,int number3,int number4)
{
	int mas;
	if(number1==0&&number2==0&&number3==0&&number4)
	{
		mas=0;
	}
	else
	{
	  mas=(number1>number2)? number1: number2;
	  mas=(mas>number3)? mas: number3;
		mas=(mas>number4)? mas: number4;
	  if(mas==number1)
	  {
			mas=1;
	  }
	  if(mas==number2)
		{
			mas=2;
		}
		if(mas==number3)
		{
			mas=3;
		}
		if(mas==number4)
		{
			mas=4;
		}
  }
	return mas;	
}
PolarCoo_t Closer_Point(int8_t a[20],uint8_t b[20],int sum)
{
	int z,min,q;
	PolarCoo_t closer;
	if(sum==1)
	{
    q=0;
	}
	else if(sum==2)
	{
		q=(b[0]<b[1])? 0:1;
	}		
	else 
	{
		min=(b[0]<b[1])? b[0]:b[1];
		q=(b[0]<b[1])? 0:1;
		for(z=2;z<sum;z++)
	  {
			q=(min<b[z])? q:z;
		  min=(min<b[z])? min:b[z];		 
	  }
	}
	closer.ang=a[q];
	closer.dis=b[q];
	return closer;
}
Coo_t Zoning(float X,float Y)
{
	Coo_t wirte;
	int m=1,o=1;
	while((X-m*480)>-2400)
	{
		m++;
	}
	wirte.hor=m;
	while((Y-o*480)>0)
	{
		o++;
	}
	wirte.ver=o;
	return wirte;
}
void First_Scan(void)
{
	int area;
		//划分区域
	if(g_plan==-1)
	{
		if(Position_t.X<=-275&&Position_t.Y<=3100)
		{
			 area=1;
		}
		else if(Position_t.X>=-275&&Position_t.Y<1700)
		{
			 area=2;
		}
		else if(Position_t.X>275&&Position_t.Y>=1700) 
		{
			 area=3;
		}
		else
		{
			 area=4;
		}
	}		 	
	if(g_plan==1)
	{
    if(Position_t.X>275&&Position_t.Y<3100)
		{
			 area=1;
		}
	  else if(Position_t.X>-275&&Position_t.Y>=3100)
		{
			 area=2;
		}
		else if(Position_t.X<=-275&&Position_t.Y>1700)
		{
			 area=3;
		}
		else 
		{
			 area=4;
		}
	}
	
	switch(area) 
   {
		case 1:
		{
			ClLineAngle(0,800);
		}break;
		case 2:
		{
			ClLineAngle(90,800);
		}break;
		case 3:
		{
		    ClLineAngle(180,800);
		}break;
		case 4:
		{
			ClLineAngle(-90,800);
		}break;
		default:
		 break;
   }								
}
int In_Or_Out(void)
{   
   //将正方形区域分成内外两部分
   if(Position_t.X>-1200&&Position_t.X<1200&&Position_t.Y>1200&&Position_t.Y<3600)
   { 
	 if(g_plan==-1) return 1;
	 if(g_plan==1)  return 0;			  
   }
   else 
   {
	 if(g_plan==-1) return 0;
	 if(g_plan==1)  return 1;	
   }	
}

int Least_H(int a1[10],int a2[10],int a3[10])
{
  int i,b1=0,b2=0,b3=0,c;
	for(i=0;i<10;i++)
	{
		if(!a1[i])
		{
			b1++;
		}
		if(!a2[i])
		{
			b2++;
		}
		if(!a3[i])
		{
			b3++;
		}
	}
	c=Mas(b1,b2,b3);
	return c;
}
int Least_S(int a1[10],int a2[10],int a3[10],int a4[10])
{
	int i,b1=0,b2=0,b3=0,b4=0,c;
	for(i=0;i<10;i++)
	{
		if(!a1[i])
		{
			b1++;
		}
		if(!a2[i])
		{
			b2++;
		}
		if(!a3[i])
		{
			b3++;
		}
		if(!a4[i])
		{
			b4++;
		}
	}
	c=Mas2(b1,b2,b3,b4);
	return c;	
}	
void change(int ar[10],int arr[10][10],int e)
{
	int k;
	for(k=0;k<10;k++)
	{
		ar[k] = arr[k][e];
	}
}
void New_Route(int down,int right,int up,int left)
{
	int side=1;
	if(side==1&&Position_t.X>(240+right*480-AD_MID_SP))
	{
		side=2;
	}
	if(side==2&&Position_t.Y>(3120+up*480-AD_MID_SP))
	{
		side=3;
	}
	if(side==3&&Position_t.X<(-2640+left*480+AD_MID_SP))
	{
		side=4;
	}
	if(side==4&&Position_t.Y<(-240+down*480+AD_MID_SP))
	{
		side=1;
	}
	switch(side)
	{
		case 1:
		{
			ClLine(0,-240+down*480,-90,800);
		}break;
		case 2:
		{
			ClLine(240+right*480,0,0,800);
		}break;
		case 3:
		{
			ClLine(0,3120+up*480,90,800);
		}break;
		case 4:
		{
			ClLine(-2640+left*480,0,180,800);
		}break;
		default:
		 break;
	}
}
