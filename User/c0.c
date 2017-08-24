#include "c0.h"

extern POSITION_T Position_t;
extern int g_plan;
float angleP,angleD,distantP,pid1,pid2;
int	yiquan,line,SPE=0; 
extern int8_t arr1[20];
extern uint8_t arr2[20];
extern int arr_number;

/*======================================================================================
函数定义	  ：		将小球相对于摄像头的角度转换成相对于陀螺仪的角度(万典学长的函数)
函数参数	  ：		diatance     小球距离摄像头的距离(mm)
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
/*======================================================================================
函数定义	  ：		避免角度溢出(万典学长的函数)
函数参数	  ：		当前角度
函数返回值    ：		修正后的角度
=======================================================================================*/
float AvoidOverAngle(float angle)
{
	if(angle<=-180)
	{
		angle+=360;
	}
	if(angle>180)
	{
		angle-=360;
	}
	return angle;
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

//角度PID调节
float PidAngle(float exAngle,float actAngle)
{
		static float error=0,error_old=0,kp,kd=0,adAngle=0;
	  kp=angleP;
	  kd=angleD;
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
//距离PID调节
float PidCoordinate(float ex,float act)
{
		static float error=0,error_old=0,kp,kd=0,ad=0;
	  kp=distantP;
	  error =ex -act ;
	  ad =kp*error+kd*(error -error_old);
		error_old=error;
	  return ad ;
}
/*======================================================================================
函数定义	  ：		走指定角度的直线闭环
函数参数	  ：		lineAngle     指定的角度（以车的视角为标准）
                  speed         小车的速度
函数返回值  ：	  无
=======================================================================================*/
void ClLineAngle(float lineAngle,int speed)
{
	 if(speed<=500)
	 {
		 angleP=80;
		 angleD=0;
		 distantP=5;
	 }
	 else if(speed>500&&speed<1000)
	 {
		 angleP=100;
		 angleD=20;
		 distantP=7;
	 }
	 else if(speed>=1000&&speed<=1200)
	 {
		 angleP=150;
		 angleD=30;
		 distantP=12;
	 }
	 else if(speed>1200&&speed<=1400)
	 {
		 angleP=200;
		 angleD=30;
		 distantP=14;
	 }
	 else if(speed>1400&&speed<2000)
	 {
		 angleP=250;
		 angleD=30;
		 distantP=15;
	 }
	 else 
	 {
		 angleP=280;
		 angleD=50;
		 distantP=20;
	 }
	 VelCrl(CAN1, 1,(speed*COUNTS_PER_ROUND)/(WHEEL_DIAMETER*PI)+PidAngle(lineAngle,Position_t.angle));
	 VelCrl(CAN1, 2, -(speed*COUNTS_PER_ROUND)/(WHEEL_DIAMETER*PI)+PidAngle(lineAngle,Position_t.angle));
                     //查看PID调节量
	 pid2=PidAngle(lineAngle,Position_t.angle);
}
/*======================================================================================
函数定义	  ：		正方向走指定的任意直线闭环
函数参数	  ：    aimX          直线过的定点的X坐标
                  aimY          直线过的定点的Y坐标
                  lineAngle     指定的角度（以车的视角为标准）
                  speed         小车的速度
函数返回值  ：	  无
=======================================================================================*/
void ClLine(float aimX,float aimY,float lineAngle,int speed)
{
		static double distant=0,k=0,degree=0,impulse=0;
		 if(speed<=500)
	 {
		 angleP=80;
		 angleD=0;
		 distantP=5;
	 }
	 else if(speed>500&&speed<1000)
	 {
		 angleP=100;
		 angleD=20;
		 distantP=7;
	 }
	 else if(speed>=1000&&speed<=1200)
	 {
		 angleP=150;
		 angleD=30;
		 distantP=12;
	 }
	 else if(speed>1200&&speed<=1400)
	 {
		 angleP=200;
		 angleD=30;
		 distantP=14;
	 }
	 else if(speed>1400&&speed<2000)
	 {
		 angleP=250;
		 angleD=30;
		 distantP=15;
	 }
	 else 
	 {
		 angleP=280;
		 angleD=50;
		 distantP=20;
	 }
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
/*======================================================================================
函数定义	  ：		反方向走指定的任意直线闭环（倒着走）
函数参数	  ：    aimX          直线过的定点的X坐标
                  aimY          直线过的定点的Y坐标
                  lineAngle     指定的角度（以车的视角为标准）（车头指向）
                  speed         小车的速度
函数返回值  ：	  无
=======================================================================================*/
void ClLine2(float aimX,float aimY,float lineAngle,int speed)
{
		static double distant=0,k=0,degree=0,impulse=0;
		 if(speed<=500)
	 {
		 angleP=80;
		 angleD=0;
		 distantP=5;
	 }
	 else if(speed>500&&speed<1000)
	 {
		 angleP=100;
		 angleD=20;
		 distantP=7;
	 }
	 else if(speed>=1000&&speed<=1200)
	 {
		 angleP=150;
		 angleD=30;
		 distantP=12;
	 }
	 else if(speed>1200&&speed<=1400)
	 {
		 angleP=200;
		 angleD=30;
		 distantP=14;
	 }
	 else if(speed>1400&&speed<2000)
	 {
		 angleP=250;
		 angleD=30;
		 distantP=15;
	 }
	 else 
	 {
		 angleP=280;
		 angleD=50;
		 distantP=20;
	 }
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
/*======================================================================================
函数定义	  ：		顺时针的正方形闭环
函数参数	  ：    speed         小车的速度
                  lineLong      正方形的边长
                  beginX        正方形左下角的点的X坐标
                  beginY        正方形左下角的点的Y坐标                       
函数返回值  ：	  无
=======================================================================================*/
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

/*======================================================================================
函数定义	  ：		逆时针的正方形闭环
函数参数	  ：    speed         小车的速度
                  lineLong      正方形的边长
                  beginX        正方形右下角的点的X坐标
                  beginY        正方形右下角的点的Y坐标                       
函数返回值  ：	  无
=======================================================================================*/
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
/*======================================================================================
函数定义	  ：    比较三个值的大小，取最大值，返回最大值是第几个数 
函数参数	  ：    number1        第一个数
                  number2        第二个数
                  number3        第三个数         
函数返回值  ：	  mas            第几个数最大
=======================================================================================*/
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
/*======================================================================================
函数定义	  ：    比较四个值的大小，取最大值，返回最大值是第几个数 
函数参数	  ：    number1        第一个数
                  number2        第二个数
                  number3        第三个数
                  number4        第四个数
函数返回值  ：	  mas            第几个数最大
=======================================================================================*/
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
/*======================================================================================
函数定义	  ：    取出最近点的角度和距离
函数参数	  ：    a[20]          一组点的角度
                  b[20]          一组点的距离      
                  sum            这组数据有对应的几个点
函数返回值  ：	  极坐标结构体（有角度和距离）
=======================================================================================*/
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
/*======================================================================================
函数定义	  ：    将场地划分成10*10的100个格子
函数参数	  ：    X            点的X坐标
                  Y            点的Y坐标             
函数返回值  ：	  含有对应横竖的第几个格子的结构体
=======================================================================================*/
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
/*======================================================================================
函数定义	  ：    摄像头第一圈找球，无球时车在不同区域要走的方向
函数参数	  ：    无
                                      
函数返回值  ：	  无
=======================================================================================*/
void First_Scan(void)
{
	static int area;
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
/*======================================================================================
函数定义	  ：    将场地分成内外，用于逃逸
函数参数	  ：    无

函数返回值  ：	  0代表内圈 1代表外圈（以逆时针看，顺时针倒过来）
=======================================================================================*/
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
/*======================================================================================
函数定义	  ：    比较三个数组谁含有的0多
函数参数	  ：    a1[10]        第一个数组
                  a2[10]        第二个数组
                  a3[10]        第三个数组            
函数返回值  ：	  c             含有最多0的是第几个数组
=======================================================================================*/
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
/*======================================================================================
函数定义	  ：    比较四个数组谁含有的0多
函数参数	  ：    a1[10]        第一个数组
                  a2[10]        第二个数组
                  a3[10]        第三个数组  
                  a4[10]        第四个数组
函数返回值  ：	  c             含有最多0的是第几个数组
=======================================================================================*/
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
/*======================================================================================
函数定义	  ：    更新路线，走之前没怎么走过的(顺时针)
函数参数	  ：    down          正方形下面三条线中哪条是要走的
                  right         正方形右面四条线中哪条是要走的         
                  up            正方形上面三条线中哪条是要走的   
                  left          正方形左面四条线中哪条是要走的
函数返回值  ：	  无
=======================================================================================*/
void New_Route(int down,int right,int up,int left)
{
	static int side=1;
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
/*======================================================================================
函数定义	  ：    扫四条边缘(一圈)
函数参数	  ：    无
                                            
函数返回值  ：	  1:              已完成
                  0:              未完成
=======================================================================================*/
int RunEdge(void)
{
	int finish=0;
	static int side=1,num=0;
	if(side==1&&num==0&&Position_t.X>1800)
	{
		num=1;
	}
	if(side==1&&num==1&&Position_t.X>1800)
	{
		side=2;
	}
	
	if(side==2&&num==1&&Position_t.Y>4200)
	{
		num=2;
	}
	if(side==2&&num==2&&Position_t.Y>4200)
	{
		side=3;
	}	
	
	if(side==3&&num==2&&Position_t.X<-1800)
	{
		num=3;
	}
	if(side==3&&num==3&&Position_t.X<-1800)
	{
		side=4;
	}	
	
	if(side==4&&num==3&&Position_t.Y<600)
	{
		num=4;
	}
	if(side==4&&num==4&&Position_t.Y<600)
	{
		side=1;num=0;finish=1;
	}		
	
	switch(side)
	{
		case 1:
		{
			ClLine(0,0,-90,1000);
		}break;
		case 2:
		{
			ClLine(2400,0,0,1000);
		}break;
		case 3:
		{
			ClLine(0,4800,90,1000);
		}break;
		case 4:
		{
			ClLine(-2400,0,180,1000);
		}break;
		default:
		 break;
	}	
	return finish;
}
/*======================================================================================
函数定义	  ：    计算判断球是否在车身范围内，就可以直接直走
函数参数	  ：    di      球离摄像头的距离
                  an      球与中线的夹角
                           
函数返回值  ：	  1:      在车宽内    
                  0:      不在车宽内
=======================================================================================*/
int Vehicle_Width(int di,int an)
{
	float gap;
	gap=di*sin(ANGTORAD(an));
	gap=fabs(gap);
	if(gap<200)
		return 1;
	else 
		return 0;
}
/*======================================================================================
函数定义	  ：    扫描轨迹是否全已走完，走完全部清零
函数参数	  ：    a[10][10]     轨迹二维数组
                                             
函数返回值  ：	  1:            已经都走过了
                  0:            还没走完
=======================================================================================*/
int ScanTrace(int a[10][10])
{
	int m,n,yes=0;
	for(m=0;m<10;m++)
	{
		for(n=0;n<10;n++)
		{
			if(!a[m][n])
			{
				yes++;
			}
		}
	}
	if(yes==0)
	{
		for(m=0;m<10;m++)
	  {
		  for(n=0;n<10;n++)
		  {
	       a[m][n]=0;
		  }
	  }
		return 1;
	}
	else 
	{
		return 0;
	}		
}
/*======================================================================================
函数定义	  ：    将摄像头看到的区域划分成四个同角度的扇形
函数参数	  ：    无
                                            
函数返回值  ：	  numbers.one            从左到右第一个扇形内球数
                  numbers.two            从左到右第二个扇形内球数
                  numbers.there          从左到右第三个扇形内球数
                  numbers.four           从左到右第四个扇形内球数
=======================================================================================*/
Four_t Apart(void)
{
	int k;
	Four_t numbers;
	for(k=0;k<arr_number;k++)
	{
		if(arr1[k]>=-25&&arr1[k]<-7.5)
			numbers.one++;
		if(arr1[k]>=-15&&arr1[k]<2.5)
			numbers.two++;
		if(arr1[k]>=-2.5&&arr1[k]<15)
			numbers.there++;
		if(arr1[k]>=7.5&&arr1[k]<25)
			numbers.four++;
	}
	return numbers;
}

/*======================================================================================
函数定义	  ：    交换顺序
函数参数	  ：    
                  
                           
函数返回值  ：	  无
=======================================================================================*/
void ChangeOrder1(int8_t a,int8_t b)
{
	int8_t c;
	c=a;
	a=b;
	b=c;
}
void ChangeOrder2(uint8_t a,uint8_t b)
{
	uint8_t c;
	c=a;
	a=b;
	b=c;
}
/*======================================================================================
函数定义	  ：    将球的坐标按照从左到右排序
函数参数	  ：    
                  
                           
函数返回值  ：	  无
=======================================================================================*/
void Left2Right(void)
{
	int c1,c2;
	for(c1=0;c1<(arr_number-1);c1++)
	{
		for(c2=0;c2<(arr_number-1);c2++)
		{
			if(arr1[c2]<arr1[(c2+1)])
			{
				ChangeOrder1(arr1[c2],arr1[(c2+1)]);
				ChangeOrder2(arr2[c2],arr2[(c2+1)]);
			}
		}
	}
}
/*======================================================================================
函数定义	  ：    将球的坐标按照从下到上排序
函数参数	  ：    
                  
                           
函数返回值  ：	  无
=======================================================================================*/
void Down2Up(void)
{
	int c1,c2;
	for(c1=0;c1<(arr_number-3);c1++)
	{
		for(c2=1;c2<(arr_number-2);c2++)
		{
			if(arr2[c2]>arr2[(c2+1)])
			{
				ChangeOrder1(arr1[c2],arr1[(c2+1)]);
				ChangeOrder2(arr2[c2],arr2[(c2+1)]);
			}
		}
	}
}
/*======================================================================================
函数定义	  ：    取较前方的平均角度
函数参数	  ：    
                  
                           
函数返回值  ：	  最优角度
=======================================================================================*/
float MostSector(void)
{
	float best,nu=0,total=0;
	int k;
	for(k=0;k<arr_number;k++)
	{
    if(arr2[k]>70)
		{
			nu++;
			total=total+arr1[k];			
		}			
	}	
	if(total==0)
	{
		for(k=0;k<arr_number;k++)
		{
			total=total+arr1[k];	
		}
		best=total/arr_number;
	}
	else 
	{
		best=total/nu;
	}
	return best;
}
/*======================================================================================
函数定义	  ：    两点的距离
函数参数	  ：    
                  
                           
函数返回值  ：	  两点的距离
=======================================================================================*/
float P2P(float a1,float a2,float b1,float b2)
{
	float dist;
	dist=sqrt(PF(a1-b1)+PF(a2-b2));
	return dist;
}


/*=====================================================函数定义（孟yu'hao学长的函数）===================================================*/
//发射航向角转换函数 由度转换为脉冲
float YawTransform(float yawAngle)
{
	return (yawAngle * YAW_REDUCTION_RATIO * COUNT_PER_DEGREE);
}

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void YawAngleCtr(float yawAngle)
{

	PosCrl(CAN1, GUN_YAW_ID, POS_ABS, YawTransform(yawAngle));
}


//送弹推球函数
void PushBall(void)
{
	PosCrl(CAN1, PUSH_BALL_ID, POS_ABS, PUSH_POSITION);
}

//送弹推球收回函数
void PushBallReset(void)
{
	PosCrl(CAN1, PUSH_BALL_ID, POS_ABS, PUSH_RESET_POSITION);
}


//收球电机速度转换函数 由转每秒转换为脉冲
float CollectBallVelTrans(float round)
{
	return round * COUNT_PER_ROUND;
}

//收球电机速度控制函数 单位：转每秒
void CollectBallVelCtr(float round)
{
	VelCrl(CAN1,COLLECT_BALL_ID,CollectBallVelTrans(round));
}
//收球电机需要初始化：	Vel_cfg(CAN1, COLLECT_BALL_ID, 50000,50000);



//发射电机速度转换函数 由转每秒转换为脉冲
int32_t shootVelTrans(float roundPerS)
{
	return (int32_t)-roundPerS * COUNT_PER_ROUND;
}

//发射电机速度控制函数 单位：转每秒
void ShootCtr(float rps)
{
    shootPara_t shootPara;
	
	shootPara.velInt32 = shootVelTrans(rps);

    //起始位
    USART_SendData(USART1, 'A');
    //通过串口1发数
    USART_SendData(USART1, shootPara.velUint8[0]);
    USART_SendData(USART1, shootPara.velUint8[1]);
    USART_SendData(USART1, shootPara.velUint8[2]);
    USART_SendData(USART1, shootPara.velUint8[3]);
    //终止位
    USART_SendData(USART1, 'J');
}