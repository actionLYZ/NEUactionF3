/*=====================================================结构体定义===================================================*/
//记录左中右小球个数
typedef struct
{
	int leftNum;
	int midNum;
	int rightNum;
}BALLNUM_T;
/*=====================================================宏定义区域==================================================*/
#define CAMERATOGYRO     32              //摄像头距离陀螺仪的距离(cm)
#define ANGTORAD(x)      ((x)*PI/180)    //将角度转换成弧度
#define RADTOANG(x)      ((x)*180/PI)    //将弧度转换成角度
#define ADJUSTDIS        50              //左右区域和中间区域的偏差距离调节量，根据实际情况更改(mm)
/*=====================================================函数定义===================================================*/
float AvoidOverAngle(float angle);                 //避免角度溢出
BALLNUM_T SeekMostBall(void);                      //寻找最多小球的区域
float DisBall2Gyro(float distance,float angle);    //将小球相对于摄像头的距离转换成相对于陀螺仪的距离
void CollectMostBall(void);                        //在球最多的区域收集球
void CollecMostBall(void);                         //2号方案粗略收集小球

