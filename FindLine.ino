/*
 * date: 11/7
 * add:  LCD
 */
//========================================定义区=======================================
void Zhua();
void LayCube();

int color_step();
void ReadColor();
void Step1();
void Step2();
void Step3();
void Step4();
void Step5();
void Step6();


void CarGoHen();
void CarGoShu();
void CarTurnAround();//转180°
void CarTurnLeft();
void CarTurnRight();
void CarStop();
int COLOR_STEP;//返回颜色拿取顺序

int LDR_pin = A1;   //定义光敏电阻模拟口A1
//int Intensity = 0;  //光照度数值

//----------------motor------------------
/*
int STBY = 7; //使能端
//Motor A
int PWMA = 8; //左电机PWM输出控制脚   
int AIN1 = 9; //左电机正极  
int AIN2 = 10; //左电机负极  
  
//Motor B  
int PWMB = 11; //右电机PWM输出控制脚
int BIN1 = 12; //右电机正极 
int BIN2 = 13; //右电机负极  
*/
//========================
int L1=A2;    //定义红外传感器引脚
int L2=A3;
int R1=A4;
int R2=A5;

int SL_1=0;   //定义红外传感器状态
int SL_2=0;
int SR_1=0;
int SR_2=0;

int index=0;  //初始化方向




//========================================机械臂=======================================
#include<Servo.h>
Servo myservo[6];
int PWMPin[5]={2,3,4,5,6};//暂定5个，其中第五个为抓取动作执行舵机

float zhuatheta[20][4]={    //由matlab计算得,20步
  {90.0000,  180.0000,   30.0000,   60.0000},
  {89.8789,  179.8897,   30.1498,   59.9605},
  {89.1091,  179.1886,   31.1020,   59.7095},
  {87.2433,  177.4893,   33.4097,   59.1010},
  {84.0309,  174.5635,   37.3830,   58.0535},
  {79.3911,  170.3378,   43.1218,   56.5404},
  {73.3872,  164.8696,   50.5478,   54.5826},
  {66.2001,  158.3239,   59.4372,   52.2389},
  {58.1026,  150.9490,   69.4528,   49.5983},
  {49.4326,  143.0526,   80.1764,   46.7710},
  {40.5674,  134.9785,   91.1414,   43.8800},
  {31.8974,  127.0822,  101.8650,   41.0528},
  {23.7999,  119.7072,  111.8806,   38.4122},
  {16.6128,  113.1616,  120.7700,   36.0685},
  {10.6089,  107.6934,  128.1960,   34.1106},
  { 5.9691,  103.4676,  133.9348,   32.5976},
  { 2.7567,  100.5419,  137.9081,   31.5500},
  { 0.8909,   98.8426,  140.2158,   30.9416},
  { 0.1211,   98.1414,  141.1681,   30.6905},
  { 0.0000,   98.0312,  141.3178,   30.6510}
};

float fangtheta[20][4]{//放置时的关节角
   {90.0000,  180.0000,   30.0000,   60.0000},
   {89.8789,  179.9850,   29.9114,   60.1036},
   {89.1091,  179.8899,   29.3479,   60.7622},
   {87.2433,  179.6594,   27.9824,   62.3582},
   {84.0309,  179.2625,   25.6313,   65.1062},
   {79.3911,  178.6892,   22.2355,   69.0753},
   {73.3872,  177.9474,   17.8414,   74.2112},
   {66.2001,  177.0594,   12.5813,   80.3593},
   {58.1026,  176.0589,    6.6549,   87.2862},
   {49.4326,  174.9877,    0.3095,   94.7028},
   {40.5674,  173.8923,   -6.1788,  102.2864},
   {31.8974,  172.8211,  -12.5242,  109.7030},
   {23.7999,  171.8206,  -18.4506,  116.6300},
   {16.6128,  170.9326,  -23.7106,  122.7780},
   {10.6089,  170.1908,  -28.1048,  127.9140},
   { 5.9691,  169.6175,  -31.5006,  131.8830},
   { 2.7567,  169.2206,  -33.8517,  134.6310},
   { 0.8909,  168.9901,  -35.2172,  136.2271},
   { 0.1211,  168.8950,  -35.7806,  136.8857},
   {-0.0000,  168.8800,  -35.8692,  136.9892}
  };

float readcolor[20][4]{//识别颜色的关节角度
  {90.0000,  180.0000,   30.0000,   60.0000},
  {89.8789,  179.8897,   30.1498,   59.9605},
  {89.1091,  179.1886,   31.1020,   59.7095},
  {87.2433,  177.4893,   33.4097,   59.1010},
  {84.0309,  174.5635,   37.3830,   58.0535},
  {79.3911,  170.3378,   43.1218,   56.5404},
  {73.3872,  164.8696,   50.5478,   54.5826},
  {66.2001,  158.3239,   59.4372,   52.2389},
  {58.1026,  150.9490,   69.4528,   49.5983},
  {49.4326,  143.0526,   80.1764,   46.7710},
  {40.5674,  134.9785,   91.1414,   43.8800},
  {31.8974,  127.0822,  101.8650,   41.0528},
  {23.7999,  119.7072,  111.8806,   38.4122},
  {16.6128,  113.1616,  120.7700,   36.0685},
  {10.6089,  107.6934,  128.1960,   34.1106},
  { 5.9691,  103.4676,  133.9348,   32.5976},
  { 2.7567,  100.5419,  137.9081,   31.5500},
  { 0.8909,   98.8426,  140.2158,   30.9416},
  { 0.1211,   98.1414,  141.1681,   30.6905},
  { 0.0000,   98.0312,  141.3178,   30.6510}
  };
//////////////////////////
void arm_ini(){//初始化舵机
  for(int i=0;i<6;i++){  //初始化各个舵机
    myservo[i].attach(PWMPin[i]);
    myservo[i].write(90);
    delay(20);
    }
  }
////////////////////////function////////////////
float BeginToRecognise(){//伸出手臂识别颜色、不退回
  float Recognise;
  for(int i=0;i<20;i++){//伸出
    for(int j=0;j<4;j++){
      myservo[j].write(readcolor[i][j]);
      Serial.println(readcolor[i][j]);
      delay(30);//动作响应时间
    }
   }
  Recognise = GetColor();//Getcolor函数读取颜色,cube的
  return Recognise;
  }

void ArmBack(){//机械臂/平滑/回归初始位（读取颜色之后）
  for(int i=19;i>=0;i--){//退回
    for(int j=0;j<4;j++){
      myservo[j].write(readcolor[i][j]);
//      Serial.println(readcolor[i][j]);
      delay(20);//动作响应时间
    }
   }  
  }

void Zhua(){  //抓取时关节动作  
    for(int i=0;i<20;i++){//伸出
    for(int j=0;j<4;j++){
      myservo[j].write(zhuatheta[i][j]);
//      Serial.println(zhuatheta[i][j]);
      delay(30);//动作响应时间
    }
   }
   myservo[4].write(90);//手爪抓，角度值可适当调整
   for(int i=19;i>=0;i--){//缩回
    for(int j=0;j<4;j++){
      myservo[j].write(zhuatheta[i][j]);
//      Serial.println(zhuatheta[i][j]);
      delay(30);
    }
   }
 }
 
void LayCube(){  //放置区放置时的关节动作  
    for(int i=0;i<20;i++){//伸出
    for(int j=0;j<4;j++){
      myservo[j].write(fangtheta[i][j]);
//      Serial.println(fangtheta[i][j]);
      delay(30);//动作响应时间
    }
   }
   myservo[4].write(0);//手爪松开
   for(int i=19;i>=0;i--){//缩回
    for(int j=0;j<4;j++){
      myservo[j].write(fangtheta[i][j]);
//      Serial.println(fangtheta[i][j]);
      delay(30);
    }
   }
 }

//=========================================================显示屏==============================================================
#include <LiquidCrystal.h> //申明1602液晶的函数库
//申明1602液晶的引脚所连接的Arduino数字端口，8线或4线数据模式，任选其一
LiquidCrystal lcd(38,39,40,41,42,43,44,45,46,47,48);   //8数据口模式连线声明
//LiquidCrystal lcd(12,11,10,5,4,3,2); //4数据口模式连线声明
void lcd_ini(){
  lcd.begin(16,2);      //初始化1602液晶工作模式//定义1602液晶显示范围为2行16列字符
}                 
void  ShowColor()
  {
    lcd.home();        //把光标移回左上角，即从头开始输出   
    lcd.print("拿取顺序为："); //显示
    lcd.setCursor(0,1);   //把光标定位在第1行，第0列
  
    COLOR_STEP = color_step();
    switch(COLOR_STEP){
    case 1: lcd.print("红-绿-蓝");break;//123
    case 2: lcd.print("红-蓝-绿");break;//132
    case 3: lcd.print("绿-红-蓝");break;//213
    case 4: lcd.print("绿-蓝-红");break;//231
    case 5: lcd.print("蓝-红-绿");break;//312
    case 6: lcd.print("蓝-绿-红");break;//321
    default:break;}
    delay(500);
    /*//特效
    for(i=0;i<3;i++)
    {
      lcd.noDisplay();
      delay(500);
      lcd.display();
      delay(500);
    }
    for(i=0;i<24;i++)
    {
      lcd.scrollDisplayLeft();
      delay(500);
    }
    */
  }

//void 


//=======================================================颜色识别=============================================================

//int LDR_pin = A1;   //定义光敏电阻模拟口A1
int Intensity = 0;  //光照度数值
//初始化///////////////////
void color_ini()
{
  //设置波特率9600
  Serial.begin(9600);

}
///////function///////////////////
//排序
void bubble(unsigned long *a, int n) /*定义两个参数：数组首地址与数组大小*/
{
  int i, j, temp;
  for (i = 0; i < n - 1; i++)
  {
    for (j = i + 1; j < n; j++) /*注意循环的上下限*/
    {
      if (a[i] > a[j])
      {
        temp = a[i];
        a[i] = a[j];
        a[j] = temp;
      }
    }
  }
}

//灰度传感器识别颜色,并返回相应的灰度值
void LDR_test()
{
  unsigned long color[5] = {0};
  int num = 0;
  while (num < 5)
  {
    color[num] = analogRead(LDR_pin);
    num++;
  }
  num = 0;
  bubble(color, 5);
  Intensity = (color[1] + color[2] + color[3]) / 3;  //去除最大最小值
  Serial.print("Intensity = ");         //串口输出"Intensity = "
  Serial.println(Intensity);            //串口输出Intensity变量的值，并换行
  return;
}

//识别颜色
//定义颜色
int RED = 1;
int GREEN = 2;
int BLUE = 3;

int GetColor(){//读取cube颜色返回颜色
  int color;
  LDR_test(); 
  if (Intensity >= 130 && Intensity <= 150)
  {
    color = GREEN;        //绿色
  }
  if (Intensity >= 190 && Intensity <= 200)
  {
    color = BLUE;   //蓝色
  }
  if (Intensity >= 165 && Intensity <= 189)
  {
    color = RED;   //红色
    }
  return color;
}

//===================================电机&寻迹================================================
int STBY = 7; //使能端

//Motor A
int PWMA = 8; //左电机PWM输出控制脚   
int AIN1 = 9; //左电机正极  
int AIN2 = 10; //左电机负极  
  
//Motor B  
int PWMB = 11; //右电机PWM输出控制脚
int BIN1 = 12; //右电机正极 
int BIN2 = 13; //右电机负极  

//========================
/*
int L1=A2;    //定义红外传感器引脚
int L2=A3;
int R1=A4;
int R2=A5;

int SL_1=0;   //定义红外传感器状态
int SL_2=0;
int SR_1=0;
int SR_2=0;*/

//int index=0;  //初始化方向

//初始化
void motor_ini(){
  //初始化电机驱动IO为输出方式
  pinMode(STBY, OUTPUT);  
  
  pinMode(PWMA, OUTPUT);  
  pinMode(AIN1, OUTPUT);  
  pinMode(AIN2, OUTPUT);  
  
  pinMode(PWMB, OUTPUT);  
  pinMode(BIN1, OUTPUT);  
  pinMode(BIN2, OUTPUT);  
  
  //初始化五路巡线器
  pinMode(L1,INPUT);
  pinMode(L2,INPUT);
  pinMode(R1,INPUT);
  pinMode(R2,INPUT);
}

///////////////////////////////基本动作///////////////////////////////

/*
 * 小车行动
 */
void CarGoHen();
void CarGoShu();
void CarTurnAround();//转180°
void CarTurnLeft();
void CarTurnRight();
void CarStop();

//int COLOR_STEP;//返回颜色拿取顺序

void runset(int motor, int speed, int direction){  //控制单个电机正反转及速度
  
  digitalWrite(STBY, HIGH); //使能驱动模块脚 
  
  boolean Pin1 = LOW;  
  boolean Pin2 = HIGH;  
  
  if(direction == 1){  
    Pin1 = HIGH;  
    Pin2 = LOW;  
  }  
  
  if(motor == 1){  
    digitalWrite(AIN1, Pin1);  
    digitalWrite(AIN2, Pin2);  
    analogWrite(PWMA, speed);  
  }else{  
    digitalWrite(BIN1, Pin1);  
    digitalWrite(BIN2, Pin2);  
    analogWrite(PWMB, speed);  
  }  
}  

/*
 * 直走
 */
void Go(){
  runset(1, 250, 1);  
  runset(2, 250, 1); 
  }

 
/*
 * 左偏移
 */
void Left(){  //
  runset(1, 150, 0); //左电机中速向后转  
  runset(2, 150, 1); //右电机中速向前转  
  }
/*
 *右偏移 
 */
void Right(){
  runset(1, 150, 1);  
  runset(2, 150, 0);   
  }

/*
 * 巡线 & 横走
 */
void CarGoHen()   
{
  if(R2==0){ 
      Left();
    }
  else{
      if(L1==0)  Right();
      else  Go();
    } 
}

/*
 * 巡线 & 竖着走
 */
 void CarGoShu()   
{
  if(R1==0){ 
      Left();
    }
  else{
      if(L2==0)  Right();
      else  Go();
    } 
}
/*
 * 综合巡线
 */
void CarGo(){
  if(index%2==0) 
    CarGoHen();
  if(index%2==1)
    CarGoShu();
  }

  
//////转向///////
 
/*
 * //旋转180°
 */
void CarTurnAround()
{
  runset(1, 200, 0); //左电机以200/255速度向后转  
  runset(2, 200, 1); //右电机以200/255速度向前转  
  delay(1000);   //通过调整时间来控制转向角度
  index += 2;
}
/*
 * 左转
 */
void CarTurnLeft()
{
  runset(1, 200, 0); //左电机以200/255速度向后转  
  runset(2, 200, 1); //右电机以200/255速度向前转  
  delay(500);   //通过调整时间来控制转向角度
  index += 1;
}
/*
 * 右转
 */
void CarTurnRight()
{
  runset(1, 200, 1);  
  runset(2, 200, 0);   
  delay(500);
  index += 3;
}
/*
 * 停车
 */
void CarStop()
{
  digitalWrite(STBY, LOW);
}


//======================================================================================================
/*
 * 
 * N多变量的初始化
 */
static int ROW = 0;//行
static int COL = 0;//列
//int index = 0;//初始标志
//int a[][2];//定义一个数组存储走过的路
int ai = 0;
int Line[][2]={0};//定义路径数组
/*
 * 初始化数据a[][2]，主路线
 */
int a[][2] = {{0,1},{6,1},{6,0},{4,3},{0,3},{0,0}};
//          1     2     3     4     5     6      

//如果后来不是4，1提取的话就改多个s数组,在这块反正好改
int s1[][2] = {{6,1},{5,1},{3,1},{3,3},{5,3},{5,1},{3,1},{3,3},{4,3},{5,3},{5,1},{3,1},{3,3}}; //13
int s2[][2] = {{6,1},{5,1},{3,1},{3,3},{5,3},{5,1},{3,1},{3,3},{5,3},{5,1},{3,1},{3,3},{4,3}};//13
int s3[][2] = {{6,1},{5,1},{3,1},{3,3},{4,3},{5,3},{5,1},{3,1},{3,3},{5,3},{5,1},{3,1},{3,3}};//13
int s4[][2] = {{6,1},{5,1},{3,1},{3,3},{4,3},{5,3},{5,1},{3,1},{3,3},{5,3},{5,1},{3,1},{3,3},{5,3}};//14
int s5[][2] = {{6,1},{5,1},{3,1},{3,3},{5,3},{5,1},{3,1},{3,3},{5,3},{5,1},{3,1},{3,3},{4,3}};//13
int s6[][2] = {{6,1},{5,1},{3,1},{3,3},{5,3},{5,1},{3,1},{3,3},{4,3},{5,3},{5,1},{3,1},{3,3},{5,3}};//14

/*
 * 转向识别
 */
 
 /*
int SL_1;//左1轨道识别状态
int SL_2;//左2轨道识别状态
int SR_1;//右1轨道识别状态
int SR_2;//右2轨道识别状态
*/


/*
 * 循环调用函数
 */
 //////////////////////////////////////////////////////
 //////////////////////////////////////////////////////

void setup() {
  //初始化各个部分
  color_ini();
  motor_ini();
  arm_ini();
  lcd_ini();
}
 /*
void loop() {
  // put your main code here, to run repeatedly:
  //Hengzhezou();
  //Shuzhezhou();
  FindLine();
}
*/
int color_step()
{
   String order;
   //readQR();//读取二维码，读到的顺序付给order
   if(order == "123")//RGB
     return 1;
   if(order == "132")
     return 2;
   if(order == "213")
     return 3;
   if(order == "231")
     return 4;
   if(order == "312")
     return 5;
   if(order == "321")
     return 6;
}


void ReadColor()//读取路径
{
   COLOR_STEP = color_step();
  switch(COLOR_STEP){
    case 1: Step1();break;
    case 2: Step2();break;
    case 3: Step3();break;
    case 4: Step4();break;
    case 5: Step5();break;
    case 6: Step6();break;
   default:break;
   }
}

void Step1(){//123
  int i = 0;
  CarTurnAround();
  if(Line[COL][ROW] == s1[i][2]){//i=0
      CarTurnLeft();
      i++;
      CarGo();
     }
     
  if(Line[COL][ROW] == s1[i][2]){//i=1
      BeginToRecognise();
      if(GetColor() == RED)
      CarStop();
      Zhua();
      CarGo();
      i++;
    }
    if(Line[COL][ROW] == s1[i][2]){//i=2
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s1[i][2]){//i=3
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s1[i][2]){//i=4
      CarStop();
      LayCube();
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s1[i][2]){//i=5
      CarTurnRight();
      i++;
      CarGo();
      BeginToRecognise();
      if(GetColor() == GREEN)
        CarStop();
        Zhua();
        CarGo();
      }
    if(Line[COL][ROW] == s1[i][2]){//i=6
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s1[i][2]){//i=7
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s1[i][2]){//i=8
      CarStop();
      LayCube();
      i++;
      CarGo();
    }
     if(Line[COL][ROW] == s1[i][2]){//i=9
       CarTurnRight();
       i++;
       CarGo();
    }
    if(Line[COL][ROW] == s1[i][2]){//i=10
      CarTurnRight();
      BeginToRecognise();
      if(GetColor() == BLUE)
      {
        CarStop();
        Zhua();
        CarGo();
      }
      i++;
    }
    if(Line[COL][ROW] == s1[i][2]){//i=11
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s1[i][2]){//i=12
      CarStop();
      LayCube();
      i++;
      CarTurnLeft();
      CarGo();
    }
}

void Step2(){//132
   int i = 0;
  CarTurnAround();
  if(Line[COL][ROW] == s2[i][2]){//i=0
      CarTurnLeft();
      i++;
      CarGo();
     }
     
  if(Line[COL][ROW] == s2[i][2]){//i=1
      BeginToRecognise();
      if(GetColor() == RED)
      CarStop();
      Zhua();
      CarGo();
      i++;
    }
    if(Line[COL][ROW] == s2[i][2]){//i=2
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s2[i][2]){//i=3
      CarStop();
      LayCube();//放下物体
      i++;
      CarTurnRight();
      CarGo();
    }

    if(Line[COL][ROW] == s2[i][2]){//i=4
      CarTurnRight();
      i++;
      CarGo();
      BeginToRecognise();
      if(GetColor() == BLUE)
        CarStop();
        Zhua();
        CarGo();
      }
    if(Line[COL][ROW] == s2[i][2]){//i=6
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s2[i][2]){//i=7
      CarStop();
      LayCube();
      i++;
      CarTurnRight();
      CarGo();
    }
    if(Line[COL][ROW] == s2[i][2]){//i=8
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s2[i][2]){//i=9
      CarTurnRight();
      BeginToRecognise();
      if(GetColor() == GREEN)
      {
        CarStop();
        Zhua();
        CarGo();
      }
      i++;
    }
    if(Line[COL][ROW] == s2[i][2]){//i=10
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s2[i][2]){//i=11
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s2[i][2]){//i=12
      CarStop();
      LayCube();
      i++;
      CarTurnAround();
      CarGo();
    }
}

void Step3(){//213
  int i = 0;
  CarTurnAround();
  if(Line[COL][ROW] == s3[i][2]){//i=0
      CarTurnLeft();
      i++;
      CarGo();
     }
     
  if(Line[COL][ROW] == s3[i][2]){//i=1
      BeginToRecognise();
      if(GetColor() == GREEN)
      CarStop();
      Zhua();
      CarGo();
      i++;
    }
    if(Line[COL][ROW] == s3[i][2]){//i=2
      CarTurnRight();
      i++;
      CarGo();
    }
     if(Line[COL][ROW] == s3[i][2]){//i=3
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s3[i][2]){//i=4
      CarStop();
      LayCube();//放下物体
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s3[i][2]){//i=5
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s3[i][2]){//i=6
      CarTurnRight();
      i++;
      CarGo();
      BeginToRecognise();
      if(GetColor() == RED)
        CarStop();
        Zhua();
        CarGo();
      }
    if(Line[COL][ROW] == s3[i][2]){//i=7
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s3[i][2]){//i=8
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s3[i][2]){//i=9
      CarStop();
      LayCube();
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s3[i][2]){//i=10
      CarTurnRight();
      BeginToRecognise();
      if(GetColor() == BLUE)
      {
        CarStop();
        Zhua();
        CarGo();
      }
      i++;
    }
    if(Line[COL][ROW] == s3[i][2]){//i=11
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s3[i][2]){//i=12
      CarStop();
      LayCube();
      i++;
      CarTurnLeft();
      CarGo();
    }
}

void Step4(){//231
  int i = 0;
  CarTurnAround();
  if(Line[COL][ROW] == s4[i][2]){//i=0
      CarTurnLeft();
      i++;
      CarGo();
     }
     
  if(Line[COL][ROW] == s4[i][2]){//i=1
      BeginToRecognise();
      if(GetColor() == GREEN)
      CarStop();
      Zhua();
      CarGo();
      i++;
    }
    if(Line[COL][ROW] == s4[i][2]){//i=2
      CarTurnRight();
      i++;
      CarGo();
    }
     if(Line[COL][ROW] == s4[i][2]){//i=3
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s4[i][2]){//i=4
      CarStop();
      LayCube();//放下物体
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s4[i][2]){//i=5
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s4[i][2]){//i=6
      CarTurnRight();
      i++;
      CarGo();
      BeginToRecognise();
      if(GetColor() == BLUE)
        CarStop();
        Zhua();
        CarGo();
      }
    if(Line[COL][ROW] == s4[i][2]){//i=7
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s4[i][2]){//i=8
      CarStop();
      LayCube();
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s4[i][2]){//i=9
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s4[i][2]){//i=10
      CarTurnRight();
      BeginToRecognise();
      if(GetColor() == RED)
      {
        CarStop();
        Zhua();
        CarGo();
      }
      i++;
    }
    if(Line[COL][ROW] == s4[i][2]){//i=11
      CarTurnRight();
      i++;
      CarGo();
    }

    if(Line[COL][ROW] == s4[i][2]){//i=12
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s4[i][2]){//i=13
      CarStop();
      LayCube();
      i++;
      CarTurnAround();
      CarGo();
    }
}


void Step5(){//312
  int i = 0;
  CarTurnAround();
  if(Line[COL][ROW] == s5[i][2]){//i=0
      CarTurnLeft();
      i++;
      CarGo();
     }
     
  if(Line[COL][ROW] == s5[i][2]){//i=1
      BeginToRecognise();
      if(GetColor() == BLUE)
      CarStop();
      Zhua();
      CarGo();
      i++;
    }
    if(Line[COL][ROW] == s5[i][2]){//i=2
      CarTurnRight();
      i++;
      CarGo();
    }
     if(Line[COL][ROW] == s5[i][2]){//i=3
      CarStop();
      LayCube();
      i++;
      CarTurnRight();
      CarGo();
    }
    
    if(Line[COL][ROW] == s5[i][2]){//i=4
     CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s5[i][2]){//i=5
      CarTurnRight();
      i++;
      CarGo();
      BeginToRecognise();
      if(GetColor() == RED)
        CarStop();
        Zhua();
        CarGo();
      }
    if(Line[COL][ROW] == s5[i][2]){//i=6
      CarTurnRight();
      i++;
      CarGo();
    }

    if(Line[COL][ROW] == s5[i][2]){//i=7
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s5[i][2]){//i=8
      CarStop();
      LayCube();
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s5[i][2]){//i=9
      CarTurnRight();
      BeginToRecognise();
      if(GetColor() == GREEN)
      {
        CarStop();
        Zhua();
        CarGo();
      }
      i++;
    }
    if(Line[COL][ROW] == s5[i][2]){//i=10
      CarTurnRight();
      i++;
      CarGo();
    }

    if(Line[COL][ROW] == s5[i][2]){//i=11
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s5[i][2]){//i=12
      CarStop();
      LayCube();
      i++;
      CarTurnAround();
      CarGo();
    }
}

void Step6(){//321
      int i = 0;
  CarTurnAround();
  if(Line[COL][ROW] == s6[i][2]){//i=0
      CarTurnLeft();
      i++;
      CarGo();
     }
     
  if(Line[COL][ROW] == s6[i][2]){//i=1
      BeginToRecognise();
      if(GetColor() == BLUE)
      CarStop();
      Zhua();
      CarGo();
      i++;
    }
    if(Line[COL][ROW] == s6[i][2]){//i=2
      CarTurnRight();
      i++;
      CarGo();
    }
     if(Line[COL][ROW] == s6[i][2]){//i=3
      CarStop();
      LayCube();
      i++;
      CarTurnRight();
      CarGo();
    }
    
    if(Line[COL][ROW] == s6[i][2]){//i=4
     CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s6[i][2]){//i=5
      CarTurnRight();
      i++;
      CarGo();
      BeginToRecognise();
      if(GetColor() == GREEN)
        CarStop();
        Zhua();
        CarGo();
      }
    if(Line[COL][ROW] == s6[i][2]){//i=6
      CarTurnRight();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s6[i][2]){//i=7
      CarTurnRight();
      i++;
      CarGo();
    }

    if(Line[COL][ROW] == s6[i][2]){//i=8
      CarStop();
      LayCube();
      i++;
      CarGo();
    }
    if(Line[COL][ROW] == s6[i][2]){//i=9
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s6[i][2]){//i=10
      CarTurnRight();
      BeginToRecognise();
      if(GetColor() == RED)
      {
        CarStop();
        Zhua();
        CarGo();
      }
      i++;
    }
    if(Line[COL][ROW] == s6[i][2]){//i=11
      CarTurnRight();
      i++;
      CarGo();
    }

    if(Line[COL][ROW] == s6[i][2]){//i=12
      CarTurnRight();
      i++;
      CarGo();
    }
    
    if(Line[COL][ROW] == s6[i][2]){//i=13
      CarStop();
      LayCube();
      i++;
      CarTurnAround();
      CarGo();
    }
}


void Hengzhezou()//
{
  //横向行走路过十字路口
    if(SL_1 == LOW && SL_2 ==HIGH && SR_1 == HIGH && SR_2 == LOW)
    {
      CarGo();//直走
      if(SL_1 == HIGH && SL_2 ==HIGH && SR_1 == HIGH && SR_2 == HIGH)
      {
        switch((int)index%4)
        {
        case 1: COL++;break;
        case 2: ROW++;break;
        case 3: COL--;break;
        case 4: ROW--;break;
        defualt:break;
      }
    }
  }
}


void Shuzhezou(){
  
    //径向行走路过十字路口
    if(SL_1 == LOW && SL_2 == LOW && SR_1 ==LOW && SR_2 == LOW)
    {
      CarGo();//直走
      if(SL_1 == HIGH && SL_2 == LOW && SR_1 ==LOW && SR_2 == HIGH)
      {
        switch(index%4)
        {
        case 1: COL++;break;
        case 2: ROW++;break;
        case 3: COL--;break;
        case 4: ROW--;break;
        defualt:break;
       }
    }
   }
}

void FindLine(){
  
    int ai = 0;//定义一个初始值
    SR_1 = digitalRead(R1);
    SR_2 = digitalRead(R2);
    SL_1 = digitalRead(L1);
    SL_2 = digitalRead(L2);
    

    if(Line[COL][ROW] == a[ai][2]){//i=0
      CarTurnRight();
      ai++;
      CarGo();
    }

    if(Line[COL][ROW] == a[ai][2]){//i=1
      CarTurnRight();
      ai++;
      CarGo();
    }

    if(Line[COL][ROW] == a[ai][2]){//i=2
      CarStop();
      ai++;
      ReadColor();//二维码读取顺序
      ShowColor();//显示颜色顺序
    }  
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
    if(Line[COL][ROW] == a[ai][2]){//i=3
      CarTurnLeft();
      ai++;
      CarGo();
    }
     if(Line[COL][ROW] == a[ai][2]){//i=4
      CarTurnLeft();
      ai++;
      CarGo();
     }

     if(Line[COL][ROW] == a[ai][2]){//i=5
      CarStop();
     }
     

}



void loop() {
  // put your main code here, to run repeatedly:
  Hengzhezou();
  Shuzhezou();
  FindLine();
}
