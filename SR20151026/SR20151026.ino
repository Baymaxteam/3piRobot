/*
 * Simple3piMazeSolver - demo code for the Pololu 3pi Robot
 * 
 * This code will solve a line maze constructed with a black line on a
 * white background, as long as there are no loops.  It has two
 * phases: first, it learns the maze, with a "left hand on the wall"
 * strategy, and computes the most efficient path to the finish.
 * Second, it follows its most efficient solution.
 *
 * http://www.pololu.com/docs/0J21
 * http://www.pololu.com
 * http://forum.pololu.com
 *
 */

// 程式需導入以下程式庫才能正常運作
#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLEDs.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>
#include <OrangutanBuzzer.h>

Pololu3pi robot;
unsigned int sensors[5]; // 定義一個大小為五的矩陣，存放機器人五個感測器的數值

// program space = 程式空間
// 這個導入的標頭檔將會允許資料存放在微控制器的程式空間部分。
// ATmega168有16k的程式空間相較只有1k的RAM，靜態資料應該放在程式空間裡
#include <avr/pgmspace.h>


// 機器人的開始訊息。"PROGMEM" 字綴表示資料存放在程式空間裡
const char welcome_line1[] PROGMEM = " Pololu";
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "Maze";
const char demo_name_line2[] PROGMEM = "solver";

const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";


// 以下為自創字元的資料，被用在 load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char levels[] PROGMEM = {
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

// 這個函式將自創字元顯示在LCD.  Up to 8
// characters can be loaded; we use them for 7 levels of a bar graph.
void load_custom_characters()
{
  OrangutanLCD::loadCustomCharacter(levels + 0, 0); // no offset, e.g. one bar
  OrangutanLCD::loadCustomCharacter(levels + 1, 1); // two bars
  OrangutanLCD::loadCustomCharacter(levels + 2, 2); // etc...
  OrangutanLCD::loadCustomCharacter(levels + 3, 3);
  OrangutanLCD::loadCustomCharacter(levels + 4, 4);
  OrangutanLCD::loadCustomCharacter(levels + 5, 5);
  OrangutanLCD::loadCustomCharacter(levels + 6, 6);
  OrangutanLCD::clear(); // the LCD must be cleared for the characters to take effect
}

// 這個函式將使用長條圖顯示感測器數值
void display_readings(const unsigned int *calibrated_values)
{
  unsigned char i;

  for (i=0;i<5;i++) {
    // Initialize the array of characters that we will use for the
    // graph.  Using the space, an extra copy of the one-bar
    // character, and character 255 (a full black box), we get 10
    // characters in the array.
    const char display_characters[10] = { 
      ' ', 0, 0, 1, 2, 3, 4, 5, 6, 255     };

    // The variable c will have values from 0 to 9, since
    // calibrated values are in the range of 0 to 1000, and
    // 1000/101 is 9 with integer math.
    char c = display_characters[calibrated_values[i] / 101];

    // Display the bar graph character.
    OrangutanLCD::print(c);
  }
}

// setup()會初始化3pi機器人，顯示歡迎訊息、感測器校正、播放起始音樂。這個函式會自動於機器人啟動時執行。
void setup()
{
  unsigned int counter; // 創造一個定時器


  // 這個函式一定得呼叫，作為初始化機器人。使用2000訂為超時，對應為2000*0.4 us = 0.8 ms 在我們的20 MHz 單晶片
  robot.init(2000);

  load_custom_characters(); // load the custom characters

    // Play welcome music and display a message
  OrangutanLCD::printFromProgramSpace(welcome_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(welcome_line2);
  OrangutanBuzzer::playFromProgramSpace(welcome);
  delay(1000);

  OrangutanLCD::clear();
  OrangutanLCD::printFromProgramSpace(demo_name_line1);
  OrangutanLCD::gotoXY(0, 1);
  OrangutanLCD::printFromProgramSpace(demo_name_line2);
  delay(1000);

  // Display battery voltage and wait for button press
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    int bat = OrangutanAnalog::readBatteryMillivolts();

    OrangutanLCD::clear();
    OrangutanLCD::print(bat);
    OrangutanLCD::print("mV");
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print("Press B");

    delay(100);
  }

  // Always wait for the button to be released so that 3pi doesn't
  // start moving until your hand is away from it.
  OrangutanPushbuttons::waitForRelease(BUTTON_B);
  delay(1000);


  // 自動校正感測器
  for (counter=0; counter<80; counter++)
  {
    if (counter < 20 || counter >= 60)
      OrangutanMotors::setSpeeds(40, -40);
    else
      OrangutanMotors::setSpeeds(-40, 40);


    // 這個函式會紀錄感測到的最大值與最小值。IR_EMITTERS_ON表示IR LEDs常開
    robot.calibrateLineSensors(IR_EMITTERS_ON);


    delay(20);
  }
  OrangutanMotors::setSpeeds(0, 0);

  // Display calibrated values as a bar graph.
  while (!OrangutanPushbuttons::isPressed(BUTTON_B))
  {
    // 讀取感測器數值並回傳黑線位置資訊
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);


    // 於LCD上展示位置量測結果，由最左感測器開始至最右感測器以0~4000表示。
    OrangutanLCD::clear();
    OrangutanLCD::print(position);
    OrangutanLCD::gotoXY(0, 1);
    display_readings(sensors);

    delay(100);
  }
  OrangutanPushbuttons::waitForRelease(BUTTON_B);

  OrangutanLCD::clear();

  OrangutanLCD::print("Go!");		

  // Play music and wait for it to finish before we start driving.
  OrangutanBuzzer::playFromProgramSpace(go);
  while(OrangutanBuzzer::isPlaying());
}



// 這個函式會使機器人循者直線移動。直到感測到交叉口或是終點。
void follow_segment()
{
  int last_proportional = 0;
  long integral=0;

  while(1)
  {
    // 將機器人最高速度訂為60來保持移動的可靠度

    // 取得位置資訊
    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

    // 如果黑線在機器人正中間"proportional"變數應該要為0 
    int proportional = ((int)position) - 2000;

    
    // 計算PID控制的微分項(改變量)以及積分項(累積量)
    int derivative = proportional - last_proportional;
    integral += proportional;

    // 紀錄上個時間點的位置
    last_proportional = proportional;

  
    // 計算兩個馬達輸出的差值。如果此項為正值機器人將會左轉彎，反之亦然。
    int power_difference = 0.2*proportional;

    // 限制馬達輸出不超過設定的最大值(60)
    // 將結果換算成真正馬達的命令，下命令給馬達
    const int maximum = 60; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    if (power_difference < 0)
      OrangutanMotors::setSpeeds(maximum + power_difference, maximum);
    else
      OrangutanMotors::setSpeeds(maximum, maximum - power_difference);


    // 我們使用中間三顆感測器來確認正在直走的黑線上，利用最旁邊兩側感測器感測是否到達交叉口

    if (sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }
    else if (sensors[0] > 200 || sensors[4] > 200)
    {
      // Found an intersection.
      return;
    }

  }
}

// 轉角的PID控制，目標轉到 1900-2000-2100
void follow_segment_turn(unsigned char dir){
  int last_proportional = 0;
  long integral=0;
  //先強制轉一些
  switch(dir){
  case 'L':
    // Turn left.
    OrangutanMotors::setSpeeds(-65, 65);
    delay(150);
    break;
  case 'l':
    // Turn left.
    OrangutanMotors::setSpeeds(-65, 65);
    delay(150);
    break;
  case 'R':
    // Turn right.
    OrangutanMotors::setSpeeds(65, -65);
    delay(150);
    break;
  case 'r':
    // Turn right.
    OrangutanMotors::setSpeeds(65, -65);
    delay(150);
    break;   
  case 'B':
    // Turn around.
    OrangutanMotors::setSpeeds(65, -65);
    delay(350);
    break;
  case 'S':
    // Don't do anything!
    break;
  case 's':
    // Don't do anything!
    break;
  }
  
  while(1)
  {

    unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);

   // 如果黑線在機器人正中間"proportional"變數應該要為0 
    int proportional = ((int)position) - 2000;

    // 計算PID控制的微分項(改變量)以及積分項(累積量)
    int derivative = proportional - last_proportional;
    integral += proportional;

    // 紀錄上一個時間點的機器人位置
    last_proportional = proportional;

    // 計算兩個馬達輸出的差值。如果此項為正值機器人將會左轉彎，反之亦然。
    int power_difference = 0.2*proportional;

    // 限制馬達輸出不超過設定的最大值(60)
    const int maximum = 60; // the maximum speed
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;

    OrangutanMotors::setSpeeds(power_difference, -power_difference);


    // 讀取感測器數值，計算黑線位置與中間的誤差，若誤差小於200即視為轉彎成功
    position = robot.readLine(sensors, IR_EMITTERS_ON);
    int error = abs(((int)position) - 2000);
    if (error<200)
    { 
      // 轉彎成功 跳回
      return;
    }

  }
}




// path變數會存放機器人走過的路徑。以字元的方式儲存，有一下七種。
// 'L' 表示唯一左轉
// 'R' 表示唯一右轉
// 'S' 表示兩路口直走
// 'B' 表示迴轉
// 'l' 表示兩路口左轉
// 'r' 表示兩路口右轉
// 's' 表示三路口直走
// 當機器人做一個迴轉的動作，表示進入死巷子，可以簡化路徑。follow_next_turn()函式會檢查這個狀況並做路徑簡化的動作
char path[200] = "";
unsigned char path_length = 0; // the length of the path

// Displays the current path on the LCD, using two rows if necessary.
void display_path()
{
  // Set the last character of the path to a 0 so that the print()
  // function can find the end of the string.  This is how strings1
  // are normally terminated in C.
  path[path_length] = 0;

  OrangutanLCD::clear();
  OrangutanLCD::print(path);

  if (path_length > 8)
  {
    OrangutanLCD::gotoXY(0, 1);
    OrangutanLCD::print(path + 8);
  }
}


// 這個函數會決定機器人在交叉口要轉哪個方向。本程式使用的是中左定則，先以中間為主，左邊為次要選項，右邊最後。
unsigned char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
{
  
  // 決定機器人在交叉口要轉哪個方向，以中間為主，左邊為次要選項，右邊最後。
  if (path[path_length-2]=='s'&& path[path_length-1]=='B')
      return 'l';
  else if (path[path_length-2]=='r'&& path[path_length-1]=='B')
      return 'S';
  else if (path[path_length-2]=='R'&& path[path_length-1]=='B')
      return 'L';
  else if (path[path_length-2]=='L'&& path[path_length-1]=='B')
      return 'R';
  else if (path[path_length-1]=='B'&& found_right)
      return 'R';
  else if (path[path_length-1]=='B'&& found_left)
      return 'L';
  else if (found_straight && found_left && found_right)
      return 's';
  else if (found_right && found_left)
      return 'r';
  else if (found_straight)
      return 'S';
  else if (found_right)
      return 'R';
  else if (found_left)
      return 'L';    

  else
      return 'B';
    
}

// 路徑簡化。目標為找出所有xBx的狀況並直接剪掉這種死路的狀況，改為直走S取代。
void simplify_path()
{
  // 若路徑長度小於三或是倒數第二個路徑不是迴轉，就不用簡化、直接跳回主程式。
  if (path_length < 3 || path[path_length-2] != 'B')
    return;

  int total_angle = 0;
  int i;
  int ten_cross=0;
  for (i = 1; i <= 3; i++)
  {
    switch (path[path_length - i])
    {
    case 'R':
      total_angle += 90;
      break;
    case 'L':
      total_angle += 270;
      break;
    case 'B':
      total_angle += 180;
      break;
    case 'r':
      total_angle += 90;
      break;
    case 'l':
      total_angle += 270;
      break;
    case 's':
      ten_cross=1;
      break;
   



      
    }
  }

  // 限制角度範圍在 0 到 360 .
  total_angle = total_angle % 360;

  // 將路徑以一個轉彎取代
  switch (total_angle)
  {
  case 0:
    path[path_length - 3] = 'S';
    break;
  case 90:
    if (ten_cross)
      path[path_length - 3] = 'r';
    else
    path[path_length - 3] = 'R';
    break;
  case 180:
    path[path_length - 3] = 'B';
    break;
  case 270:
    path[path_length - 3] = 'L';
    break;
  }

  // 總長度減二
  path_length -= 2;
}



// loop為主程式
void loop()
{
  while (1)
  {
    follow_segment();

    
    // 再直走一小段路，可以幫助機器人的感測器移動到交叉點路徑上方。
    // 注意這裡會減慢，以免機器人往前滑移太多
    OrangutanMotors::setSpeeds(45, 45);
    delay(50);

    
    // 以下變數會紀錄當下迷宮交叉點可以走的路徑
    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;

    // 讀取感測器數值並決定轉角方向
    unsigned int sensors[5];
    robot.readLine(sensors, IR_EMITTERS_ON);

    // 檢查交叉點是否有往左右的路徑存在
  
    if (sensors[0] > 200)
      found_left = 1;

    if (sensors[4] > 200 )
      found_right = 1;


    // 再直走一小段路，可以幫助機器人移動到交叉點路徑上方。
    OrangutanMotors::setSpeeds(40, 40);
    delay(150); //default:200

    // 檢查是否有直走的路徑
    robot.readLine(sensors, IR_EMITTERS_ON);
    if (sensors[1] > 200 || sensors[2] > 200 || sensors[3] > 200)
      found_straight = 1;

    // 檢查是否為死路
    // 如果三個感測器皆讀到黑色表示走到迷宮終點
    if (sensors[1] > 600 && sensors[2] > 600 && sensors[3] > 600)
      break;

    // 到這邊轉角的鑑定完成
    // 如果迷宮已解完，則我們就可以按照現存的路徑，換句話說我們需要求解。
    unsigned char dir = select_turn(found_left, found_straight, found_right);

    // 決定要往哪邊轉
    follow_segment_turn(dir);

    // 儲存轉彎路徑
    path[path_length] = dir;
    path_length++;

    
    // 應該要確定迷宮轉角的數量不會超過path矩陣變數的最大數量

    // 簡化路徑
    simplify_path();

    // 展示路徑
    display_path();
  }

  // Solved the maze!

  // 進入一無限循環，讓我們可以一再重複呈現機器人解完的路徑
  while (1)
  {
    // Beep to show that we solved the maze.
    OrangutanMotors::setSpeeds(0, 0);
    OrangutanBuzzer::play(">>a32");

    // Wait for the user to press a button, while displaying
    // the solution.
    while (!OrangutanPushbuttons::isPressed(BUTTON_B))
    {
      if (millis() % 2000 < 1000)
      {
        OrangutanLCD::clear();
        OrangutanLCD::print("Solved!");
        OrangutanLCD::gotoXY(0, 1);
        OrangutanLCD::print("Press B");
      }
      else
        display_path();
      delay(30);
    }
    while (OrangutanPushbuttons::isPressed(BUTTON_B));

    delay(1000);

    // 跑迷宮，因為我們不需要再鑑別交叉點了，所以這個迴圈非常簡單  
    int i;
    for (i = 0; i < path_length; i++)
    {
      follow_segment();

      // 減速，理由和上面程式一樣
      OrangutanMotors::setSpeeds(40, 40);
    delay(150); 

      // 轉彎，按照path變數裡紀錄的方向
      follow_segment_turn(path[i]);
    }

    // 走直線控制
    follow_segment();

    // Now we should be at the finish!  Restart the loop.
  }
}


