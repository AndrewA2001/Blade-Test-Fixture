/*
  Analog input, serial output

  Reads an analog input pin, converts it into voltage and converts the voltage into a psi value
  Also prints the results to the Serial Monitor which can be captured using RealTerm/TeraTerm.

  The circuit:
  - Signal pin connected to analog pin 0.
    Rest of the 2 wires of M2000 pressure sensor go to +5V and ground
  - Spits out data every 200 milliseconds

  created 2 Feb. 2021
  modified 3 March 4 2021
  by Andrew Abrego

*/



#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <SD.h>

//file to write to

File myFile;

//Relays Pin
const int dirPin =  A1;
const int stepPin = 4;

//LED Pins
const int RedPin = 10;
const int GreenPin = 9;
const int BluePin = 8;

//Relay Pin
const int FanPin = 7;
const int startPin = 6;

//TFT pins

#define TFT_CS        5                                                   //48                              
#define TFT_DC        A4                                                   //44
#define TFT_RST       A5
#define TFT_SDCS      A3

//FSR Pins


int fsrPin = 0;     // the FSR and 10K pulldown are connected to a0
float fsrReading;     // the analog reading from the FSR resistor divider

float fsrForce;       // Finally, the resistance converted to force



void check_sum();
void take_measurement();


//adafruit TFT screen

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//tft function
//void setCursor(uint16_t x0, uint16_t y0);
//void setTextColor(uint16_t color);
//void setTextColor(uint16_t color, uint16_t backgroundcolor);



//Used for the debounce of the pause button.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;

//How much the stepper motor moves by
int steps = 2170 ;
int speed_delay = 50; //delay is in microseconds

//How many steps the Carriage moves when inputted from the Serial Monitor
int Remote_Movement_Steps = 620;

int PausePinState;
int LastPausePinState = LOW;

int count = 0;
int KillSwitchState = 0;
int StartPinState = 0;
int StartFlag = 0;

//sum for the Newtons

float sum = 0;


//commands for buttons in PCB
String command;
String KillSwitch = "false";
bool PauseButtonBool = false;
bool StartButton = false;


//data line for writing the data to the file
String data_line;

//Function Prototypes

void Kill();
void Pause2();
//void printNum(unsigned long num);
void Move_Motor();
void Start();
void RGB_color(int red_light_value, int green_light_value, int blue_light_value);
void my_interrupt_handler();
void update_count(int number);
//void Error_flag();



//These Functions Take Care of Measuring
void getNewtons();

String FileName = "BTF_R4.csv";




void setup() {

  //kill switch
  attachInterrupt(digitalPinToInterrupt(2), Kill, RISING);
  //pause button
  attachInterrupt(digitalPinToInterrupt(3), my_interrupt_handler, RISING);
  // Sets the two pins as Output
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(startPin, INPUT);
  pinMode(FanPin, OUTPUT);

  // lc.shutdown (0, false);                                 // wake up display
  //  lc.setIntensity (0, 4);                                 // set brightness level (0 to 15)
  //  lc.clearDisplay (0);                                    // clear display register
  //  printNum(count);                                        //sets count to zero



  //initializes the TFT Screen


  tft.init(135, 240);
  tft.setRotation(1);
  //Fills The screen with black
  tft.fillScreen(ST77XX_BLACK);


  //creates the current count variable on the
  tft.setCursor(30, 10);
  tft.setTextSize(3);
  tft.setTextColor(0x7FF);
  tft.println("BLADE TEST");
  tft.setCursor(45, 40);
  tft.println("FIXTURE");

  //prints the current status of the rig
  tft.setTextSize(2);
  tft.setCursor(0, 75);
  tft.setTextColor(0xFFFF);
  tft.println("PRESS START BUTTON");
  tft.setCursor(30, 90);
  tft.println("TO BEGIN RUNNING ");

  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  if (!SD.begin(TFT_SDCS)) {
    //Error_flag();
    while (1)
    {
      RGB_color(0, 0 , 255); //Green
      delay(500);
      RGB_color(0, 0 , 0);
      delay(500);
    }
  }
  Serial.println("initialization done.");

  //pinmodes for multicolor LED
  pinMode(RedPin, OUTPUT);
  pinMode(GreenPin, OUTPUT);
  pinMode(BluePin, OUTPUT);
  RGB_color(255, 0 , 0); //Green
  pinMode(A1, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);



  //CopyRight TXT lol for andy and cole

  //writes the initial column headers for the csv file
  myFile = SD.open(FileName, FILE_WRITE);

  // if the file opened okay, write to it:


  //********************************************************************WRITE DATA FOR FILE HERE ***********************************************************************

  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("Date: , 5/12/2021");
    myFile.println("Time: , 1:13");
    myFile.println("Cycles : , 10000");
    myFile.println("Displacment: , 10mm");
    myFile.println("Speed: , 250 uS");
    myFile.println("");

    myFile.println("Count," "Average Newtons Per Step");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening File");

  }
}


void loop() {


  if (KillSwitch == "false" && PauseButtonBool == false && StartButton == true) {

    //Green

    RGB_color(0, 255, 0);
    tft.setCursor(0, 60);
    tft.fillRect(0, 90 , 100 , 24, 1); //used to clear the screen
    tft.setCursor(0, 90);
    tft.setTextColor(0x07E0);
    tft.print("Running");

    digitalWrite(dirPin, HIGH); //Changes the rotations direction
    // Makes 400 pulses for making two full cycle rotation
    digitalWrite(FanPin, HIGH);
    //310 steps == 1mm
    for (int x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speed_delay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speed_delay);
      //takes measurement every 10 steps (Cant be any faster or Program will not compile)
      //take_measurement();

      if ((x % 200) == 0)
      {
        Serial.println("working");
        getNewtons();
      }

    }


    delay(500);
    digitalWrite(dirPin, LOW); // Enables the motor to move in a particular direction
    // Makes 200 pulses for making one full cycle rotation
    for (int x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speed_delay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speed_delay);

      //takes measurement every 10 steps (Cant be any faster or Program will not compile)
     
    }

    check_sum();

    delay(500); // One second delay



    digitalWrite(dirPin, LOW); //Changes the rotations direction
    // Makes 400 pulses for making two full cycle rotation

    //310 steps == 1mm
    for (int x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speed_delay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speed_delay);

    }
    delay(500);
    //Kill();
    // Pause();

    digitalWrite(dirPin, HIGH); // Enables the motor to move in a particular direction
    // Makes 200 pulses for making one full cycle rotation
    for (int x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speed_delay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speed_delay);

      //Kill();
      // Pause();
    }

    delay(500); // One second delay
    //Kill();
    //Pause();
    count++;
    update_count(count);
  }
  else
  {

    if (PauseButtonBool == true && KillSwitch == "false" )
    {
      Move_Motor();
      //Yellow
      RGB_color(0, 50, 255);
      tft.setCursor(0, 60);
      tft.fillRect(0, 90 , 100 , 24, 1); //used to clear the screen
      tft.setCursor(0, 90);
      tft.setTextColor(0xFFE0);
      tft.print("Paused");
      delay(1000);
    }

    if (KillSwitch != "false")
    {
      Move_Motor();
      digitalWrite(FanPin, LOW);
      RGB_color(0, 0, 255); //Red
      tft.setCursor(0, 60);
      tft.fillRect(0, 90 , 100 , 24, 1); //used to clear the screen
      tft.setCursor(0, 90);
      tft.setTextColor(0xF800);
      tft.print("KILLED");
      delay(1000);

    }
    Start();
  }

}
//function kills the motor
void Kill()
{
  KillSwitch = "Not true";
  Serial.println("Kill Switch has been Pressed");
}

//Function Pauses the Motor
void Pause2()
{
  int reading = digitalRead(3);
  if (reading == HIGH || count == 10000)
  {
    PauseButtonBool = !PauseButtonBool;
    Serial.print("PauseButtoBoolean state: ");
    Serial.println(PauseButtonBool);
  }

}
//Function Starts the Fixture
void Start()
{

  StartPinState = digitalRead(startPin);
  //Serial.println(StartPinState);
  if (StartPinState == HIGH && StartFlag == 0)
  {
    StartButton = true;
    Serial.print("Operation has been Started");

    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(0, 0);
    tft.setTextSize(2);
    tft.println("Current Count: ");

    //prints the current status of the rig
    tft.setCursor(0, 60);
    tft.println("Current Status: ");
    StartFlag++;
    count++;
    //updates count on tft
    update_count(count);
  }
}


//7SD numbers

//function to print number to the screen
//void printNum(unsigned long num)                          // print the generated number to the 8-number display
//{
//  unsigned long dig = 1;
//  unsigned long digCnt = 0;
//
//  if ( num > 0 )                                           // if number is greater than zero
//  {
//    while ( num >= dig )                                    // count # of digits
//    {
//      dig *= 10;
//      digCnt++;
//    }
//    for (int d = 0, dig = 10; d < digCnt; d++)           // while more digits
//    {
//      lc.setDigit(0, d, byte(num % 10), false);         // write this digit
//      num /= 10;                                        // writing from right to left
//    }
//  }
//  else
//  {
//    lc.setDigit(0, 0, (byte)0, false);                  // just write a zero
//  }
//}

//Function to move the motor once the device is either paused or killed
void Move_Motor() {
  if (Serial.available() > 0)
  {
    command = Serial.readStringUntil('\n');

    if (command  == "f")
    {
      digitalWrite(dirPin, HIGH); //Changes the rotations direction
      // Makes 400 pulses for making two full cycle rotation
      //310 steps == 1mm
      for (int x = 0; x < Remote_Movement_Steps; x++) {
        digitalWrite(stepPin, LOW);
        delayMicroseconds(250);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(250);
      }
      delay(500);
    }
    else if (command == "b")
    {
      digitalWrite(dirPin, LOW); //Changes the rotations direction
      // Makes 400 pulses for making two full cycle rotation
      //310 steps == 1mm
      for (int x = 0; x < Remote_Movement_Steps; x++) {
        digitalWrite(stepPin, LOW);
        delayMicroseconds(250);
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(250);
      }
    }
  }
}


void RGB_color(int red_light_value, int green_light_value, int blue_light_value)
{
  analogWrite(RedPin, red_light_value);
  analogWrite(GreenPin, green_light_value);
  analogWrite(BluePin, blue_light_value);
}


void my_interrupt_handler()
{
  int count_test;
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore

  if (interrupt_time - last_interrupt_time > 200)
  {
    int reading2 = digitalRead(3);


    //unsure about code below look at later
    //count_test = count % 10000

    if (reading2 == HIGH)
    {
      PauseButtonBool = !PauseButtonBool;
      Serial.print("PauseButtoBoolean state: ");
      Serial.println(PauseButtonBool);
    }
  }
  last_interrupt_time = interrupt_time;
}

//function to constantly update the count of the device on the TFT

void update_count(int number)
{
  tft.fillRect(0, 30 , 80 , 24, 1); //used to clear the screen
  tft.setTextSize(2);
  tft.setTextColor(0xF800);
  tft.setCursor(0, 30);
  tft.print(number);
}

//takes measuremnt in newtons of the force sensor;

//function triggers kill switch
void check_sum()
{
  //divides the sum of newtons force by the amount of steps (going forward -> then backward <- to origin

  sum = sum/10 ;
  Serial.print(" ");
  Serial.print("Sum: ");
  Serial.println(sum);




  if (sum < .25)
  {
    KillSwitch = "Not true";
  }

  write_to_file(sum, count);

  sum = 0;
}

void write_to_file(float newtons, int count_number)
{
  //strings to convert taken in data to string format in order to write them into data file

  String newton_string, count_string;
  myFile = SD.open(FileName, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to File...");

    newton_string = String(newtons);
    count_string = String(count_number);

    myFile.println(newton_string + "," + count_string);

    // close the file:
    myFile.close();

    //Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println('E');
  }

}

void getNewtons() {
  fsrReading = analogRead(fsrPin);

  float fsrVoltage;     // the analog reading converted to voltage
  float fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
  float fsrConductance;
  // Serial.print("Analog reading = ");
  // Serial.println(fsrReading);

  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
  // Serial.print("Voltage reading in mV = ");
  // Serial.println(fsrVoltage);

  if (fsrVoltage == 0) {
    // Serial.println("No pressure");
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    fsrConductance = 1000000;           // we measure in micromhos so
    fsrConductance /= fsrResistance;


    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      //Serial.print("Force in Newtons: ");
      //Serial.println(fsrForce);
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;
      //Serial.print("Force in Newtons: ");
      //Serial.println(fsrForce);
    }
  }
  //Serial.println("--------------------");

  sum = sum + fsrForce;
  Serial.print(sum);
}

//This Function Displays an Error Message
//void Error_flag()
//{
//  tft.fillScreen(ST77XX_BLACK);
//  Serial.println("2");
//  tft.setCursor(0, 10);
//  tft.setTextSize(3);
//  tft.setTextColor(0xF800);
//  tft.println("MAJOR ERROR!");
//  tft.setTextSize(2);
//  tft.setTextColor(0xFFFF);
//  tft.setCursor(30, 60);
//  tft.println("INSERT SD CARD");
//  tft.setCursor(20, 80);
//  tft.println("AND RESET DEVICE");
//  tft.setCursor(20, 100);
//  tft.println("VIA RESET BUTTON");
//  Serial.println("initialization failed!");
//}
