#include <Protocentral_FDC1004.h>
#include <Adafruit_INA219.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <string.h>

#define KEYCAP_THRESHOLD   12000
#define N   2
#define CURRENT_LIMIT   3000.0
#define CURRENT_STEP   0.1

void calibrate_channel(void);
void TaskINA220Read(void);
void TaskFDC1004Read(void);
void TaskLCDDisplay(void);
void TaskMain(void);

int gate_pin = 6;

FDC1004 FDC;
Adafruit_INA219 ina219(0x41);
Adafruit_INA219 ina219_2(0x44);

const int rs = 3, en = 4, d4 = 5, d5 = 7, d6 = 8, d7 = 9;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

float setI;
float measI;
float measV;
float measV2;

int min_gate_level = 170;
int max_gate_level = 255;
int gate_level = min_gate_level;

//from MATLAB
float KL = 0;
float KI = 0.00531;
float KD = 0.001;


void setup() {
  // put your setup code here, to run once:
  Wire.begin();        //i2c begin
  Serial.begin(115200); // serial baud rate

  if (! ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
  }
  if (! ina219_2.begin())
  {
    Serial.println("Failed to find INA219 2 chip");
  }

  measV = ina219.getBusVoltage_V();
  measI = ina219.getCurrent_mA();
  measV2 = ina219_2.getBusVoltage_V();
  
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("ISET : ");
  lcd.setCursor(0, 1);
  lcd.print("I:      V:      ");

  delay(100);
  FDC.initFDC(0X8000); //Reset
  delay(100);
  calibrate_channel();
  delay(100);
  FDC.triggerRepeatMeasurement(FDC1004_100HZ); //Configure trigger register
  delay(100);
  
  setI = 0.0;

  pinMode(gate_pin, OUTPUT);
  analogWrite(gate_pin, gate_level);

  pinMode(LED_BUILTIN, OUTPUT);

  delay(2000);

  Serial.print("Set to ");
  Serial.println(setI);
  
}

int led_val = 0;

int start_meas = 0;
int loop_cnt = 0;
int mul_val = 0;

void loop() {

  TaskINA220Read();
  
  //TaskFDC1004Read();

  TaskLCDDisplay();

  TaskMain();

  led_val = !led_val;
  digitalWrite(LED_BUILTIN, led_val);
  
  delay(100);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskINA220Read(void)
{
    measV = ina219.getBusVoltage_V();
    measI = ina219.getCurrent_mA();

    measV2 = ina219_2.getBusVoltage_V();
/*
    Serial.print("Current = ");
    Serial.print(measI);
    Serial.print(", Voltage = ");
    Serial.println(measV);
*/
}

int cnt_main = 0;
float I_accum = 0.0;
float I_p;
float I_d;

float diffI_prev = 0;

void TaskMain(void)
{
  //cnt_main = cnt_main + 1;
  /*
  if (measI < setI)
  {
    if(gate_level<255)
    {
      gate_level = gate_level + 1;
    }
  }
  else
  {
    if(gate_level>0)
    {
      gate_level = gate_level - 1;
    }
  }
  */
/*
  if(cnt_main>20)
  {
    if(gate_level>0)
    {
      gate_level = gate_level - 1;
    }
  }
*/

  float diffI;
  float setV;
  float err_d;

  diffI = setI - measI;
  err_d = diffI_prev - diffI;
  
  I_p = KL * diffI;
  I_accum = I_accum + KI*diffI;
  I_d = KD * err_d;

  if(I_accum<min_gate_level)
  {
    I_accum = min_gate_level;
  }
  if(I_accum>max_gate_level)
  {
    I_accum = max_gate_level;
  }

  setV = I_p + I_accum + I_d;
  if(setV<min_gate_level)
  {
    setV = min_gate_level;
  }
  else if(setV>max_gate_level)
  {
    setV = max_gate_level;
  }
  gate_level = (int)setV;

  diffI_prev = diffI;
  
  Serial.print("MeasI = ");
  Serial.print(measI);
  Serial.print(", MeasV = ");
  Serial.print(measV);
  
  Serial.print(", I_p = ");
  Serial.print(I_p);
  Serial.print(", I_accum = ");
  Serial.print(I_accum);
  
  Serial.print(", VDS = ");
  Serial.print(measV - measV2);
  Serial.print(", Set Voltage = ");
  Serial.print(setV);
  Serial.print(", Gate Level = ");
  Serial.println(gate_level);
  analogWrite(gate_pin, gate_level);
}


int cnt[N];

void TaskFDC1004Read(void)
{
  
  int i;
  int16_t msb;
  int16_t lsb;
  int32_t capacitance[N];
  int32_t cap_pre[N];
  uint16_t value[2];
  
  
    for(i=0;i<N;i++)
    {
      if (! FDC.readMeasurement(i, value))
      {
        //delay(10);
        msb = ((int16_t) value[0]);
        
        capacitance[i] = ((int32_t)488) * ((int32_t)msb); //in attofarads
        capacitance[i] /= 1000;   //in femtofarads

        if (capacitance[i]>KEYCAP_THRESHOLD)
        {
          cnt[i] = cnt[i] + 1;
        }
        else
        {
          cnt[i] = 0;
        }

        if (i==0)
        {
          if ( (cnt[i]==1) || (cnt[i]>50) )
          {
            //Serial.println("Increasing set current");
            if(setI<(CURRENT_LIMIT-0.005))
            {
              setI = setI + CURRENT_STEP;
            }
          }
        }
        

        if (i==1)
        {
          if ( (cnt[i]==1) || (cnt[i]>50) )
          {
            //Serial.println("Decreasing set current");
            if(setI>0.0+0.005)
            {
              setI = setI - CURRENT_STEP;
            }
          }
          
        }
        
      }
    }
    
    
}

void TaskLCDDisplay(void)
{
      lcd.setCursor(2, 1);
      lcd.print(measI);
      lcd.setCursor(8, 1);
      lcd.print("V:");
      //lcd.print("I:      V:0.00  ");
      lcd.setCursor(10, 1);
      lcd.print(measV);
      //lcd.setCursor(14, 1);
      //lcd.print("  ");
      lcd.setCursor(6, 0);
      lcd.print(setI);
    
}


/*--------------------------------------------------*/
/*------------------- Functions --------------------*/
/*--------------------------------------------------*/
void calibrate_channel(void)
{
  int i;
  int16_t msb;
  int32_t capacitance;
  int capdac;
  int capdac_pre;
  int same_cnt;
  int start_meas = 1;

  Serial.println("Start Calibration");
  
  capdac = 0;
  same_cnt = 0;
  i=0;

  


  while(start_meas)
  {
    capdac_pre = capdac;
    FDC.configureMeasurementSingle(i, i, capdac);
    FDC.triggerSingleMeasurement(i, FDC1004_100HZ);
    
    //wait for completion
    delay(15);
    uint16_t value[2];
    if ((! FDC.readMeasurement(i, value)))
    {
      msb = (int16_t) value[0];
      capacitance = ((int32_t)488) * ((int32_t)msb); //in attofarads
      capacitance /= 1000;   //in femtofarads
      //capacitance += ((int32_t)3125) * ((int32_t)capdac);

      //Serial.print((((float)capacitance/1000)),4);Serial.print(" pf");Serial.print(", CAPDAC=");Serial.println(capdac);
      
      if (capacitance > 0)               // adjust capdac accordingly
      {
        if (capdac < FDC1004_CAPDAC_MAX)
          capdac++;
      }
      else if (capacitance<0)
      {
        Serial.print("Channel ");Serial.print(i);Serial.print(" CAPDAC = ");Serial.println(capdac);
        i++;
        capdac = 0;
      }
      
      if(i==4)
      {
        start_meas = 0;
        //Serial.println("FDC1004 Setup is Complete");
      }
    }
  }
}

char i1[2] = "i1";
char i2[2] = "i2";
int byte_pos = 0;
int incomingByte = 0; // for incoming serial data

char in_byte[2];

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    incomingByte = Serial.read();
    //Serial.println(incomingByte);
    if(byte_pos<2)
    {
      in_byte[byte_pos] = incomingByte;
      byte_pos = byte_pos + 1;
    }
    if (incomingByte == 13) //if press enter
    {
      //if(!strcmp(in_byte, "i1"))
      //if((in_byte == "i1"))
      if( (in_byte[0]=='i') && (in_byte[1]=='0') )
      {
        setI = 0;
        Serial.println("Set to 0");
      }
      else if( (in_byte[0]=='i') && (in_byte[1]=='1') )
      {
        setI = 300;
        Serial.println("Set to 300");
      }
      else if( (in_byte[0]=='i') && (in_byte[1]=='2') )
      {
        setI = 600;
        Serial.println("Set to 600");
      }
      else if( (in_byte[0]=='i') && (in_byte[1]=='3') )
      {
        setI = 900;
        Serial.println("Set to 900");
      }
      else
      {
        Serial.print(in_byte);
        Serial.print(", ");
        Serial.print(i1);
        Serial.print(", ");
        Serial.print(i2);
        Serial.print(", ");
        Serial.println("No match !!");
      }
      //Serial.println(in_byte);
      byte_pos = 0;
    }
    // say what you got:
    //Serial.print("PWM value is: ");
    //Serial.println(pwm_out, DEC);
  }
}
