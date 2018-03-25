#include <RBD_Timer.h>          //библиотека для таймера
#include <Wire.h>               
#include <LiquidCrystal_I2C.h>  //библиотека для дисплея
#include <EEPROM.h>             //библиотека для хранения в EEPROM
#include <SoftwareSerial.h>     //библиотека для uart
#include <TinyGPS.h>            //библиотека для gps
LiquidCrystal_I2C lcd(0x27,16,2); // Устанавливаем дисплей

int temp1, temp2;                                     // для чтения аналогового входа с датчика температуры
int grad1, grad2;                                     // для преобразованияиз фаренгейтов в градусы цельсия
long tempBuff1, tempBuff2;                            // для накапливания значений с датчика температуры, чтобы потом посчитать среднее
int resultTemp1, resultTemp2;                         // для подсчета итогового среднего значения за интервал времени
int temp1MeasureCount, temp2MeasureCount;             // количество циклов, для подсчета среднего значения заи интервал времени

int rainSenseReading;             // переменная для чтения значения аналогового порта датчика дождя
int rainGoCounter;                 // переменная для подсчета циклов, в течении которых дождь шел 
int rainStopCounter;
long rainBuff;                     // для накапливания значений с датчика дождя
int rainResult;                  // для подсчета итогового среднего значения за интервал времени
int rainMeasureCount;             // количество циклов, для подсчета среднего значения заи интервал времени
bool isRain;                      // если дождь есть, то true

boolean buttonIncrementState;       // состояние кнопки 
boolean buttonIncrementPrevState;   // предыдущее состояние кнопки 
boolean buttonDecrementState;       // состояние кнопки 
boolean buttonDecrementPrevState;   // предыдущее состояние кнопки 
boolean buttonManualOilState;       // состояние кнопки 
boolean buttonManualOilPrevState;   // предыдущее состояние кнопки 
                     


#define PIN_RELAY 7
#define PIN_TEMP_SENSOR_1 A0
#define PIN_TEMP_SENSOR_2 A1
#define PIN_RAIN_SENSOR A3

#define BUTTON_MANUAL_OIL 9
#define SOUND_PIN 10
#define BUTTON_PIN_DECREMENT_OIL_TIME 11
#define BUTTON_PIN_INCREMENT_OIL_TIME 12

RBD::Timer timerTempCounter;        // таймера для расчета значений через определенный промежуток времени
RBD::Timer timerRainCounter;        // т.е. мы накапливали значение в буфер, потом посчитали среднее и все по новой
RBD::Timer timerOdometerCounter;

TinyGPS gps;
SoftwareSerial ss(4, 3);
float flat, flon;
unsigned long age;

int iSpeed;
int distanceMoved;  //пройденная дистанция за intervalOdo
int averageSpeed;   //средняя скорость за intervalOdo
int speedBuff;
int speedMeasureCount;

int distanceToOil;  //расстояние до смазки в метрах
byte hi;                          //байты для хранения в EEPROM
byte low;


uint8_t temp_cel[8] =
{
B00111,
B00101,
B00111,
B00000,
B00000,
B00000,
B00000
}; //закодировано в двоичной системе. Значок градуса



//*************************************************************************************************************************
void setup()
{

  Serial.begin(19200); // подключаем монитор порта

//GPS
  ss.begin(9600);

//Buzzer пикает при включении
  pinMode(SOUND_PIN, OUTPUT);
  digitalWrite(SOUND_PIN, HIGH);  // Звук вкл
  delay(100);
  digitalWrite(SOUND_PIN, LOW);  //Звук выкл
  

//OilDistance
  hi = EEPROM.read(1);
  low = EEPROM.read(2);
  int res = word(hi, low);
  distanceToOil = res;
  
//Button
   pinMode(BUTTON_PIN_INCREMENT_OIL_TIME, INPUT_PULLUP);
   pinMode(BUTTON_PIN_DECREMENT_OIL_TIME, INPUT_PULLUP);
   pinMode(BUTTON_MANUAL_OIL, INPUT_PULLUP);
  
//Relay
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);  //Реле закрыто

//Rain Sensor
  pinMode(PIN_RAIN_SENSOR, INPUT);    // Подключаем аналоговый вход с датчика дождя к A3
  rainSenseReading = 0;
  rainGoCounter = 0;
  rainBuff = 0;
  rainMeasureCount = 0;
  isRain = false;

//Analog temp
  pinMode(PIN_TEMP_SENSOR_1, INPUT); // сенсор LM35 подключим к аналоговому входу A0
  pinMode(PIN_TEMP_SENSOR_2, INPUT); // Второй сенсор к A1
  tempBuff1 = 0;
  tempBuff2 = 0;
  temp1MeasureCount = 0;
  temp2MeasureCount = 0;
  
  
//Display
  lcd.init();                     
  lcd.backlight();    // Включаем подсветку дисплея
  lcd.clear();
  lcd.print("Oil");
  lcd.setCursor(0, 1);
  lcd.print("Air");
  
  lcd.setCursor(5, 0);  //При запуске значения температуры не отображаются intervalTemp милисекунд
  lcd.print("##");
  lcd.setCursor(5, 1);
  lcd.print("##");
  
  lcd.createChar(0, temp_cel); //создаем сивол градуса


  


  timerTempCounter.setTimeout(3000);        // время усреднения температуры
  timerTempCounter.restart();

  timerRainCounter.setTimeout(3000);        // время усреднения дождя
  timerRainCounter.restart();

  timerOdometerCounter.setTimeout(1500);    // время усреднения пройденного расстояния
  timerOdometerCounter.restart();

  
}
//**********************************************************************************************************
void loop()
{

//GPS speed
  getSpeedFromGPS();

//GPS speed counter
  speedCount();
 
//Count moved distance
  if (timerOdometerCounter.onRestart()) {
    countMovedDistance();
  }



//Buttons state read 
  readButtonsState();


  
//Auto oiling term
  chainOiling(50); //условие на автоматическую подачу масла, в скобках время срабатывания насоса в милисекундах

//Manual Oiling
  if (buttonManualOilState == LOW){
      digitalWrite(PIN_RELAY, LOW);
  } else 
      digitalWrite(PIN_RELAY, HIGH);
  

  
//Increment
  if (distanceToOil < 9950){
    if (buttonClickCheck(buttonIncrementPrevState, buttonIncrementState))
      incrementDistanceToOil();
  }
  buttonIncrementPrevState= buttonIncrementState;         // предыдущее состояние кнопки = текущему


//Decrement
  if ( distanceToOil > 0){
    if (buttonClickCheck(buttonDecrementPrevState, buttonDecrementState))
      decrementDistanceToOil();
  }
  buttonDecrementPrevState = buttonDecrementState;         // предыдущее состояние кнопки = текущему

//Oiler distance print
  lcd.setCursor(12, 0);
  lcd.print(distanceToOil);




//********READ FROM SENSORS*****

  //Read analog signal from rain sensor
  rainSensorRead();
  if (timerRainCounter.onRestart()) {
    rainCountAndPrint();
  }
  
  //Analog temp read
  tempSensorRead();
  if(timerTempCounter.onRestart()) {   
    tempCountAndPrint();
  }

  //This is because NOFIX must print above other prints
  if (iSpeed < 0) {
    lcd.setCursor(11, 1);
    lcd.print("NOFIX");
  }

 
  
}







//*************************FUNCTION******************************************************

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void tempCountAndPrint() {
 
  resultTemp1 = round(tempBuff1 / temp1MeasureCount);  //получаем среднюю температура за interval
    resultTemp2 = round(tempBuff2 / temp2MeasureCount);
    
    
    //Display
    if (resultTemp1 < 10) {  //Проверяем на количество цифр в значении температуры, чтобы определить, куда поставим значение на дисплее
      lcd.setCursor(4,0);
      lcd.print("  ");
      lcd.setCursor(6, 0); 
    } else 
    if (resultTemp1 < 100){
      lcd.setCursor(4,0);
      lcd.print(" ");         //убираем единицу, если температура перевалила за сотню, а потом вернулась
      lcd.setCursor(5, 0); 
    } else
    if (resultTemp1 > 100){
      lcd.setCursor(4, 0); 
    }
    lcd.print(resultTemp1);

     
    if (resultTemp2 < 10) {  //Проверяем на количество цифр в значении температуры, чтобы определить, куда поставим значение на дисплее
      lcd.setCursor(4,1);
      lcd.print("  ");
      lcd.setCursor(6, 1); 
    }
    if (resultTemp2 < 100){
      lcd.setCursor(4,1);
      lcd.print(" ");         //убираем единицу, если температура перевалила за сотню, а потом вернулась
      lcd.setCursor(5, 1); 
    } else
    if (resultTemp2 > 100){
      lcd.setCursor(4, 1); 
    }
    lcd.print(resultTemp2);

    //отобразить градусы цельсия
    lcd.setCursor(7, 0);
    lcd.print(char(0));
    lcd.print("C");
    lcd.setCursor(7, 1);
    lcd.print(char(0)); 
    lcd.print("C");

    //сбросить перед следующим циклом
    temp1MeasureCount = 0;
    temp2MeasureCount = 0;
    tempBuff1 = 0;
    tempBuff2 = 0;   
}

static void tempSensorRead() {
  delay(5);
  temp1 = analogRead(PIN_TEMP_SENSOR_1); // переменная находится в интервале 0 - 1023 
  
  delay(5);
  temp2 = analogRead(PIN_TEMP_SENSOR_2);
  delay(5);

  if (temp1 < 1017){                    //пытаемся отсечь выбросы, которые у меня наблюдались при мониторинге порта
  grad1 = ( temp1/1023.0 )*4.9*1000/10; // преобразуем в градуся цельсия
  tempBuff1 +=  grad1;                  // накапливаем температуру, чтобы потом получить среднее
  temp1MeasureCount +=  1;     //считаем число циклов для того, чтобы вычислить среднюю температуру
  }
  
  if(temp2 < 1017){
  grad2 = ( temp2/1023.0 )*4.9*1000/10;
  tempBuff2 +=  grad2;
  temp2MeasureCount +=  1;     //считаем число циклов для того, чтобы вычислить среднюю температуру
  }
}

static void rainSensorRead() {
  delay(5);
  rainSenseReading = analogRead(PIN_RAIN_SENSOR);   //Rain Sensor
  delay(5);
  rainBuff += rainSenseReading;
  rainMeasureCount += 1;
}

static void rainCountAndPrint() {
  //Когда дождь идет, то значение с датчика будет маленьким. 
  //Когда дождя нет, то датчик будет высыхать и значения, приходящие с него будут увеличиваться  
  rainResult = rainBuff / rainMeasureCount;
 
  if (rainResult < 200 && !isRain) {  //тут устанавливается порог срабатывания
      rainGoCounter++;                //если дождь идет, но режим дождя еще не запущен
      rainStopCounter = 0;
  } 
  if (rainResult < 200 && isRain) {   //условие написано, чтобы лишний раз не инкрементировать переменнную
      rainStopCounter = 0;            //если дождь идет и режим дождя запущен
  }   
  if (rainResult > 300 && isRain) {   // порог отключения режима дождя
      rainStopCounter++;
      rainGoCounter = 0;              // если дождь закончился, но режим дождя еще стоит
  }
  if (rainResult > 300 && !isRain) {  
      rainGoCounter = 0;              // если дождь закончился и режим дождя выключен
  } 

  if (rainGoCounter > 2){    // если после X циклов дождь не кончился, то включается режим RAIN
    isRain = true;
    lcd.setCursor(11, 1); 
    lcd.print(" RAIN");
  }  
  if (rainStopCounter > 3) {
    isRain = false; 
    lcd.setCursor(11, 1); 
    lcd.print("     "); 
  }

  rainMeasureCount = 0;
  rainBuff = 0; 
}

static void decrementDistanceToOil() {
  if ( (buttonDecrementPrevState == HIGH) && (buttonDecrementState == LOW) ) {
      distanceToOil -= 50;  
       hi = highByte(distanceToOil);
       low = lowByte(distanceToOil);
       EEPROM.write(1, hi);
       EEPROM.write(2, low);
       digitalWrite(SOUND_PIN, HIGH); //лень делать цикл, поэтому делэй
       delay(85);                     //особой роли не сыграет, так как на кнопки не часто планируется нажимать
       digitalWrite(SOUND_PIN, LOW);
    } 
}

static void incrementDistanceToOil() {
   if ( (buttonIncrementPrevState == HIGH) && (buttonIncrementState == LOW) ) {
      distanceToOil += 50;  
      hi = highByte(distanceToOil);
      low = lowByte(distanceToOil);
      EEPROM.write(1, hi);
      EEPROM.write(2, low);  
      digitalWrite(SOUND_PIN, HIGH);    //лень делать цикл, поэтому делэй
      delay(85);                        //особой роли не сыграет, так как на кнопки не часто планируется нажимать
      digitalWrite(SOUND_PIN, LOW);     //да и даже при большой скорости нажатий все равно темпераура отлично считывается
    } 
}

static bool buttonClickCheck(bool previousState, bool currentState) {
  if ( (previousState == HIGH) && (currentState == LOW) ) return true;
  else return false; 
}

static void readButtonsState() {
  buttonIncrementState= digitalRead(BUTTON_PIN_INCREMENT_OIL_TIME);
  buttonDecrementState= digitalRead(BUTTON_PIN_DECREMENT_OIL_TIME);
  buttonManualOilState= digitalRead(BUTTON_MANUAL_OIL);
}

static int countMovedDistance() {
  averageSpeed = speedBuff / speedMeasureCount;           //average speed for timerOdometerCounter in m/s
    if (averageSpeed >= 3) {
      distanceMoved += averageSpeed * (timerOdometerCounter.getTimeout() / 1000);
    }
    
    Serial.println(distanceMoved);
   

    speedBuff = 0;
    speedMeasureCount = 0;
}


static void chainOiling(int oilingTime) {
   if (distanceMoved > distanceToOil) {
      digitalWrite(PIN_RELAY, LOW);
      delay(oilingTime);     //и тут захерачим пока на delay, пока буду тестировать насос дома
      digitalWrite(PIN_RELAY, HIGH);
      distanceMoved = 0;
    }
}

static void speedCount() {
  speedBuff += iSpeed;
  speedMeasureCount += 1;
}

static void getSpeedFromGPS() {
  if (age == TinyGPS::GPS_INVALID_AGE || age > 2000)
  {
    //GPS connection lost
    iSpeed = 0;
    Serial.println("connection lost");
  }
  else
  {
    //GPS connection is up to date
    iSpeed = round(gps.f_speed_mps()); // speed in meters per second
    
  }
  smartdelay(100);    // без этого вроде не будут нормально данные читаться с GPS, надо будет попробовать без этой хрени
}

