# internet-of-things


trafic signal: https://wokwi.com/projects/333716006026347092
https://wokwi.com/projects/333716006026347092

RGB using 3 led: https://wokwi.com/projects/new/arduino-uno

trafic signal: https://wokwi.com/projects/new/arduino-uno


RGB using rgbLED:  https://wokwi.com/projects/333802413751272018

Hello word: https://wokwi.com/projects/322062421191557714

simple servo motor: https://wokwi.com/projects/334978658864202323
servo meter using potentio meter: https://wokwi.com/projects/334980073732964946
https://wokwi.com/projects/334980838317883987

servo meter with slide: https://wokwi.com/projects/335065101785629268


buzzer: https://wokwi.com/projects/334982296663753299

program using :buzzer
https://wokwi.com/projects/335065425157030483

buzzer program with pushbotton:https://wokwi.com/projects/335067685745328723

ultrasonic :
https://wokwi.com/projects/335071386779255379

ultrasonic with buzer; https://wokwi.com/projects/335072522958537298

buzzer with sliding potentio meter: https://wokwi.com/projects/335609047071851090

Buzzer,led,ultrasonic sensor: https://wokwi.com/projects/335610067809206868


multiple buzer: https://wokwi.com/projects/335615786380952147

motion sensor with LED: https://wokwi.com/projects/335700704128664148

Ultrasonic sensor controls servo motor
https://wokwi.com/projects/334980059722941011

Potentiometer with LED brightness variation: https://wokwi.com/projects/335701627023393363

Servo with push button:14.Servo with pushbutton: https://wokwi.com/projects/335704431741895252

Ultra sonic sensor,pushbutton led: https://wokwi.com/projects/337603413334295124
LAB list:

1) LED/Buzzer:
https://wokwi.com/projects/337601765761352274

3)DHT11 sensor :https://wokwi.com/projects/337604199645708884

5)LCD with Humidity reading on it: https://wokwi.com/projects/337605873710924370


   Internals:
    
        1)LED:
   
               void setup() {
              // put your setup code here, to run once:
              pinMode(13, OUTPUT);

            }

            void loop() {
              // put your main code here, to run repeatedly:
            digitalWrite(13, HIGH);
            delay(500);
            digitalWrite(13, LOW);
            delay(500);
            }


      2)3 LED: 
      
                            void setup() {
                        // put your setup code here, to run once:
                      pinMode(1, OUTPUT);
                      pinMode(11, OUTPUT);
                      pinMode(5, OUTPUT);
                      }

                      void loop() {
                        // put your main code here, to run repeatedly:
                      digitalWrite(1, HIGH);
                      delay(1000);
                      digitalWrite(1, LOW);
                      delay(1000);
                      digitalWrite(5, HIGH);
                      delay(1000);
                      digitalWrite(5, LOW);
                      delay(1000);
                      digitalWrite(11, HIGH);
                      delay(1000);
                      digitalWrite(11, LOW);
                      delay(1000);
                      }


      3) RGB LED:
      
                  void setup() {
                  pinMode(11, OUTPUT);
                  pinMode(9, OUTPUT);
                  pinMode(5, OUTPUT);

                }

                void loop() {
                  displayColor(0b100); //RED
                  delay(1000);
                  displayColor(0b010); //GREEN
                  delay(1000);
                  displayColor(0b001); //BLUE
                  delay(1000);
                  displayColor(0b101); //MAGENTA
                  delay(1000);
                  displayColor(0b011); //CYAN
                  delay(1000);
                  displayColor(0b110); //YELLOW
                  delay(1000);
                  displayColor(0b111); //WHITE
                  delay(1000);
                }

                void displayColor(byte color) {
                  digitalWrite(11, !bitRead(color, 2));
                  digitalWrite(9, !bitRead(color, 1));
                  digitalWrite(5, !bitRead(color, 0));
                }
                
     4) Lequid crystal:
     
                   /* Hello Wokwi! */

              #include <LiquidCrystal_I2C.h>

              LiquidCrystal_I2C lcd(0x27, 20, 4);

              void setup() {
                lcd.init();
                lcd.backlight();
                lcd.setCursor(1, 0);
                lcd.print("Hello, Wokwi!");
              }

              void loop() {
                lcd.setCursor(7, 1);
                lcd.print(millis() / 1000);
              }
     
     
   5) Servo button + push button:
   
   
                 #include <Servo.h>
              // constants won't change
              const int BUTTON_PIN = 7; // Arduino pin connected to button's pin
              const int SERVO_PIN  = 9; // Arduino pin connected to servo motor's pin
              Servo servo; // create servo object to control a servo
              // variables will change:
              int angle = 0;          // the current angle of servo motor
              int lastButtonState;    // the previous state of button
              int currentButtonState; // the current state of button
              void setup() {
                Serial.begin(1000);                // initialize serial
                pinMode(BUTTON_PIN, INPUT_PULLUP); // set arduino pin to input pull-up mode
                servo.attach(SERVO_PIN);           // attaches the servo on pin 9 to the servo object

                servo.write(angle);
                currentButtonState = digitalRead(BUTTON_PIN);
              }
              void loop() {
                lastButtonState    = currentButtonState;      // save the last state
                currentButtonState = digitalRead(BUTTON_PIN); // read new state

                if(lastButtonState == HIGH && currentButtonState == LOW) {
                  Serial.println("The button is pressed");
                  // change angle of servo motor
                  if(angle == 0)
                    angle = 180;
                  else
                  if(angle == 180)
                    angle = 0;
                  // control servo motor arccoding to the angle
                  servo.write(angle);
                }
              }


  6)   servo motor+ sliding potentio meter:
  
  
                          #include <Servo.h>
                    Servo servo;
                    int potpin=A0; //Analog pin used to connect Potentiometer 
                    int val;
                    void setup()
                    {
                      servo.attach(11);   //Servo attached to pin 11
                    }
                    void loop()
                    {
                      val=analogRead(potpin);    //Reads theb value of PM(from 0 to 1023)
                      val=map(val,A0,1023,0,180); //A0,1023(8 bit reference ) for PM. 0-180 for servo
                      servo.write(val);
                      delay(15);  //waits for the servo to get there
                    }

7) buzzer+ pushbutton:

                  const int push_button=7; //connect push button to pin number 8
                const int BUZZER =  5;  //connect Buzzer to pin 5
                void setup() {
                  Serial.begin(9600); //intiallize serial communication
                  pinMode(BUZZER, OUTPUT);    // Define buzzer as output.   
                  pinMode(push_button, INPUT);  // Define push button as Input.   

                }


                void loop(){

                  int state = digitalRead(push_button); // variable to store status of push button

                  if (state == 1) // check the status of push button either HIGH (1) OR LOW (0)
                   { 
                     Serial.println("the button is been pressed"); 
                      tone(5,500);    
                     digitalWrite(BUZZER, 1); 
                    // If input is High, buzzer operated
                     delay(1000); 
                   } 
                  else  
                  {
                    Serial.println("the button is not been pressed"); 
                     noTone(5);   
                     digitalWrite(BUZZER, 0);
                     delay(1000);  // For every other condition make buzzer off
                  }
                }
8) altrasonic sensor+ buzzer:


               const int TRIG_PIN   = 10; // Arduino pin connected to Ultrasonic Sensor's TRIG pin
              const int ECHO_PIN   = 9; // Arduino pin connected to Ultrasonic Sensor's ECHO pin
              const int BUZZER_PIN = 2; // Arduino pin connected to Piezo Buzzer's pin
              const int DISTANCE_THRESHOLD = 100; // centimeters

              // variables will change:
              float duration_us, distance_cm;

              void setup() {
                Serial.begin (9600);         // initialize serial port
                pinMode(TRIG_PIN, OUTPUT);   // set arduino pin to output mode
                pinMode(ECHO_PIN, INPUT);    // set arduino pin to input mode
                pinMode(BUZZER_PIN, OUTPUT); // set arduino pin to output mode
              }

              void loop() {
                // generate 10-microsecond pulse to TRIG pin
                digitalWrite(TRIG_PIN, HIGH);
                delayMicroseconds(10);
                digitalWrite(TRIG_PIN, LOW);

                // measure duration of pulse from ECHO pin
                duration_us = pulseIn(ECHO_PIN, HIGH);
                // calculate the distance
                distance_cm = 0.017 * duration_us;

                if(distance_cm > DISTANCE_THRESHOLD)
                {
                  tone(2,1000);
                  digitalWrite(BUZZER_PIN, HIGH);
                } // turn on Piezo Buzzer
                else
                {
                  noTone(2);
                  digitalWrite(BUZZER_PIN, LOW);  // turn off Piezo Buzzer
                }
                // print the value to Serial Monitor
                Serial.print("distance: ");
                Serial.print(distance_cm);
                Serial.println(" cm");

                delay(500);
              }


9) Potentio meter + LED:


            const int LED_PIN=10;
        const int POTENTIOMETER_PIN=A0;
        void setup()
        {
          pinMode(LED_PIN, OUTPUT);
        }
        void loop()
        {
          int potentiometerValue = analogRead(POTENTIOMETER_PIN);
          int brightness = potentiometerValue / 4;
          analogWrite(LED_PIN, brightness);
        }
        
        
        
 10) DHT 22:
 
 
                  #include <DHT.h>;
              #define DHTPIN 7     // what pin we're connected to
              #define DHTTYPE DHT22   // DHT 22  (AM2302)
              DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
              //Variables
              int chk;
              float hum;  //Stores humidity value
              float temp; //Stores temperature value
              void setup()
              {
                Serial.begin(9600);
                dht.begin();
              }
              void loop()
              {
                  delay(2000);
                  //Read data and store it to variables hum and temp
                  hum = dht.readHumidity();
                  temp= dht.readTemperature();
                  //Print temp and humidity values to serial monitor
                  Serial.print("Humidity: ");
                  Serial.print(hum);
                  Serial.print(" %, Temp: ");
                  Serial.print(temp);
                  Serial.println(" Celsius");
                  delay(1000); //Delay 2 sec.
              }


11) LDR:

                const int ldrPin=A0;
          void setup() {
            Serial.begin(9600);
            pinMode(ldrPin,INPUT);
          }
          void loop() {
            int rawData = analogRead(ldrPin);   
            Serial.println(rawData);
            delay(1000);
          }


12) LDR+ LED

  
           int ldr=A0;//Set A0(Analog Input) for LDR.
           int value=0;
           int led=7;
           void setup() {
           Serial.begin(9600);
           pinMode(led,OUTPUT);
           }

           void loop() {
           value=analogRead(ldr);//Reads the Value of LDR(light).
           Serial.println("LDR value is :");//Prints the value of LDR to Serial Monitor.
           Serial.println(value);
           if(value<50)
             {
               digitalWrite(led,HIGH);//Makes the LED glow in Dark.
             }
             else
             {
               digitalWrite(led,LOW);//Turns the LED OFF in Light.
             }
             delay(1000);
           }
           
           
           
    13) LED chaser:
       
        
                        int pinsCount=7;                        // declaring the integer variable pinsCount
                int pins[] = {2,3,4,5,6,7,8};          // declaring the array pins[]

                void setup() {                
                  for (int i=0; i<pinsCount; i=i+1){    // counting the variable i from 0 to 9
                    pinMode(pins[i], OUTPUT);            // initialising the pin at index i of the array of pins as OUTPUT
                  }
                }

                void loop() {
                  for (int i=0; i<pinsCount; i=i+1){    // chasing right
                    digitalWrite(pins[i], HIGH);         // switching the LED at index i on
                    delay(100);                          // stopping the program for 100 milliseconds
                    digitalWrite(pins[i], LOW);          // switching the LED at index i off
                  }
                  for (int i=pinsCount-1; i>0; i=i-1){   // chasing left (except the outer leds)
                   digitalWrite(pins[i], HIGH);         // switching the LED at index i on
                    delay(100);                          // stopping the program for 100 milliseconds
                    digitalWrite(pins[i], LOW);          // switching the LED at index i off

                  }
                }



     1) https://wokwi.com/projects/340854137667191380
     2) https://wokwi.com/projects/340854470627820115
     3) https://wokwi.com/projects/340854790239027795
     4) https://wokwi.com/projects/340855309309313619
     5) https://wokwi.com/projects/340856211886834260
     6) https://wokwi.com/projects/340857527721787988
     7) https://wokwi.com/projects/340857898569564756
     8) 





            12/9/2022
 

          1. Seven segment LED display example: Counter Reference: https://wokwi.com/arduino/libraries/SevSeg/SevSeg_Counter
         2. Analog Joystick with two axes (horizontal/vertical) and an integrated push button. Etch-a-sketch - A simple drawing game using a MAX7219 LED Dot Matrix                     Reference: https://wokwi.com/projects/296234816685212169
         3. DHT22 on the ESP32 Reference: https://wokwi.com/projects/322410731508073042
                        4. Display distance on LCD screen with buzzer and LED  Reference 1: https://wokwi.com/projects/290056311044833800 Reference2:                                  https://wokwi.com/projects/290043622233997832
         5. Electronic Safe, powered by an Arduino Uno. Reference: https://wokwi.com/arduino/libraries/demo/electronic-safe
         6. Arduino LED Graph Bar - Move the potentiometer knob to control the LEDs Reference:  https://wokwi.com/projects/309829489359061570

       Answer:
   1) https://wokwi.com/projects/342585755492680276
   2)https://wokwi.com/projects/342586174174397012
   4)
   a)https://wokwi.com/projects/342586650855998035
   
   b)  https://wokwi.com/projects/342587630330839635
   5) https://wokwi.com/projects/342588971434377811
