int sensorvklop = 12;
int tipka = 6;
int sprejemnik = A2;
int sprejemnik1 = A3;
int vrednostX = A0;
int vrednostY = A1;
int rele1 = 11;

int stikalo = 7;


int playtime = 12;
bool start = false;
bool starttill10 = false;
bool endofgame = false;
bool stopcount = false;
bool stopcount1 = false;
bool A = true;
bool B = false;
bool izbiraCounter = false;
bool izbiraTime = false;
bool ON = false;
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);


int  vrednostXa = 0;
int vrednostSprejem = 0;
int vrednostSprejem1 = 0;
int stanje = 0;
int prejsStanje = 0;
unsigned long stevec = 0;
unsigned long stevec1 = 0;
unsigned long zacetek = 0;
unsigned long zacetek1 = 0;
unsigned long zacetek2 = 0;
unsigned long konec = 0;
unsigned long konec1 = 0;
unsigned long konec2 = 0;
unsigned long previousTime = 0;

////////////////////////////////////////////////////////////////////////
//definition of the dimensions of the board
#define brdXd 4000      //in steps
#define brdYd 6000
#define pixXd 333       //in pixels
#define pixYd 566

//definition of the motors conections
#define stepmotA 5
#define stepmotB 3
#define dirmotA 4
#define dirmotB 2

// endstops definitions
#define endstopX 8
#define endstopY 9

#define spee 400        //define the speed of the system

bool calibrated = 0;

long cmicros = 0;
long pmicrosA = 0;
long pmicrosB = 0;

bool prevA = 0;
bool prevB = 0;


String inSer;         // incoming serial string
int stpmotA = 0;      //counter for the motor steps
int stpmotB = 0;
int posX = 0;         //real system positions
int posY = 0;
//used in the mov2 function
int finX, finY;       //final positions
int finSmA, finSmB, stepsmA, stepsmB;
long speedmA, speedmB;
//

bool actdirecA = 0;   //variables to save the actual direction of each motor
bool actdirecB = 0;
bool newsermssg = false;

int steps2goA = 0;
int steps2goB = 0;

boolean toggle0 = 0;
boolean toggle1 = 0;




void setup() {

 Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.setTimeout(1);
  Serial.println("connected!");
  pinMode(stepmotA, OUTPUT);
  pinMode(stepmotB, OUTPUT);
  pinMode(dirmotA, OUTPUT);
  pinMode(dirmotB, OUTPUT);
  pinMode(endstopX, INPUT);
  pinMode(endstopY, INPUT);




//////////////////////////////////////////////////////////
  pinMode(sensorvklop, OUTPUT);
  pinMode(tipka, INPUT);
  pinMode(rele1, OUTPUT);
  pinMode(sprejemnik, INPUT);
  pinMode(sprejemnik1, INPUT);
  pinMode(vrednostX, INPUT);
  pinMode(vrednostY, INPUT);
  pinMode(stikalo, INPUT);

   digitalWrite(rele1,HIGH);
  digitalWrite(tipka, HIGH);
  digitalWrite(stikalo, HIGH);

  lcd.begin();                      // initialize the lcd

  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("hello");
  digitalWrite(sensorvklop, HIGH);
  }










void loop() {


 if (Serial.available() > 0) {
    inSer = Serial.readString();
//    Serial.println(inSer);
    newsermssg = true;
  }

  if (newsermssg == true and start) {
    //    newsermssg = false;
    if (inSer == "calib") {                               //Function to calibrate the position of the system
      inSer = "";
     // Serial.println("calibrating");
      calibrated = 0;
      calibrateSystem();
    }

    if (inSer == "movL") {                               //Function to move the stick at the Left
      inSer = "";
    //  Serial.println("movL");
      moveLeft();
    }

    if (inSer == "movI") {                               //Function to move the stick Forward
      inSer = "";
//      Serial.println("movI");
      moveI();
    }

    if (inSer == "movFB") {                              //Function to move the stick Forward and Backward repetitely
      inSer = "";
//      Serial.println("movFB");
      moveFB();
    }

    if (inSer.startsWith("mov2")) {                     //Function to move the stick to a gived position Sintax: mov2,xxx,yyy
      String input = inSer;
      inSer = "";
      int auxX, auxY, auxi, firstVal, secondVal;
      bool nf = false;
      for (int i = 0; i < input.length(); i++) {
        if (input.substring(i, i + 1) == ",") {
          if (nf == true) {
            firstVal = input.substring(auxi + 1, i).toInt();
            secondVal = input.substring(i + 1).toInt();
            break;
          }
          auxi = i;
          nf = true;
        }
      }
//      Serial.print(firstVal);
//      Serial.print("     ");
//      Serial.println(secondVal);
      move2(firstVal, secondVal);

    }

    if (inSer.startsWith("def2")) {                     //Function to move the stick to a gived position Sintax: mov2,xxx,yyy
      String input = inSer;
      inSer = "";
      int auxX, auxY, auxi, firstVal, secondVal;
      for (int i = 0; i < input.length(); i++) {
        if (input.substring(i, i + 1) == ",") {
          firstVal = input.substring(0, auxi).toInt();
          secondVal = input.substring(i + 1).toInt();
          break;
          auxi = i;
        }
      }
//      Serial.print(firstVal);
//      Serial.print("     ");
//      Serial.println(secondVal);
      defense2(secondVal);

    }
  }


  if (calibrated) {
    //    Serial.print(stpmotA);
    //    Serial.print("     ");
    //    Serial.print(steps2goA);
    //    Serial.print("     ");
    //    Serial.print(stpmotB);
    //    Serial.print("     ");
    //    Serial.println(steps2goB);
    if (stpmotA != steps2goA) {
      cmicros = micros();
      if (stpmotA > steps2goA)
        setdirA(1);
      else
        setdirA(0);
      dostepA();
    }
    if (stpmotB != steps2goB) {
      cmicros = micros();
      if (stpmotB > steps2goB)
        setdirB(1);
      else
        setdirB(0);
      dostepB();
    }
  }




 
  ON = digitalRead (stikalo);
  if (ON == 0 ) {


    if (start == false) {
      chooseGame();
    }
    else if ( start == true) {
    }


    if (A and start) {  Function for "playfor 2min" game mode
      
      counter();

      zacetek = millis();

      if ((zacetek - konec) > 1000) {
        playtime--;
        konec = zacetek;

        lcd.setCursor(12, 1);
        lcd.print(playtime);

      }
      if ( playtime == 0) {
        if (stevec > stevec1) {
          lcd.clear();
          lcd.setCursor(4, 0);
          lcd.print("You lost");
          delay(2000);
          endofgame = true;
        }
        else if (stevec < stevec1) {
          lcd.clear();
          lcd.setCursor(4, 0);
          lcd.print("You win");
          delay(2000);
          endofgame = true;
        }
        else {
           lcd.clear();
          lcd.setCursor(4, 0);
          lcd.print("Train more");
          delay(2000);
          endofgame = true;
          }

      }
      delay(100);
    }

    if (B and start) {        //Function for "play till 10" game mode
      if ((stevec or stevec1) < 3) {
        
       counter ();

      }
      if (stevec > 2) {
        lcd.clear();
        lcd.setCursor(5, 0);
        lcd.print("You lost");
        delay(2000);
        endofgame = true;

      }
      else if (stevec1 > 2) {
        lcd.clear();
        lcd.setCursor(5, 0);
        lcd.print("You win");
        delay(2000);
        endofgame = true;

      }
    }
  }
  if (ON == 1 or endofgame == true) {     //Function to reset variables
    start = false;
    pregame();
    B = false;
    A = true;
    endofgame = false;
    stevec1 = 0;
    stevec = 0;
    playtime = 12;
   digitalWrite(rele1,HIGH);
  }
}



void counter() {      ///function for counting goals

 zacetek2 = millis();

 if ((zacetek2-konec2)>200){      //Avoding to reed the analog value every cicle (increase speed)
  Serial.println(vrednostSprejem);
  vrednostSprejem = analogRead(sprejemnik);
  vrednostSprejem1 = analogRead(sprejemnik1);
  konec2 = zacetek2;
 }

  if (vrednostSprejem < 760 and not stopcount) {  // commparing analog value from photo resistor
    stanje =  1;
    if (stanje != prejsStanje) {
      stevec ++;
       lcd.setCursor(2, 1);
       calibrateSystem();
       lcd.print(stevec);
      
      

      stopcount1 = true;
      zacetek1 = millis();
      prejsStanje = stanje;
    }
  }
  else if (vrednostSprejem1 < 800 and not stopcount and false) {    // commparing analog value from photo resistor
    stanje =  1;

    if (stanje != prejsStanje) {   //icreasing counter
      stevec1 ++;
      lcd.setCursor(7, 1);
      lcd.print(stevec1);
 

      stopcount1 = true;
      zacetek1 = millis();
      prejsStanje = stanje;
    }
  }
  else {
    stanje = 0;
    prejsStanje = 0;
    
  }
  if (stopcount1) {
    stopcount = true;
    konec1 = millis();
    
    if ((konec1-zacetek1) > 4000) {
      stopcount = false;
      stopcount1 = false;

    }
  }

 
}



void pregame() {        //If system is off
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("WELCOME ");
  lcd.setCursor(1, 1);
  lcd.print("Please turn on");
  delay(1000);
}


void chooseGame() {   //function for choosing game mode

  bool potrditev = true;

  lcd.setCursor(0, 0);
  lcd.print("   Game mode:   ");

  vrednostXa = analogRead(vrednostX);
  potrditev = digitalRead(tipka);



  if (vrednostXa < 50 ) {       //Compering analog value from yostick
    A = true;
    B = false;


  }
  else if (vrednostXa > 1000) {
    B = true;
    A = false;


  }
  if (A == true) {
    lcd.setCursor(0, 1);
    lcd.print("Play for 2 min   ");
  }
  else if (B) {
    lcd.setCursor(0, 1);
    lcd.print("Play to reach 10");
  }

  if (A == true and potrditev == 0) { 
    izbiraCounter = true;
    izbiraTime = false;
calibrateSystem();
for2min();
    digitalWrite(rele1,LOW);
     start = true;
  }
  else if ( B and potrditev == 0) {
    izbiraCounter = false;
    rech10();
    izbiraTime = true;
calibrateSystem();
   digitalWrite(rele1,LOW);
    start = true;
  }
  delay(20);
}



void for2min() {      //function for printing on screen 
  lcd.setCursor(0, 0);
  lcd.print("Play for 2 min");
  lcd.setCursor(0, 1);
  lcd.print("M:    ");
  lcd.setCursor(2, 1);
  lcd.print(stevec);
  lcd.setCursor(5, 1);
  lcd.print("P: ");
  lcd.setCursor(7, 1);
  lcd.print(stevec1);
  lcd.setCursor(8, 1);
  lcd.print("  T:     ");



}

void rech10() {     function for printing on screen 
  lcd.setCursor(0, 0);
  lcd.print("Play to reach 10");
  lcd.setCursor(0, 1);
  lcd.print("M:    ");
  lcd.setCursor(2, 1);
  lcd.print(stevec);
  lcd.setCursor(5, 1);
  lcd.print("P:         ");
  lcd.setCursor(7, 1);
  lcd.print(stevec1);

}


/////////////////////////////////////////////////////






void calibrateSystem() {    //Function to calibrate the position of the system
  Serial.println("Starting calib");

  setdirA(0);
  setdirB(1);
  digitalRead(endstopX);
  while (digitalRead(endstopX) == LOW) {
    cmicros = micros();
    dostepA();
    dostepB();
    delayMicroseconds(spee);
  }
  setdirA(1);
  setdirB(1);
  digitalRead(endstopY);
  while (digitalRead(endstopY) == LOW) {
    cmicros = micros();
    dostepA();
    dostepB();
    delayMicroseconds(spee);
  }

  stpmotA = 0;      //counter for the motor steps to 0
  stpmotB = 0;
  steps2goA = 0;    //reset to the desired final position
  steps2goB = 0;
  posX = 0;         //real system positions to 0
  posY = 0;
//  Serial.println("System calibrated");
  calibrated = 1;

}

void moveFB() {
  int counting = 0;
  while (counting <= 3) {
    setdirA(1);
    setdirB(0);
    int count = 0;
    while (count < 2000) {
      dostepA();
      dostepB();
      delayMicroseconds(spee);
      count ++;
    }
    setdirA(0);
    setdirB(1);
    count = 0;
    while (count < 2000) {
      dostepA();
      dostepB();
      delayMicroseconds(spee);
      count ++;
    }
    counting ++;
  }
}

void moveI() {
  setdirA(1);
  setdirB(0);
  while (digitalRead(endstopX) == LOW) {
    dostepA();
    dostepB();
    delayMicroseconds(spee);
  }
}

void moveLeft() {       //Function to move the stick at the Left
  setdirA(1);
  setdirB(1);
  while (digitalRead(endstopY) == LOW) {
    dostepA();
    dostepB();
    delayMicroseconds(spee);
  }
//  Serial.println(stpmotB);
}

void defense2(int x) {
  Serial.println("defending");
  int finX;
  int delX;
  bool Xdir2goA, Xdir2goB;
  finX = map(x, 0, pixXd, 0, brdXd);     //we convert the coordinates recived from the serial port from pixels to steps
  //  Serial.print("stpmotB:    ");
  //  Serial.print(stpmotB);
  //  Serial.print("     stpmotA:    ");
  //  Serial.println(stpmotA);
  posX = (stpmotB - stpmotA) / 2;      //we compute the real position in our coordinates using the actual steps of the motors
  posY = (stpmotB + stpmotA) / 2;


  delX = finX - posX;                   //we compute the distance from the real position to the desired position
  //  Serial.print("delX:    ");
  //  Serial.print(delX);
  //  Serial.print("    finX:    ");
  //  Serial.print(finX);
  //  Serial.print("    posX:    ");
  //  Serial.println(posX);
  if (delX >= 0) {                     //we compute the steps of each motor and the direction of it
    steps2goA = delX;
    steps2goB = delX;
    Xdir2goA = 0;
    Xdir2goB = 0;
    //    Serial.println("right");

  }
  else {
    steps2goA = delX;
    steps2goB = delX;
    Xdir2goA = 1;
    Xdir2goB = 1;
    //    Serial.println("left");
  }
  //  Serial.println("motor steps 2");
  //  Serial.print(delX);
  //  Serial.print("     ");
  //  Serial.println(Xdir2goA);
  //  setdirA(Xdir2goA);
  //  setdirB(Xdir2goB);
  speedmA = spee;
  speedmB = spee;
}

void move2(int pixX, int pixY) {       //Function to move the stick to a gived position Sintax: mov2,xxx,yyy
  int finX, finY;
  int delX, delY;
  int Xsteps2doA, Xsteps2doB;
  bool Xdir2goA, Xdir2goB;
  int Ysteps2doA, Ysteps2doB;
  bool Ydir2goA, Ydir2goB;
  long auxSpeed, coefSpeed;


  finX = map(pixX, 0, pixXd, 0, brdXd);     //we convert the coordinates recived from the serial port from pixels to steps
  finY = map(pixY, 0, pixYd, 0, brdYd);

  posX = (stpmotB - stpmotA) / 2;      //we compute the real position in our coordinates using the actual steps of the motors
  posY = (stpmotB + stpmotA) / 2;

  delX = finX - posX;
  delY = finY - posY;


  if (abs(delX) >= 0) {
    Xsteps2doA = delX;
    Xsteps2doB = delX;
    Xdir2goA = 0;
    Xdir2goB = 0;
  }
  else {
    Xsteps2doA = delX;
    Xsteps2doB = delX;
    Xdir2goA = 1;
    Xdir2goB = 1;
  }
  if (abs(delY) >= 0) {
    Ysteps2doA = delY;
    Ysteps2doB = delY;
    Ydir2goA = 1;
    Ydir2goB = 0;
  }
  else {
    Ysteps2doA = delY;
    Ysteps2doB = delY;
    Ydir2goA = 0;
    Ydir2goB = 1;
  }
//  Serial.println("motor steps 2 do and dir Y");
//  Serial.print(Ysteps2doA);
//  Serial.print("     ");
//  Serial.println(Ysteps2doB);
//  Serial.print(Ydir2goA);
//  Serial.print("     ");
//  Serial.println(Ydir2goB);
//  Serial.println("motor steps 2 do and dir X");
//  Serial.print(Xsteps2doA);
//  Serial.print("     ");
//  Serial.println(Xsteps2doB);
//  Serial.print(Xdir2goA);
//  Serial.print("     ");
//  Serial.println(Xdir2goB);
  if ((Xdir2goA == 0 and Ydir2goA == 0) or (Xdir2goA == 1 and Ydir2goA == 1))
    finSmA = Xsteps2doA + Ysteps2doA;
  else
    finSmA = Xsteps2doA - Ysteps2doA;
  if ((Xdir2goB == 0 and Ydir2goB == 0) or (Xdir2goB == 1 and Ydir2goB == 1))
    finSmB = Xsteps2doB + Ysteps2doB;
  else
    finSmB = Xsteps2doB - Ysteps2doB;
//  Serial.println("Final steps each motor:");
//  Serial.print(finSmA);
//  Serial.print("     ");
//  Serial.println(finSmB);
  //  if(finSmA>=0)
  //    setdirA(1);
  //  else
  //    setdirA(0);
  //  if(finSmB>=0)
  //    setdirB(1);
  //  else
  //    setdirB(0);

  stepsmA = abs(finSmA);
  stepsmB = abs(finSmB);
  steps2goA = stpmotA + finSmA;
  steps2goB = stpmotB + finSmB;
//  Serial.print(stepsmA);
//  Serial.print("     ");
//  Serial.println(stepsmB);
  //  setdirA(finSmA < 0);
  //  setdirB(finSmB < 0);
  if (stepsmA > stepsmB) {
//    Serial.println("cas1");
    coefSpeed = stepsmA / stepsmB;
    speedmA = spee;
    speedmB = spee * coefSpeed;
  }
  else if (stepsmA < stepsmB) {
//    Serial.println("cas2");
    coefSpeed = stepsmB / stepsmA;
    speedmB = spee;
    speedmA = spee * coefSpeed;
  }
  else {
//    Serial.println("cas3");
    speedmA = spee;
    speedmB = spee;
  }
//  Serial.print(spee);
//  Serial.print("     ");
//  Serial.print(coefSpeed);
//  Serial.print("     ");
//  Serial.print(speedmA);
//  Serial.print("     ");
//  Serial.println(speedmB);
  delay(3000);




}

void dostepA() {            //function to do a step with the A motor
  if (cmicros - pmicrosA >= speedmA) {        //cheks the real time and decide if it's the time to do the next step
    pmicrosA = cmicros;
    if (prevA == 0) {
      digitalWrite(stepmotA, HIGH);
      prevA = 1;
    }
    else {
      digitalWrite(stepmotA, LOW);
      prevA = 0;
    }
    if (actdirecA == 1) {
      stpmotA --;
    }
    else {
      stpmotA ++;
    }
  }
}

void dostepB() {            //function to do a step with the B motor
  if (cmicros - pmicrosB >= speedmB) {        //cheks the real time and decide if it's the time to do the next step
    pmicrosB = cmicros;
    if (prevB == 0) {
      digitalWrite(stepmotB, HIGH);
      prevB = 1;
    }
    else {
      digitalWrite(stepmotB, LOW);
      prevB = 0;
    }
    if (actdirecB == 1) {
      stpmotB --;
    }
    else {
      stpmotB ++;
    }
  }
}

void setdirA(bool direc) {             //function to configure the direction of the A motor
  digitalWrite(dirmotA, direc);
  actdirecA = direc;
//  Serial.println(direc);


}

void setdirB(bool direc) {             //function to configure the direction of the B motor

  digitalWrite(dirmotB, direc);
//  Serial.println(direc);
  actdirecB = direc;
}
