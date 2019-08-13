// Created by William McColl and Djouzar ABASSEBAY
// 16/09/19  - Heriot Watt University
// Send a packet with force sensors and joint angles values for a hand exoskeleton 

#define Force1 A1
#define Force2 A5
#define ForceMCP A0
#define dip_potent A4
#define pip_potent A6
#define ledGreen 2 // Led to check if assessment started 
#define ledBlue 3
#define ledRed 4 // Led to check if the motors are ON

// Variables
char ForceResult1[5];
char AngleResult[5];
char Final[11];
int ForceValue1, ForceValue2;
int ForceValue_1, ForceValue_2;
int dip_angle, pip_angle;
int dip_angle_final, pip_angle_final;
int forceVlaueMCP_final;

void setup() {
  Serial.begin(9600);  // Baud rate
  pinMode (Force1, INPUT);  
  pinMode (Force2, INPUT);
  pinMode (dip_potent, INPUT);  
  pinMode (pip_potent, INPUT);
  pinMode (ForceMCP, INPUT);
  pinMode (ledGreen, OUTPUT);
  pinMode (ledBlue, OUTPUT);
  pinMode (ledRed, OUTPUT);
}

void loop() {
 // Serial.println("help");
  //Serial.println(Servo_Speed);
  
  // Read all the input
  ForceValue1 = analogRead(Force1);
  ForceValue2 = analogRead(Force2);
  int dip_angle = analogRead(dip_potent); 
  int pip_angle = analogRead(pip_potent); 
  int forceVlaueMCP = analogRead(ForceMCP); 

  // Map all the values to fit the defined scale
  ForceValue_1 = map(ForceValue1, 0, 1024, 0, 999); 
  ForceValue_2 = map(ForceValue2, 0, 1024, 0, 999); 
  dip_angle_final = map(dip_angle, 889, 502, 0, 90); 
  pip_angle_final = map(pip_angle, 894, 505, 0, 90); 
  forceVlaueMCP_final = map(forceVlaueMCP,0, 1024, 0, 999);

  // Create a packet with a fix lengh a three digit number for each sent value 
  sprintf(Final, "%03d, %03d, %03d, %03d ",ForceValue_1,ForceValue_2,dip_angle_final,pip_angle_final);

  // This part is to light on the LEDs
  char cst = Serial.read();
  //Serial.print(dip_angle);
  //Serial.print("  ,  " );
  Serial.println(Final);
  //Serial.print("  ,  " );
  //Serial.println(forceVlaueMCP_final);
  
 /* if (cst == 's'){
      digitalWrite(led,HIGH);
    }else if (cst == 'e'){
      digitalWrite(led,LOW);     
    }*/

  delay(50);
}
