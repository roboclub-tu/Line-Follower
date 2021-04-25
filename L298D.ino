 int E1 = 10;
int M1 = 12;
int E2 =11;
int M2 = 13;

void setup()
{
pinMode(M1, OUTPUT);
pinMode(M2, OUTPUT);
  }

void forward()
{
  int value;
for(value = 0 ; value <= 255; value+=5)
    {
      //HIGH - BACKWARDS
      //LOW - FORWARD
      
     digitalWrite(M1,LOW);
     analogWrite(E1, value);

     //RIGHT WHEEL
     digitalWrite(M2, LOW);
     analogWrite(E2, value);
    }
    delay(1000);
 }

void backward()
{
 int value;
for(value = 0 ; value <=255; value+=5)
    {
      //HIGH - BACKWARDS
      //LOW - FORWARD
      
     digitalWrite(M1,HIGH);
     analogWrite(E1, value);

     //RIGHT WHEEL
     digitalWrite(M2, HIGH);
     analogWrite(E2, value);
    }
    delay(100);
}
void loop()
{
   forward();
   //backward();
}
 
