/***************************************************************************************************/
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

char valorRecebido;

/***************************************************************************************************/


/***************************************************************************************************/


#include <ESP32Servo.h>
int servoPin = 5;
Servo myservo; // create servo object to control a servo

//Definindo os pinos sensor frente

#define trigPin 26     //Pino TRIG do sensor no pino analógico 26
#define echoPin 27     //Pino ECHO do sensor no pino analógico 27


//Definindo os pinos sensor tras.
#define trigPintras 2     //Pino TRIG do sensor no pino analógico 2
#define echoPintras 15     //Pino ECHO do sensor no pino analógico 15

#define Bip 4

// motor um            // Ligação dos pinos da Ponte H L298N
#define enA  70        //pino enA na porta digital 10
#define in1  25         //pino in1 na porta digital 9
#define in2  33         //pino in2 na porta digital 8

// motor dois          // Ligação dos pinos da Ponte H L298N
#define enB  70         //pino enB na porta digital 5
#define in3  23         //pino in3 na porta digital 7
#define in4  22         //pino in4 na porta digital 6


/***************************************************************************************************/
       
//função para procurar obtasculo a todo o tempo
int Procurar (void) {
float duracao = 0.0;              // variavael para Guardar a duração do retorno do som
float CM = 0.0;                   // variavael para Guardar a distancia

digitalWrite(trigPin, LOW);       //não envia som
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);      //envia som
delayMicroseconds(10);
digitalWrite(trigPin, LOW);       //não envia o som e espera o retorno do som enviado
duracao = pulseIn(echoPin, HIGH); //Captura a duração em tempo do retorno do som.

CM = (duracao / 58.8);            //Calcula a distância em centimetros

Serial.print("Distancia em CM: "); //Imprimi no monitor serial a distancia
Serial.print(CM);              



return CM;                        // Return to CM.
}

/***************************************************************************************************/

/***************************************************************************************************/
//Variaveis
int DistanciaDireita, DistanciaEsquerda;  // variavel de Distâncias de ambos os lados
float Distancia = 0.00;                   // variavel para guardar a distancia

//Velocidades dos motores (você pode calibrar cada motor colocando os valores de 0 a 254)

int velocidadeMotorUm = 200;
int velocidadeMotorDois = 200;

// Função que é executado na inicialização do Arduino

/***************************************************************************************************/

/***************************************************************************************************/
void setup() {
  SerialBT.begin("APOLO-2");
  Serial.print("o dispositivo ja pode ser conectado");


  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);// Standard 50hz servo
  myservo.attach(servoPin, 500, 2400);
  
Serial.begin(9600);
// Definir todos os pinos de controle do motor como saídas

//pinMode(pinoS,INPUT);



pinMode(enA, OUTPUT);
pinMode(enB, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);

pinMode(Bip,OUTPUT);



//Configuraçõs do sensor ultrassonico

pinMode(trigPin, OUTPUT);     //define o pino TRIG como saída
pinMode(echoPin, INPUT);      //define o pino ECHO como entrada
}

/***************************************************************************************************/

/***************************************************************************************************/
void loop() {
  int modos = 0;
  
  valorRecebido = (char)SerialBT.read();
  if(Serial.available() ){
    SerialBT.write(Serial.read());
  }

 if(valorRecebido =='0'){modos = 0;
 
 }
 if(valorRecebido =='1'){modos = 1;}
    
 if(modos==0){controleTotal();}
 if(modos==1){autonomo();}
  }


/***************************************************************************************************/

/***************************************************************************************************/

void controleTotal(){
//  
    if(valorRecebido =='f'){Frente();}
    
    if(valorRecebido =='t'){ParaTras ();}

    if(valorRecebido =='p'){Parar ();}



    
    if(valorRecebido =='d'){
Serial.println("Robô: Direita ");
digitalWrite(in1, HIGH);                           //MOTOR 1 
digitalWrite(in2, LOW);

digitalWrite(in3, LOW);                          //MOTOR 2
digitalWrite(in4, HIGH);
    }
    //else{Parar ();}
    
    if(valorRecebido =='e'){
Serial.println("Robô: Esquerda ");                
digitalWrite(in1, LOW);                           //MOTOR 1 
digitalWrite(in2, HIGH);

digitalWrite(in3, HIGH);                          //MOTOR 2
digitalWrite(in4, LOW);
      }

      
}




void autonomo(){
 // SINAL();
  myservo.write(90);                           // Gira o Servo com o sensor a 90 graus
 delay (100);                                      // Aguarda 100 milesugodos

Distancia = Procurar ();                          // Medindo a Distancia em CM.
if (Distancia < 30) {                             // Se há obstáculo encontrado a menos de 40cm.
direcao ();                                      // Se Frente estiver bloqueado, mude de direção
}

else if (Distancia >= 30)  {                      // Se o obstáculo for encontrado entre a mais de 40cm 
Frente ();    
}
}



void direcao () {        
Parar ();                                         // O robô Para
ParaTras ();                                      // O robô vai para tras
Parar ();                                         // O robô Para
myservo.write (180);                          // Gira o Servo com o sensor a 180 graus
delay (1000);              
DistanciaEsquerda = Procurar ();                  // Defina a Distancia da Esquerda 
delay (500);               
myservo.write (0);                            // Gira o Servo com o sensor a 0 graus
delay (500);               
DistanciaDireita = Procurar ();                   // Defina a Distancia da Direita
delay (500);               
myservo.write (90);                           // Gira o Servo com o sensor a 90 graus
delay (500);              
CompareDistance ();                               // Encontre a distância mais longa.
}
/***************************************************************************************************/

/***************************************************************************************************/
void CompareDistance () {  // Função para calcular qual a distancia é melhor para o robô ir                 
if (DistanciaDireita > DistanciaEsquerda) {       // Se a direita está menos obstruída.
Vireadireita ();                                // O robô vai virar a direita 
}
else if (DistanciaEsquerda > DistanciaDireita) {  // Se Esquerda estiver menos obstruída.
VireaEsquerda ();                               // Robô Vire na direção esquerda.
}

else {                                            // Se ambos estiverem igualmente obstruídos. 
Retorne ();                        
}
}
/***************************************************************************************************/

/***************************************CRIAR VOIDS************************************************************/
void SINAL(){
  digitalWrite(Bip, HIGH);
  delay(500);
  digitalWrite(Bip, LOW);
  delay(200);
  digitalWrite(Bip, HIGH);
  delay(500);
  digitalWrite(Bip, LOW);
}


void Parar()
{// Função para fazer o carro parar
Serial.println("Robô: Parar ");
digitalWrite(in1, LOW);                           //Configurar a ponte h 
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
}
// Função para fazer o robô andar para frente

void Frente()
{
Serial.println("Robô: Frente ");
digitalWrite(in1, HIGH);                          //Configurar a ponte h 
digitalWrite(in2, LOW);

digitalWrite(in3, HIGH); 
digitalWrite(in4, LOW);
analogWrite(enA, velocidadeMotorUm);              // Defina a velocidade do motor Um
analogWrite(enB, velocidadeMotorDois);            // Defina a velocidade do motor Dois                         
}


void Vireadireita (){
Serial.println("Robô: Direita ");
digitalWrite(in1, HIGH);                           //MOTOR 1 
digitalWrite(in2, LOW);

digitalWrite(in3, LOW);                          //MOTOR 2
digitalWrite(in4, HIGH);

delay(900);                                       //aguarda um tempo


analogWrite(enA, velocidadeMotorUm);              // Defina a velocidade do motor Um
analogWrite(enB, velocidadeMotorDois);            // Defina a velocidade do motor Dois                         
}




void VireaEsquerda(){

Serial.println("Robô: Esquerda ");                
digitalWrite(in1, LOW);                           //MOTOR 1 
digitalWrite(in2, HIGH);

digitalWrite(in3, HIGH);                          //MOTOR 2
digitalWrite(in4, LOW);

delay(900);                                       //aguarda um tempo


analogWrite(enA, velocidadeMotorUm);              // Defina a velocidade do motor Um
analogWrite(enB, velocidadeMotorDois);            // Defina a velocidade do motor Dois                         
}                        


// Função que faz o robô andar para trás e emite som quando ele dá ré
void ParaTras()
{
Serial.println("Robô: Ré ");
digitalWrite(in1, LOW);                          //Configurar a ponte h 
digitalWrite(in2, HIGH);

digitalWrite(in3, LOW); 
digitalWrite(in4, HIGH);
                       
}

void Retorne () {    
Serial.println("Robô: Girar ");      
digitalWrite(in1, HIGH);                          //Configurar a ponte h 
digitalWrite(in2, LOW);
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
delay (1000);                                      //aguarda um tempo
                       
}

/*
void obstacolo () {    
Serial.println("Robô: Parar ");
digitalWrite(in1, LOW);                           //Configurar a ponte h 
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);
delay(100);                                       //aguarda um tempo

Serial.println("Robô: Ré ");
digitalWrite(in1, LOW);                           //Configurar a ponte h 
digitalWrite(in2, HIGH);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
delay(300);                                       //aguarda um tempo
analogWrite(enA, velocidadeMotorUm);              // Defina a velocidade do motor Um
analogWrite(enB, velocidadeMotorDois);  

Serial.println("Robô: Direita ");
digitalWrite(in1, LOW);                           //Configurar a ponte h 
digitalWrite(in2, HIGH);
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
delay(300);                                       //aguarda um tempo
analogWrite(enA, velocidadeMotorUm);              // Defina a velocidade do motor Um
analogWrite(enB, velocidadeMotorDois); 

Serial.println("Robô: Ré ");
digitalWrite(in1, LOW);                           //Configurar a ponte h 
digitalWrite(in2, HIGH);
digitalWrite(in3, LOW);
digitalWrite(in4, HIGH);
delay(300);                                       //aguarda um tempo
analogWrite(enA, velocidadeMotorUm);              // Defina a velocidade do motor Um
analogWrite(enB, velocidadeMotorDois);  

Serial.println("Robô: Direita ");
digitalWrite(in1, LOW);                           //Configurar a ponte h 
digitalWrite(in2, HIGH);
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);
delay(300);                                       //aguarda um tempo
analogWrite(enA, velocidadeMotorUm);              // Defina a velocidade do motor Um
analogWrite(enB, velocidadeMotorDois); 
} */  
