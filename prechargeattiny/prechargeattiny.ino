#define charge_state_pin 3
#define airn_closed_pin 4
#define begin_pre_pin 2


#define AIRn_pin 6
#define AIRp_pin 7
#define PRE_pin 5

void setup(){

  pinMode(charge_state_pin, INPUT);
  pinMode(airn_closed_pin, INPUT);
  pinMode(begin_pre_pin, INPUT);

  pinMode(AIRn_pin, OUTPUT);
  pinMode(AIRp_pin, OUTPUT);
  pinMode(PRE_pin, OUTPUT);

  digitalWrite(AIRn_pin, LOW);
  digitalWrite(AIRp_pin, LOW);
  digitalWrite(PRE_pin, LOW);

  //Serial.begin(9600);
};

typedef enum
{
  espera, //Aguarda os 24V do shutdown
  massa,  //Fecha o air- e confirma que fechou   
  precharge, //Fecha o relay do precharge
  precharge_complete,  //Fecha o air+ e espera 50ms para confirmar que o air+ esta fechado
  OHGROSSA,    //Carro a andar, air+ permanece fechado, precharge relay aberto
}estados; //maquina de estados                                                               

//char* estadosStrings[] = {"espera", "massa", "precharge", "precharge_complete", "OHGROSSA"};     //estados para print


bool AIRn = 0, AIRp = 0, PRE = 0;                               //saidas de comando
bool charge_state = 0, airn_closed = 0, begin_pre = 0;          //entradas logicas como estava bool charge_state = 1, airn_closed = 1, begin_pre = 0;

unsigned int tempo = 0;                                         //utilizado para definir 

estados EstadoAtual, ProximoEstado = espera;                    //variaveis de posicao da maquina


void readinputs()
{
  charge_state = digitalRead(charge_state_pin);
  airn_closed = digitalRead(airn_closed_pin);
  begin_pre = digitalRead(begin_pre_pin);
}


/* 
//Print dos estados para debug
 void printestados()
 {
   if(EstadoAtual != ProximoEstado)
   {
     Serial.println(estadosStrings[ProximoEstado]);
   }
 }
*/

void inicio()
{
  AIRn = 0;
  AIRp = 0;
  PRE = 0;
}

void maquina()
{
  switch (EstadoAtual) {
  case espera:
  {
    if(begin_pre == 1)
    {
      ProximoEstado = massa;
    }
    break;
  }
  case massa:
  {
    if(airn_closed == 1)
    {
      tempo = 0;
      ProximoEstado = precharge;
    }
    if(begin_pre == 0)
    {
      ProximoEstado = espera;
    }
    break;
  }
  case precharge:
  {
    if(charge_state == 1)
    {
      tempo = millis();
      ProximoEstado = precharge_complete;
    }
    if(begin_pre == 0)
    {
      ProximoEstado = espera;
    }    
    break;
  }
  case precharge_complete:
  {
    if(millis() >= tempo + 500)
    {
      ProximoEstado = OHGROSSA;
    }
    if(begin_pre == 0)
    {
      ProximoEstado = espera;
    }    
    break;
  }
  case OHGROSSA:
  {
    if(begin_pre == 0)
    {
      ProximoEstado = espera;
    } 
    break;
  }
  default:
  {
    break;
  }
  }

}

void writeoutputs()
{
  digitalWrite(AIRn_pin, AIRn);
  digitalWrite(AIRp_pin, AIRp);
  digitalWrite(PRE_pin, PRE); 
}

// "espera", "massa", "precharge", "precharge_complete", "OHGROSSA"    //estados

void loop(){

  inicio();
  // Serial.println(estadosStrings[ProximoEstado]);

  while(true)
  {
    readinputs();
    maquina();

    AIRn = ((EstadoAtual == massa) || (EstadoAtual == precharge)) || ((EstadoAtual == precharge_complete) || (EstadoAtual == OHGROSSA));
    PRE =  ((EstadoAtual == precharge) || (EstadoAtual == precharge_complete));
    AIRp = ((EstadoAtual == precharge_complete) || (EstadoAtual == OHGROSSA));
    

    //AIRn = LOW;
    //AIRp = LOW;
    //PRE = LOW;

    // printestados();
    EstadoAtual = ProximoEstado;

    writeoutputs();
    delay(100);
  }
};
