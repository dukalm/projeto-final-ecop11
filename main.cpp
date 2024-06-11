#include <Arduino.h>
#include "ModbusRTU.h"

#define ID_SLAVE 1
#define TEMP 0x00  // 00 dec
#define UMID 0x01// 01 dec
#define PRES 0x59 // 89 dec

// Criar o objeto modbus
ModbusRTU mb;

void setup()
{
    Serial.begin(9600,SERIAL_8N1); //9.600bps e 8bits de dados e 1bit paridade
    mb.begin(&Serial); // ligou modbus na serial
    // Definir o número do slave
    mb.slave(ID_SLAVE);
    // Definir as variáveis do slave matr 010089
    mb.addHreg(TEMP);
    mb.addHreg(UMID);
    mb.addHreg(PRES);
}

long int tempo = 0;

void loop()
{
    if (millis() - tempo > 1000) // passo 1 segundo
    {
        // gerando valores sinteticos para as medidas
          int temp = random(0, 400); 
          int umid = random(0, 100);
          int pres = random(0, 200);
        // inserindo medidas no Modbus
        mb.Hreg(TEMP, temp);
        mb.Hreg(UMID, umid);
        mb.Hreg(PRES, pres);
        // atualizar para próximo segundo
        tempo = millis();
    }
    // rotina do modbus
    mb.task();
    yield();
}