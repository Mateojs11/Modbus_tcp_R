/*
 * Gateway_H_V1.ino
 *
 * Created: 11/19/2014 2:45:00 PM
 * Author: AndrésFelipe
 */ 

#include <Arduino.h>
#include <avr/io.h>//librerias para las interrupciones
#include <avr/interrupt.h>//librerias para las interrupciones
#include <avr/wdt.h>//libreria para el watchdog timer
#include <EEPROM.h>//libreria para la memoria Eeprom
#include <DVJA.h> // librerias para el manejo del protocolo DVJA
#include <MODBUS_TCP.h> //libreria para el manejo del protocolo modbus con DVJA
#include <HardwareSerial.h>


enum
{
	MB_SLAVE = 1
};

enum
{
	MB_REGS=50	 	/* total number of registers on slave */
};

unsigned char query2[];
char limpiador2 = 0;

int regs[MB_REGS];	/* this is the slave's modbus data map */
DVJA UCR,radio(&Serial1); // por defecto en el puerto 3, si se desea cambiar de puerto es necesario poner un argumento con el apuntador al puerto (&Serail3)
MODBUS_TCP gprs; // por defecto en el puerto 2
volatile int enviar = 0;
#define SERIAL_BUFFER_SIZE 255
#define timeReset 2400000

void setup()
{
	  //desactiva el watchdog timer
	  wdt_disable();
	  pinMode(13,OUTPUT);
	  digitalWrite(13,LOW);
	  // Funciones para la configuración de los puertos.
	  radio.config(1200,'n',0);
	  gprs.config(1200,'n',0);
	  UCR.config(1200,'n',0);
	  pinMode(PTT,OUTPUT);
	  digitalWrite(PTT,LOW);
	  Origen = EEPROM.read(0); //ID_esta
	  Destino = EEPROM.read(1); //ID_maestra
	  tipo_estacion = EEPROM.read(2);
	  //EEPROM.write(3,50);
	  //destino_mod = EEPROM.read(3);
	  if((Destino == 255)||(Origen == 255) || (tipo_estacion == 255))
	  {
		  No_configurado = 1;
	  }
	  //activa el watchdog timer a 8 segundos
		wdt_enable(WDTO_8S);
		// configuración del timer para la interrupción que corresponde a la respuesta por radio
		cli();
		OCR2A = 5;//62
		TCCR2A |= (1 << WGM21);
		// Set to CTC Mode
		//TIMSK2 |= (1 << OCIE2A);
		//Set interrupt on compare match
		TCCR2B |= (1 << CS21);
		// set prescaler to 64 and starts PWM
		sei();
}


void loop()
{

	int problema = radio.updateR(Origen, Destino, &Serial3);
	/* This is all for the Modbus slave */
	wdt_reset();
	int problema3 = gprs.update_mb(MB_SLAVE,regs, MB_REGS);
	// reinicio del contador del watchdog timer
	wdt_reset();
	//funcion para DVJA slave
	  
	int problema2 = UCR.update(Origen,Destino,start_addr);
	
	if (bandera == 1)
	{
		cli();
		TIMSK2 |= (1 << OCIE2A);
		sei();
		}else if(OCIE2A != 0 && bandera == 0){
		cli();
		TIMSK2 |= (0 << OCIE2A);
		sei();
	}
	
	
	reset_all();
	if(No_configurado == 1){
		int i = 1;
		Serial3.write(i);
		delay(500);
		Destino = EEPROM.read(0); //ID_esta
		Origen = EEPROM.read(1); //ID_maestra
		tipo_estacion = EEPROM.read(2);
		  
		if((Destino != 255) && (Origen != 255) || (tipo_estacion != 255))
		{
			No_configurado = 0;
		}
		wdt_reset();
	}

}


// Interrupción para el envio de la trama por radio, se usa el timer2 para manejar el PTT.
ISR(TIMER2_COMPA_vect)
{
	if (bandera >= 1)
	{
		digitalWrite(PTT,HIGH);
		bandera++;
		if (bandera > 65) // 65
		{	
			if (enviar < 3)
			{
				for (int i = 0; i < (query2[3] + 3); i++)
				{
					Serial1.write(query2[i]);
				}
				enviar++;
				bandera = 1;
			}
			else
			{
				bandera = 0;
				enviar = 0;
				//UCR.clear_buffer(&Serial3);// es esto necesario?
				digitalWrite(PTT,LOW);
				//TIMSK2 |= (0 << OCIE2A);
			}
		}
	}
}

void reset_all() // Criterio de reset del arduino 1.5 Horas.
{
	if (millis() > timeReset)
	{
		wdt_disable();
		wdt_enable(WDTO_15MS);
		while (1)
		{
		}
	}
}