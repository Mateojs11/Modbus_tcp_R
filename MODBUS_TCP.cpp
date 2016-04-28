#include <MODBUS_TCP.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <HardwareSerial.h>
//#include <DVJAV1.h>
#include <Arduino.h>

/*-----------------------------------------------------------------------------*/
/*--------------Declaración de variables globales------------------------------*/
/*-----------------------------------------------------------------------------*/

unsigned long Nowdt = 0;
unsigned int lastBytesReceived = 0;
const unsigned long T35 = 15;
volatile byte segundos = 0;
volatile int tip_esta = 5, pruebas = 0;


	enum {
		FC_READ_DISC  = 0x02,   //Read discrete inputs
		FC_READ_REGS  = 0x03,   //Read contiguous block of holding register
		FC_READ_INPUT = 0x04,   //Read Input Registers
		FC_WRITE_REG  = 0x06,   //Write single holding register
		FC_WRITE_REGS = 0x10,    //Write block of contiguous registers
		FC_REPORT_ID = 0x11,    //Report server id
		FC_USUAR_FUNT = 0x41,    //funcion de usuario 65
		FC_USUAR_INC = 0x42      //funcion de usuario 66
	};
	
	enum {
		//-------------------------------------------------
		//MODIFICACION:
		//ADU Modbus TCP
		TRANS_ID_H =0,
		TRANS_ID_L,
		PROT_ID_H,
		PROT_ID_L,
		LENGTH_H,
		LENGTH_L,
		UNIT_ID,
		FUNC,
		START_H,
		START_L,
		REGS_H,
		REGS_L,
		BYTE_CNT
		//-------------------------------------------------
	};

ISR(TIMER1_COMPA_vect)
{
	segundos++;
	if(segundos == tip_esta)
	{	
		timeout = 1;
		segundos = 0;
	}
}
const unsigned char fsupported[] = { FC_READ_DISC, FC_READ_REGS, FC_READ_INPUT, FC_WRITE_REG, FC_WRITE_REGS, FC_REPORT_ID, FC_USUAR_FUNT, FC_USUAR_INC};

/*++++++++++++++++++++++++++++CONSTRUCTOR+++++++++++++++++++++++++*/

MODBUS_TCP::MODBUS_TCP(HardwareSerial *PuertoModbus)
{
	usedPort = PuertoModbus;
}

/* -------------------------------------------------------------------------------------------------*/
/*-------------------FUNCIONE PARA LA CONFIGURACIÓN DEL PUERTO--------------------------------------*/
/* -------------------------------------------------------------------------------------------------*/

int MODBUS_TCP::config( unsigned long baudRate, // Velocidad de transmisión de los datos 1200 para este caso.
					unsigned char parity, // la paridad que manejarán los datos, 8n1 para nuestro caso.
					unsigned int tXEnable) // el pin que habiliatará las comunicaciones.
{
	_baudRate = baudRate;
	_parity = parity;
	_txEnable = tXEnable;
	(*usedPort).begin(_baudRate);
	switch (_parity) {
		case 'e': // 8E1
		UCSR0C |= ((1<<UPM01) | (1<<UCSZ01) | (1<<UCSZ00));
		break;
		case 'o': // 8O1
		UCSR0C |= ((1<<UPM01) | (1<<UPM00) | (1<<UCSZ01) | (1<<UCSZ00));
		break;
		case 'n': // 8N1
		UCSR0C |= ((1<<UCSZ01) | (1<<UCSZ00));
		break;
		default:
		break;
	}
	return 0;
}

/* -------------------------------------------------------------------------------------------------*/
/*-------------------------------------Funcion principal de modbus----------------------------------*/
/* -------------------------------------------------------------------------------------------------*/
int MODBUS_TCP::update_mb(unsigned char slave, int *regs,unsigned int regs_size)
{
	unsigned char query[MAX_MESSAGE_LENGTH];
	unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
	unsigned int start_addr;
	int exception, length;
	unsigned long now;

	length = (*usedPort).available();
	now = millis();

	if (length == 0)
	{
		lastBytesReceived = 0;
		return 0;

	}

	if (lastBytesReceived != length)
	{
		lastBytesReceived = length;
		Nowdt = now + T35;
		return 0;
	}
	if (now < Nowdt)
	return 0;

	lastBytesReceived = 0;
	length = MODBUS_TCP::request(slave, query);

	if (length < 1)
		return length;

	exception = MODBUS_TCP::validate_request(query, length, regs_size);

	if (exception)
	{
		MODBUS_TCP::build_error_packet(slave,query[FUNC],exception,errpacket);
		MODBUS_TCP::send_reply(errpacket,EXCEPTION_SIZE);
		return (exception);
	}

	start_addr = ((int) query[START_H] << 8) +(int) query[START_L];
	/*int prueba_de_sonido = 0;*/
	switch (query[FUNC])
	{
		case FC_READ_DISC:
			return MODBUS_TCP::read_discrete_inputs(slave,start_addr,query[REGS_L],regs);
			break;
		case FC_READ_REGS:
			return MODBUS_TCP::read_holding_registers(slave,start_addr,query[REGS_L],regs);
			break;
		case FC_READ_INPUT:
			return MODBUS_TCP::read_input_registers(slave,start_addr,query[REGS_L],regs);
			break;
		case FC_WRITE_REGS:
			return MODBUS_TCP::preset_multiple_registers(slave,start_addr,query[REGS_L],query,regs);
			break;
		case FC_WRITE_REG:
			return MODBUS_TCP::write_single_register(slave,start_addr,query,regs);
			break;
		case FC_USUAR_FUNT:
			return MODBUS_TCP::Usuar_function(slave,start_addr,query,regs);
			break;
		case FC_USUAR_INC:
			return MODBUS_TCP::usuar_function_inicia(slave,query,regs);
			break;
	}
}


/* -------------------------------------------------------------------------------------------------*/
/*------------------------------------------Funciones de modbus.------------------------------------*/
/* -------------------------------------------------------------------------------------------------*/
int MODBUS_TCP::read_discrete_inputs(unsigned char slave, unsigned int start_addr,unsigned char reg_count, int *regs)
{
	unsigned char function = FC_READ_DISC; 	/* Function 03: Read Holding Registers */
	int packet_size = 9;
	int status;
	int exception = 1;
	unsigned int i;
	unsigned char packet[MAX_MESSAGE_LENGTH];
	unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
	DVJA UCR(&Serial3);
	//---------------------------------------------------------
	UCR.clear_buffer();
	UCR.consulta();
	sincrona = 1;
	timeout = 0;
	
	while ((exception == 1) && (timeout == 0 )) {
		//--------------------------------------------------------------
		//configuracion timer 1
		cli();//deshabilita las interrupciones globales
		TCCR1A=0;
		TCCR1B=0;
		OCR1A=15624;//valor a comparar para timer de 1 segundo
		//15624 62534
		//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
		TCCR1B |= (1<<WGM12);
		TCCR1B |= (1<<CS12);
		TCCR1B |= (1<<CS10);
		TIMSK1=(1<<OCIE1A); // flag comparacion
		sei(); //habilita las interrupciones globales
		
		//-------------------------------------------------------------------
		
		exception = UCR.update(Origen,Destino,start_addr);

		
	}
	
	
	//--------------------------------------------------------------
	//configuracion timer 1 	 detener
	cli();//deshabilita las interrupciones globales
	TCCR1A=0;
	TCCR1B=0;
	OCR1A=15624;//valor a comparar para timer de 1 segundo
	//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
	TCCR1B |= (1<<WGM12);
	//timer/counter stopped csn0,1,2 = 0
	TCCR1B |= (0<<CS10);
	TCCR1B |= (0<<CS11);
	TCCR1B |= (0<<CS12);
	//TIMSK1=(1<<TOIE1); // flag sobreflujo
	TIMSK1=(1<<OCIE1A); // flag comparacion
	sei(); //habilita las interrupciones globales
	
	//-------------------------------------------------------------------
	segundos = 0;
	//si corta es igual a 1 entonces la trama es corta
	//si la trama es corta debe responder error 4
	if(corta == 1){
		corta = 0;
		exception = 1;
	}
	if (exception) {
		MODBUS_TCP::build_error_packet(slave,function,4,errpacket);
		MODBUS_TCP::send_reply(errpacket,EXCEPTION_SIZE);
		return (exception);
	}
	
	sincrona = 0;
	//----------------------------------------------------------
	byte integridad = byte(Integri);
	
	byte puerta = B1;
	byte bateria= B10;
	byte sin110 = B100;
	byte sirena = B1000;
	byte sensor = B10000;
	
	byte PUERTA = puerta & integridad;
	byte BATERIA = bateria & integridad;
	byte SIN110 = sin110 & integridad;
	byte SIRENA = sirena & integridad;
	byte SENSOR = sensor & integridad;
	byte NADA = B000000;
	
	
	byte INTEGRIDAD[]= {PUERTA, BATERIA, SIN110, SIRENA, SENSOR, NADA, NADA, NADA};
	byte paket = B00000000;
	MODBUS_TCP::build_read_discrete_packet(slave,function,reg_count,packet);

	for (i = start_addr; i < (start_addr + (unsigned int) reg_count); i++) 
	{
		
		paket = paket + INTEGRIDAD[i];
		
		
		//                packet[packet_size] = regs[i] >> 8;
		//                packet_size++;
		//                packet[packet_size] = regs[i] & 0x00FF;
		//                packet_size++;
	}
	int Inte=int(paket);
	packet[packet_size] = Inte ;
	packet_size++;
	//-----------------------------------------------------
	//MODIFICACION:
	//asignacion de length en cabecera tcp

	packet[LENGTH_H] = (packet_size-6) >> 8;
	packet[LENGTH_L] = (packet_size-6) & 0x00FF;
	//-----------------------------------------------------
	status = MODBUS_TCP::send_reply(packet,packet_size);

	return (status);
}


int MODBUS_TCP::read_holding_registers(unsigned char slave, unsigned int start_addr,unsigned char reg_count, int *regs)
{
	unsigned char functions = FC_READ_REGS; 	/* Function 03: Read Holding Registers */
	int packet_size = 9;
	int status;
	unsigned int i, _start_addr = start_addr;
	int exception=1;
	unsigned char packet[MAX_MESSAGE_LENGTH], _reg_count = reg_count;
	unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
	DVJA UCR(&Serial3);
	//---------------------------------------------------------
	// en desarrollo
	//funcion que generara la consulta dvja
	
	for (i = _start_addr; i < (_start_addr + (unsigned int) _reg_count); i++)
	{

		if( (i == 5) || ( i == 6) || (i == 7))
		{
			//nada
			exception = 0;
			corta = 1;
		}
		else
		{
			UCR.clear_buffer();
			UCR.lectura_eeprom(i);
			sincrona = 1;
			timeout = 0;
			exception = 1;
			segundos = 0;
			corta = 1;
			confirmacion = 0;
			
			while ((exception == 1) && (timeout == 0 ) )
			{
				//while (exception == 1) {
				//--------------------------------------------------------------
				//configuracion timer 1
				cli();//deshabilita las interrupciones globales
				TCCR1A=0;
				TCCR1B=0;
				OCR1A=15624;//valor a comparar para timer de 1 segundo
				//15624 62534
				//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
				TCCR1B |= (1<<WGM12);
				TCCR1B |= (1<<CS12);
				TCCR1B |= (1<<CS10);
				TIMSK1=(1<<OCIE1A); // flag comparacion
				sei(); //habilita las interrupciones globales
				
				//-------------------------------------------------------------------
				exception = UCR.update(Origen,Destino,i); //0
				
				//si corta es diferente de 1 entonces la trama no es corta (es larga )
				//si la trama no es corta debe responder error 4
				if(corta != 1)
				{
					//corta =0;
					exception = -1;
				}
				if(confirmacion != 0)
				{
					exception = -1;
				}
				
			}
			
			//error consulta multiple
			if(exception != 0)
			{
				i = _start_addr + (unsigned int) _reg_count ;
			}
			if ((_reg_count != 1) && (i < (_start_addr + (unsigned int) _reg_count)))
			{
				wdt_reset();
				delay(2000);
			}

		}
	}
	//--------------------------------------------------------------
	//configuracion timer 1 	 detener
	cli();//deshabilita las interrupciones globales
	TCCR1A=0;
	TCCR1B=0;
	OCR1A=15624;//valor a comparar para timer de 1 segundo
	//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
	TCCR1B |= (1<<WGM12);
	//timer/counter stopped csn0,1,2 = 0
	TCCR1B |= (0<<CS10);
	TCCR1B |= (0<<CS11);
	TCCR1B |= (0<<CS12);
	//TIMSK1=(1<<TOIE1); // flag sobreflujo
	TIMSK1=(1<<OCIE1A); // flag comparacion
	sei(); //habilita las interrupciones globales
	//-------------------------------------------------------------------
	if (exception)
	{
		MODBUS_TCP::build_error_packet(slave,functions,4,errpacket);
		MODBUS_TCP::send_reply(errpacket,EXCEPTION_SIZE);
		return (exception);
	}
	segundos = 0;
	sincrona = 0;
	//----------------------------------------------------------
	MODBUS_TCP::build_Read_packet(slave,functions,_reg_count,packet);

	for (i = _start_addr; i < (_start_addr + (unsigned int) _reg_count);
	i++) 
	{
		if( i == 5){
			
			packet[packet_size] = tipo_estacion >> 8;
			packet_size++;
			packet[packet_size] = tipo_estacion & 0x00ff;
			packet_size++;
			
		}
		else if( i == 6){
			
			packet[packet_size] = Destino >> 8;
			packet_size++;
			packet[packet_size] = Destino & 0x00ff;
			packet_size++;
			
		}
		else if( i == 7){
			
			packet[packet_size] = Origen >> 8;
			packet_size++;
			packet[packet_size] = Origen & 0x00ff;
			packet_size++;
			
		}

		else {
			packet[packet_size] = holding_registers[i] >> 8;
			packet_size++; 
			packet[packet_size] = holding_registers[i] & 0x00FF;
			packet_size++;
		}
	}
	
	 //-------------------------------------------------
	 //MODIFICACION:
	 //asignacion de length en cabecera tcp
	 
	 packet[LENGTH_H] = (packet_size-6) >> 8;
	 packet[LENGTH_L] = (packet_size-6) & 0x00FF;
	 //return packet_size;
	 //-------------------------------------------------
	status = MODBUS_TCP::send_reply(packet,packet_size);
	return (status);
}


int MODBUS_TCP::read_input_registers(unsigned char slave, unsigned int start_addr,unsigned char reg_count, int *regs)
{
	unsigned char functions = FC_READ_INPUT; 	/* Function 04: Read Input Registers */
	int packet_size = 9;
	int status;
	int exception = 1;
	unsigned int i;
	DVJA UCR(&Serial3);
	unsigned char packet[MAX_MESSAGE_LENGTH];
	unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
	//---------------------------------------------------------
	UCR.clear_buffer();
	UCR.consulta();
	
	sincrona = 1;
	timeout = 0;
	
	while ((exception == 1) && (timeout == 0 ) ) 
	{
		//while (exception == 1) {
		//--------------------------------------------------------------
		//configuracion timer 1
		cli();//deshabilita las interrupciones globales
		TCCR1A=0;
		TCCR1B=0;
		OCR1A=15624;//valor a comparar para timer de 1 segundo
		//15624 62534
		//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
		TCCR1B |= (1<<WGM12);
		TCCR1B |= (1<<CS12);
		TCCR1B |= (1<<CS10);
		TIMSK1=(1<<OCIE1A); // flag comparacion
		sei(); //habilita las interrupciones globales
		//UCR.clear_buffer(&Serial3);
		//-------------------------------------------------------------------
			exception = UCR.update(Origen,Destino,start_addr);
		
	}
	//--------------------------------------------------------------
	//configuracion timer 1 	 detener
	cli();//deshabilita las interrupciones globales
	TCCR1A=0;
	TCCR1B=0;
	OCR1A=15624;//valor a comparar para timer de 1 segundo
	//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
	TCCR1B |= (1<<WGM12);
	//timer/counter stopped csn0,1,2 = 0
	TCCR1B |= (0<<CS10);
	TCCR1B |= (0<<CS11);
	TCCR1B |= (0<<CS12);
	//TIMSK1=(1<<TOIE1); // flag sobreflujo
	TIMSK1=(1<<OCIE1A); // flag comparacion
	sei(); //habilita las interrupciones globales
	//-------------------------------------------------------------------
	segundos = 0;
	//si corta es igual a 1 entonces la trama es corta
	//si la trama es corta debe responder error 4
	
	if(corta == 1)
	{
		corta = 0;
		exception = 1;
	}
	//return exception;
	if (exception) 
	{
		MODBUS_TCP::build_error_packet(slave,functions,4,errpacket);
		MODBUS_TCP::send_reply(errpacket,EXCEPTION_SIZE);
		return (exception);
	}
	
	sincrona = 0;
	//----------------------------------------------------------
	
	
	MODBUS_TCP::build_Read_packet(slave,functions,reg_count,packet);

	for (i = start_addr; i < (start_addr + (unsigned int) reg_count);
	i++) {
		packet[packet_size] = input_registers[i] >> 8;
		packet_size++;
		packet[packet_size] = input_registers[i] & 0x00FF;
		packet_size++;
	}
	 //-------------------------------------------------
	 //MODIFICACION:
	 //asignacion de length en cabecera tcp

	 packet[LENGTH_H] = (packet_size-6) >> 8;
	 packet[LENGTH_L] = (packet_size-6) & 0x00FF;
	 //-------------------------------------------------
	status = MODBUS_TCP::send_reply(packet,packet_size);
	
	return (status);
}


int MODBUS_TCP::preset_multiple_registers(unsigned char slave,unsigned int start_addr,unsigned char count,unsigned char *Trama1,int *regs)
{
	unsigned char function = FC_WRITE_REGS;	/* Preset Multiple Registers */
	int status = 0;
	int exception = 1;
	unsigned int i;
	DVJA UCR;
	unsigned char packet[RESPONSE_SIZE + CHECKSUM_SIZE];
	unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
	
	MODBUS_TCP::write_regs(start_addr, Trama1, regs);
	
	//---------------------------------------------------------
	// en desarrollo
	//funcion que generara la consulta dvja
	
	for (i = 0; i < Trama1[REGS_L]; i++){
		
		unsigned int addr= start_addr + i;
		UCR.clear_buffer();
		if(addr == 5){
			
			if ((regs[addr] >= 0) && (regs[addr] <= 5)){
				exception = 0;
				corta = 1 ;
				confirmacion = 1;
				EEPROM.write(2, regs[addr]);
				tipo_estacion = EEPROM.read(2);
			}
			else {
				exception = 1;
				corta = 1 ;
			}
			
		}
		else {
			//se debe hacer esta comprobacion antes para evitar problemas por las consultas multiples
			//debido a los cambios de id estacion e id maestra en el gateway
			if (cambio_Destino == 1){
				cambio_Destino = 0;
				
				
				
				EEPROM.write(0, regs[0]);
				
				Destino = EEPROM.read(0); //ID_esta
				//Origen = EEPROM.read(1); //ID_maestra
				
			}
			
			if ( cambio_Origen == 1){
				
				cambio_Origen = 0;
				
				
				EEPROM.write(1, regs[1]);
				
				//Destino = EEPROM.read(0); //ID_esta
				Origen = EEPROM.read(1); //ID_maestra
				
			}
			//----------------------------------------------------------
			
			UCR.escritura_eeprom(addr,regs[(start_addr) + i]);
			
			sincrona = 1;
			timeout = 0;
			exception = 1;
			segundos = 0;
			confirmacion = 1;
			corta = 1;
			
			
			while ((exception == 1) && (timeout == 0 ) ) {
				//while (exception == 1) {
				//--------------------------------------------------------------
				//configuracion timer 1
				cli();//deshabilita las interrupciones globales
				TCCR1A=0;
				TCCR1B=0;
				OCR1A=15624;//valor a comparar para timer de 1 segundo
				//15624 62534
				//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
				TCCR1B |= (1<<WGM12);
				TCCR1B |= (1<<CS12);
				TCCR1B |= (1<<CS10);
				TIMSK1=(1<<OCIE1A); // flag comparacion
				sei(); //habilita las interrupciones globales
				
				//-------------------------------------------------------------------
				exception = UCR.update(Origen,Destino,i);
				
				//si corta es diferente de 1 entonces la trama no es corta (es larga )
				//si la trama no es corta debe responder error 4
				if(corta != 1){
					//corta =0;
					exception = -1;
				}
				if(confirmacion != 1){
					exception = -1;
				}

			}
			
			//error consulta multiple
			if(exception != 0)
			{
				i = Trama1[REGS_L] ;
			}
			
			if ((Trama1[REGS_L] != 1) && (i<(Trama1[REGS_L]-1))){
				wdt_reset();
				delay(2000);
			}

		}
		
		
	}
	//--------------------------------------------------------------
	//configuracion timer 1 	 detener
	cli();//deshabilita las interrupciones globales
	TCCR1A=0;
	TCCR1B=0;
	OCR1A=15624;//valor a comparar para timer de 1 segundo
	//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
	TCCR1B |= (1<<WGM12);
	//timer/counter stopped csn0,1,2 = 0
	TCCR1B |= (0<<CS10);
	TCCR1B |= (0<<CS11);
	TCCR1B |= (0<<CS12);
	//TIMSK1=(1<<TOIE1); // flag sobreflujo
	TIMSK1=(1<<OCIE1A); // flag comparacion
	sei(); //habilita las interrupciones globales
	//-------------------------------------------------------------------
	segundos = 0;
	
	if (exception) {
		MODBUS_TCP::build_error_packet(slave,function,4,errpacket);
		MODBUS_TCP::send_reply(errpacket,EXCEPTION_SIZE);
		return (exception);
	}
	
	sincrona = 0;
	confirmacion = 0 ;

	
	MODBUS_TCP::build_Write_packet(slave, function, start_addr, count, packet);

		//-------------------------------------------------
		//MODIFICACION:
		//asignacion de length en cabecera tcp

		packet[LENGTH_H] = (RESPONSE_SIZE-6) >> 8;
		packet[LENGTH_L] = (RESPONSE_SIZE-6) & 0x00FF;
		//-------------------------------------------------
	status = MODBUS_TCP::send_reply(packet,RESPONSE_SIZE);
	//status = send_reply(packet, RESPONSE_SIZE);
	//}

	return (status);
}

int MODBUS_TCP::write_single_register(unsigned char slave,unsigned int write_addr, unsigned char *Trama1, int *regs)
{
	 unsigned char function = FC_WRITE_REG; /* Function: Write Single Register */
        int status = 0;
		int exception = 1;
        unsigned int reg_val;
		DVJA UCR;
        unsigned char packet[RESPONSE_SIZE + CHECKSUM_SIZE];
		unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
 //--------------------------------------------------------- 
        // en desarrollo      
        //funcion que generara la consulta dvja (no se ha hecho en tcp)
	    reg_val = Trama1[REGS_H] << 8 | Trama1[REGS_L];
		UCR.clear_buffer();
	    if(write_addr == 5){
			 
		  if ((reg_val >= 0) && (reg_val <= 5)){
		     exception = 0;
             corta = 1 ;
			 confirmacion = 1;
		     EEPROM.write(2, reg_val);
		     tipo_estacion = EEPROM.read(2);
		   }
		  else {
			exception = 1;
			corta = 1 ;
		    }
		
		 }
	     else
		 {
			 UCR.escritura_eeprom(write_addr,reg_val);
		
	   
		
		sincrona = 1;
		timeout = 0;
		 
	     while ((exception == 1) && (timeout == 0 ) ) {  
	   //while (exception == 1) {   
	
	//--------------------------------------------------------------	
	       //configuracion timer 1 	 
           cli();//deshabilita las interrupciones globales
	       TCCR1A=0;
	       TCCR1B=0;
	       OCR1A=15624;//valor a comparar para timer de 1 segundo
		   //15624 62534
		   //-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
	       TCCR1B |= (1<<WGM12);
	       TCCR1B |= (1<<CS12);
		   TCCR1B |= (1<<CS10);
		   TIMSK1=(1<<OCIE1A); // flag comparacion
	       sei(); //habilita las interrupciones globales 
		   
	//-------------------------------------------------------------------	   
		   //para tcp recordar cambiar parametros de la funcion *undate_dvja_slave
		   //ademas de las declaraciones globales de Origen , Destino , star_addr, sincrona , timeout (no se ha hecho en tcp)
		   //recordar cambiar los return de la funcion * para que solo devuelvan 0 al final de la misma (no se ha hecho en tcp)
		   // funcion de interrupcion por timer (no se ha hecho en tcp)
		   //recordar hacer el cambio de sincrona dentor de update_dvja_slave (no se ha hecho en tcp)
		   //recordar añadir el errpacket (no se ha hecho en tcp)
		   exception = UCR.update(Origen,Destino,write_addr);		 
		 }
	    }	 
//--------------------------------------------------------------	
	       //configuracion timer 1 	 detener
           cli();//deshabilita las interrupciones globales
	       TCCR1A=0;
	       TCCR1B=0;
	       OCR1A=15624;//valor a comparar para timer de 1 segundo
		   //-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
	       TCCR1B |= (1<<WGM12);
		   //timer/counter stopped csn0,1,2 = 0
	       TCCR1B |= (0<<CS10);
		   TCCR1B |= (0<<CS11);
	       TCCR1B |= (0<<CS12);
		   //TIMSK1=(1<<TOIE1); // flag sobreflujo
	       TIMSK1=(1<<OCIE1A); // flag comparacion
	       sei(); //habilita las interrupciones globales 
		   
//-------------------------------------------------------------------
		 segundos = 0;
		 //si corta es diferente de 1 entonces la trama no es corta (es larga )
		 //si la trama no es corta debe responder error 4
		 if(corta != 1){
			 corta =0;
			 exception = 1;
		 }
                 if(confirmacion != 1){
			 exception = 1;
		 }
		 if (exception) {
						MODBUS_TCP::build_error_packet(slave,function,4,errpacket);
						MODBUS_TCP::send_reply(errpacket,EXCEPTION_SIZE);
                        return (exception);
         }
		 
		 sincrona = 0;
		 confirmacion = 0;
		 
		  if (cambio_Destino == 1){
			 cambio_Destino = 0; 

		     EEPROM.write(0, reg_val);
		
			 Destino = EEPROM.read(0); //ID_esta
		     //Origen = EEPROM.read(1); //ID_maestra
			  
		  }
		  
		  if ( cambio_Origen == 1){
			
			 cambio_Origen = 0; 
			 
  	
		     EEPROM.write(1, reg_val);
		
			 //Destino = EEPROM.read(0); //ID_esta
		     Origen = EEPROM.read(1); //ID_maestra
			  
		  }
 //---------------------------------------------------------- 

        reg_val = Trama1[REGS_H] << 8 | Trama1[REGS_L];
		MODBUS_TCP::build_write_single_packet(slave,function,write_addr,reg_val,packet);
        //holding_registers[write_addr] = (int) reg_val;
/*
        written.start_addr=write_addr;
        written.num_regs=1;
*/		
		//-------------------------------------------------
		//MODIFICACION:
		//asignacion de length en cabecera tcp

		packet[LENGTH_H] = (RESPONSE_SIZE-6) >> 8;
		packet[LENGTH_L] = (RESPONSE_SIZE-6) & 0x00FF;
		//-------------------------------------------------
		status = MODBUS_TCP::send_reply(packet,RESPONSE_SIZE);

        return (status);
}

/* -------------------------------------------------------------------------------------------------*/
/*----------------------------------Funciones de usuario para MODBUS.-------------------------------*/
/* -------------------------------------------------------------------------------------------------*/
int MODBUS_TCP::Usuar_function(unsigned char slave,unsigned int write_addr, unsigned char *Trama1, int *regs)
{  
	unsigned char function = FC_USUAR_FUNT; 	/* Function 41: Funcion de usuario. */
	int packet_size = 8;
	int status, salida = 0;
	int exception = 1;
	unsigned int i;
	DVJA UCR;
	unsigned char packet[MAX_MESSAGE_LENGTH];
	unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
	unsigned int crc_calc = 0;
	unsigned int crc_received = 0;
	unsigned char recv_crc_hi;
	unsigned char recv_crc_lo;

	//---------------------------------------------------------
	UCR.clear_buffer();
	UCR.consulta_fc_usuario(Trama1,2);
	sincrona = 1;
	timeout = 0;
	
	
	do 
	{
		//--------------------------------------------------------------
		//configuracion timer 1
		cli();//deshabilita las interrupciones globales
		TCCR1A=0;
		TCCR1B=0;
		OCR1A=15624;//valor a comparar para timer de 1 segundo
		//15624 62534
		//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
		TCCR1B |= (1<<WGM12);
		TCCR1B |= (1<<CS12);
		TCCR1B |= (1<<CS10);
		TIMSK1=(1<<OCIE1A); // flag comparacion
		sei(); //habilita las interrupciones globales
		
		//-------------------------------------------------------------------
		exception = UCR.update(Origen,Destino,start_addr);
		/*Criterio para re envio de la trama de consulta para asegurarse de que llegue a la UCR
		  Unicamente implementado para esta funcion*/
		while (segundos == 3 && salida == 0)
		{
			salida = UCR.consulta_fc_usuario(Trama1,2);
		}
		if (timeout != 0)
		{
			break;
		}
	} while ((exception == 1));
	
	//--------------------------------------------------------------
	//configuracion timer 1 	 detener
	cli();//deshabilita las interrupciones globales
	TCCR1A=0;
	TCCR1B=0;
	OCR1A=15624;//valor a comparar para timer de 1 segundo
	//-bits CSn0,CSn1,CSn2 usados para programar el tiempo del timer
	TCCR1B |= (1<<WGM12);
	//timer/counter stopped csn0,1,2 = 0
	TCCR1B |= (0<<CS10);
	TCCR1B |= (0<<CS11);
	TCCR1B |= (0<<CS12);
	//TIMSK1=(1<<TOIE1); // flag sobreflujo
	TIMSK1=(1<<OCIE1A); // flag comparacion
	sei(); //habilita las interrupciones globales
	
	//-------------------------------------------------------------------
	segundos = 0;
	
	if (exception) {
		MODBUS_TCP::build_error_packet(slave,function,4,errpacket);
		MODBUS_TCP::send_reply(errpacket,EXCEPTION_SIZE);
		return (exception);
	}
	
	sincrona = 0;
	//----------------------------------------------------------
	if(corta == 1)
	{
		corta == 0;
		MODBUS_TCP::build_User_packet(slave,function,packet);
		
		packet[packet_size + CABECERA_H] = D_;
		packet[packet_size + CABECERA_L] = V_;
		packet[packet_size + ID_DESTINO] = Destino;
		packet[packet_size + No_DATOS] = 9;
		packet[packet_size + ID_ORIGEN] = Origen;
		
		packet_size += 5; // queda en 7
		
		for (i = 0; i < 3; i++)
		{
			packet[packet_size] = query2[5 + i] & 0x00FF;
			packet_size++;
		}
		crc_calc = UCR.calc_crc(packet, 10, packet_size);
		//cambio de 4 por 3 ojo
		recv_crc_hi = crc_calc >> 8;
		recv_crc_lo = crc_calc & 0x00FF;
		
		packet[packet_size] = recv_crc_lo;
		packet_size++;
		packet[packet_size] = recv_crc_hi;
		packet_size++;
		
		packet[packet_size] = J_;
		packet_size++;
		packet[packet_size] = A_;
		packet_size++;
		
		//-----------------------------------------------------
		//MODIFICACION:
		//asignacion de length en cabecera tcp

		packet[LENGTH_H] = (packet_size-6) >> 8;
		packet[LENGTH_L] = (packet_size-6) & 0x00FF;
		//-----------------------------------------------------
		
		status = MODBUS_TCP::send_reply(packet,packet_size);
		return (status);
		
	}
	
	MODBUS_TCP::build_User_packet(slave,function,packet);
	
	packet[packet_size + CABECERA_H] = D_;
	packet[packet_size + CABECERA_L] = V_;
	packet[packet_size + ID_DESTINO] = Destino;
	packet[packet_size + No_DATOS] = 30;
	packet[packet_size + ID_ORIGEN] = Origen;
	//packet[packet_size + FIN_TRAMA_H] = J_;
	//packet[packet_size + FIN_TRAMA_L] = A_;
	
	packet_size += 5; // queda en 7
	packet[packet_size] = 1;
	packet_size++;
	packet[packet_size] = Integri;
	packet_size++;

	for (i = 0; i < 12; i++)
	{
		
		if ((i == 1) || (i == 4))
		{
			packet[packet_size] = input_registers[i] & 0x00FF;
			packet_size++;
		}
		else {
			
			packet[packet_size] = input_registers[i] & 0x00FF;
			packet_size++;
			packet[packet_size] = input_registers[i] >> 8;
			packet_size++;
		}
	}
	crc_calc = UCR.calc_crc(packet, 10, packet_size);
	//cambio de 4 por 3 ojo
	recv_crc_hi = crc_calc >> 8;
	recv_crc_lo = crc_calc & 0x00FF;
	
	packet[packet_size] = recv_crc_lo;
	packet_size++;
	packet[packet_size] = recv_crc_hi;
	packet_size++;
	packet[packet_size] = J_;
	packet_size++;
	packet[packet_size] = A_;
	packet_size++;
	
	//-----------------------------------------------------
	//MODIFICACION:
	//asignacion de length en cabecera tcp

	packet[LENGTH_H] = (packet_size-6) >> 8;
	packet[LENGTH_L] = (packet_size-6) & 0x00FF;
	//-----------------------------------------------------
	
	status = MODBUS_TCP::send_reply(packet,packet_size);
	//status = send_reply(packet, packet_size);
	return (status);
}

int MODBUS_TCP::usuar_function_inicia(unsigned char slave,unsigned char *Trama1, int *regs)
{
	unsigned char function = FC_USUAR_INC; 	/* Function 03: Read Holding Registers */
	int packet_size = 8;
	int status;
	int exception = 1;
	unsigned int i;
	DVJA UCR;
	unsigned char packet[MAX_MESSAGE_LENGTH];
	//unsigned char errpacket[EXCEPTION_SIZE + CHECKSUM_SIZE];
	UCR.clear_buffer();
	//----------------------------------------------------------
	for (i = 0; i < 3; i++)
	{	
		EEPROM.write(i, Trama1[FUNC + (1 + i)]);
	}
	
	Destino = EEPROM.read(0); //ID_esta
	Origen = EEPROM.read(1); //ID_maestra
	tipo_estacion = EEPROM.read(2);
	MODBUS_TCP::build_User_packet(slave,function,packet);
	
	packet[packet_size] = Destino & 0x00FF;
	packet_size++;
	packet[packet_size] = Origen & 0x00FF;
	packet_size++;
	packet[packet_size] = tipo_estacion & 0x00FF;
	packet_size++;
	//-----------------------------------------------------
	//MODIFICACION:
	//asignacion de length en cabecera tcp
	packet[LENGTH_H] = (packet_size-6) >> 8;
	packet[LENGTH_L] = (packet_size-6) & 0x00FF;
	//-----------------------------------------------------
	status = MODBUS_TCP::send_reply(packet,packet_size);
	return (status);
}

/* -------------------------------------------------------------------------------------------------*/
/*----------------------Rutinas auxiliares para el desarrollo del programa.-------------------------*/
/* -------------------------------------------------------------------------------------------------*/


int MODBUS_TCP::receive_Request(unsigned char *Trama1)
{
	 int bytes_received = 0;
	 int longitud = 260; // se usa para tener una referencia del tamaño maximo de la trama.
	 /* FIXME: does Serial.available wait 1.5T or 3.5T before exiting the loop? */
	 while (((*usedPort).available()) && (bytes_received < longitud)) 
	 {
		 Trama1[bytes_received] = (*usedPort).read();
		 if (bytes_received == 5)
		 {
			 longitud = ((int) Trama1[bytes_received - 1] << 8) + (int) Trama1[bytes_received];
			 longitud = longitud + 6 ;
		 }
		 bytes_received++;
		 if (bytes_received >= MAX_MESSAGE_LENGTH)
			return NO_REPLY; 	/* port error */
	 }
	 return (bytes_received);
}

void MODBUS_TCP::build_read_discrete_packet(unsigned char slave, unsigned char functions,unsigned char count, unsigned char *packet)
{
	//packet[SLAVE] = slave;
	packet[TRANS_ID_H] = TID_H;
	packet[TRANS_ID_L] = TID_L;
	packet[PROT_ID_H] = PID_H;
	packet[PROT_ID_L] = PID_L;
	packet[LENGTH_H];
	packet[LENGTH_L];
	packet[UNIT_ID] = UID;
	packet[FUNC] = functions;
	count = count / 8 ;
	if(count == 0){
		packet[8] =  1 ;
	}
	else{
		packet[8] =  1;
	}
}


void MODBUS_TCP::build_error_packet(unsigned char slave, unsigned char functions,unsigned char exception, unsigned char *packet)
{
	  //-------------------------------------------------
	  //MODIFICACION:
	  //ADU paquete de error modbus tpc

	  //packet[SLAVE] = slave;
	  packet[TRANS_ID_H] = TID_H;
	  packet[TRANS_ID_L] = TID_L;
	  packet[PROT_ID_H] = PID_H;
	  packet[PROT_ID_L] = PID_L;
	  packet[LENGTH_H] = 0x00;
	  packet[LENGTH_L] = 0x03;
	  packet[UNIT_ID] = UID;
	  packet[FUNC] = functions + 0x80;
	  packet[8] = exception;
	  //-------------------------------------------------
}


int MODBUS_TCP::send_reply(unsigned char *Trama1, unsigned char string_length)
{
	unsigned char i;

	if (_txEnable > 1) 
	{ // set MAX485 to speak mode
		UCSR0A=UCSR0A |(1 << TXC0);
		digitalWrite( _txEnable, HIGH);
		delay(1);
	}

	for (i = 0; i < string_length; i++) 
	{
		(*usedPort).write(Trama1[i]);
	}

	if (_txEnable > 1) {// set MAX485 to listen mode
		while (!(UCSR0A & (1 << TXC0)));
		digitalWrite( _txEnable, LOW);
	}

	return i; 		/* it does not mean that the write was succesful, though */
}


void MODBUS_TCP::reply(unsigned char *Trama1, unsigned char string_length)
{
	int temp_crc;
	temp_crc = MODBUS_TCP::calc_crc(Trama1,0,string_length);
	Trama1[string_length] = temp_crc >> 8;
	string_length++;
	Trama1[string_length] = temp_crc & 0x00FF;
}


unsigned int MODBUS_TCP::calc_crc(unsigned char *Trama1, unsigned char start,unsigned char cnt)
{
	unsigned char i, j;
	unsigned temp, temp2, flag;

	temp = 0xFFFF;

	for (i = start; i < cnt; i++) 
	{
		temp = temp ^ Trama1[i];

		for (j = 1; j <= 8; j++) 
		{
			flag = temp & 0x0001;
			temp = temp >> 1;
			if (flag)
			temp = temp ^ 0xA001;
		}
	}

	/* Reverse byte order. */
	temp2 = temp >> 8;
	temp = (temp << 8) | temp2;
	temp &= 0xFFFF;

	return (temp);
}


int MODBUS_TCP::request(unsigned char slave,unsigned char *Trama1)
{
	DVJA UCR1(&Serial3);
	int response_length;
	unsigned int crc_calc = 0;
	unsigned int crc_received = 0;
	unsigned char recv_crc_hi;
	unsigned char recv_crc_lo;
	response_length = MODBUS_TCP::receive_Request(Trama1);
	//--------------------------------------------------------------
	/* check for slave id */
	if (slave != Trama1[UNIT_ID]) 
	{
		return NO_REPLY;
	}
	//}
	//-------------------------------------------------
	//-------------------------------------------------
	//MODIFICACION:
	TID_H = Trama1[TRANS_ID_H];
	TID_L = Trama1[TRANS_ID_L];
	PID_H = Trama1[PROT_ID_H];
	PID_L = Trama1[PROT_ID_L];
	UID = Trama1[UNIT_ID];

	//-----------------------------------------------------------------
	return (response_length);
}


int MODBUS_TCP::validate_request(unsigned char *Trama1, unsigned char length,unsigned int regs_size)
{
	int i, fcnt = 0;
	unsigned int regs_num = 0;
	unsigned int start_addr = 0;
	unsigned char max_regs_num;

	/* check function code */
	for (i = 0; i < sizeof(fsupported); i++) {
		if (fsupported[i] == Trama1[FUNC]) {
			fcnt = 1;
			break;
		}
	}
	if (0 == fcnt)
		return EXC_FUNC_CODE;

	if (FC_WRITE_REG == Trama1[FUNC]) {
		/* For function write single reg, this is the target reg.*/
		regs_num = ((int) Trama1[START_H] << 8) + (int) Trama1[START_L];
		if ((regs_num >= 6)|| (regs_num < 0))
		return EXC_ADDR_RANGE;
		return 0;
	}
	
	
	if (FC_USUAR_FUNT == Trama1[FUNC]) {
		/* For function write single reg, this is the target reg.*/
		
		return 0;
	}
	
	
	if (FC_USUAR_INC == Trama1[FUNC]) {
		/* For function write single reg, this is the target reg.*/
		
		return 0;
	}
	
	/* For functions read/write regs, this is the range. */
	regs_num = ((int) Trama1[REGS_H] << 8) + (int) Trama1[REGS_L];
	
	/* check quantity of registers */
	if (FC_READ_REGS == Trama1[FUNC])
	max_regs_num = MAX_READ_REGS;
	//funcion nueva
	else if (FC_READ_INPUT == Trama1[FUNC])
	max_regs_num = MAX_READ_INPUT;
	else if (FC_READ_DISC == Trama1[FUNC])
	max_regs_num = MAX_READ_DISC;
	else if (FC_WRITE_REGS == Trama1[FUNC])
	max_regs_num = MAX_WRITE_REGS;

	if ((regs_num < 1) || (regs_num > max_regs_num))
	return EXC_REGS_QUANT;

	/* check registers range, start address is 0 */
	
	start_addr = ((int) Trama1[START_H] << 8) + (int) Trama1[START_L];
	
	if (FC_READ_DISC == Trama1[FUNC]){
		int min_regs_addr =0;
		if (((start_addr + regs_num) > (max_regs_num )) || (start_addr < min_regs_addr)){
			return EXC_ADDR_RANGE;
		}
	}
	else if (FC_READ_INPUT == Trama1[FUNC]){
		int min_regs_addr =0;
		int max_regs_addr = 12;
		if (((start_addr + regs_num) > max_regs_addr) || (start_addr < min_regs_addr)){
			return EXC_ADDR_RANGE;
		}
	}
	else if (((start_addr + regs_num) > 8) || (start_addr < 0)){
		return EXC_ADDR_RANGE;
	}
	
	return 0; 		/* OK, no exception */
}

void MODBUS_TCP::build_Read_packet(unsigned char slave, unsigned char functions,unsigned char count, unsigned char *packet)
{
	//-------------------------------------------------
	//MODIFICACION:
	//ADU paquete de lectura modbus tpc

	//packet[SLAVE] = slave;
	packet[TRANS_ID_H] = TID_H;
	packet[TRANS_ID_L] = TID_L;
	packet[PROT_ID_H] = PID_H;
	packet[PROT_ID_L] = PID_L;
	packet[LENGTH_H];
	packet[LENGTH_L];
	packet[UNIT_ID] = UID;
	packet[FUNC] = functions;
	packet[8] = count * 2;
	
	//de 2 a 8 debido a la que la cabecera TCP es mas grande
	//-------------------------------------------------
}


void MODBUS_TCP::build_Write_packet(unsigned char slave, unsigned char functions,unsigned int start_addr,unsigned char count,unsigned char *packet)
{
	 //-------------------------------------------------
	 //MODIFICACION:
	 //ADU paquete de escritura  multiples registros modbus tpc
	 
	 //packet[SLAVE] = slave;
	 packet[TRANS_ID_H] = TID_H;
	 packet[TRANS_ID_L] = TID_L;
	 packet[PROT_ID_H] = PID_H;
	 packet[PROT_ID_L] = PID_L;
	 packet[LENGTH_H];
	 packet[LENGTH_L];
	 packet[UNIT_ID] = UID;
	 packet[FUNC] = functions;
	 packet[START_H] = start_addr >> 8;
	 packet[START_L] = start_addr & 0x00ff;
	 packet[REGS_H] = 0x00;
	 packet[REGS_L] = count;
	 //-------------------------------------------------
}

void MODBUS_TCP::build_User_packet(unsigned char slave, unsigned char functions,unsigned char *packet)
{
      //packet[SLAVE] = slave;
      packet[TRANS_ID_H] = TID_H;
      packet[TRANS_ID_L] = TID_L;
      packet[PROT_ID_H] = PID_H;
      packet[PROT_ID_L] = PID_L;
      packet[LENGTH_H];
      packet[LENGTH_L];
      packet[UNIT_ID] = UID;
      packet[FUNC] = functions;
      //packet[2] = count * 2;
}


void MODBUS_TCP::build_write_single_packet(unsigned char slave, unsigned char functions,unsigned int write_addr, unsigned int reg_val, unsigned char *packet)
{
	//-------------------------------------------------
	//MODIFICACION:
	//ADU paquete de escritura  registros modbus tpc

	//packet[SLAVE] = slave;
	packet[TRANS_ID_H] = TID_H;
	packet[TRANS_ID_L] = TID_L;
	packet[PROT_ID_H] = PID_H;
	packet[PROT_ID_L] = PID_L;
	packet[LENGTH_H];
	packet[LENGTH_L];
	packet[UNIT_ID] = UID;
	packet[FUNC] = functions;
	packet[START_H] = write_addr >> 8;
	packet[START_L] = write_addr & 0x00ff;
	packet[REGS_H] = reg_val >> 8;
	packet[REGS_L] = reg_val & 0x00ff;
	//-------------------------------------------------
}




int MODBUS_TCP::write_regs(unsigned int start_addr, unsigned char *Trama1, int *regs)
{
	 int temp;
	 unsigned int i;

	 for (i = 0; i < Trama1[REGS_L]; i++) {
		 /* shift reg hi_byte to temp */
		 temp = (int) Trama1[(BYTE_CNT + 1) + i * 2] << 8;
		 /* OR with lo_byte           */
		 temp = temp | (int) Trama1[(BYTE_CNT + 2) + i * 2];

		 //holding_registers[(start_addr) + i] = temp;
		 regs[(start_addr) + i] = temp;
	 }
	 return i;
}
