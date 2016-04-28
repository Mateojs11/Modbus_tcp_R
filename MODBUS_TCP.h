/*  Se crea una clase que manejará las funciones de comunicación en radio del gateway.
	Estas funciones se encargarán de configurar y actualizar los datos*/

/*
Versión 1 de la libreria modbus para el manejo del gateway hibrido, sus funciones son las mismas que se 
implementaron en el gateway anterior, solo que se tendrá en cuenta que el Radio es un nuevo canal de respuesta.
*/

/*Se continua con el desarrollo de la libreria de modbus con las funciones de usuario para el manejo del Gateway 2.0; en la ultima semana
se trabajo en la homologación del codigo original al codigo que hace parte de la libreria. */

/*Se realizan pruebas de las funciones mas acogidas de modbus y se reporta el funcionamiento esperado. */

#ifndef MODBUS_TCP_h
#define MODBUS_TCP_h

#include "Arduino.h"
#include <DVJA.h>
#include <avr/io.h>
#include <avr/interrupt.h>

//-------------------------------------------------
//Modificacion
extern unsigned long Nowdt;
extern unsigned int lastBytesReceived;
extern const unsigned long T35;
extern volatile byte segundos;
extern volatile int tip_esta, pruebas;

//-------------------------------------------------------------------------------------
class MODBUS_TCP
{
	public:
	MODBUS_TCP(HardwareSerial *PuertoModbus = &Serial2);
	int config(unsigned long baudRate,unsigned char parity,unsigned int tXEnable);
	int update_mb(unsigned char slave, int *regs,unsigned int regs_size);
enum {
	//-------------------------------------------------
	//MODIFICACION:
	// el 6 por el 12 ya que aumenta el tamaño de la respuesta
	
	RESPONSE_SIZE = 12,
	EXCEPTION_SIZE = 9,
	CHECKSUM_SIZE = 2
	//-------------------------------------------------
	
};

	enum {
		NO_REPLY = -1,
		EXC_FUNC_CODE = 1,
		EXC_ADDR_RANGE = 2,
		EXC_REGS_QUANT = 3,
		EXC_EXECUTE = 4,
		EXC_ERR_DVJA = 5
	};


	enum {
		MAX_READ_DISC = 0x08,
		MAX_READ_REGS = 0x7D,
		MAX_READ_INPUT = 0x7D,
		MAX_WRITE_REGS = 0x7B,
		MAX_MESSAGE_LENGTH = 256
	};
	
	enum {
		D_ = 100,
		V_ = 118,
		ACUMULADO = 1,
		J_ = 106,
		A_ = 97,
		T_C = 12,
		T_L = 33
		
	};
	
	enum {
		CABECERA_H = 0,
		CABECERA_L,
		ID_DESTINO,
		No_DATOS,
		ID_ORIGEN,
		No_ACUM,
		INTEGRIDAD,
		FIN_TRAMA_H = 31,
		FIN_TRAMA_L = 32
	};
	
	unsigned char TID_H;
	unsigned char TID_L;
	unsigned char PID_H;
	unsigned char PID_L;
	unsigned char UID;
	
	private:
	unsigned char _parity,_txEnable;
	HardwareSerial *usedPort;
	unsigned long _baudRate;
	int receive_Request(unsigned char *Trama1);
	unsigned int calc_crc(unsigned char *Trama1, unsigned char start,unsigned char cnt);
	void reply(unsigned char *Trama1, unsigned char string_length);
	void build_error_packet(unsigned char slave, unsigned char functions,unsigned char exception, unsigned char *Trama1);
	int send_reply(unsigned char *Trama1, unsigned char string_length);
	void build_Write_packet(unsigned char slave, unsigned char functions,unsigned int start_addr,unsigned char count,unsigned char *Trama1);
	void build_Read_packet(unsigned char slave, unsigned char functions,unsigned char count, unsigned char *Trama1);
	int validate_request(unsigned char *Trama1, unsigned char length,unsigned int regs_size);
	void build_read_discrete_packet(unsigned char slave, unsigned char function,unsigned char count, unsigned char *Trama1);
	int request(unsigned char slave, unsigned char *Trama1);
	int usuar_function_inicia(unsigned char slave,unsigned char *Trama1, int *regs);
	void build_User_packet(unsigned char slave, unsigned char functions,unsigned char *Trama1);
	int read_discrete_inputs(unsigned char slave, unsigned int start_addr,unsigned char reg_count, int *regs);
	void build_write_single_packet(unsigned char slave, unsigned char functions,unsigned int write_addr, unsigned int reg_val, unsigned char* Trama1);
	int read_holding_registers(unsigned char slave, unsigned int start_addr,unsigned char reg_count, int *regs);
	int read_input_registers(unsigned char slave, unsigned int start_addr,unsigned char reg_count, int *regs);
	int preset_multiple_registers(unsigned char slave,unsigned int start_addr,unsigned char count,unsigned char *Trama1,int *regs);
	int write_regs(unsigned int start_addr, unsigned char *Trama1, int *regs);
	int write_single_register(unsigned char slave,unsigned int write_addr, unsigned char *query, int *regs);
	int Usuar_function(unsigned char slave,unsigned int write_addr, unsigned char *query, int *regs);
};

#endif