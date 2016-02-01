// Router & Receiver
#include "contiki.h"
#include "net/rime.h"
#include <string.h>
#include "dev/button-sensor.h"
#include "dev/sensinode-sensors.h"
#include "dev/leds.h"
#include <stdio.h>

#define TABLELENGTH        10
#define COMMAND_ROUTREQUEST   0x80   //Command for requesting route
#define COMMAND_ROUTERESPONSE 0x81   //Command for route response
#define COMMAND_DTATTX        0x82   //Command for sending data through unicast
#define COMMAND_ACK			  0x83	 //Command for sending ACK to sender
#define BATTERY_AVERGE_LVL    3000

// This is the backward table,
typedef struct
{
	uint16_t u16Dest;                   //The destation of sender
	uint16_t u16NextHop;                //The next hop which point to the destination address
	uint8_t brdcstID;
} rBackwardTable;
static rBackwardTable rBwTable[TABLELENGTH];

// This is the forward table
typedef struct
{
	uint16_t u16Dest;                   //The destation address
	uint16_t u16NextHop;                //The next hop which point to the destination address
	uint16_t u16Battery;
	uint16_t u16Rssi;
	uint8_t brdcstID;
} tsRouteTable;
static tsRouteTable sRouteTable[TABLELENGTH];

static rimeaddr_t addr;
static uint8_t destination;
static struct unicast_conn uc;
static struct broadcast_conn bc;
static uint8_t u8DataBuffer[50];

static const struct broadcast_callbacks broadcast_callbacks = {recv_bc};
static const struct unicast_callbacks unicast_callbacks = {recv_uc};


static int rv;
static struct sensors_sensor * sensor;
static float sane = 0;
static uint16_t battery;
static uint8_t temperature1=0;
static uint8_t temperature2=0;
static uint8_t brdcstCounter=1;
static uint8_t brdcstLimit=4;
static uint8_t brdcstID=1;
static int m=0;
static int i=0;
/*---------------------------------------------------------------------------*/
PROCESS(routenode_process, "Example unicast");
AUTOSTART_PROCESSES(&routenode_process);
/*---------------------------------------------------------------------------*/

// 1. Receive broadcast:
//		Create a backward table
//			{
//				1. where is the sender
//				2. where the broadcast sent from
//				3. Broadcast id
//			}
//		if the destination is current node, send unicast back to where the broadcast sent from
//		if current node is not the destination, continue braodcast
static void
recv_bc(struct broadcast_conn *c, const rimeaddr_t *from)
{
	uint8_t * data;
	uint16_t dest=0;
	uint16_t sender = 0;
	uint16_t nexthop=0;
	uint16_t src=0;
	unsigned int bFound=0;

	// Test use
	if (rimeaddr_node_addr.u8[0] == 0x12 && rimeaddr_node_addr.u8[1] == 0x34 && from->u8[0] == 0x45 && from->u8[1] == 0x45){
		packetbuf_clear();
	}
	else if (rimeaddr_node_addr.u8[0] == 0x45 && rimeaddr_node_addr.u8[1] == 0x45 && from->u8[0] == 0x12 && from->u8[1] == 0x34){
		packetbuf_clear();
	}
	else{

  	data = packetbuf_dataptr();

	switch (data[0]){
	case COMMAND_ROUTREQUEST:
		src = from->u8[0];
		src = src << 8;
		src = src | from->u8[1];
		//printf("test!!!!!!!!!!!!!!!!!!src:%x",src);

		sender = data[3];
		sender = sender << 8;
		sender = sender | data[4];

		brdcstID = data[5];
		brdcstCounter = data[6];

		//if the connection exists, already find the node
		for (m = 0; m < TABLELENGTH; m++){
			if (rBwTable[m].u16NextHop == src && rBwTable[m].u16Dest == sender){
				bFound = 1;
				if (brdcstID > rBwTable[m].brdcstID) 
				{
					rBwTable[m].brdcstID = brdcstID;
				}
				break;
			}
		}

		if (!bFound){
			for (m = 0; m < TABLELENGTH; m++){
				if (rBwTable[m].u16Dest == 0x0000){
					rBwTable[m].u16Dest = sender;
					rBwTable[m].u16NextHop = src;
					rBwTable[m].brdcstID = brdcstID;
					printf("Create the backward table -> \n Sender: %x, last hop: %x \r\n", rBwTable[m].u16Dest, rBwTable[m].u16NextHop);
					break;
				}
			}
		}
		printf("Backwrad table: %d %d %d\r\n", rBwTable[m].u16Dest, rBwTable[m].u16NextHop, rBwTable[m].brdcstID);

		// Route table update finished ---------------------------------- //

		// dest: the receiver destination
		dest = data[1];
		dest = dest << 8;
		dest = dest | data[2];

		//if current node is the destination, send back the addr of current node,
		// as well as which node i want to send
		if (rimeaddr_node_addr.u8[0] == data[1] && rimeaddr_node_addr.u8[1] == data[2])
		{
			u8DataBuffer[0] = COMMAND_ROUTERESPONSE;
			u8DataBuffer[1] = data[3];
			u8DataBuffer[2] = data[4];
			u8DataBuffer[3] = rimeaddr_node_addr.u8[0]; // this is the reveiver address, not just current address
			u8DataBuffer[4] = rimeaddr_node_addr.u8[1];
			// get the battery status
			sensor = sensors_find(ADC_SENSOR);
			rv = sensor->value(ADC_SENSOR_TYPE_VDD);
			if (rv != -1)
			{
				sane = rv*3.75 / 2047;
				battery = sane * 1000;
			}

			u8DataBuffer[5] = battery;
			u8DataBuffer[6] = battery >> 8;
			u8DataBuffer[7] = brdcstID;

			packetbuf_copyfrom(u8DataBuffer, 8);

			addr.u8[1] = rBwTable[m].u16NextHop;
			addr.u8[0] = rBwTable[m].u16NextHop >> 8;
			//printf("next %d%d",addr.u8[0],addr.u8[1]);
			print("broadcast id: %d; broadcast counter: %d\n", data[5], data[6])
			unicast_send(&uc, &addr);
			printf("Current node is the destination -> \n response sent to: %x, sender will be %x\r\n", rBwTable[m].u16NextHop, rBwTable[m].u16Dest);
		}
		else{
			//continue broadcast
			//printf("timer:%f",CLOCK_SECOND * rand()/(float)(RAND_MAX));
			if (brdcstCounter < brdcstLimit)
			{
				data[6]++;
				packetbuf_copyfrom(data, 7);
				print("broadcast id: %d; broadcast counter: %d\r\n", data[5], data[6])
				broadcast_send(&bc);
			}
			else 
			{
				printf("Broadcast extends limit !\r\n");
				packetbuf_clear();
			}
			
		}
		break;
	default:
		break;
	}
  	}
}


// 2.reveive unicast
//		1. COMMAND_ROUTERESPONSE
//			Create forward table. By searching the backward table, send the unicast back to last hop
//		2. COMMAND_DTATTX
//			Route the data sent from sender to receiver, if current node is the receiver, get the data and send ACK back to sender
//		3. COMMAND_ACK
//			Route the ACK to sender
static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
	uint8_t * data;
  	uint16_t dest=0;
  	uint16_t nexthop=0;
  	uint16_t src=0;
	uint16_t resend=0;
  	unsigned int bFound=0;
	uint16_t rssi = 0;

  	data = packetbuf_dataptr();

 		switch(data[0])
 		{
			// receiver will not get response
			case COMMAND_ROUTERESPONSE: // the response from braodcast
				// get the receiver's address
				dest = data[3];
				dest = dest << 8;
				dest = dest | data[4];

				//who send the response
				src = from->u8[0];
				src = src << 8;
				src = src | from->u8[1];

				// the address that router will re-send to
				resend = data[1];
				resend = resend << 8;
				resend = resend | data[2];

				// get the battery infromation of response
				battery = data[6];
				battery = battery << 8;
				battery = battery | data[5];

				brdcstID = data[7];

				printf("has respo %x%x\r\n", from->u8[0], from->u8[1], battery);
				rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI); //get the signal

				// find if the route table exists
				for (i = 0; i < TABLELENGTH; i++)
				{
					if (sRouteTable[i].u16Dest == dest)
					{
						bFound = 1;
						printf("Current router alreay has the route\r\n");
						sRouteTable[i].u16Rssi = rssi;
						sRouteTable[i].u16Battery = battery;
						if (brdcstID > sRouteTable[i].brdcstID)
						{
							sRouteTable[i].brdcstID = brdcstID;
						}
						break;
					}
				}

				if (!bFound)
				{
					for (i = 0; i<TABLELENGTH; i++)
					{
						if (sRouteTable[i].u16Dest == 0x0000)
						{
							printf("New route table %x, %x, %d, %d\r\n", dest, src, rssi, battery);
							sRouteTable[i].u16Dest = dest;
							sRouteTable[i].u16NextHop = src;
							sRouteTable[i].u16Rssi = rssi;
							sRouteTable[i].u16Battery = battery;
							sRouteTable[i].brdcstID = brdcstID;
							break;
						}
					}
				}
				printf("forward table: %d %d %d %d %d\r\n", sRouteTable[i].u16Dest, sRouteTable[i].u16NextHop, sRouteTable[i].u16Rssi, sRouteTable[i].u16Battery, sRouteTable[i].brdcstID);

				// get the battery information
				sensor = sensors_find(ADC_SENSOR);
				rv = sensor->value(ADC_SENSOR_TYPE_TEMP);
				if (rv != -1)
				{
					sane = rv * 3.75 / 2047;
					battery = sane * 1000;
				}
				//u8DataBuffer[5] = battery;
				//u8DataBuffer[6] = battery >> 8;
				data[5] = battery;
				data[6] = battery >> 8;

				//find the reverse route
				for (i = 0; i<TABLELENGTH; i++)
				{
					if (rBwTable[i].u16Dest == resend)
					{
						if (brdcstID > rBwTable[i].brdcstID)
						{
							rBwTable[i].brdcstID = brdcstID;
						}
						break;
					}
				}
				addr.u8[1] = rBwTable[i].u16NextHop;
				addr.u8[0] = rBwTable[i].u16NextHop >> 8;
				packetbuf_copyfrom(data, 8);

				printf("send data to %x%x\r\n", addr.u8[0], addr.u8[1]);
				unicast_send(&uc, &addr);
			break;

 			case COMMAND_DTATTX:
	 			dest = data[1];
	  			dest = dest << 8;
	  			dest = dest | data[2];

				if (rimeaddr_node_addr.u8[0] == data[1] && rimeaddr_node_addr.u8[1] == data[2] )
				{
           			temperature1 = data[5];
           			temperature2 = data[6];
					battery = data[7];
					battery = battery << 8;
					battery = battery | data[8];

					//printf("has respo %d%d\r\n",from->u8[0],from->u8[1],battery);
					printf("Data received. Battery level: %d Temperature 1: %d Temperature 2: %d\r\n",battery,temperature1,temperature2);
					printf("Send ACK back to sender ...\n");

					// Prepare ACK data
					u8DataBuffer[0] = COMMAND_ACK;
					u8DataBuffer[1] = data[3];
					u8DataBuffer[2] = data[4];
 					packetbuf_copyfrom(u8DataBuffer, 3);

					// Find the reverse table
					resend = data[3];
					resend = resend << 8;
					resend = resend | data[4];
					for (m = 0; m < TABLELENGTH; m++){
						if (rBwTable[m].u16Dest == resend){
							printf("ACK back route found-> \n Sender: %x, last hop: %x \r\n", rBwTable[m].u16Dest, rBwTable[m].u16NextHop);
							break;
						}
					}

					addr.u8[1] = rBwTable[m].u16NextHop;
					addr.u8[0] = rBwTable[m].u16NextHop >> 8;
					printf("next %d%d",addr.u8[0],addr.u8[1]);
					unicast_send(&uc, &addr);
				}
				else // Current node is not the destination, search the fowrad table
				{
					for (i = 0; i<TABLELENGTH; i++)
					{
						if (sRouteTable[i].u16Dest == dest)
						{
							break;
						}
					}
					addr.u8[1] = sRouteTable[i].u16NextHop;
					addr.u8[0] = sRouteTable[i].u16NextHop >> 8;
					packetbuf_copyfrom(data, 9);
					unicast_send(&uc, &addr);
				}
	 		break;
			case COMMAND_ACK: //receiver should not get COMMAND_ACK literally
				dest = data[1];
				dest = dest << 8;
				dest = dest | data[2];
				for (m = 0; m < TABLELENGTH; m++){
					if (rBwTable[m].u16Dest == dest){
						printf("ACK back route found-> \n Sender: %x, last hop: %x \r\n", rBwTable[m].u16Dest, rBwTable[m].u16NextHop);
						break;
					}
				}
				addr.u8[1] = sRouteTable[i].u16NextHop;
				addr.u8[0] = sRouteTable[i].u16NextHop >> 8;
				packetbuf_copyfrom(data, 3);
				unicast_send(&uc, &addr);
			break;
 			default:
 			break;
}
	packetbuf_clear();
}

// Main thread: initialize forward table as well as backward table
PROCESS_THREAD(routenode_process, ev, data)
{
	static struct etimer et;
	static uint8_t i=0;
	static uint8_t m=0;
	static int dec;
	static float frac;
	static uint16_t u16Dest=0x0000;

	PROCESS_EXITHANDLER(unicast_close(&uc);)
	PROCESS_BEGIN();

	// initialize the backward table
	for(i=0; i<TABLELENGTH; i++)
	{
		rBwTable[i].u16Dest=0x0000;
		rBwTable[i].u16NextHop=0xffff;
		rBwTable[i].brdcstID = 0;
	}

	// initialize the forward table
	for (i = 0; i < TABLELENGTH; i++)
	{
		sRouteTable[i].u16Dest = 0x0000;
		sRouteTable[i].u16NextHop = 0xffff;
		sRouteTable[i].u16Battery = 0;
		sRouteTable[i].u16Rssi = 0;
		sRouteTable[i].brdcstID = 0;
	}

	puts("start");
	broadcast_open(&bc, 128, &broadcast_callbacks);
	unicast_open(&uc, 129, &unicast_callbacks);
	etimer_set(&et, CLOCK_SECOND * 2);
	while (1)
	{
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		etimer_reset(&et);
	}

	PROCESS_END();
}
