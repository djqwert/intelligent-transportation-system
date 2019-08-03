#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "dev/sht11/sht11-sensor.h"
#include "sys/etimer.h"
#include "net/rime/rime.h"
#include "stdio.h"

//#define COOJA
#define DEBUG

#ifdef COOJA
	#define G1_ADDR 		1 	
	#define G2_ADDR 		2	
	#define TL1_ADDR 		3   
	#define TL2_ADDR 		4   
#else
	#define G1_ADDR 		49 	// 49.0
	#define G2_ADDR 		158	// 158.0
	#define TL1_ADDR 		42  // 42.0
	#define TL2_ADDR 		21  // 21.0
#endif

#define bool 				char 
#define true 				1
#define false 				0
#define MAX_RETRANSMISSIONS 5

typedef struct {
	char type;
	int value;
} measurement_t;

typedef enum { NONE, NORMAL, EMERGENCY } vehicle_t;
typedef enum { DEFAULT, NOTIFY_VEHICLE, RESTORE_VEHICLE } state_t;

PROCESS(g2, "G2 SkyMote");
AUTOSTART_PROCESSES(&g2);

static state_t state = DEFAULT;

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	#ifdef DEBUG
		printf("DEBUG: runicast message received from %d.%d, seqno %d, msg: %s\n", from->u8[0], from->u8[1], seqno, (char *)packetbuf_dataptr());
	#endif
	state = RESTORE_VEHICLE;					// Ripristino lo stato del sensore quando questo riceve notifica dal TL
	process_post(&g2, PROCESS_EVENT_MSG, NULL);
}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	#ifdef DEBUG
		printf("DEBUG: runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
	#endif
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	printf("//// Timeout\n");
	state = RESTORE_VEHICLE;
	process_post(&g2, PROCESS_EVENT_MSG, NULL);
}

static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from){}

static void broadcast_sent(struct broadcast_conn *c, int status, int num_tx){
	#ifdef DEBUG
		printf("DEBUG: Sent measurements to Sink\n");
	#endif
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv, broadcast_sent}; 
static struct broadcast_conn broadcast;

PROCESS_THREAD(g2, ev, data){

	static struct etimer double_press_timer, waiting_notify_timer, sensing_timer;

	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_EXITHANDLER(broadcast_close(&broadcast));

	PROCESS_BEGIN();

	static bool tl_notified = false;
	static bool etimer_active = false;
	static char message[2];
	static int temperature = 0, humidity = 0;
	static measurement_t sensing;
	static vehicle_t vehicle = NONE;
	static linkaddr_t recv;

	recv.u8[0] = TL2_ADDR;
	recv.u8[1] = 0;

	etimer_set(&sensing_timer, CLOCK_SECOND * 5);

	runicast_open(&runicast, 144, &runicast_calls);
	broadcast_open(&broadcast, 150, &broadcast_call);
	SENSORS_ACTIVATE(button_sensor);

	while(1){
		// EVENTI:
		//	- sensing timer scaduto
		//	- bottone
		//	- timer scaduto
		//	- ricezione msg
		PROCESS_WAIT_EVENT();

		// Sensing e broadcast
		if(etimer_expired(&sensing_timer)){

			SENSORS_ACTIVATE(sht11_sensor);
			sensing.type = 'T';
			sensing.value = (sht11_sensor.value(SHT11_SENSOR_TEMP)/10 - 396)/10;
			temperature = sensing.value;
			packetbuf_copyfrom(&sensing, sizeof(sensing));
			broadcast_send(&broadcast);
			
			sensing.type = 'H';
			humidity = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
			// Fix umidità
			sensing.value = -4 + 0.0405 * humidity + (-2.8 * 0.000001) * (humidity * humidity);
			sensing.value = (temperature - 25) * (0.01 + 0.00008 * humidity) + sensing.value;
			packetbuf_copyfrom(&sensing, sizeof(sensing));
			broadcast_send(&broadcast);
			SENSORS_DEACTIVATE(sht11_sensor);

			etimer_reset(&sensing_timer);
			continue;

		}

		// Eventi legati al bottone
		if(state == DEFAULT && ev == sensors_event && data == &button_sensor){

			printf("STATO: DEFAULT\n");

			if(vehicle == NONE){
				leds_on(LEDS_RED);
				leds_off(LEDS_RED);
				vehicle = NORMAL;
				state = NOTIFY_VEHICLE;
				etimer_set(&double_press_timer, CLOCK_SECOND * 0.5);	
				continue;
		
			}

			if(vehicle == NORMAL && etimer_expired(&double_press_timer) == 0){
				leds_on(LEDS_RED);
				leds_off(LEDS_RED);
				vehicle = EMERGENCY;
				etimer_stop(&double_press_timer); // Fai scadere il timer..
				continue;

			}

		}

		// Timer scaduto per definire il veicolo: invio notifica al semaforo
		if(state == NOTIFY_VEHICLE && tl_notified == false){

			// Condizione necessaria nel caso il semaforo TL* abbia già ricevuto un veicolo da meno di 5 secondi.
			if(etimer_active == false || (etimer_active == true && etimer_expired(&waiting_notify_timer))){

				printf("STATO: NOTIFY_VEHICLE\n");

				tl_notified = true;
				SENSORS_DEACTIVATE(button_sensor);
				sprintf(message, "%d", vehicle);
				if(!runicast_is_transmitting(&runicast)) {
					packetbuf_copyfrom(message, 2);
					runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
				}

			}

			continue;

		}

		// Ricezione msg dal semaforo
		if(state == RESTORE_VEHICLE){ // Arriva l'okay dal semaforo

			printf("STATO: RESTORE_VEHICLE\n");

			state = DEFAULT;
			vehicle = NONE;
			tl_notified = false;
			etimer_active = true;
			etimer_set(&waiting_notify_timer, CLOCK_SECOND * 5);
			SENSORS_ACTIVATE(button_sensor);

		}

	}

	PROCESS_END();

}