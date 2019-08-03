// Nodo G1: 49.0
// Nodo G2: 158.0
// Nodo TL1:42.0
// Nodo TL2:21.0

#include "contiki.h"
#include "dev/button-sensor.h"
#include "dev/sht11/sht11-sensor.h"
#include "dev/leds.h"
#include "sys/etimer.h"
#include "net/rime/rime.h"
#include "stdio.h"
#include "stdlib.h"

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
#define MAX_RETRANSMISSIONS	5

typedef struct {
	char type;
	int value;
} measurement_t;

// Vehicle states, VOID is default state
typedef enum { NONE, NORMAL, EMERGENCY, VOID } vehicle_t;
// TL state, BLINK is default state
typedef enum { BLINK, SEND_NOTIFY_TL, MANAGE_TRAFFIC, SEND_NOTIFY_CAR, RED_TL, GREEN_TL, RESTORE_TL } state_t;

PROCESS(tl,"TL SkyMote");
AUTOSTART_PROCESSES(&tl);

static const linkaddr_t g1_addr  = {{G1_ADDR, 0}};  	// Strutture contenenti l'indirizzo dei Mote
static const linkaddr_t g2_addr = {{G2_ADDR, 0}};
static const linkaddr_t tl1_addr = {{TL1_ADDR, 0}};
static const linkaddr_t tl2_addr = {{TL2_ADDR, 0}};
static bool tl_notified = false;		// Did other tl notify its vehicle?
static struct runicast_conn runicast;
static state_t state = BLINK;			
static vehicle_t my_vehicle = NONE;		// State of my vehicle
static vehicle_t its_vehicle = VOID;	// State of its vehicle

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){

	#ifdef DEBUG
		printf("DEBUG: runicast message received from %d.%d, seqno %d, msg: %s\n", from->u8[0], from->u8[1], seqno, (char *)packetbuf_dataptr());
	#endif
	if(linkaddr_cmp(from, &tl1_addr) || linkaddr_cmp(from, &tl2_addr)){	// Ho ricevuto il veicolo da TL*

		its_vehicle = atoi((char *) packetbuf_dataptr());

		if(my_vehicle == NONE && its_vehicle == NONE)
			return;

		if(tl_notified == true)
			state = MANAGE_TRAFFIC;
		else
			state = SEND_NOTIFY_TL;

	}else{	// Altrimenti giunge dallo SkyMote G*

		my_vehicle = atoi((char *) packetbuf_dataptr());
		state = SEND_NOTIFY_TL;
		tl_notified = false;

	}

	process_post(&tl, PROCESS_EVENT_MSG, NULL);

}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	#ifdef DEBUG
		printf("DEBUG: runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
	#endif
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	printf("//// Timeout\n");
	state = RESTORE_TL;
	process_post(&tl, PROCESS_EVENT_MSG, NULL);
}

static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from){}

static void broadcast_sent(struct broadcast_conn *c, int status, int num_tx){
	#ifdef DEBUG
		printf("DEBUG: broadcast message sent to sink\n");
	#endif
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv, broadcast_sent}; 
static struct broadcast_conn broadcast;

PROCESS_THREAD(tl, ev, data){

	static struct etimer et, sensing_timer;

	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_EXITHANDLER(broadcast_close(&broadcast));

	PROCESS_BEGIN();

	static size_t battery_level = 100;			// Default battery level of tf
	static size_t counter_until_20 = 0;			// Counter variable for tf, when timeout's sensing_timer is 1 second
	static bool red_tl_enable = false;			// Wheter tf is red
	static bool timer10_flag = false;			// When timeout's sensing_timer is 10 second
	static bool timer20_flag = false;			// When timeout's sensing_timer is 20 second
	static bool button_activated = false;		// Variable state of button when battery level is below to 20
	static bool et_expired = false;				// Il blink_timer (et) o sensing_timer sono scaduti, quindi è possibile per fare alcuni controlli
	static char message[2];
	static int temperature = 0, humidity = 0;
	static measurement_t sensing;				// Where to store sensing values
	static linkaddr_t recv;

	runicast_open(&runicast, 144, &runicast_calls);
	broadcast_open(&broadcast, 150, &broadcast_call);
	leds_on(LEDS_GREEN);
	leds_off(LEDS_RED);
	etimer_set(&et, CLOCK_SECOND);
	etimer_set(&sensing_timer, CLOCK_SECOND * 5);

	while(1){

		PROCESS_WAIT_EVENT();

		if(et_expired == true && button_activated == false){

			if(battery_level < 20){
				SENSORS_ACTIVATE(button_sensor);
				button_activated = true;
			}
			if(timer10_flag == false && battery_level <= 50){
				etimer_set(&sensing_timer, CLOCK_SECOND * 10);
				timer10_flag = true;
			}else if(timer20_flag == false && battery_level <= 20){
				etimer_set(&sensing_timer, CLOCK_SECOND);
				timer20_flag = true;
			}
			et_expired = false;

		}

		if(button_activated == true && ev == sensors_event && data == &button_sensor){

			battery_level = 100;
			counter_until_20 = 0;
			timer10_flag = false;
			timer20_flag = false;
			button_activated = false;
			SENSORS_DEACTIVATE(button_sensor);
			leds_off(LEDS_BLUE);
			etimer_set(&sensing_timer, CLOCK_SECOND * 5);
			continue;

		}

		if(state == BLINK && etimer_expired(&et)){

			printf("STATO: BLINK\n");

			leds_toggle(LEDS_GREEN);
			leds_toggle(LEDS_RED);
			battery_level = (int)(battery_level - 5) > 0 ? (battery_level - 5) : 0;
			et_expired = true;
			etimer_reset(&et);

		}

		// Se la batteria va a 0, non faccio più sensing
		if(etimer_expired(&sensing_timer)){

			etimer_reset(&sensing_timer);
			if(button_activated == true){
				counter_until_20++;
				leds_toggle(LEDS_BLUE);
				if(counter_until_20 < 20 || battery_level <= 0)
					continue;
				else
					counter_until_20 = 0;
			}

			SENSORS_ACTIVATE(sht11_sensor);	// Burst sensor time
			et_expired = true;
			battery_level = (int)(battery_level - 10) > 0 ? (battery_level - 10) : 0;
			
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
			continue;

		}

		if(tl_notified == false && ((state == SEND_NOTIFY_TL && red_tl_enable == false) 
			|| (state == SEND_NOTIFY_TL && red_tl_enable == true && etimer_expired(&et)))){ // Comunica l'informazione all'altro semaforo, così si gestiranno le priorità

			printf("STATO: SEND_NOTIFY_TL\n");

			red_tl_enable = false;

			if(linkaddr_cmp(&linkaddr_node_addr, &tl1_addr)){
				recv.u8[0] = TL2_ADDR;
				recv.u8[1] = 0;
			}else{
				recv.u8[0] = TL1_ADDR;
				recv.u8[1] = 0;
			}

			sprintf(message, "%d", my_vehicle);
			if(!runicast_is_transmitting(&runicast)) {
				packetbuf_copyfrom(message, 2);
				runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
			}

			if(its_vehicle != VOID)		// Nel caso abbia ricevuto la macchina vuol dire che è già stato contattato
				state = MANAGE_TRAFFIC;	// Scambio auto avvenuto, ora entrambi i sensori vedono gli stessi dati

			tl_notified = true;
			
		}

		if((state == MANAGE_TRAFFIC && red_tl_enable == false) || (state == MANAGE_TRAFFIC && red_tl_enable == true && etimer_expired(&et))){

			printf("STATO: MANAGE_TRAFFIC\n");

			red_tl_enable = true;
			tl_notified = false;
			if(its_vehicle == VOID)	// FIX: Può capitare di saltare in questo stato da RED_TL
				its_vehicle = NONE;

			// ho la priorità se ho una qualunque auto e:
			// - lui non ha macchine
			// - lui ha auto normali
			// - ho un auto di emergenza e lui anche
			if(my_vehicle != NONE){

				if(linkaddr_cmp(&linkaddr_node_addr, &tl1_addr)){ 		// Prioritario
					if(its_vehicle != EMERGENCY || (my_vehicle == EMERGENCY && its_vehicle == EMERGENCY))
						state = SEND_NOTIFY_CAR;
					else
						state = RED_TL;
					
				}

				// ho la priorità se ho una qualunque auto e:
				// - lui non ha macchine
				// - ho un auto di emergenza e lui no
				if(linkaddr_cmp(&linkaddr_node_addr, &tl2_addr)){ 		// NON prioritario
					if(its_vehicle == NONE || (my_vehicle == EMERGENCY && its_vehicle != EMERGENCY))
						state = SEND_NOTIFY_CAR;
					else
						state = RED_TL;
				}

			} else {

				state = RED_TL;

			}

		}

		if(state == RED_TL){

			printf("STATO: RED_TL\n");

			its_vehicle = VOID;
			if(my_vehicle == NONE)
				state = RESTORE_TL;
			else
				state = MANAGE_TRAFFIC;
			leds_on(LEDS_RED);
			leds_off(LEDS_GREEN);
			etimer_set(&et, CLOCK_SECOND * 5);
			continue;
		}

		if(state == SEND_NOTIFY_CAR){ // Dico sì alla macchinuccia

			printf("STATO: SEND_NOTIFY_CAR\n");

			if(linkaddr_cmp(&linkaddr_node_addr, &tl1_addr)){
				recv.u8[0] = G1_ADDR;
				recv.u8[1] = 0;
			}else{
				recv.u8[0] = G2_ADDR;
				recv.u8[1] = 0;
			}

			sprintf(message, "%d", my_vehicle);
			if(!runicast_is_transmitting(&runicast)) {
				packetbuf_copyfrom(message, 2);
				runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
			}
			state = GREEN_TL;

		}

		if(state == GREEN_TL){

			printf("STATO: GREEN_TL\n");

			my_vehicle = NONE;			
			leds_on(LEDS_GREEN);
			leds_off(LEDS_RED);
			if(its_vehicle == NONE)
				state = RESTORE_TL;
			else
				state = MANAGE_TRAFFIC;
			etimer_set(&et, CLOCK_SECOND * 5);
			continue;
		}

		if(state == RESTORE_TL && etimer_expired(&et)){

			printf("RESTORE_TL\n");

			state = BLINK;
			red_tl_enable = false;
			tl_notified = false;
			my_vehicle = NONE;
			its_vehicle = VOID;
			leds_toggle(LEDS_GREEN);
			leds_toggle(LEDS_RED);
			etimer_set(&et, CLOCK_SECOND * 1);
			
		}

	}

	PROCESS_END();

}