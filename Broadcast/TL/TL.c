// Nodo G1: 49.0
// Nodo G2: 158.0

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
#define HUMIDITY_SENS		2 // Parametro divisore di CLOCK_SECOND; il risultato sarà: CLOCK_SECOND / HUMIDITY_SENS

typedef struct {
	char type;
	int value;
} measurement_t;

// Vehicle states, VOID is default state
typedef enum { NONE, NORMAL, EMERGENCY } vehicle_t;
// TL state, BLINK is default state
typedef enum { BLINK, MANAGE_TRAFFIC, SEND_NOTIFY_CAR, RED_TL, GREEN_TL, RESTORE_TL } state_t;

PROCESS(tl,"TL SkyMote");
AUTOSTART_PROCESSES(&tl);

static const linkaddr_t g1_addr  = {{G1_ADDR, 0}};  	// Strutture contenenti l'indirizzo dei Mote
static const linkaddr_t g2_addr = {{G2_ADDR, 0}};
static const linkaddr_t tl1_addr = {{TL1_ADDR, 0}};
static const linkaddr_t tl2_addr = {{TL2_ADDR, 0}};
static state_t state = BLINK;					// Variabile che tiene lo stato della macchina (Mote)
static vehicle_t my_vehicle = NONE;				// Variabile che tiene lo stato del veicolo sulla propria strada (G1, TL1) e (G2, TL2)
static vehicle_t its_vehicle = NONE;			// Variabile che tiene lo stato del veicolo sull'altra strada (G1, TL2) o (G2, TL1)

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){
	#ifdef DEBUG
		printf("DEBUG: runicast message received from %d.%d\n", from->u8[0], from->u8[1]);
	#endif
}

static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	#ifdef DEBUG
		printf("DEBUG: runicast message sent to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
	#endif
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){}

static const struct runicast_callbacks runicast_calls = {recv_runicast, sent_runicast, timedout_runicast};
static struct runicast_conn runicast;

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from){

	#ifdef DEBUG
		printf("DEBUG: broadcast message received from %d.%d\n", from->u8[0], from->u8[1]);
	#endif

	// La funzione riceve un msg broadcast inviato dall'auto
	if(linkaddr_cmp(&linkaddr_node_addr, &tl1_addr) && linkaddr_cmp(from, &tl2_addr) == 0){
		
		if(linkaddr_cmp(from, &g1_addr))
			my_vehicle = atoi((char *) packetbuf_dataptr());
		else
			its_vehicle = atoi((char *) packetbuf_dataptr());

		state = MANAGE_TRAFFIC;
		process_post(&tl, PROCESS_EVENT_MSG, NULL);

	} else if(linkaddr_cmp(&linkaddr_node_addr, &tl2_addr) && linkaddr_cmp(from, &tl1_addr) == 0){

		if(linkaddr_cmp(from, &g2_addr))
			my_vehicle = atoi((char *) packetbuf_dataptr());
		else
			its_vehicle = atoi((char *) packetbuf_dataptr());

		state = MANAGE_TRAFFIC;
		process_post(&tl, PROCESS_EVENT_MSG, NULL);

	}

}

static void broadcast_sent(struct broadcast_conn *c, int status, int num_tx){
	#ifdef DEBUG
		printf("DEBUG: message sent in broadcast\n");
	#endif
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv, broadcast_sent}; 
static struct broadcast_conn broadcast;

PROCESS_THREAD(tl, ev, data){

	static struct etimer et, sensing_timer, humidity_timer;		// et: timer per fare blinking ed attendere per il rosso/verde
																// sensing_timer: timer per fare sensing ogni CLOCK_SEC * k secondi 
	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_EXITHANDLER(broadcast_close(&broadcast));

	PROCESS_BEGIN();

	static size_t battery_level = 100;			// Livello della batteria
	static size_t counter_until_20 = 0;			// Contatore fino a 20 necessario quando sensing_timer ha frequenza CLOCK_SEC * 1
	static bool red_tl_enable = false;			// Flag attivo quando si è evviata lo stato MANAGE_TRAFFIC e si setta et a CLOCK_SEC * 5  
	static bool timer10_flag = false;			// Flag attivo quando sensing_timer è passato ha CLOCK_SEC * 10
	static bool timer20_flag = false;			// Flag attivo quando sensing_timer deve campionare ogni 20 sec
	static bool button_activated = false;		// Flag attivo quando battery_level <= 20
	static bool et_expired = false;				// Flag attivo quando sensing_timer o et (in stato BLINK) scadono
	static bool transmit = false;				// Flag per inviare umidità in differità
	static int humidity = 0, temperature = 0;
	static measurement_t sensing;				// Struct per salvare i valori di sensing
	static linkaddr_t recv;

	runicast_open(&runicast, 144, &runicast_calls);
	broadcast_open(&broadcast, 150, &broadcast_call);
	leds_on(LEDS_GREEN);
	leds_off(LEDS_RED);
	etimer_set(&et, CLOCK_SECOND);
	etimer_set(&sensing_timer, CLOCK_SECOND * 5);

	while(1){

		PROCESS_WAIT_EVENT();

		// Invia l'umidità dopo 500ms dall'invio della temperatura
		if(transmit == true && etimer_expired(&humidity_timer) && !runicast_is_transmitting(&runicast)){
			recv.u8[0] = G1_ADDR;
			recv.u8[1] = 0;
			transmit = false;
			packetbuf_copyfrom(&sensing, sizeof(sensing));
			runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
		}

		// Se sensing_timer o et (in stato = BLINK) sono scaduti e il pulsante non è stato ancora attivato ...
		if(et_expired == true && button_activated == false){

			if(battery_level < 20 && button_activated == false){
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

		// Quando il pulsante è stato attivato, e viene premuto ripristina lo stato di sensing del mote
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

		// Quando scade sensing timer raccogli temperatura e umidità
		if(etimer_expired(&sensing_timer)){

			etimer_reset(&sensing_timer);
			// se il pulsante è attivo significa che la batteria ha carica infereriore (<=) a 20
			if(button_activated == true){
				counter_until_20++;
				leds_toggle(LEDS_BLUE);
				if(counter_until_20 < 20 || battery_level <= 0)	// Non fa sensing, si rimanda quando counter = 20 o batteria sarà full
					continue;
				else
					counter_until_20 = 0;
			}

			// Destinatario
			recv.u8[0] = G1_ADDR;
			recv.u8[1] = 0;

			SENSORS_ACTIVATE(sht11_sensor);	// Burst sensor time
			et_expired = true;
			battery_level = (int)(battery_level - 10) > 0 ? (battery_level - 10) : 0;

			sensing.type = 'T';
			sensing.value = (sht11_sensor.value(SHT11_SENSOR_TEMP)/10 - 396)/10;
			temperature = sensing.value;

			if(!runicast_is_transmitting(&runicast)) {
				packetbuf_copyfrom(&sensing, sizeof(sensing));
				runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
			}
			
			sensing.type = 'H';
			humidity = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
			// Fix umidità
			sensing.value = -4 + 0.0405 * humidity + (-2.8 * 0.000001) * (humidity * humidity);
			sensing.value = (temperature - 25) * (0.01 + 0.00008 * humidity) + sensing.value;
			SENSORS_DEACTIVATE(sht11_sensor);

			transmit = true;
			etimer_set(&humidity_timer, CLOCK_SECOND / HUMIDITY_SENS);

			continue;

		}

		if((state == MANAGE_TRAFFIC && red_tl_enable == false ) || (state == MANAGE_TRAFFIC && red_tl_enable == true && etimer_expired(&et))){

			printf("STATO: MANAGE_TRAFFIC\n");

			red_tl_enable = true;

			if(my_vehicle != NONE){

				// ho la priorità se ho una qualunque auto e:
				// - lui non ha macchine
				// - lui ha auto normali
				// - ho un auto di emergenza e lui anche
				if(linkaddr_cmp(&linkaddr_node_addr, &tl1_addr)){ 							// Prioritario
					if(its_vehicle != EMERGENCY || (my_vehicle == EMERGENCY && its_vehicle == EMERGENCY))
						state = SEND_NOTIFY_CAR;
					else
						state = RED_TL;
					
				}

				// ho la priorità se ho una qualunque auto e:
				// - lui non ha macchine
				// - ho un auto di emergenza e lui no
				if(linkaddr_cmp(&linkaddr_node_addr, &tl2_addr)){				 		// NON prioritario
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

			its_vehicle = NONE;
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

			packetbuf_copyfrom("0", 2);
			broadcast_send(&broadcast);
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

			printf("STATO: RESTORE_TL\n");

			state = BLINK;
			red_tl_enable = false;
			my_vehicle = NONE;
			its_vehicle = NONE;
			leds_toggle(LEDS_GREEN);
			leds_toggle(LEDS_RED);
			etimer_set(&et, CLOCK_SECOND * 1);
			
		}

	}

	PROCESS_END();

}