#include "contiki.h"
#include "dev/button-sensor.h"
#include "dev/sht11/sht11-sensor.h"
#include "dev/serial-line.h"
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
#define SIZE 				4
#define MAX_CHARSET			25

typedef struct {
	char type;
	int value;
} measurement_t;

typedef enum { NONE, NORMAL, EMERGENCY } vehicle_t;
typedef enum { DEFAULT, NOTIFY_VEHICLE, RESTORE_VEHICLE } state_t;

PROCESS(g1, "G1 SkyMote");
AUTOSTART_PROCESSES(&g1);

static const linkaddr_t g2_addr = {{G2_ADDR,0}};						// Strutture contenenti l'indirizzo dei Mote
static const linkaddr_t tl1_addr = {{TL1_ADDR,0}};
static const linkaddr_t tl2_addr = {{TL2_ADDR,0}};
static int temperature[SIZE], humidity[SIZE];							// Array contenenti le informazioni di sensing dei 4 mote, temp/hum memorizza 
static int local_temperature = -100, local_humidity = 0;				// Variabili temporanee per fixare il valore dell'umidità relativa
static bool temp_from_g2 = false, temp_from_tl1 = false, temp_from_tl2 = false; 	// Flag che tengono traccia delle trasmissioni di sensing
static bool hum_from_g2 = false, hum_from_tl1 = false, hum_from_tl2 = false;
static char warning_message[MAX_CHARSET];								// Buffer di testo per il messaggio di warning
static state_t state = NONE;											// Variabile che tiene lo stato della macchina (Mote)

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno){

	#ifdef DEBUG
		printf("DEBUG: runicast message received from %d.%d\n", from->u8[0], from->u8[1]);
	#endif

	static size_t index;									// Indice usato nel for
	static int temperature_avg = 0, humidity_avg = 0;		// Variabili locali per il calcolo del valore medio
	static measurement_t sensing;							// Struttura di appoggio per le informazioni ricevute

	memcpy(&sensing, packetbuf_dataptr(), sizeof(sensing));

	#ifdef DEBUG
		printf("DEBUG: Sens: %c, value: %d\n", sensing.type, sensing.value);
	#endif

	if(linkaddr_cmp(from,&g2_addr)){						// Controllo da chi proviene il pacchetto e setto il flag del dato
		if(sensing.type == 'T'){
			temperature[1] = sensing.value;
			temp_from_g2 = true;
		} else {
			humidity[1] = sensing.value;
			hum_from_g2 = true;
		}
	} else if(linkaddr_cmp(from,&tl1_addr)){
		if(sensing.type == 'T'){
			temperature[2] = sensing.value;
			temp_from_tl1 = true;
		} else {
			humidity[2] = sensing.value;
			hum_from_tl1 = true;
		}
	} else { // if(linkaddr_cmp(from,&tl2_addr))
		if(sensing.type == 'T'){
			temperature[3] = sensing.value;
			temp_from_tl2 = true;
		} else {
			humidity[3] = sensing.value;
			hum_from_tl2 = true;
		}
	}
	
	// Se ho ricevuto tutte le temperature o umidità, calcolo la media e stampo le informazioni
	if((temp_from_g2 && temp_from_tl1 && temp_from_tl2) || (hum_from_g2 && hum_from_tl1 && hum_from_tl2)){

		SENSORS_ACTIVATE(sht11_sensor);	// Burst sensor time
		if(temp_from_g2 && temp_from_tl1 && temp_from_tl2){
			temperature[0] = (sht11_sensor.value(SHT11_SENSOR_TEMP)/10 - 396)/10;
			local_temperature = temperature[0];
		}else{
			// Fix umidità: http://tinyos.stanford.edu/tinyos-wiki/index.php/Boomerang_ADC_Example
			// Volendo si potrebbe fare prima la media di temperatura ed umidità, e correggere l'umidità di conseguenza con
			// la temperatura media, così da ridurre ulteriormente la complessità all'interno della rete
			local_humidity = sht11_sensor.value(SHT11_SENSOR_HUMIDITY);
			humidity[0] = -4 + 0.0405 * local_humidity + (-2.8 * 0.000001) * (local_humidity * local_humidity);
			if(local_temperature != -100)
				humidity[0] = (local_temperature - 25) * (0.01 + 0.00008 * local_humidity) + humidity[0];
		}
		SENSORS_DEACTIVATE(sht11_sensor);

		for(index = 0; index < 4; index++){
			if(temp_from_g2 && temp_from_tl1 && temp_from_tl2)
				temperature_avg += temperature[index]; 
			else
				humidity_avg += humidity[index];
		}
		temperature_avg /= SIZE;
		humidity_avg /= SIZE;

		if(strlen(warning_message) != 0)
			printf("%s\n", warning_message);
		if(temp_from_g2 && temp_from_tl1 && temp_from_tl2){
			printf("TEMP: %d°C\t", temperature_avg);
			temp_from_tl2 = false; temp_from_tl1 = false; temp_from_g2 = false;
		} else {
			printf("HUMIDITY: %d%%\n", humidity_avg);
			hum_from_tl2 = false; hum_from_tl1 = false; hum_from_g2 = false;
		}

		memset(warning_message, '\0', 25);

	}

}

static const struct runicast_callbacks runicast_calls = {recv_runicast};
static struct runicast_conn runicast;

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from){

	#ifdef DEBUG
		printf("DEBUG: broadcast message received from %d.%d\n", from->u8[0], from->u8[1]);
	#endif

	// Ripristino lo stato del sensore quando questo riceve notifica dal TL1
	if(linkaddr_cmp(from, &tl1_addr) && state == NOTIFY_VEHICLE){
		state = RESTORE_VEHICLE;						
		process_post(&g1, PROCESS_EVENT_MSG, NULL);
	}

}

static void broadcast_sent(struct broadcast_conn *c, int status, int num_tx){
	#ifdef DEBUG
		printf("DEBUG: message sent in broadcast\n");
	#endif
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv, broadcast_sent}; 
static struct broadcast_conn broadcast;

PROCESS_THREAD(g1, ev, data){

	static struct etimer double_press_timer, waiting_notify_timer;	// Timer per la doppia pressione del tasto, e per inviare 
																	// la notifica a TL dopo una già inviata
	PROCESS_EXITHANDLER(runicast_close(&runicast));
	PROCESS_EXITHANDLER(broadcast_close(&broadcast));

	PROCESS_BEGIN();

	static vehicle_t vehicle = NONE;		// Variabile che tiene lo stato del veicolo sulla propria strada (G1, TL1) e (G2, TL2)
	static bool tl_notified = false;		// Flag attivo quando il mote ha notificato l'arrivo del veicolo al suo TL*
	static bool etimer_active = false;		// Flag attivo quando si attiva waiting_notify_timer
	static bool auth = false;				// Flag attivo quando si effettua correttamente il login
	static size_t msg_size, i;				// msg_size contiene la dimensione in caratteri del warning msg inserito da console, i è un indice
	static char message[2];					// Buffer per inviare msg

	runicast_open(&runicast, 144, &runicast_calls);
	broadcast_open(&broadcast, 150, &broadcast_call);
	SENSORS_ACTIVATE(button_sensor);

	while(1){
		// EVENTI:
		//  - ricezione msg via cmd
		//	- bottone
		//	- double_press_timer scaduto per tasto
		//	- ricezione msg da TL
		PROCESS_WAIT_EVENT();

		// Eventi legati al cmd: login e settaggio warning
		if(ev == serial_line_event_message){

			if(auth == false){

				if(!strcmp((char *) data, "NES\0")){
						printf("Autenticato! Inserisci Warning message: \n");
						auth = true;
					} else
						printf("Password errata! Inserisci password:\n");
			
			} else {
				
				msg_size = strlen((char *) data);
				if(msg_size <= (MAX_CHARSET - 1)){
					if(strcmp((char *) data, "\n\0")){
						strcpy(warning_message, (char *) data);
						for(i = 0; i <= msg_size; i++)	// Tutte le lettere maiuscole
				      		if(warning_message[i] >= 'a' && warning_message[i] <= 'z')
				        		warning_message[i] = warning_message[i] - 32; // A - a
				    }
				    printf("Connessione terminata.\n");
					auth = false;

				}else
					printf("Messaggio troppo lungo! Inserisci Warning message: \n");
			
			}

		}

		// Eventi legati al bottone
		if(state == DEFAULT && ev == sensors_event && data == &button_sensor){

			printf("STATO: DEFAULT\n");

			if(vehicle == NONE){

				vehicle = NORMAL;
				state = NOTIFY_VEHICLE;
				etimer_set(&double_press_timer, CLOCK_SECOND * 0.5);
				continue;
		
			}

			if(vehicle == NORMAL && etimer_expired(&double_press_timer) == 0){

				vehicle = EMERGENCY;
				etimer_stop(&double_press_timer);
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
				packetbuf_copyfrom(message, 2);
				broadcast_send(&broadcast);

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