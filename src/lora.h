#ifndef LORA
	#define LORA

	void onEvent(ev_t ev);
	void do_send(osjob_t* j, double txdata);
	void loraSetup(void);
#endif
