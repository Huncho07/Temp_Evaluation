; Si realizzi un firmware che ogni 8 secondi esca dallo sleep, acquisisca il valore di
; temperatura dal sensore presente sulla board e lo stampi su porta seriale (EUSART)
; tramite numero a due cifre decimali.
		
		
		
                #include "p16f887.inc"
                #include "macro.inc"
		
		; configuration bits
		__CONFIG _CONFIG1, _INTRC_OSC_NOCLKOUT & _WDT_OFF & _LVP_OFF
		__CONFIG _CONFIG2, _BOR21V
		
    		; variabili in RAM (shared RAM)
		udata_shr
tmp             res    .1    ; variabile per il calcolo del valore di temperatura
temperature     res    .1    ; variabile per il calcolo del valore di temperatura
usart_counter   res    .1    ; contatore per la trasmissione dell'intero buffer
print_buffer    res    .5    ; buffer di stampa usato per la trasmissione
    
     		; reset vector
rst_vector		code	0x0000
		pagesel start
		goto start
		
		; programma principale
		code
		
start           ; inizializzazione hardaware
		pagesel init_hardware
		call init_hardware
		
main_loop	; disabilita l' interrupt global enable, abilita interrupt timer1 e
		; abilita gli interrupt delle periferiche aggiuntive (tra cui timer1)
		; con queste impostazioni è possibile svegliare il micro dalla condizione
		; di sleep con l' overflow del timer senza che entri nella routine di interrupt
		; visto che il GIE è disabilitato
	        bcf	INTCON, GIE
		banksel	PIE1
		bsf	PIE1, TMR1IE
		bsf	INTCON, PEIE
		; carica il timer per il conteggio degli 8 secondi, senza precaricare il timer
		; visto che da inizializzazione hardware il timer prescalato a 1:4 e con
		; l' uso del quarzo esterno conta esattamente il tempo richiesto
 	        call load_timer
	        ; inizio conversione A/D
		banksel ADCON0
		bsf     ADCON0, GO  
wait_adc		
                ; polling per attesa del completamento della conversione A/D
		btfsc   ADCON0, GO
		goto    wait_adc  
		; banksel su ADRESH che è il registro in cui si trova il risultato della
		; conversione A/D, in particolare solo 8 bit piu' significativi del risultato,
		; faccio poi una mov che porta il risultato in w
		banksel ADRESH
		movf    ADRESH, w
		; porto poi il valore dal registro accumulatore alla variabile tmp
		movwf   tmp         ; tmp = valore di tensione (0 - 255) campionato
		; chiamata delle funzioni che portano alla computazione del valore
		; di temperatura in due cifre decimali, la seconda subroutine carica anche
		; il buffer usato per la trasmissione
		call compute_temp
		call compute_decimal
		; subroutine che inizia il processo per la trasmissione seriale
		call print_eusart
		; ritorno al loop iniziale
		goto    main_loop
		
init_hardware   ; inizializzazione hardware per scheda PIC Board - Studio

		; configure clock
		setRegK OSCCON, B'01110001' ; 8 MHz internal oscillator

		;OPTION_REG:
		;RBPU = 0  (pull-ups enabled on port B)
		;INTEDG = 0
		;T0CS = 0
		;T0SE = 0
		;PSA = 0
		;PS<2:0> = 111 (timer0 prescaler = 1:256)
		; -> freq. = 7812.5 Hz, tick = 128us, max period = 32.768 ms
		setRegK OPTION_REG, B'00000111'

		;port A:
		; RA0-RA5: analog inputs
		; RA6-RA7: digital outputs (flash_ce, bus_switch)
		setRegK PORTA, B'01000000' ; flash_ce = 1
		setRegK ANSEL, B'11111111' ; set RE0-RE2 as analog too
		setRegK TRISA, B'00111111'

		;port B:
		; RB0-RB4: digital inputs (buttons, sd_detect) with pull-up
		; RB5: digital output (sd_ce)
		; RB6-RB7: used by ICSP
		setReg0 ANSELH
		setRegK PORTB, B'00100000' ; sd_ce = 1
		setRegK TRISB, B'11011111'
		setRegK WPUB, B'00011111'
		movf PORTB, w ; read PORTB to clear mismatch condition
		banksel INTCON
		bcf INTCON, RBIF
		movlw 0x0F
		banksel IOCB  ; enable interrupt on-change for pins 0 ... 3
		movwf IOCB
		banksel INTCON
		bsf INTCON, RBIE  ; enable port B interrupt

		;port C:
		; RC2: digital output for buzzer
		; others: used by peripherals
		setReg0 PORTC
		setRegK TRISC, B'11111011'

		;port D:
		; RD0-RD3: digital outputs (LEDs)
		; RD4-RD7: not used (digital inputs)
		setReg0 PORTD
		setRegK TRISD, 0xF0
		;port E:
		; RE0-RE2: analog inputs (see ANSEL above)
		; RE3: used by reset
		setReg0 PORTE
		
		;timer1
		; - usa quarzo esterno (32768 Hz)
		; - modalita' asincrona (funziona con quarzo esterno anche durante sleep)
		; - prescaler = 1:4
		; - timer off
		; Con la frequenza del quarzo ed il prescaler a 1:4 si ha:
		; - singolo tick ~= 12.207 us
		; - periodo max = 8 s (contatore a 16 bit)
		banksel	T1CON
		movlw	B'00101110'     ;TMR1 OFF
		movwf	T1CON
		
		; ADC
		setRegK ADCON0, B'10011001' ; clock = Fosc/32,
		                            ; channel 6 (temperature sensor),
		                            ; ADC enabled
		setReg0 ADCON1 ; use vdd and vss as reference
		
		; EUSART
		; baud rate = 19200 (BRGH = 1, BRG16 = 0)
		; TXEN = 1
		; SPEN = 1
		; CREN = 1
		setRegK TXSTA, B'00100100'
		setRegK RCSTA, B'10010000'
		setReg0 BAUDCTL
		setRegK SPBRG, .25
		
		return
		
load_timer      ; attiva timer 1
		banksel	T1CON
		bsf	T1CON, TMR1ON
		; spegnimento led per testare lo sleep
		banksel	PORTD
		bcf	PORTD, 3
		; porto il micro in sleep come da specifiche
	        sleep
		; accensione led per testare lo sleep
		banksel	PORTD
		bsf	PORTD, 3
		; azzera flag interrupt di timer1
		banksel PIR1
		bcf	PIR1, TMR1IF
		return
			
compute_temp    ; routine di conversione da tensione a gradi centigradi
		; input:
		;   tmp = tensione campionata (0-255, corrispondente a 0 - 3.3 V)
		;   (il valore iniziale di tmp viene perso)
		; output:
		;   temperature = risultato in gradi
		;
		; Dal datasheet del sensore di temperatura MCP9701A:
		;  T = (Vadc - V0) / Tc   [ dove V0 = 400 mV, Tc = 19.5 mV/C]
		; Convertendo da tensioni a valori binari, si ha:
		;  T = (Nadc - 31) / 1.51
		; che puo' essere approssimata in calcoli interi a 8 bit come:
		;  T = (Nadc - 31) * 2 / 3  [ approssimaz. 1.51 ~= 1.5 = 3/2 ]
		; Questa formula permette di calcolare temperature fino a 84 C
		;  senza incorrere nell'overflow della variabile a 8 bit
		movlw .31
		subwf tmp, f  ; tmp = tmp - 31
		bcf STATUS, C
		rlf tmp, f    ; tmp = tmp * 2 (usando lo shift a sinistra)
		;  la divisione per 3 è effettuata con semplice algoritmo di sottrazioni
		;  successive del valore 3, incrementando ogni volta il risultato,
		;  fino a che il minuendo non diventa negativo
		clrf temperature  ; valore iniziale del risultato = 0
loop_div3
		movlw .3
		subwf tmp, w          ; w = tmp - 3
		btfss STATUS, C
		goto end_div3         ; se risultato negativo (C=0): fine divisione
		movwf tmp             ; tmp = tmp - 3
		incf temperature, f   ; incrementa risultato di 1
		goto loop_div3        ; continua sottrazione
end_div3
		return
		
compute_decimal   ; calcolo delle 2 cifre decimali
		  ; input:
		  ;   temperature = valore in gradi da riprodurre
		  ;   (il valore iniziale di temperature viene perso)
		  ;
		  ; tmp e' usata come variabile temporanea
		  ; Il metodo di calcolo delle 2 cifre e' il seguente:
		  ;  si divide la temperatura per 10, il quoziente rappresenta le
		  ;  decine mentre il resto rappresenta le unita'
		  ; La divisione per 10 e' effettuata come sopra con sottrazioni
		  ;  successive
		clrf tmp  ; risultato della divisione per 10 (decine)	
loop_div10      
		movlw .10
		subwf temperature, w  ; w = temperature - 10
		btfss STATUS, C
		goto end_div10        ; se risultato negativo (C=0): fine divisione
		movwf temperature     ; temperature = temperature - 10
		incf tmp, f           ; incrementa risultato di 1
		goto loop_div10       ; continua sottrazione		
end_div10       
		; la variabile "tmp" contiene le decine, la variabile "temperature" 
		; contiene le unità ( è il resto della divisione)
		
		movlw '0'	        
		addwf tmp, w            ; sommo il codice ASCII di '0' per avere il codice
		                        ; del numero che voglio
		movwf (print_buffer+0)	; carico le decine
		movlw '0'
		addwf temperature, w
		movwf (print_buffer+1)	; carico le unità
		movlw ' '
		movwf (print_buffer+2)  ; carico il carattere 'spazio'
		movlw 'C' 
		movwf (print_buffer+3)  ; carico il carattere 'C' di grado centigrado
		movlw .10
		movwf (print_buffer+4)	; carico il carattere invio
		
		movlw .5                ; lunghezza stringa da stampare
		
		return	
		
print_eusart	; si sposta la lunghezza della stringa da stampare sul counter, poi si 
		; fa una mov del buffer su FSR (file select register), l'FSR è un puntatore
		; usato per scorrere i vari caratteri nella trasmissione seriale
		movwf usart_counter     ; numero di caratteri presenti nel buffer di trasmissione
		movlw print_buffer      ; sposto print_buffer nel registro accumulatore
		movwf FSR               ; sposto w nell'FSR
wait_usart	; polling per la trasmissione di tutti i caratteri nel buffer
         	movf usart_counter, w
		btfsc STATUS, Z   ; controllo dell' operazione precedente se il risualtato è zero
		                  ; dal test esce 1 e quindi ritorno al loop principale, se
				  ; il risultato è uno dal test esce 0 e quindi si skippa la 
				  ; return visto che significa che il counter non è ancora nullo
		return
		; si entra in questa parte di codice se si ha un altro carattere da inviare
		movf INDF, w      ; INDF ha il contenuto dell' indirizzo puntato da FSR, faccio
		                  ; una mov del contenuto di INDF in w e poi trasmetto
		banksel TXREG
		movwf TXREG       ; inzia la trasmissione del byte
	        incf FSR          ; incrementa puntatore dati per passare al prossimo carattere
		
wait_buffer	; polling per aspettare che il byte sia effettivamente trasmesso prima di entrare
		; nel main loop, altrimenti la seriale non fa in tempo a trasmettere
		; il carattere tirando fuori un carattere non valido
		banksel TXSTA     
		btfss TXSTA, TRMT ; se TRMT è settato significa che il carattere è stato
		                  ; trasmesso
		goto wait_buffer
		
		decfsz usart_counter, f ; decrementa contatore dati e salva il valore
		                        ; nuovamente sul counter, se il valore del counter
					; è zero skippa la goto e ritorna al loop principale
		goto wait_usart ; aspetto che tutti i caratteri siano trasmessi, senza questo 
		                ; polling dopo la trasmissione del primo carattere il micro 
				; ricomincia il loop 
		return
		
		end