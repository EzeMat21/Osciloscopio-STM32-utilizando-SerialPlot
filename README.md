TP1 ESPECIAL OSCILOSCOPIO

Implemente un "osciloscopio" que permita tomar medidas de tensión del
ADC a una tasa mínima de 1 kHz, almacenarlas en un buffer tipo
ping-pong, y enviarlas por UART a la PC, donde se visualizarán con el
programa Serial Plot (o similar)
(https://hackaday.io/project/5334-serialplot-realtime-plotting-software

Utilizando un buffer ping pong, se muestran por pantalla los valores de
tensión obtenidos del ADC. El ADC escribe sobre el buffer PING mientras
el puerto UART transmite el contenido del PONG, y luego se intercambian
es decir, el ADC comienza a escribir sobre el buffer PONG y el puerto
UART lo hace sobre el buffer PING. La tasa de adquisión del ADC es de
1kHz y el UART transmite los datos cada vez que se llena. El uart se
configuro a una tasa de 115200 baud rate.

La maquina de estados tiene 3 estados: guardando_en_ping,
guardando_en_pong e inicio. En el inicio se configura el ADC como el
UART. Luego del inicio se pasa al estado guardando_en_ping. Aquí el ADC
va llenando el buffer ping. Cuando este se llena, se activa el evento de
buffer lleno pasando a su vez al estado guardando_en_pong. En esta
transición el puntero de escritura del buffer se cambia, apuntando ahora
al buffer pong y se transmite por UART el contenido del buffer ping.
Ahora el ADC esta llenando el buffer pong, cuando este se llene
nuevamente se activa el evento de buffer lleno y transicionando
nuevamente al estado guardando_en_ping. En esta transición, el puntero
escritura vuelve a apuntar al buffer ping y se envia por el puerto uart
el contenido del buffer pong.

Bibliografía o links consultados: Mastering STM32: Cap 12:
Analog-To-Digital Conversion STM32F103x8 Reference Manual Interrupciones
por timer:
https://deepbluembedded.com/stm32-timer-interrupt-hal-example-timer-mode-lab/
