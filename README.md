# Implementacion de un Controlador PID para un ServoMotor.
### Lema, Adan Juan Angel - **adanlema@hotmail.com**

Para el funcionamiento de nuestro programa se debe emplear los siguientes comandos en la terminal:

| Comando | Descripcion |
| --- | --- |
| make | Compila el codigo |
| make doc | Documento el codigo usando doxygen |
| make download | Sube el codigo a la placa |
| make clean | Borra la carpeta *build* |

Se adquirieron datos utilizando el convertidor analógico-digital (ADC). Además, se implementaron dos posibles señales de salida para alimentar el motor al activar el botón y empezar la adquisición de datos. La primera salida consistía en un solo pulso: comenzaba en estado OFF, permanecía así hasta alcanzar el tiempo **TON**, luego cambiaba a estado ON hasta llegar al tiempo **TOFF**. Mientras tanto, la segunda salida se generaba mediante PWM. Inicialmente, comenzaba con un ciclo de trabajo del 0% al presionar el botón, luego pasaba a un 75% al alcanzar **TON**. En un segundo instante, **T2**, el ciclo de trabajo se establecía en 50%, y finalmente, en **TOFF**, la salida se ponía en estado OFF. Los pines a utilizar de la blue-pill en este proyecto son:

| Pin | Funcion |
| --- | --- |
| PA0 | Entrada del ADC |
| PA6 | Salida PWM |
| PA9 | Transmisor Serial (Tx) |
| PA10 | Receptor Serial (Rx) |
| PB12 | Boton activador |
| PB13 | Salida 1 |
