# Implementacion de un Controlador PID para un ServoMotor.
### Lema, Adan Juan Angel - **adanlema@hotmail.com**

Para el funcionamiento de nuestro programa se debe emplear los siguientes comandos en la terminal:

| Comando | Descripcion |
| --- | --- |
| make | Compila el codigo |
| make doc | Documento el codigo usando doxygen |
| make download | Sube el codigo a la placa |
| make clean | Borra la carpeta *build* |

Los pines a utilizar de la blue-pill en este proyecto son:

| Pin | Funcion |
| --- | --- |
| PA0 | Entrada del ADC |
| PA6 | Salida PWM |
| PA9 | Transmisor Serial (Tx) |
| PA10 | Receptor Serial (Rx) |
