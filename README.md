
# MULTIROTORFLEX  

 Con el IDE (Integrated Development & Environment) de **Arduino** o **PlatformIO**, cargaremos el programa principal **VERSION.ino** en un microcontrolador **ESP32**, cuyas caracteristicas resumidas serian:
 * Doble nucleo
 * 1M de RAM dinamica
 * 4M de memoria FLASH
 * 240MHz reloj
 * WiFi, BT, ... integrados
 * 802.11mc (Fine Timing Measurement) FMT para medir distancias WiFi
 * ...

 La carga del programa principal **VERSION.ino** lleva aparejada la de los siguientes módulos y cabeceras de programa escritos en C/C++:
 1. *globales* : en donde definimos las variables globales del sistema.
 2. *BMP180* : donde gestionaremos el sensor barométrico BMP180 para calcular alturas y niveles.
 3. *MS5611* : donde gestionaremos el sensor barométrico MS5611 para calcular alturas y niveles.
 4. *HMC5883L* : software del compás.
 5. *MPU6050* : el sensor principal compuesto por un giroscopio y un acelerómetro.
 6. *GPS* : software para el GPSNEO utilizando la libreria TinyGPSplus.   
 7. *PIDcontrol* : control PID monofrecuencia para todos los grados de libertad del drone.
 8. *propulsion* : acceso directo y a bajo nivel a motores y servos.
 9. *WEB* : codigo HTML+CSS+JavaScript embebido.

 Los utilidades para gestionar el sistema se encuentran en los siguientes directorios:
 1. *I2C* : software de gestion del bus I2C.
 2. *calibracion_ESC* : para la calibración de motores y controladores eléctricos.
 3. *calibracion_servos* : para la calibración de los servos.
 4. *filtros* : filtros "Kalman" y "Complementario".
 5. *sockets* : utilidades de conexion TCP y UDP.
 6. *WEB* : codigo HTML+CSS+JavaScript embebido, pero sin restricciones de seguridad.

 