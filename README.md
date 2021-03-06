# Proyecto

Vamos a medir la posición 3D de una persona con un acelerómetro y generaremos una alarma cuando esta posisición sea una determinada


## Hardware

### MCU/SOC

[Adafruit HUZZAH32 - ESP32 Feather](https://tienda.bricogeek.com/placas-adafruit-feather/1108-adafruit-huzzah32-esp32-feather.html) [(Details)](https://learn.adafruit.com/adafruit-huzzah32-esp32-feather?view=all)

![](./images/feather_3405_kit_ORIG.jpg)

![](./images/feather_3405_quarter_ORIG.jpg)

* 240 MHz dual core Tensilica LX6 microcontroller with 600 DMIPS
* 520 KB SRAM
* 802.11b/g/n HT40 Wi-Fi t
* Dual mode Bluetooth (classic and BLE)
* 4 MByte flash
* Hall sensor
* 10x capacitive touch interface
* 3 x UARTs 
* 3 x SPI 
* 2 x I2C 
* 12 x ADC de 12 bits
* 2 x I2S for Audio
* 2 x DAC de 10 bits

Conector JST para batería lipo, con cargador incorporado

Podemos leer el valor de la batería en A13/35. Debemos multiplicar por 2 el valor leído


Debemos soldar los pines

![](./images/feather_solder1.jpg)

## [Acelerómetro 3 ejes adxl345](https://tienda.bricogeek.com/acelerometros/1158-acelerometro-3-ejes-adxl345-2g4g8g16g.html) [(Details)](https://learn.adafruit.com/adxl345-digital-accelerometer?view=all)

![](./images/acelerometro-3-ejes-adxl345-2g4g8g16g.jpg)

* Sensor Analog Devices ADXL 345
* Alimentación: 3.3 a 5V
* Comunicación: I2C o SPI
* Sensibilidad configurable: +-2g/4g/8g/16g

## [Sensor de temperatura, humedad y presión BME280](https://tienda.bricogeek.com/sensores-temperatura/1116-sensor-de-temperatura-humedad-y-presion-bme280.html)[(Details)](https://www.adafruit.com/product/2652)

![](./images/BME280.jpg)

* Sensor: Bosh BME280
* Temperatura: +/- 1Cº
* Humedad: +/- 3%
* Presión: +/- 1 hPa
* Alimentación: 3.3 a 5V
* Comunicación: I2C o SPI

## [Pantalla TFT color 1.14' 240x135 - ST7789](https://tienda.bricogeek.com/pantallas-lcd/1372-pantalla-tft-color-114-240x135-st7789.html) [(Details)](https://learn.adafruit.com/adafruit-1-14-240x135-color-tft-breakout?view=all)

![](./images/pantalla-tft-color-114-240x135-st7789.jpg)


* Tipo: TFT IPS
* Chip: ST7789
* Resolución: 240x135 píxeles (260ppi)
* Diagonal: 1.14 pulgadas
* Alimentación: 3.3 - 5V
* Interfaz: SPI
* Zócalo para tarjeta MicroSD

### Herramientas de programación

Comenzaremos usando [ArduinoBlocks.com](http://www.arduinoblocks.com): una herramienta de programación con bloques ideal para iniciarse.

Instalamos [ArduinoBlock Connector](http://www.arduinoblocks.com/web/site/abconnector5)

![](./images/ArduinoBlockConnector.png)

[Creamos nuestra cuenta](http://www.arduinoblocks.com/web/site/register)

![](./images/ArduinoBlocks_registro.png)


## Led de la placa

La placa tiene incluido un led conectado al pin 13 que encenderemos y apagaremos cada cierto tiempo

![](./images/programa_led13digital.png)

[Led 13 Adafruit Huzzah ESP32](http://www.arduinoblocks.com/web/project/782635)


## Controlando el brillo de un led

Vamos a controlar ahora el brillo de un led analógicamente

Además vamos a enviar datos a la consola

[led 13 analógico Adafruit Huzzah](http://www.arduinoblocks.com/web/project/782636)

![](./images/programa_ledAnalogico.png)


Activamos la consola, seleccionamos la velocidad adecuada y pulsamos "Conectar"

![](./images/ConsolaSerie.png)

## Placa de prototipo

Es como una regleta de conexiones

![¿cómo funciona una placa prototipo?](./images/breadboard1.gif)

## Potenciómetro

Vamos a medir ahora un valor analógico, el de un potenciómetro. A partir de esta medida calcularemos el brillo de un led. Para ello usaremos unas variables.

![](./images/led_Pot_bb.png)

![](./images/led_Pot_esquematico.png)

[Potenciómetro y led 13 analógico Huzzah](http://www.arduinoblocks.com/web/project/782641)
![](./images/programa_potenciometro.png)

## LED RGB

Son 3 leds de colores Rojo, Azul y Verde en un único encapsulado, con una de las patillas conectadas, en nuestro caso es el pin negativo. Conectamos unas resistencias en serie para limitar la cantidad de corriente y el voltaje que llega al led.

![](./images/led_RGB_bb.png)

![](./images/led_RGB_esquematico.png)


Podemos crear colores combinando el brillo de cada uno

![](./images/Colores-MezclaRGB.jpeg)

![](./images/programa_ledRGB.png)

[Proyecto led RGB](http://www.arduinoblocks.com/web/project/782626)


## Led RGB con función

Para controlar mejor el color del led vamos a crear una función, que es una manera de reutilizar nuestro código muchas veces de manera sencilla

![](./images/ledRGB_funcion.png)

[Proyecto: led RGB con función](http://www.arduinoblocks.com/web/project/782650)

## Acelerómetro 3 ejes adxl345

![](./images/accel-pitch-roll-yaw.jpg)

Vamos a conectar un Acelerómetro de 3 ejes con conexión I2C


![](./images/led_RGB_accelerometro_bb_arduinoblocks.png)

Vamos a mostrar gráficamente los valores


![](./images/programa_plotter_acelerometro.png)

[Plotter acelerometro ADXL345 ESP32](http://www.arduinoblocks.com/web/project/782655)

![](./images/grafico_Accel.png)


Mostramos la aceleración con colores

![](./images/programa_ledRGB_accel.png)

[Acelerómetro y led RGB](http://www.arduinoblocks.com/web/project/782651)

Ahora podemos desconectar el dispositivo del PC y comprobar el funcionamiento


### IOT

¿Qué es IOT?


![](./images/Internet_de_las_Cosas.jpg)

### MQTT

¿Qué es MQTT?

![](./images/MQTT_arquitectura.png)

![](./images/mqtt-architecture.png)

### Publicación en Adafruit IO

![](./images/IOT%2Breles.png)


![](./images/programa_accel_mqtt_adafruitIO.png)

[Publicación de la Acceleración en adafruit.IO con MQTT](http://www.arduinoblocks.com/web/project/782713)

### Medida de humedad y temperatura

Vamos a medir ahora la temperatura, la presión y la humedad usando un sensor BME280, que conectamos también por I2C, en este caso de la siguiente manera, conectando SDA al pin SDI del sensor y SCL al SCK del sensor

![](./images/led_RGB_bme280_bb.png)

Desgraciadamente, arduinoBlocks no dispone de bloque para el sensor BME280, por eso vamos a usar el código de [ejemplo](./codigo/test_BME280/test_BME280.ino) de la librería Adafruit BME280 desde el Gestor de librerías (Programa->Librerías->Administrador de librerías) de [Arduino IDE](https://arduino.cc/software)

Al conectar el sensor y subir el código obtendremos valores bastante precisos:


    Temperature = 22.58 °C
    Pressure = 933.06 hPa
    Approx. Altitude = 690.11 m
    Humidity = 53.76 %


## Pantalla

Conexión SPI:

![](./images/led_bme280_pantalla_bb.png)

Necesitamos instalar las librerías "adafruit ST7778" y usaremos uno de [sus ejemplos](./codigo/test_ST7789/test_ST7789.ino) para probar que funciona

```C++
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>


#define TFT_CS         14
#define TFT_RST        15
#define TFT_DC         4

// For 1.14", 1.3", 1.54", 1.69", and 2.0" TFT with ST7789:
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);



void setup(void) {
  Serial.begin(115200);
  Serial.print("Temperatura, presion y humedad");

  // OR use this initializer (uncomment) if using a 1.14" 240x135 TFT:
  tft.init(135, 240);           // Init ST7789 240x135
  
  Serial.println(F("Initializada"));

  uint16_t time = millis();
  tft.fillScreen(ST77XX_BLACK);
  time = millis() - time;

  Serial.println(time);
  
}

int counter = 10;
void loop() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(4);
  tft.print(counter++);
  delay(10);
}

```