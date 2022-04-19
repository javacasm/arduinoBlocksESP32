#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp280;

void setup()
{
    Serial.begin(115200);
  Serial.flush();
  while(Serial.available()>0)Serial.read();

  bmp280.begin(0x76);
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,Adafruit_BMP280::STANDBY_MS_500);
}


void loop()
{
  yield();

    Serial.println((bmp280.readPressure()/100.0));
    Serial.println(bmp280.readTemperature());
    delay(1000);

}
