#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>

BLEAdvertising *advertising;

RTC_DATA_ATTR static uint8_t bootCount = 0;

void setup()
{
    Serial.begin(115200);

    if (bootCount++ == UINT8_MAX)
    {
        bootCount = 0;
    }

    BLEDevice::init("");
    advertising = BLEDevice::getAdvertising();

    BLEAdvertisementData advertisement = BLEAdvertisementData();
    BLEAdvertisementData scanResponse = BLEAdvertisementData();

    advertisement.setFlags(0x04);

    std::string manufactureData = "";
    manufactureData += (char)4;
    manufactureData += (char)0xFF;
    manufactureData += (char)0x12;
    manufactureData += (char)0x21;
    manufactureData += (char)bootCount;
    advertisement.addData(manufactureData);

    advertising->setAdvertisementData(advertisement);
    advertising->setAdvertisementType(ADV_TYPE_NONCONN_IND);

    Serial.println("advertising");
    advertising->start();
    delay(100);
    advertising->stop();
    Serial.println("sleeping");

    esp_deep_sleep(1000000LL * 10);
}

void loop()
{
}