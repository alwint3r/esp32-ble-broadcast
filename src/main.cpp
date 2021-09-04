#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <Wire.h>
#include <shtc3.h>

BLEAdvertising *advertising;

typedef struct __manufacture_data_t
{
    uint16_t manufacturer_id;
    uint8_t measurement_ok;
    int16_t temperature;
    int16_t humidity;
} __attribute__((packed)) manufacture_data_t;

shtc3_error_t i2c_write(shtc3_i2c_config_t *config, const uint8_t *data, size_t length)
{
    Wire.beginTransmission(config->i2c_address);
    size_t written = Wire.write(data, length);
    uint8_t result = Wire.endTransmission();

    log_i("written=%d result=%d", written, result);

    if (result == I2C_ERROR_OK)
    {
        return SHTC3_OK;
    }
    return SHTC3_ERROR;
}

shtc3_error_t i2c_read(shtc3_i2c_config_t *config, uint8_t *data, size_t length)
{
    Wire.beginTransmission(config->i2c_address);
    uint8_t available = Wire.requestFrom(config->i2c_address, length);
    shtc3_error_t result = SHTC3_OK;

    log_i("available=%d requested=%d", available, length);

    if (available == length)
    {
        Wire.readBytes(data, length);
    }
    else
    {
        result = SHTC3_ERROR;
    }

    Wire.endTransmission();

    return result;
}

void delay_fn(uint32_t milliseconds)
{
    delay(milliseconds);
}

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    manufacture_data_t data = {};
    data.manufacturer_id = 0x1221;

    delay(1000);

    shtc3_t *sensor = shtc3_init(i2c_read, i2c_write, delay_fn, 0x70);
    shtc3_measurement_t measurement = {};

    shtc3_error_t err = shtc3_wakeup(sensor);
    
    delay(500);

    err = shtc3_measure(sensor, SHTC3_MEASUREMENT_POLL, SHTC3_MEASUREMENT_PWR_NORMAL, &measurement);
    if (err == SHTC3_OK)
    {
        data.humidity = (int16_t)(measurement.humidity * 100);
        data.temperature = (int16_t)(measurement.temperature * 100);
        data.measurement_ok = 1;
    }
    else
    {
        data.humidity = 0;
        data.temperature = 0;
        data.measurement_ok = 0;
    }

    err = shtc3_sleep(sensor);

    log_i("measurement_ok=%d\ttemperature=%f\thumidity=%f", data.measurement_ok, measurement.temperature, measurement.humidity);

    std::string packed_measurement = std::string((char *)&data, sizeof(data));

    BLEDevice::init("");
    advertising = BLEDevice::getAdvertising();

    BLEAdvertisementData advertisement = BLEAdvertisementData();
    BLEAdvertisementData scanResponse = BLEAdvertisementData();

    advertisement.setFlags(0x06);

    std::string manufactureData = "";
    manufactureData += (char)8;
    manufactureData += (char)0xFF;
    manufactureData += packed_measurement;
    advertisement.addData(manufactureData);

    advertising->setAdvertisementData(advertisement);
    advertising->setAdvertisementType(ADV_TYPE_NONCONN_IND);

    log_i("advertising");
    advertising->start();
    delay(100);
    advertising->stop();
    log_i("sleeping");

    esp_deep_sleep(1000000LL * 10);
}

void loop()
{
}