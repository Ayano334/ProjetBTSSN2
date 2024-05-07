#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2CSfa3x.h>
#include <Adafruit_BME280.h>
#include <VOCGasIndexAlgorithm.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Déclaration des variables et des objets pour les capteurs
float sampling_interval = 1.f;
SensirionI2CSgp40 sgp40;
SensirionI2CSfa3x sfa3x;
Adafruit_BME280 bme;
VOCGasIndexAlgorithm voc_algorithm(sampling_interval);
float Temp;  // Température
int Alde;      // Formaldéhyde
float Hum;    // Humidité

// Déclaration des UUID pour le service BLE et les caractéristiques
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TEMPERATURE_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define HUMIDITY_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define ALDEHYDE_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define VOC_INDEX_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26ab"


BLEServer* pServer;
BLEService* pService;
BLECharacteristic* pCharacteristicTemperature;
BLECharacteristic* pCharacteristicHumidity;
BLECharacteristic* pCharacteristicAldehyde;
BLECharacteristic* pCharacteristicVOCIndex;


// Déclaration de la fonction pour mesurer la valeur brute du signal SGP40 en mode faible consommation
void sgp40MeasureRawSignalLowPower(uint16_t compensationRh, uint16_t compensationT, uint16_t* error, int32_t voc_index);
void initSensors();
void measureSensors();

void setup() {
    // Initialisation de la communication série
    Serial.begin(115200);
    // Attente que la communication série soit disponible
    while (!Serial) {
        delay(100);
    }
    // Initialisation de la communication I2C
    Wire.begin();
    // Initialisation du BLE
    BLEDevice::init("ESP32_BLE_Sensors");
    pServer = BLEDevice::createServer();
    pService = pServer->createService(BLEUUID(SERVICE_UUID));
    // Création des caractéristiques BLE
    pCharacteristicTemperature = pService->createCharacteristic(BLEUUID(TEMPERATURE_UUID), BLECharacteristic::PROPERTY_READ);
    pCharacteristicHumidity = pService->createCharacteristic(BLEUUID(HUMIDITY_UUID), BLECharacteristic::PROPERTY_READ);
    pCharacteristicAldehyde = pService->createCharacteristic(BLEUUID(ALDEHYDE_UUID), BLECharacteristic::PROPERTY_READ);
    pCharacteristicVOCIndex = pService->createCharacteristic(BLEUUID(VOC_INDEX_UUID), BLECharacteristic::PROPERTY_READ);
    pCharacteristicVOCIndex->setValue("0");
    pCharacteristicTemperature->setValue("0.00");
    pCharacteristicHumidity->setValue("0.00");
    pCharacteristicAldehyde->setValue("0.00");
    
    // Démarre le service BLE
    pService->start();
    // Démarre l'annonce BLE
    pServer->getAdvertising()->start();
    // Initialise les capteurs
    initSensors();
}

void initSensors(){
    // Initialisation du capteur BME280
    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
    // Initialisation du capteur SGP40
    sgp40.begin(Wire);
    // Initialisation du capteur SFA3x
    sfa3x.begin(Wire);
    // Démarrage de la mesure continue avec SFA3x
    int error = sfa3x.startContinuousMeasurement();
    if (error) {
        Serial.print("Error trying to execute startContinuousMeasurement(): ");
        // Gérer l'erreur si nécessaire
    }
}

void loop() {
    // Mesure des valeurs des capteurs
    measureSensors();
    // Attente avant la prochaine mesure
    delay(2000);
}

void measureSensors() {
    // Mesure de la température et de l'humidité avec le capteur BME280
    Temp = bme.readTemperature();
    Hum = bme.readHumidity();
    // Mesure de la valeur brute du signal SGP40 en mode faible consommation
    uint16_t error;
    char errorMessage[256];
    uint16_t compensationRh = 0x8000;  // Valeur de compensation à ajuster
    uint16_t compensationT = 0x6666;    // Valeur de compensation à ajuster
    int32_t voc_index = 0;
    sgp40MeasureRawSignalLowPower(compensationRh, compensationT, &error, voc_index);
    // Mise à jour des valeurs des caractéristiques BLE
    pCharacteristicTemperature->setValue(String(Temp).c_str());
    pCharacteristicHumidity->setValue(String(Hum).c_str());
    pCharacteristicAldehyde->setValue(String(Alde).c_str());
    pCharacteristicVOCIndex->setValue(String(Alde).c_str());

}

// Fonction pour mesurer la valeur brute du signal SGP40 en mode faible consommation
void sgp40MeasureRawSignalLowPower(uint16_t compensationRh, uint16_t compensationT, uint16_t* error, int32_t voc_index) {
    uint16_t srawVoc = 0;
    // Demande d'une première mesure pour chauffer la plaque (ignorant le résultat)
    *error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
    if (*error) {
        return;
    }
    // Délai pour laisser la plaque chauffer
    delay(170);
    // Demande des valeurs de mesure
    *error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
    if (*error) {
        return;
    }
    // Désactiver le chauffage
    *error = sgp40.turnHeaterOff();
    if (*error) {
        return;
    }
    // Traitement des signaux bruts par l'algorithme d'indice de gaz VOC
    voc_index = voc_algorithm.process(srawVoc);
    Alde = voc_index;
}
