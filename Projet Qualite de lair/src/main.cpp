#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2CSfa3x.h>
#include <Adafruit_BME280.h>
#include <VOCGasIndexAlgorithm.h>

// Déclaration des variables et des objets pour les capteurs
float sampling_interval = 1.f;
SensirionI2CSgp40 sgp40;
SensirionI2CSfa3x sfa3x;
Adafruit_BME280 bme;
VOCGasIndexAlgorithm voc_algorithm(sampling_interval);
float Temp;  // Température
int Alde;      // Formaldéhyde
float Hum;    // Humidité

// Déclaration de la fonction pour mesurer la valeur brute du signal SGP40 en mode faible consommation
void sgp40MeasureRawSignalLowPower(uint16_t compensationRh, uint16_t compensationT, uint16_t* error, int32_t voc_index);
void initcapteurT(){
    // Initialisation du capteur BME280
    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
}
void initcapteurC(){
    // Initialisation du capteur SGP40
    sgp40.begin(Wire);

    // Récupération du numéro de série du capteur SGP40
    uint16_t serialNumber[3];
    uint8_t serialNumberSize = 3;
    uint16_t error = sgp40.getSerialNumber(serialNumber, serialNumberSize);
    Serial.print("Sampling interval (sec):\t");
    Serial.println(voc_algorithm.get_sampling_interval());
    Serial.println("");

    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        // Gérer l'erreur si nécessaire
    } else {
        Serial.print("SerialNumber: ");
        for (size_t i = 0; i < serialNumberSize; i++) {
            Serial.print("0x");
            Serial.print(serialNumber[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    uint16_t testResult;

    // Test d'auto-étalonnage du capteur SGP40
    error = sgp40.executeSelfTest(testResult);
    if (error) {
        Serial.print("Error trying to execute executeSelfTest(): ");
        // Gérer l'erreur si nécessaire
    } else if (testResult != 0xD400) {
        Serial.print("executeSelfTest failed with error: ");
        Serial.println(testResult, HEX);
    }

}
void initcapteurF(){
   sfa3x.begin(Wire);

    // Démarrage de la mesure continue avec SFA3x
    int error = sfa3x.startContinuousMeasurement();
    if (error) {
        Serial.print("Error trying to execute startContinuousMeasurement(): ");
        // Gérer l'erreur si nécessaire
    }
}

void setup() {
    // Initialisation de la communication série
    Serial.begin(115200);
    // Attente que la communication série soit disponible
    while (!Serial) {
        delay(100);
    }
    // Initialisation de la communication I2C
    Wire.begin();
    initcapteurC();
    initcapteurT();
    initcapteurF();

}

void capteurTFC() {
 // Mesure de la valeur brute du signal SGP40 en mode faible consommation
    uint16_t error;
    char errorMessage[256];
    uint16_t compensationRh = 0x8000;  // Valeur de compensation à ajuster
    uint16_t compensationT = 0x6666;    // Valeur de compensation à ajuster
    int32_t voc_index = 0;

    // Appel de la fonction pour mesurer la valeur brute du signal SGP40
    sgp40MeasureRawSignalLowPower(compensationRh, compensationT, &error, voc_index);

    // Mesure des valeurs SFA3x
    delay(1000);
    int16_t hcho;      // Formaldéhyde
    int16_t humidity;  // Humidité
    int16_t temperature;
    error = sfa3x.readMeasuredValues(hcho, humidity, temperature);
    if (error) {
        Serial.print("Error trying to execute readMeasuredValues(): ");
        // Gérer l'erreur si nécessaire
    } else {
        Alde = hcho / 5.0;
        Serial.print("Hcho:");
        Serial.print(Alde);
        Serial.println("\t");
    }

    // Mesure des valeurs BME280
    Serial.println("\t");
    Serial.println("\t");
    Temp = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(Temp);
    Serial.println(" °C");

    Hum = bme.readHumidity();
    Serial.print("Humidité = ");
    Serial.print(Hum);
    Serial.println(" %");


}
void loop() {
    capteurTFC();
    delay(2000);  // Attendre 2 secondes entre chaque lecture
}

// Fonction pour mesurer la valeur brute du signal SGP40 en mode faible consommation
void sgp40MeasureRawSignalLowPower(uint16_t compensationRh, uint16_t compensationT, uint16_t* error, int32_t voc_index) {
    uint16_t srawVoc = 0;

    // Demande d'une première mesure pour chauffer la plaque (ignorant le résultat)
    *error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
    if (*error) {
        return;
    }

    // Délai de 170 ms pour laisser la plaque chauffer.
    // En gardant à l'esprit que la commande de mesure inclut déjà un délai de 30 ms
    delay(140);

    // Demande des valeurs de mesure
    *error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
    if (*error) {
        return;
    }

    Serial.print("srawVOC: ");
    Serial.println(srawVoc);

    // Désactiver le chauffage
    *error = sgp40.turnHeaterOff();
    if (*error) {
        return;
    }

    // Traitement des signaux bruts par l'algorithme d'indice de gaz VOC pour obtenir les valeurs de l'indice VOC
    voc_index = voc_algorithm.process(srawVoc);
    int COV = voc_index;
    Serial.print("COV Indice: ");
    Serial.println(COV);
}
