#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define GPS_RX 4  // Arduino reçoit (TX du SIM808)
#define GPS_TX 3  // Arduino émet (RX du SIM808)

#define BUZZER_PIN 8
#define SPEED_THRESHOLD_KMH 20.0

TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

bool isBuzzing = false;
unsigned long buzzerStartTime = 0;
const unsigned long BUZZER_DURATION_MS = 15000;

void setup() {
  Serial.begin(9600);           // Pour monitor série
  gpsSerial.begin(9600);        // Pour GPS
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("GPS Speed Monitoring Started");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());

    if (gps.location.isValid() && gps.speed.isValid()) {
      double speed_kmh = gps.speed.kmph();
      Serial.print("Speed (km/h): ");
      Serial.println(speed_kmh);

      if (speed_kmh > SPEED_THRESHOLD_KMH && !isBuzzing) {
        isBuzzing = true;
        buzzerStartTime = millis();
        digitalWrite(BUZZER_PIN, HIGH);  // Allumer le buzzer
        Serial.println("!! Vitesse > 20 km/h : Buzzer activé");
      }
    }
  }

  // Arrêter le buzzer après 15 secondes
  if (isBuzzing && millis() - buzzerStartTime >= BUZZER_DURATION_MS) {
    digitalWrite(BUZZER_PIN, LOW);
    isBuzzing = false;
    Serial.println("Buzzer arrêté après 15 secondes");
  }
}
