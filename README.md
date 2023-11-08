# Projet de main robotisée partie carte main robotisée

Objectif : piloter une main robotisée à partir d'un gant équipé de capteurs de flexions

Le projet se décompose en deux parties :

- Une partie main robotisée pilotée par un ESP32 qui reçoit des ordres du gant équipé capteur via MQTT
- Une partie gant capteur équipée d’un ESP32 chargé de lire les capteurs placé sur les doigts de l’utilisateur et de transmettre les données à la carte main robotisée via MQTT
Le développement du code se fera sous l'IDE Visual Studio Code avec l'extension PlatformIO.
