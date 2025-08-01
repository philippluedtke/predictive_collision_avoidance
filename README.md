# Über die Dateien
Repository für meine Projektarbeit über "Prädikative Kollisionsvermeidung in Co-Bot-Umgebung.

Der "streamlined_ubertragung"-Code ist für den Pico. Falls mehrere Sensoren verwendet werden muss nur sichergestellt werden
dass das Format der Datenübertragung das selbe ist und ggf. die Sensor-id angepasst werden, da ich diese im Code festlege.

Der "clusteranalyse_mit_kollisionsanalyse"-Code ist das eigentliche Programm.
Die Performance ist mäßig, wird jedoch erheblich verbessert wenn man die Visualisierung deaktiviert und die Ergebnisse stattdessen durch die Konsole ausgibt.
Wahrscheinlich muss man den Port anpassen (Standard ist COM4) wenn man das Programm ausprobieren will.
