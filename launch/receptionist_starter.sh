#!/bin/bash

# Wechsle zum angegebenen Verzeichnis
#cd /home/suturo/suturo23_24/nlp_ws/src/suturo_nlp/activate_language_processing/scripts

cd /home/jule/nlp_ws/src/suturo_nlp/activate_language_processing/scripts

source /home/jule/nlp_ws/rasa_venv/bin/activate


# Führe das Python-Skript aus
python3 activate_language_processing.py

# Rufe die Funktion run_rasa aus der .bashrc-Datei auf
source ~/.bashrc  # Lädt die .bashrc-Datei, um sicherzustellen, dass run_rasa verfügbar ist
#run_rasa
