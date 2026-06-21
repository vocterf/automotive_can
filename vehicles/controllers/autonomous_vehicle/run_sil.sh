#!/bin/bash

# Definicja pancernego zestawu ścieżek
export WEBOTS_HOME="/snap/webots/current/usr/share/webots"
WEBOTS_LAUNCHER="/snap/webots/current/usr/share/webots/webots-controller"
SCRIPT_DIR="/home/vocter/projects/automotive_can/vehicles/controllers/autonomous_vehicle"

# 1. Przejście do#!/bin/bash

# Definicja pancernego zestawu ścieżek
export WEBOTS_HOME="/snap/webots/current/usr/share/webots"
WEBOTS_LAUNCHER="/snap/webots/current/usr/share/webots/webots-controller"
SCRIPT_DIR="/home/vocter/projects/automotive_can/vehicles/controllers/autonomous_vehicle"

# 1. Przejście do katalogu
cd "$SCRIPT_DIR" || exit 1

# 2. Uruchomienie z flagą -E, która pozwala sudo "zobaczyć" Twoje zmienne
sudo -E "$WEBOTS_LAUNCHER" --protocol=tcp --ip-address=127.0.0.1 ./autonomous_vehicle
