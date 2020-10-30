#!/bin/sh
xterm -title "server" -e "cd ~/argos3-kilobot ; argos3 -c src/examples/experiments/kilobot_ALF_dhtf_server.argos"  &
xterm -title "client" -e "sleep 0.01 ; cd ~/argos3-kilobot ; argos3 -c src/examples/experiments/kilobot_ALF_dhtf_client.argos"
