#!/bin/sh
xterm -title "server" -e "cd ~/argos3-kilobot ; argos3 -c src/examples/experiments/configSERVER_num_of_kbs20_timeout_const05_augmented_knowledge005_seed001"  &
xterm -title "client" -e "sleep 0.01 ; cd ~/argos3-kilobot ; argos3 -c src/examples/experiments/configCLIENT_num_of_kbs20_timeout_const05_augmented_knowledge005_seed001"