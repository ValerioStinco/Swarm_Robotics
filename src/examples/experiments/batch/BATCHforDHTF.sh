#!/bin/bash

### ./src/examples/experiments/batch/BATCHforDHTF.sh src/examples/experiments/batch/ kilobot_ALF_dhtf_server_TEMP.argos kilobot_ALF_dhtf_client_TEMP.argos
if [ "$#" -ne 3 ]; then
    echo "Usage: simple_experiment_dhtf.sh (from src folder) <base_config_dir> <base_configSERVER_file_name> <base_configCLIENT_file_name>"
    exit 11
fi


wdir=`pwd`
base_configSERVER=$1$2
base_configCLIENT=$1$3


if [ ! -e $base_configSERVER ]; then
    base_configSERVER=$wdir/$1/$2
    if [ ! -e $base_configSERVER ]; then
        echo "Error: missing configuration file '$base_configSERVER'" 1>&2
        exit 1
    fi
fi
if [ ! -e $base_configCLIENT ]; then
    base_configCLIENT=$wdir/$1/$3
    if [ ! -e $base_configCLIENT ]; then
        echo "Error: missing configuration file '$base_configCLIENT'" 1>&2
        exit 1
    fi
fi


res_dir=$wdir/"results/batch_for_dhtf"
if [[ ! -e $res_dir ]]; then
    mkdir $res_dir
    echo "CARTELLA RES PRONTA"
#else
    # echo "Error: directory '$res_dir' already exists" 
    # exit 1
fi

###
#CONTROLLARE
###
base_dir=`dirname $base_configCLIENT`
echo base_dir $base_dir
echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1



augmented_knowlege="true false"
timeout_const="5 10 15 20"
entity_quantity="20" # 30 40"

#################################
# experiment_length is in seconds
#################################
experiment_length="3600"
date_time=`date "+%Y-%m-%d"`
RUNS=50

for _K_ in $entity_quantity; do
    for _T_ in $timeout_const; do
        for _A_ in $augmented_knowlege; do
        param_dir=$res_dir/$date_time"_robots#"$_K_"_timeout_const#"$_T_"_augmmented_knowledge#"$_A_
        if [[ ! -e $param_dir ]]; then
            mkdir $param_dir
            #echo "CARTELLA PARAM PRONTA"
        fi

            for it in $(seq 1 $RUNS); do
            #SERVER
            configSERVER=`printf 'configSERVER_num_of_kbs%d_timeout_const%02d_augmented_knowledge%03d_seed%03d.argos' $_K_ $_T_ $_T_ $it`
                echo configSERVER $configSERVER
                cp $base_configSERVER $configSERVER
                sed -i "s|__ENTITY_QUANTITY__|$_K_|g" $configSERVER
                sed -i "s|__TIMEOUT_CONST__|$_T_|g" $configSERVER
                sed -i "s|__AUGMENTED_KNOWLEDGE__|$_A_|g" $configSERVER                
                sed -i "s|__EXPERIMENT_LENGTH__|$experiment_length|g" $configSERVER
                sed -i "s|__SEED__|$it|g" $configSERVER                
                output_file="seed#${it}_results.csv"
                sed -i "s|__OUTPUT__|$output_file|g" $configSERVER

            #CLIENT
            configCLIENT=`printf 'configCLIENT_num_of_kbs%d_timeout_const%02d_augmented_knowledge%03d_seed%03d.argos' $_K_ $_T_ $_T_ $it`
                echo configCLIENT $configCLIENT
                cp $base_configCLIENT $configCLIENT
                sed -i "s|__ENTITY_QUANTITY__|$_K_|g" $configCLIENT
                sed -i "s|__TIMEOUT_CONST__|$_T_|g" $configCLIENT
                sed -i "s|__AUGMENTED_KNOWLEDGE__|$_A_|g" $configCLIENT                
                sed -i "s|__EXPERIMENT_LENGTH__|$experiment_length|g" $configCLIENT
                sed -i "s|__SEED__|$it|g" $configCLIENT                
                sed -i "s|__OUTPUT__|$output_file|g" $configCLIENT
                
                echo "argos3 -c $1$configCLIENT"

                xterm -title "server" -e "cd ~/argos3-kilobot ; argos3 -c './'$configSERVER" &
                xterm -title "client" -e "sleep 2 ; cd ~/argos3-kilobot ; argos3 -c './'$configCLIENT"
                echo "-----Running next configuration-----"

            mv $output_file $param_dir
            done
        done
    done
done

rm *.argos



