#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: simple_experiment.sh (from src folder) <base_config_dir> <base_config_file_name>"
    exit 11
fi

wdir=`pwd`
base_config=$1$2
if [ ! -e $base_config ]; then
    base_config=$wdir/$1/$2
    if [ ! -e $base_config ]; then
        echo "Error: missing configuration file '$base_config'" 1>&2
        exit 1
    fi
fi

res_dir=$wdir/"results/simple_experiment_dhtf"
if [[ ! -e $res_dir ]]; then
    mkdir $res_dir
else
    echo "Error: directory '$res_dir' already exists" 
    exit 1
fi

base_dir=`dirname $base_config`
echo base_dir $base_dir
echo "$CONFIGURATION_FILE" | egrep "^$SHARED_DIR" &> /dev/null || exit 1



augmented_knowlege="true false"
timeout_const="5 15"
entity_quantity="20" ### 30 40"

#################################
# experiment_length is in seconds
#################################
experiment_length="1800"
date_time=`date "+%Y-%m-%d"`
RUNS=20

for _K_ in $entity_quantity; do
    for _T_ in $timeout_const; do
        for _A_ in $augmented_knowlege; do
        param_dir=$res_dir/$date_time"_robots#"$_K_"_timeout_const#"$_T_"_augmmented_knowledge#"$_A_
        "_"$experiment_length
        if [[ ! -e $param_dir ]]; then
            mkdir $param_dir
        fi

            for it in $(seq 1 $RUNS); do

                config=`printf 'config_num_of_kbs%d_timeout_const%02d_augmented_knowledge%03d_seed%03d.argos' $_K_ $_T_ $_T_ $it`
                echo config $config
                cp $base_config $config
                sed -i "s|__NUMROBOTS__|$_K_|g" $config
                sed -i "s|__TIMEOUT_CONST__|$_T_|g" $config
                sed -i "s|__AUGMENTED_KNOWLEDGE__|$_A_|g" $config                
                sed -i "s|__TIMEEXPERIMENT__|$experiment_length|g" $config
                sed -i "s|__SEED__|$it|g" $config                
                output_file="seed#${it}_time_results.tsv"
                sed -i "s|__OUTPUT__|$output_file|g" $config
                
                echo "Running next configuration Robots $num_of_kbs timeout_const $timeout augmmented_knowledge $knowlege"
                echo "argos3 -c $1$config"
                argos3 -c './'$config
            mv $output_file $param_dir
            done
        done
    done
done

rm *.argos



