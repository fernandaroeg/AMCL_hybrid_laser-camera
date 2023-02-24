#!/usr/bin/env bash
source /home/fer/Desktop/catkin_ws/devel/setup.bash 

for i in {1..20}
do
	echo "Ejecutando pruebas AMCL y exportar datos como .txt"
	roslaunch testing_node export_data.launch file_name:=test$i
	
	sleep 180
	
	echo "Tomando screenshot de Rviz al final de la trayectoria"
	shutter -a -o amcl_test$i.png -e
	
	echo "Calculando métricas y compilando plots"
	roslaunch test_node compute_metrics_plots.launch param i num de la prueba
	
	echo "Finalizado. Se realizaron n pruebas"
done

#!/bin/bash

# define signal handler and its variable
allowAbort=true;
myInterruptHandler()
{
    if $allowAbort; then
        exit 1;
    fi;
}

# register signal handler
trap myInterruptHandler SIGINT;

# some commands...

# before calling the inner program,
# disable the abortability of the script
allowAbort=false;
# now call your program
./my-inner-program
# and now make the script abortable again
allowAbort=true;
