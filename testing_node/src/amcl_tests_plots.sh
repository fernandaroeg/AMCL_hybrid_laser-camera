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
trap myInterruptHandler INT;

# before calling the inner program,
# disable the abortability of the script
allowAbort=false;

#now call your program
echo "Ejecutando pruebas AMCL y exportar datos como .txt"
source /home/fer/Desktop/catkin_ws/devel/setup.bash
roslaunch testing_node export_data.launch #file_name:=test$i

# and now make the script abortable again
allowAbort=true;

sleep 8

echo "Tomando screenshot de Rviz al final de la trayectoria"
shutter -a -o amcl_test_probando_bash.png -e


echo "Calculando métricas y compilando plots"
source /home/fer/Desktop/catkin_ws/devel/setup.bash
roslaunch test_node compute_metrics_plots.launch param i num de la prueba

echo "Finalizado. Se realizaron n pruebas"
