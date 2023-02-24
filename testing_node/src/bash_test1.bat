#!/bin/bash
for i in {0..20}
do
   echo "========================================================\n"
   echo "This is the $i test"
   echo "========================================================\n"
   source /home/fer/Desktop/catkin_ws/devel/setup.bash
   python /home/fer/Desktop/catkin_ws/src/AMCL_Hybrid/testing_node/src/run_tests_from_bash.py "hola"
   echo "Tomando screenshot de Rviz al final de la trayectoria"
   shutter -a -o amcl_test$i.png -e
done
