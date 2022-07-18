echo "Ejecutando pruebas AMCL"
roslaunch odom_data_adapter test_amcl.launch

echo "Exportando resultados como csv"
roslaunch test_node export_data.launch
sleep 3min

echo "Calculando métricas y compilando plots"
roslaunch test_node compute_metrics_plots.launch

echo "Finalizado. Se realizaron n pruebas"


for i in 'seq 1 10'
do
	touch hola$i.txt
done
