set terminal png size 800,600
set output "final_throughput.png"
set title 'Throughput Over Time'
set xlabel 'Time (s)'
set ylabel 'Throughput (Mbit/s)'
n_variants = 3

array file_list[3] = ["throughput_TcpIllinois.txt","throughput_TcpBic.txt","throughput_TcpScalable.txt"]
array label_list[3] = ["TCP Illinois", "TCP Bic","TCP Scalable"]

plot for[i =1 : n_variants] file_list[i] using 1:2 with lines title sprintf("%s",label_list[i])
