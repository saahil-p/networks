set terminal png size 800,600
set output "final_pdr.png"
set title 'Packet loss'
set xlabel 'Time(s)'
set ylabel 'Packets Lost'
n_variants = 3

array file_list[3] = ["flow_stats_TcpIllinois.txt","flow_stats_TcpBic.txt","flow_stats_TcpScalable.txt"]
array label_list[3] = ["TCP Illinois", "TCP Bic","TCP Scalable"]


set multiplot layout 3,3 columns
plot for[i =1 : n_variants] file_list[i] using 1:8 with lines title sprintf("%s",label_list[i])

set style data histograms
set style fill solid
do for[i = 1:n_variants]{
	set xlabel 'Sender Number'
	set title label_list[i]
	plot file_list[i] using 6 with histogram title "TxPkts","" using 7 with histogram title "RxPkts"
}

set xlabel 'Variants'
set title 'Packet Tx,Rx vs Variant'
set ylabel 'Packet count'

plot 'packet_stats.txt' using 2 with histogram title "TxPkts",'' using 3 with histogram title "RxPkts"
