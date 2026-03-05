package main 

/*
todo1: coba coba parameter  inetial flow , dengan tujuan minimze cut edges di graph partition -> jumlah shortcuts lebih dikit -> p2p query runtime jauh lebih cepet
INERTIAL_FLOW_ITERATION             = 35
BISECTION_WORKERS                   = 20
SOURCE_SINK_RATE                    = 0.1
INERTIAL_FLOW_ITERATION_LARGE_GRAPH = 4

kemarin pake kaffpa (https://github.com/KaHIP/KaHIP) bisa ngehasilin 1jt shorcuts jauh lebih dikit dari implementasi inertial flow (best 3.7 jt) disini, pas load test p2p query runtime nya lebih cepet dari osrm (avg 5.99 ms ini vs 7 ms osrm)...


todo2: benerin bug di alternative routes setelah update QueryHeap
todo3: coba2 buat ngurangin memory alloc hasil dari init routing engine (3.5 gb vs osrm 490mb buat osm diy_solo_semarang)

todo4: update implmeentasi algoritma intertial flow agar jumlah cut edges minimized -> minimize jumlah shortcuts. 
todo4 DONE: tambahin prepartitionWithSCC  dari 3.7 jt to 700k shortcuts
dengan 700 k shortcuts navigatorx multilevel-ALT query:  avg latency 6.23 ms, p95 latency 11.28ms, ngalahin osrm MLD: avg latency 10.25 ms, p95 latency 23.6ms
mungkin bisa coba todo1 buat kurangin jumlah shorctuts lagi...
*/


