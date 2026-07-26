# Input OpenStreetMap Jateng Jabar

## Preprocessing

```
./bin/preprocessor --osm_file=./data/jateng_jabar.osm.pbf --mlp_file=./data/jateng_jabar.mlp --region=jateng_jabar  --us=8,11,14,17,18
2026-07-26T12:52:53.034061368+07:00     info    parsing openstreetmap .pbf file......
2026-07-26T12:53:38.222423492+07:00     info    processing openstreetmap .pbf file: 0%....
2026-07-26T12:53:48.184605597+07:00     info    processing openstreetmap .pbf file: 5 % .....
2026-07-26T12:53:49.694401473+07:00     info    processing openstreetmap .pbf file: 10 % .....
2026-07-26T12:53:51.131821745+07:00     info    processing openstreetmap .pbf file: 15 % .....
2026-07-26T12:53:51.861753205+07:00     info    processing openstreetmap .pbf file: 20 % .....
2026-07-26T12:53:52.562359039+07:00     info    processing openstreetmap .pbf file: 25 % .....
2026-07-26T12:53:53.586968994+07:00     info    processing openstreetmap .pbf file: 30 % .....
2026-07-26T12:53:54.865582265+07:00     info    processing openstreetmap .pbf file: 35 % .....
2026-07-26T12:53:55.91229715+07:00      info    processing openstreetmap .pbf file: 40 % .....
2026-07-26T12:53:57.001931763+07:00     info    processing openstreetmap .pbf file: 45 % .....
2026-07-26T12:53:58.221417677+07:00     info    processing openstreetmap .pbf file: 50 % .....
2026-07-26T12:53:59.319201994+07:00     info    processing openstreetmap .pbf file: 55 % .....
2026-07-26T12:54:00.390655315+07:00     info    processing openstreetmap .pbf file: 60 % .....
2026-07-26T12:54:01.097961482+07:00     info    processing openstreetmap .pbf file: 65 % .....
2026-07-26T12:54:01.90424344+07:00      info    processing openstreetmap .pbf file: 70 % .....
2026-07-26T12:54:03.437453262+07:00     info    processing openstreetmap .pbf file: 75 % .....
2026-07-26T12:54:04.168524012+07:00     info    processing openstreetmap .pbf file: 80 % .....
2026-07-26T12:54:05.288847465+07:00     info    processing openstreetmap .pbf file: 85 % .....
2026-07-26T12:54:06.051387116+07:00     info    processing openstreetmap .pbf file: 90 % .....
2026-07-26T12:54:06.841046945+07:00     info    processing openstreetmap .pbf file: 95 % .....
2026-07-26T12:54:07.570421264+07:00     info    processing openstreetmap .pbf file: 100 % .....
2026-07-26T12:54:07.77575955+07:00      info    building road network graph....
2026-07-26T12:54:26.703076775+07:00     info    number of vertices: 2955569
2026-07-26T12:54:26.703109354+07:00     info    number of edges: 7105876
2026-07-26T12:54:26.713067458+07:00     info    partitioning level 5 with max cell size 262144
2026-07-26T12:56:54.08261594+07:00      info    level 5 done, total cells: 18
2026-07-26T12:56:54.082652566+07:00     info    partitioning level 4 with max cell size 131072
2026-07-26T12:57:21.179062099+07:00     info    level 4 total cells: 56
2026-07-26T12:57:21.179095913+07:00     info    partitioning level 3 with max cell size 16384
2026-07-26T12:58:49.226863975+07:00     info    level 3 total cells: 348
2026-07-26T12:58:49.226894932+07:00     info    partitioning level 2 with max cell size 2048
2026-07-26T12:59:25.696600632+07:00     info    level 2 total cells: 2666
2026-07-26T12:59:25.696629025+07:00     info    partitioning level 1 with max cell size 256
2026-07-26T13:00:16.698243882+07:00     info    level 1 total cells: 20941
2026-07-26T13:00:22.814042234+07:00     info    done partitioning... time taken: 449.779629851 s
2026-07-26T13:00:22.918506457+07:00     info    Starting preprocessing step of Customizable Route Planning...
2026-07-26T13:00:22.926403236+07:00     info    Building Overlay Graph of each levels...
2026-07-26T13:00:25.328545546+07:00     info    Overlay graph built and written to ./data/overlay_graph.ngraph
2026-07-26T13:00:25.328598825+07:00     info    overlay graph level 5: number of overlay vertices 2454
2026-07-26T13:00:25.328603069+07:00     info    overlay graph level 4: number of overlay vertices 4454
2026-07-26T13:00:25.328608274+07:00     info    overlay graph level 3: number of overlay vertices 15598
2026-07-26T13:00:25.328611167+07:00     info    overlay graph level 2: number of overlay vertices 63000
2026-07-26T13:00:25.328613509+07:00     info    overlay graph level 1: number of overlay vertices 259982
2026-07-26T13:00:25.328615981+07:00     info    Running Kosaraju's algorithm to find strongly connected components (SCCs)...
2026-07-26T13:00:57.128819618+07:00     info    Writing graph to ./data/original.ngraph
2026-07-26T13:00:57.489259554+07:00     info    writing 73975 graph tiles to files...
2026-07-26T13:01:28.571796335+07:00     info    completed writing tiles to files
```

## Customization

```
./bin/customizer --region=jateng_jabar
2026-07-26T12:48:52.097880627+07:00	info	Starting customization step of Customizable Route Planning...
2026-07-26T12:48:52.097949828+07:00	info	Reading graph from ./data/profiles/car/jateng_jabar_original.ngraph
2026-07-26T12:48:54.282415319+07:00	info	Reading overlay graph from ./data/profiles/car/jateng_jabar_overlay_graph.ngraph
2026-07-26T12:48:54.424799649+07:00	info	Building cliques for each cell for each overlay graph level...
2026-07-26T12:48:54.426552619+07:00	info	number of shortcuts: 2096059
2026-07-26T12:48:58.562722613+07:00	info	computing landmarks....
2026-07-26T12:49:46.714220446+07:00	info	number of shortcuts overlay graph level 1: 1111102
2026-07-26T12:49:46.7142813+07:00	info	finished crp customization level 1
2026-07-26T12:50:05.397130774+07:00	info	number of shortcuts overlay graph level 2: 497066
2026-07-26T12:50:05.397214592+07:00	info	finished crp customization level 2
2026-07-26T12:50:10.850350985+07:00	info	number of shortcuts overlay graph level 3: 242130
2026-07-26T12:50:10.850455803+07:00	info	finished crp customization level 3
2026-07-26T12:50:12.80424388+07:00	info	number of shortcuts overlay graph level 4: 153647
2026-07-26T12:50:12.804363305+07:00	info	finished crp customization level 4
2026-07-26T12:50:13.759899217+07:00	info	number of shortcuts overlay graph level 5: 92114
2026-07-26T12:50:13.759962626+07:00	info	finished crp customization level 5
2026-07-26T12:50:13.759986702+07:00	info	Building stalling tables...
2026-07-26T12:50:36.392572029+07:00	info	done computing landmarks....
2026-07-26T12:50:37.356199399+07:00	info	Customization step completed successfully.
```

## Avg runtime

```
2026-07-26T14:37:34.853551686+07:00     info    Starting query engine....
2026-07-26T14:37:34.853633908+07:00     info    Reading graph....
2026-07-26T14:37:37.159534059+07:00     info    Reading overlay graph....
2026-07-26T14:37:37.192360159+07:00     info    Reading stalling tables & metrics...
2026-07-26T14:37:38.436172718+07:00     info    starting benchmark
2026-07-26T14:37:41.465156887+07:00     info    done query 1000
2026-07-26T14:37:44.100198159+07:00     info    done query 2000
2026-07-26T14:37:46.490120068+07:00     info    done query 3000
2026-07-26T14:37:48.939290811+07:00     info    done query 4000
2026-07-26T14:37:51.383286901+07:00     info    done query 5000
2026-07-26T14:37:53.637442529+07:00     info    done query 6000
2026-07-26T14:37:55.965888995+07:00     info    done query 7000
2026-07-26T14:37:58.253408005+07:00     info    done query 8000
2026-07-26T14:38:00.394736613+07:00     info    done query 9000
2026-07-26T14:38:02.686036227+07:00     info    done query 10000
Algoritma kueri kombinasi CRP dan ALT (with turn costs) :
avg query times: 1.927000
avg efficiency: 0.515226
avg number of vertices scanned: 1336
avg query runtime: 1.398400
avg path unpacking runtime: 0.098200
2026-07-26T14:38:05.433202629+07:00     info    done query 1000
2026-07-26T14:38:08.118557624+07:00     info    done query 2000
2026-07-26T14:38:10.772213243+07:00     info    done query 3000
2026-07-26T14:38:13.452375514+07:00     info    done query 4000
2026-07-26T14:38:16.300474194+07:00     info    done query 5000
2026-07-26T14:38:19.02717408+07:00      info    done query 6000
2026-07-26T14:38:21.712126875+07:00     info    done query 7000
2026-07-26T14:38:24.408301582+07:00     info    done query 8000
2026-07-26T14:38:27.090987732+07:00     info    done query 9000
2026-07-26T14:38:29.83621424+07:00      info    done query 10000
Algoritma kueri CRP (with turn costs):
avg query times: 2.225400
avg efficiency: 0.349395
avg number of vertices scanned: 1907
avg query runtime: 1.923200
avg path unpacking runtime: 0.012100
2026-07-26T14:38:32.426314354+07:00     info    done query 1000
2026-07-26T14:38:35.251948549+07:00     info    done query 2000
2026-07-26T14:38:37.798928447+07:00     info    done query 3000
2026-07-26T14:38:40.676744062+07:00     info    done query 4000
2026-07-26T14:38:43.375158792+07:00     info    done query 5000
2026-07-26T14:38:45.965306617+07:00     info    done query 6000
2026-07-26T14:38:48.574678122+07:00     info    done query 7000
2026-07-26T14:38:51.373740551+07:00     info    done query 8000
2026-07-26T14:38:54.009945308+07:00     info    done query 9000
2026-07-26T14:38:56.678124226+07:00     info    done query 10000
Algoritma kueri kombinasi CRP dan ALT (without turn costs):
avg query times: 2.182000
avg efficiency: 0.728240
avg number of vertices scanned: 966
avg query runtime: 2.018500
avg path unpacking runtime: 0.005400
2026-07-26T14:39:00.869739627+07:00     info    done query 1000
2026-07-26T14:39:04.976453364+07:00     info    done query 2000
2026-07-26T14:39:09.12311316+07:00      info    done query 3000
2026-07-26T14:39:13.199156052+07:00     info    done query 4000
2026-07-26T14:39:17.360105562+07:00     info    done query 5000
2026-07-26T14:39:21.389868501+07:00     info    done query 6000
2026-07-26T14:39:25.301104517+07:00     info    done query 7000
2026-07-26T14:39:29.5155562+07:00       info    done query 8000
2026-07-26T14:39:33.908817769+07:00     info    done query 9000
2026-07-26T14:39:38.270522166+07:00     info    done query 10000
Algoritma kueri CRP (without turn costs):
avg query times: 3.662000
avg efficiency: 0.440029
avg number of vertices scanned: 1523
avg query runtime: 3.482700
avg path unpacking runtime: 0.003800
```

runtime in milliseconds

# Input DIMACS California (CAL)

## Preprocessing

```
2026-07-26T13:14:58.591390276+07:00     info    partitioning level 5 with max cell size 262144
2026-07-26T13:16:18.149112799+07:00     info    level 5 done, total cells: 11
2026-07-26T13:16:18.149154032+07:00     info    partitioning level 4 with max cell size 131072
2026-07-26T13:16:34.138553351+07:00     info    level 4 total cells: 32
2026-07-26T13:16:34.138600796+07:00     info    partitioning level 3 with max cell size 16384
2026-07-26T13:16:59.87859227+07:00      info    level 3 total cells: 217
2026-07-26T13:16:59.878621215+07:00     info    partitioning level 2 with max cell size 2048
2026-07-26T13:17:11.491446152+07:00     info    level 2 total cells: 1697
2026-07-26T13:17:11.491473324+07:00     info    partitioning level 1 with max cell size 256
2026-07-26T13:17:32.701821051+07:00     info    level 1 total cells: 13559
2026-07-26T13:17:37.319248544+07:00     info    Starting preprocessing step of Customizable Route Planning...
2026-07-26T13:17:37.31930609+07:00      info    Building Overlay Graph of each levels...
2026-07-26T13:17:38.432043214+07:00     info    Overlay graph built and written to ./data/overlay_graph.ngraph
2026-07-26T13:17:38.43208778+07:00      info    overlay graph level 5: number of overlay vertices 664
2026-07-26T13:17:38.432092453+07:00     info    overlay graph level 4: number of overlay vertices 1320
2026-07-26T13:17:38.432096256+07:00     info    overlay graph level 3: number of overlay vertices 9824
2026-07-26T13:17:38.432101481+07:00     info    overlay graph level 2: number of overlay vertices 48452
2026-07-26T13:17:38.432106204+07:00     info    overlay graph level 1: number of overlay vertices 222768
2026-07-26T13:17:38.432111158+07:00     info    Running Kosaraju's algorithm to find strongly connected components (SCCs)...
2026-07-26T13:17:38.838230939+07:00     info    Writing graph to ./data/original.ngraph
2026-07-26T13:17:39.036933389+07:00     info    writing 1 graph tiles to files...
2026-07-26T13:17:41.651687383+07:00     info    completed writing tiles to files
```

## Customization

```
2026-07-26T13:17:45.72566132+07:00      info    Building cliques for each cell for each overlay graph level...
2026-07-26T13:17:45.729639577+07:00     info    number of shortcuts: 2240520
2026-07-26T13:17:46.849466194+07:00     info    computing landmarks....
2026-07-26T13:18:07.071342843+07:00     info    number of shortcuts overlay graph level 1: 1462784
2026-07-26T13:18:07.071379826+07:00     info    finished crp customization level 1
2026-07-26T13:18:10.819027865+07:00     info    number of shortcuts overlay graph level 2: 558520
2026-07-26T13:18:10.819065239+07:00     info    finished crp customization level 2
2026-07-26T13:18:11.609785268+07:00     info    number of shortcuts overlay graph level 3: 181738
2026-07-26T13:18:11.609824155+07:00     info    finished crp customization level 3
2026-07-26T13:18:11.734832031+07:00     info    number of shortcuts overlay graph level 4: 25312
2026-07-26T13:18:11.734862906+07:00     info    finished crp customization level 4
2026-07-26T13:18:11.788891404+07:00     info    number of shortcuts overlay graph level 5: 12166
2026-07-26T13:18:11.788937652+07:00     info    finished crp customization level 5
2026-07-26T13:18:11.788968707+07:00     info    Building stalling tables...
2026-07-26T13:18:16.279172788+07:00     info    done computing landmarks....
2026-07-26T13:18:16.97157318+07:00      info    Customization step completed successfully.
```

# Input DIMACS New York (NY)

## Preprocessing

```
2026-07-26T13:23:37.263980574+07:00     info    partitioning level 5 with max cell size 262144
2026-07-26T13:23:40.562121153+07:00     info    level 5 done, total cells: 3
2026-07-26T13:23:40.562153069+07:00     info    partitioning level 4 with max cell size 131072
2026-07-26T13:23:43.245462142+07:00     info    level 4 total cells: 7
2026-07-26T13:23:43.245494799+07:00     info    partitioning level 3 with max cell size 16384
2026-07-26T13:23:47.728576969+07:00     info    level 3 total cells: 34
2026-07-26T13:23:47.728610818+07:00     info    partitioning level 2 with max cell size 2048
2026-07-26T13:23:49.507577093+07:00     info    level 2 total cells: 244
2026-07-26T13:23:49.507605311+07:00     info    partitioning level 1 with max cell size 256
2026-07-26T13:23:50.777903914+07:00     info    level 1 total cells: 1908
2026-07-26T13:23:51.311755255+07:00     info    Starting preprocessing step of Customizable Route Planning...
2026-07-26T13:23:51.311814145+07:00     info    Building Overlay Graph of each levels...
2026-07-26T13:23:51.436145563+07:00     info    Overlay graph built and written to ./data/overlay_graph.ngraph
2026-07-26T13:23:51.436189968+07:00     info    overlay graph level 5: number of overlay vertices 64
2026-07-26T13:23:51.436193464+07:00     info    overlay graph level 4: number of overlay vertices 184
2026-07-26T13:23:51.436196209+07:00     info    overlay graph level 3: number of overlay vertices 1908
2026-07-26T13:23:51.436199034+07:00     info    overlay graph level 2: number of overlay vertices 9476
2026-07-26T13:23:51.436201197+07:00     info    overlay graph level 1: number of overlay vertices 44268
2026-07-26T13:23:51.436203381+07:00     info    Running Kosaraju's algorithm to find strongly connected components (SCCs)...
2026-07-26T13:23:51.486021052+07:00     info    Writing graph to ./data/original.ngraph
2026-07-26T13:23:51.51378468+07:00      info    writing 1 graph tiles to files...
2026-07-26T13:23:51.873060968+07:00     info    completed writing tiles to files
```

## Customization

```
2026-07-26T13:23:52.446516085+07:00     info    Building cliques for each cell for each overlay graph level...
2026-07-26T13:23:52.447205987+07:00     info    number of shortcuts: 586352
2026-07-26T13:23:52.586345297+07:00     info    computing landmarks....
2026-07-26T13:23:53.620780812+07:00     info    number of shortcuts overlay graph level 1: 399116
2026-07-26T13:23:53.620827492+07:00     info    finished crp customization level 1
2026-07-26T13:23:53.816612277+07:00     info    number of shortcuts overlay graph level 2: 142012
2026-07-26T13:23:53.816651775+07:00     info    finished crp customization level 2
2026-07-26T13:23:53.865601061+07:00     info    number of shortcuts overlay graph level 3: 41560
2026-07-26T13:23:53.86564162+07:00      info    finished crp customization level 3
2026-07-26T13:23:53.87462867+07:00      info    number of shortcuts overlay graph level 4: 3152
2026-07-26T13:23:53.874659933+07:00     info    finished crp customization level 4
2026-07-26T13:23:53.877289713+07:00     info    number of shortcuts overlay graph level 5: 512
2026-07-26T13:23:53.877327558+07:00     info    finished crp customization level 5
2026-07-26T13:23:53.87733454+07:00      info    Building stalling tables...
2026-07-26T13:23:54.72685706+07:00      info    done computing landmarks....
2026-07-26T13:23:54.831106509+07:00     info    Customization step completed successfully.
```
