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
2026-07-26T18:47:37.495520606+07:00     info    Starting customization step of Customizable Route Planning...
2026-07-26T18:47:37.49558639+07:00      info    Reading graph from ./data/profiles/car/jateng_jabar_original.ngraph
2026-07-26T18:47:39.753160525+07:00     info    Reading overlay graph from ./data/profiles/car/jateng_jabar_overlay_graph.ngraph
2026-07-26T18:47:39.925748618+07:00     info    Building cliques for each cell for each overlay graph level...
2026-07-26T18:47:39.930410146+07:00     info    number of shortcuts: 2096585
2026-07-26T18:47:44.069544804+07:00     info    computing landmarks....
2026-07-26T18:47:53.776486579+07:00     info    number of shortcuts overlay graph level 1: 1111496
2026-07-26T18:47:53.776522767+07:00     info    finished crp customization level 1
2026-07-26T18:47:56.750562633+07:00     info    number of shortcuts overlay graph level 2: 497198
2026-07-26T18:47:56.750606836+07:00     info    finished crp customization level 2
2026-07-26T18:47:57.518189218+07:00     info    number of shortcuts overlay graph level 3: 242130
2026-07-26T18:47:57.518220266+07:00     info    finished crp customization level 3
2026-07-26T18:47:57.770858218+07:00     info    number of shortcuts overlay graph level 4: 153647
2026-07-26T18:47:57.770890639+07:00     info    finished crp customization level 4
2026-07-26T18:47:57.897526281+07:00     info    number of shortcuts overlay graph level 5: 92114
2026-07-26T18:47:57.897572328+07:00     info    finished crp customization level 5
2026-07-26T18:47:57.89757883+07:00      info    Building stalling tables...
2026-07-26T18:48:07.362363699+07:00     info    done computing landmarks....
2026-07-26T18:48:08.259523643+07:00     info    Customization step completed successfully.
```

## Avg runtime

```
go run eval/crp_alt/random_queries_crp_alt/main.go
2026-07-26T18:49:15.613693372+07:00     info    Starting query engine....
2026-07-26T18:49:15.613752944+07:00     info    Reading graph....
2026-07-26T18:49:17.701097074+07:00     info    Reading overlay graph....
2026-07-26T18:49:17.729039953+07:00     info    Reading stalling tables & metrics...
2026-07-26T18:49:19.302300549+07:00     info    starting benchmark
2026-07-26T18:49:22.215519159+07:00     info    done query 1000
2026-07-26T18:49:24.603233949+07:00     info    done query 2000
2026-07-26T18:49:26.855519638+07:00     info    done query 3000
2026-07-26T18:49:29.001407333+07:00     info    done query 4000
2026-07-26T18:49:31.15316394+07:00      info    done query 5000
2026-07-26T18:49:33.326255022+07:00     info    done query 6000
2026-07-26T18:49:35.717246186+07:00     info    done query 7000
2026-07-26T18:49:37.822076248+07:00     info    done query 8000
2026-07-26T18:49:39.930674648+07:00     info    done query 9000
2026-07-26T18:49:42.016647392+07:00     info    done query 10000
Algoritma kueri kombinasi CRP dan ALT (with turn costs) :
avg query times: 1.770700
avg efficiency: 0.521192
avg number of vertices scanned: 1330
avg query runtime: 1.276200
avg path unpacking runtime: 0.089400
2026-07-26T18:49:44.556833172+07:00     info    done query 1000
2026-07-26T18:49:47.355992789+07:00     info    done query 2000
2026-07-26T18:49:50.263336425+07:00     info    done query 3000
2026-07-26T18:49:53.214668856+07:00     info    done query 4000
2026-07-26T18:49:55.887012893+07:00     info    done query 5000
2026-07-26T18:49:58.573310807+07:00     info    done query 6000
2026-07-26T18:50:01.231573008+07:00     info    done query 7000
2026-07-26T18:50:03.919417947+07:00     info    done query 8000
2026-07-26T18:50:06.631417632+07:00     info    done query 9000
2026-07-26T18:50:09.198355224+07:00     info    done query 10000
Algoritma kueri CRP (with turn costs):
avg query times: 2.222400
avg efficiency: 0.349779
avg number of vertices scanned: 1904
avg query runtime: 1.926500
avg path unpacking runtime: 0.012300
2026-07-26T18:50:11.780335615+07:00     info    done query 1000
2026-07-26T18:50:14.385208544+07:00     info    done query 2000
2026-07-26T18:50:16.954200778+07:00     info    done query 3000
2026-07-26T18:50:19.560496358+07:00     info    done query 4000
2026-07-26T18:50:22.120877258+07:00     info    done query 5000
2026-07-26T18:50:24.851210691+07:00     info    done query 6000
2026-07-26T18:50:27.68077765+07:00      info    done query 7000
2026-07-26T18:50:30.412524157+07:00     info    done query 8000
2026-07-26T18:50:33.072350012+07:00     info    done query 9000
2026-07-26T18:50:35.764059948+07:00     info    done query 10000
Algoritma kueri kombinasi CRP dan ALT (without turn costs):
avg query times: 2.161100
avg efficiency: 0.728894
avg number of vertices scanned: 966
avg query runtime: 1.985000
avg path unpacking runtime: 0.004800
2026-07-26T18:50:39.789232207+07:00     info    done query 1000
2026-07-26T18:50:43.758095415+07:00     info    done query 2000
2026-07-26T18:50:47.584968365+07:00     info    done query 3000
2026-07-26T18:50:51.519729999+07:00     info    done query 4000
2026-07-26T18:50:55.466712918+07:00     info    done query 5000
2026-07-26T18:50:59.413221856+07:00     info    done query 6000
2026-07-26T18:51:03.31040437+07:00      info    done query 7000
2026-07-26T18:51:07.213257423+07:00     info    done query 8000
2026-07-26T18:51:11.056914648+07:00     info    done query 9000
2026-07-26T18:51:14.930710788+07:00     info    done query 10000
Algoritma kueri CRP (without turn costs):
avg query times: 3.415100
avg efficiency: 0.442335
avg number of vertices scanned: 1523
avg query runtime: 3.242800
avg path unpacking runtime: 0.003100
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

# Correctness test, programming contest

## A Journey To Greece

```
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsample/1 (0.04s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/01_expensivebridge1 (0.03s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/02_expensivebridge2 (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/03_expensivebridge3 (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/04_expensivebridge4 (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/05_double_edge (0.02s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/06_wantOnlyAthens1 (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/07_wantOnlyAthens2 (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/08_staytooShortForSites (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/09_pureTSP1 (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/10_pureTSP2 (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/11_pathimpossible (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/12_pathpossible (0.01s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/13_largest (1.61s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/13_largest2 (1.62s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/14_large.0 (1.60s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/15_large.1 (1.48s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/16_large.2 (1.80s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/17_large.3 (2.03s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/18_large.4 (1.60s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/19_large.5 (1.66s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/20_large.6 (1.53s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/21_large.7 (1.48s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/22_large.8 (1.56s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/23_large.9 (1.52s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/24_manySmall.0 (0.42s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/25_manySmall.1 (0.41s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/26_manySmall.2 (0.42s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/27_manySmall.3 (0.48s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/28_manySmall.4 (0.49s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/29_manySmall.5 (0.49s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/30_manySmall.6 (0.46s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/31_manySmall.7 (0.40s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/32_manySmall.8 (0.41s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/33_manySmall.9 (0.40s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/34_manySmall.10 (0.43s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/35_manySmall.11 (0.47s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/36_manySmall.12 (0.48s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/37_manySmall.13 (0.47s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/38_manySmall.14 (0.43s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/39_manySmall.15 (0.42s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/40_manySmall.16 (0.40s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/41_manySmall.17 (0.42s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/42_manySmall.18 (0.42s)
    --- PASS: TestCRPQueryAJourneyToGreeceMALT/Multilevel-ALT_without_turn_costsecret/43_manySmall.19 (0.43s)

```

## Delft Distance

```
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsample/1 (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsample/2 (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/01-small-X (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/02-small-O (0.00s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/03-small (0.00s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/04-small (0.00s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/05-small (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/06-row (0.00s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/07-col (0.00s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/08-medium (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/09-medium (0.04s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/10-medium (0.45s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/12-large-few-o (6.11s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/13-large-few-o (16.74s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/17-large-many-o (5.73s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/20-tall (0.35s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/21-tall (0.53s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/22-tall (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/23-tall (0.12s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/24-wide (0.10s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/25-wide (0.08s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/26-wide (0.39s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/27-wide (0.51s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/39-diag (25.89s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/46-diag-400 (11.79s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/54-manual (0.02s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/55-manual (0.02s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/56-manual (0.02s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/57-manual (0.02s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/58-manual (0.03s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/59-manual (0.02s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/65-corner1 (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/66-corner2 (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/67-sidel (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/68-sider (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/69-sideu (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/70-sided (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/71-moon1 (0.01s)
    --- PASS: TestCRPQueryDelftDistanceMALT/Multilevel-ALT_without_turn_costsecret/72-moon2 (0.02s)
```

## Galaxy Quest
```
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsample/1 (0.01s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsample/2 (0.02s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/01-cancellation (0.00s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/02-min (0.00s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/26-random-small (0.00s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/27-random-small (0.00s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/28-random-small (0.00s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/29-random-small (0.00s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/30-random-small (0.00s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/31-random-small (0.01s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/32-random-small (0.01s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/33-random-small (0.01s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/34-random-small (0.02s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/35-random-small (0.02s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/41-random (12.93s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/43-random (1.20s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/44-random (25.15s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/45-random (56.80s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/46-random-small-coords (1.73s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/47-random-small-coords (1.56s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/48-random-small-coords (1.53s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/49-random-small-coords (2.35s)
    --- PASS: TestCRPQueryGalaxyQuestMALT/Multilevel-ALT_without_turn_costsecret/50-random-small-coords (1.72s)
```

## Shopping Malls

```
    --- PASS: TestCRPQueryShoppingMallsMALT/Multilevel-ALT_without_turn_costsecret/1 (0.01s)
    --- PASS: TestCRPQueryShoppingMallsMALT/Multilevel-ALT_without_turn_costsecret/2 (0.11s)
    --- PASS: TestCRPQueryShoppingMallsMALT/Multilevel-ALT_without_turn_costsecret/3 (0.09s)
    --- PASS: TestCRPQueryShoppingMallsMALT/Multilevel-ALT_without_turn_costsecret/4 (0.18s)
    --- PASS: TestCRPQueryShoppingMallsMALT/Multilevel-ALT_without_turn_costsecret/5 (0.22s)
    --- PASS: TestCRPQueryShoppingMallsMALT/Multilevel-ALT_without_turn_costsecret/6 (0.16s)
```

## Showroom

```
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/1.random (0.02s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/10.large-circle (0.02s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/11.all-cars-max (0.02s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/12.400 (81.61s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/13.400 (105.94s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/14.400 (95.04s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/15.400 (97.04s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/16.bigzag (72.31s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/2.random (0.03s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/3.doorcut (0.01s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/4.harder-random (0.02s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/5.harder-random (0.03s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/6.harder-random (0.02s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/7.circle (0.00s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/8.all-doors (0.00s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsecret/9.smallest (0.00s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsample/1 (0.01s)
    --- PASS: TestShowroomMALT/Multilevel-ALT_without_turn_costsample/2 (0.00s)
```

## Kereta Rel Listrik

```
    --- PASS: TestOSN2024KRLMALT/tc/practice_10_10 (17.65s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_10_11 (0.03s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_10_3 (107.64s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_10_4 (114.24s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_10_5 (117.25s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_10_6 (144.42s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_10_7 (3.33s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_1 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_10 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_11 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_12 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_16 (12.39s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_17 (132.69s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_18 (144.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_19 (198.95s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_21 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_3 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_4 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_5 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_6 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_7 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_8 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_11_9 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_1_1 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_2_1 (0.06s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_2_2 (0.09s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_2_3 (0.07s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_2_4 (0.06s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_2_5 (0.06s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_2_6 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_2_7 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_2_8 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_4_1 (174.64s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_6_1 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_6_10 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_1 (0.06s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_10 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_2 (0.07s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_3 (0.04s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_4 (0.02s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_5 (0.06s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_6 (0.07s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_7 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_8 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_7_9 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_8_1 (154.84s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_8_2 (128.93s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_8_3 (114.94s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_8_5 (308.99s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_8_6 (137.00s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_8_8 (155.45s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_1 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_10 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_11 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_17 (278.96s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_19 (0.04s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_2 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_3 (0.07s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_4 (0.07s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_5 (0.06s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_6 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_7 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_8 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_9_9 (0.05s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_sample_1 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_sample_2 (0.01s)
    --- PASS: TestOSN2024KRLMALT/tc/practice_sample_3 (0.01s)

```
