// Package mobile provides online map matching mobile app library.
package mobile

import (
	_ "golang.org/x/mobile/bind"
	_ "golang.org/x/mod/modfile"
)

/*
ini terinspirasi dari: https://eng.lyft.com/using-client-side-map-data-to-improve-real-time-positioning-a382585ac6e?gi=0f02a4844208
golang library untuk client-side (mobile app) realtime map matching
run: gomobile bind -v -target=android -androidapi 21 -o online_mapmatcher.aar ./mobile
or gomobile bind -target android -o online_mapmatcher.aar -v  ./mobile  (following this tutorial: https://github.com/miguelespinoza/react-goku)
copy online_mapmatcher.aar ke react native project directory "<rn_working_dir>/modules/map-matcher/android/libs/"

inti dari file ini dan ref artikel diatas:
1. client side mobile app (android & ios) secara berkala merequest MapElements (roadNetworkGraph) ke backend setiap kali user pindah cell s2 .
2. DynamicGraph (go localization library) bakal ngebuat/rebuild DynamicGraph dari beberapa MapElements yang dekat dengan lokasi user (dari s2 Cell user dan its s2 neighbor cells).
3. Melakukan real time map matching (https://eng.lyft.com/a-new-real-time-map-matching-algorithm-at-lyft-da593ab7b006) atau mapmatch_mht.go pakai MobileMapMatcher (go localization library) di dalam mobile app navigatorx pakai DynamicGraph yang dibuild dari MapElements tadi .
4. Lyft pakai SQLite di mobile appnya buat caching MapAttributes. kita mungkin bisa pakai bolt (https://github.com/etcd-io/bbolt).
*/
