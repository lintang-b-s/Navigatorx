package geo

var (
	// https://mathworld.wolfram.com/InverseGudermannian.html
	// https://oeis.org/A091912
	invGudermanMaclaurinNumerators = []float64{
		1,
		1,
		1,
		61,
		277,
		50521,
		41581,
		199360981,
		228135437,
		2404879675441,
		14814847529501,
		69348874393137901,
		238685140977801337,
		4087072509293123892361,
	}
	// https://oeis.org/A136606
	invGudermanMaclaurinDenominators = []float64{
		1,
		6,
		24,
		5040,
		72576,
		39916800,
		95800320,
		1307674368000,
		4184557977600,
		121645100408832000,
		2043637686868377600,
		25852016738884976640000,
		238634000666630553600000,
		10888869450418352160768000000,
	}

	maxNumMaclaurinTerms  = 14
	bestNumMaclaurinTerms = calcBestNumsOfTermsInvGudermanMaclaurinSeries()
)
