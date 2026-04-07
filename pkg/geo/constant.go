package geo

var (
	// https://mathworld.wolfram.com/InverseGudermannian.html
	// https://oeis.org/A091912
	// https://en.wikipedia.org/wiki/Gudermannian_function#Taylor_series
	// buat coefficients horner rule https://rosettacode.org/wiki/Horner%27s_rule_for_polynomial_evaluation
	eulerSecantNumbers = []float64{
		1,
		0,
		1,
		0,
		5,
		0,
		61,
		0,
		1385,
		0,
		50521,
		0,
		2702765,
		0,
		199360981,
		0,
		19391512145,
		0,
		2404879675441,
		0,
		370371188237525,
		0,
		69348874393137901,
		0,
		15514534163557086905,
		0,
		4087072509293123892361,
		0,
		1252259641403629865468285,
		0,
		441543893249023104553682821,
		0,
		177519391579539289436664789665,
		0,
		80723299235887898062168247453281,
		0,
		41222060339517702122347079671259045,
		0,
		23489580527043108252017828576198947741,
		0,
		10364622733519612119397957304745185976310201,
	}
	// https://oeis.org/A136606

	maxNumMaclaurinTerms       = len(eulerSecantNumbers)
	invGudermanMaclaurinCoeffs = invGudermanMaclaurinSeriesCoefficients()
	bestNumMaclaurinTerms      = calcBestNumsOfTermsInvGudermanMaclaurinSeries()
)
