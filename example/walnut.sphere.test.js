const jscad = require('@jscad/modeling')

const { cube, sphere } = jscad.primitives
const { translate } = jscad.transforms
const { subtract } = jscad.booleans


function main({// @jscad-params
	segments=99,
}){
	return subtract(
		sphere({radius:10, segments}),
		translate([5,5,5],sphere({radius:10, segments})),
	)	
}

module.exports = { main }
