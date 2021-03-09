
const path = require('path');

module.exports = {
	entry: './static/js/main.js',
	mode: 'development',
	watch: true,
	resolve: {
        alias: {
          'three-examples': path.join(__dirname, './node_modules/three/examples/js')
        },
		extensions: [ '.js' ],
	},
	output: {
		filename: 'bundle.js',
		path: path.resolve(__dirname, 'static/js'),
	},
};
