/* @lint-ignore-every TXT2 Tab Literal */
/**
 * @author qiao / https://github.com/qiao
 * @author mrdoob / http://mrdoob.com
 * @author alteredq / http://alteredqualia.com/
 * @author WestLangley / http://github.com/WestLangley
 * @author erich666 / http://erichaines.com
 */
/*global THREE, console */

// This set of controls performs orbiting, dollying (zooming), and panning. It maintains
// the "up" direction as +Y, unlike the TrackballControls. Touch on tablet and phones is
// supported.
//
//    Orbit - left mouse / touch: one finger move
//    Zoom - middle mouse, or mousewheel / touch: two finger spread or squish
//    Pan - right mouse, or arrow keys / touch: three finter swipe
//
// This is a drop-in replacement for (most) TrackballControls used in examples.
// That is, include this js file and wherever you see:
//    	controls = new THREE.TrackballControls( camera );
//      controls.target.z = 150;
// Simple substitute "OrbitControls" and the control should work as-is.

THREE.OrbitControls = function ( object, domElement ) {

	this.object = object;
	this.domElement = ( domElement !== undefined ) ? domElement : document;

	// API

	// Set to false to disable this control
	this.enabled = true;

	// "target" sets the location of focus, where the control orbits around
	// and where it pans with respect to.
	this.target = new THREE.Vector3();

	this.animationTarget = new THREE.Vector3();
	this.animationPosition = new THREE.Vector3();
	this.animationPosition.copy(object.position);
	this.animationSpeed = 0.1;

	// This option actually enables dollying in and out; left as "zoom" for
	// backwards compatibility
	this.noZoom = false;
	this.zoomSpeed = 1.0;

	// Limits to how far you can dolly in and out
	this.minDistance = 0;
	this.maxDistance = Infinity;

	// Set to true to disable this control
	this.noRotate = false;
	this.rotateSpeed = 1.0;
	this.noLookAround = false;
	this.lookAroundSpeed = 0.5;

	// Set to true to disable this control
	this.noPan = false;
	this.keyPanSpeed = 7.0;	// pixels moved per arrow key push

	// Set to true to disable use of the keys
	this.noKeys = false;

	// The four arrow keys
	this.keys = { LEFT: 37, UP: 38, RIGHT: 39, BOTTOM: 40 };

	// Mouse buttons
	this.mouseButtons = { ORBIT: THREE.MOUSE.LEFT, ZOOM: THREE.MOUSE.MIDDLE, PAN: THREE.MOUSE.RIGHT };

	////////////
	// internals

	var scope = this;

	var EPS = 0.000001;

	var rotateStart = new THREE.Vector2();
	var rotateEnd = new THREE.Vector2();
	var rotateDelta = new THREE.Vector2();

	var lookAroundStart = new THREE.Vector2();
	var lookAroundEnd = new THREE.Vector2();
	var lookAroundDelta = new THREE.Vector2();

	var panStart = new THREE.Vector2();
	var panEnd = new THREE.Vector2();
	var panDelta = new THREE.Vector2();
	var panOffset = new THREE.Vector3();

	var offset = new THREE.Vector3();

	var dollyStart = new THREE.Vector2();
	var dollyEnd = new THREE.Vector2();
	var dollyDelta = new THREE.Vector2();

	var phiDelta = 0;
	var thetaDelta = 0;
	var laPhiDelta = 0;
	var laThetaDelta = 0;
	var scale = 1;
	var pan = new THREE.Vector3();

	var lastPosition = new THREE.Vector3();
	var lastQuaternion = new THREE.Quaternion();

	var STATE = { NONE : -1, ROTATE : 0, DOLLY : 1, PAN : 2, TOUCH_ROTATE : 3, TOUCH_DOLLY : 4, TOUCH_PAN : 5, LOOK_AROUND: 6 };

	var state = STATE.NONE;

	// for reset

	this.target0 = this.target.clone();
	this.position0 = this.object.position.clone();

	// events

	var changeEvent = { type: 'change' };
	var startEvent = { type: 'start'};
	var endEvent = { type: 'end'};

	this.rotateLeft = function ( angle ) {
		thetaDelta -= angle;
	};

	this.rotateUp = function ( angle ) {
		phiDelta -= angle;
	};

	this.lookAroundLeft = function ( angle ) {
		laThetaDelta += angle;
	};

	this.lookAroundUp = function ( angle ) {
		laPhiDelta -= angle;
	};

	// pass in distance in world space to move left
	this.panLeft = function ( distance ) {
		// get X column of matrix
		var te = this.object.matrix.elements;
		panOffset.set( te[ 0 ], te[ 1 ], te[ 2 ] );
		panOffset.multiplyScalar( - distance );
		pan.add( panOffset );
	};

	// pass in distance in world space to move up
	this.panUp = function ( distance ) {
		// get Y column of matrix
		var te = this.object.matrix.elements;
		panOffset.set( te[ 4 ], te[ 5 ], te[ 6 ] );
		panOffset.multiplyScalar( distance );
		pan.add( panOffset );
	};

	// pass in x,y of change desired in pixel space,
	// right and down are positive
	this.pan = function ( deltaX, deltaY ) {
		var element = scope.domElement === document ? scope.domElement.body : scope.domElement;

		if ( scope.object.fov !== undefined ) {
			// perspective
			var position = scope.object.position;
			var offset = position.clone().sub( scope.target );

			// half of the fov is center to top of screen
			var targetDistance = offset.length() * Math.tan( ( scope.object.fov / 2 ) * Math.PI / 180.0 );

			// we actually don't use screenWidth, since perspective camera is fixed to screen height
			scope.panLeft( 2 * deltaX * targetDistance / element.clientHeight );
			scope.panUp( 2 * deltaY * targetDistance / element.clientHeight );
		} else if ( scope.object.top !== undefined ) {
			// orthographic
			scope.panLeft( deltaX * (scope.object.right - scope.object.left) / element.clientWidth );
			scope.panUp( deltaY * (scope.object.top - scope.object.bottom) / element.clientHeight );
		} else {
			// camera neither orthographic or perspective
			console.warn( 'WARNING: OrbitControls.js encountered an unknown camera type - pan disabled.' );
		}

	};

	this.dollyIn = function (dollyScale) {
		scale /= dollyScale;
	};

	this.dollyOut = function ( dollyScale ) {
		scale *= dollyScale;
	};

	this.updateAnimationTargetsMouse = function () {
		// Handle look around.
		offset.copy(this.animationTarget).sub(this.animationPosition);
		var laTheta = Math.atan2(offset.y, offset.x);
		var laPhi = Math.atan2(Math.sqrt(offset.x * offset.x + offset.y * offset.y), offset.z);
		laTheta += laThetaDelta;
		laPhi += laPhiDelta;
		laPhi = Math.max(EPS, Math.min(Math.PI - EPS, laPhi));

		// Compute new target position
		var radius = offset.length();
		offset.x = radius * Math.sin(laPhi) * Math.cos(laTheta);
		offset.y = radius * Math.sin(laPhi) * Math.sin(laTheta);
		offset.z = radius * Math.cos(laPhi);
		this.animationTarget.copy(this.animationPosition).add(offset);


		offset.copy(this.animationPosition).sub(this.animationTarget);

		// Rotate
		var theta = Math.atan2(offset.y, offset.x);
		var phi = Math.atan2(Math.sqrt(offset.x * offset.x + offset.y * offset.y), offset.z);
		theta += thetaDelta;
		phi += phiDelta;
		phi = Math.max(EPS, Math.min(Math.PI - EPS, phi));

		// Dolly
		radius = offset.length() * scale;
		radius = Math.max(this.minDistance, Math.min(this.maxDistance, radius));

		this.animationTarget.add(pan);

		// Compute new camera position
		offset.x = radius * Math.sin(phi) * Math.cos(theta);
		offset.y = radius * Math.sin(phi) * Math.sin(theta);
		offset.z = radius * Math.cos(phi);
		this.animationPosition.copy(this.animationTarget).add(offset);

		// Reset deltas
		thetaDelta = 0;
		phiDelta = 0;
		laThetaDelta = 0;
		laPhiDelta = 0;
		scale = 1;
		pan.set(0, 0, 0);
	};

	this.update = function () {
		this.updateAnimationTargetsMouse();

		this.target.lerp(this.animationTarget, this.animationSpeed);
		this.object.position.lerp(this.animationPosition, this.animationSpeed);
		this.object.lookAt(this.target);

		// update condition is:
		// min(camera displacement, camera rotation in radians)^2 > EPS
		// using small-angle approximation cos(x/2) = 1 - x^2 / 8
		if ( lastPosition.distanceToSquared( this.object.position ) > EPS
		    || 8 * (1 - lastQuaternion.dot(this.object.quaternion)) > EPS ) {

			this.dispatchEvent( changeEvent );

			lastPosition.copy( this.object.position );
			lastQuaternion.copy (this.object.quaternion );
		}
	};

	this.goto_shot = function(cam, shot) {
		if (cam.projection_type == 'spherical' || cam.projection_type == 'equirectangular') {
			this.animationTarget.add(opticalCenter(shot)).sub(this.animationPosition);
		} else {
			var offset = pixelToVertex(cam, shot, 0, 0, 20);
			this.animationTarget.copy(offset);
		}
		this.animationPosition.copy(opticalCenter(shot));
	};

	this.goto = function(position, target) {
		this.animationPosition.copy(position);
		this.animationTarget.copy(target);
	};

	this.reset = function () {
		state = STATE.NONE;
		this.target.copy( this.target0 );
		this.object.position.copy( this.position0 );
		this.update();
	};

	function getZoomScale() {
		return Math.pow( 0.95, scope.zoomSpeed );
	}

	function onMouseDown( event ) {
		if ( scope.enabled === false ) return;
		event.preventDefault();

		if ( event.button === scope.mouseButtons.ORBIT ) {
			if (event.shiftKey) {
				if (scope.noLookAround === true) return;
				state = STATE.LOOK_AROUND;
				lookAroundStart.set(event.clientX, event.clientY);
			} else {
				if (scope.noRotate === true) return;
				state = STATE.ROTATE;
				rotateStart.set(event.clientX, event.clientY);
			}
		} else if ( event.button === scope.mouseButtons.ZOOM ) {
			if ( scope.noZoom === true ) return;
			state = STATE.DOLLY;
			dollyStart.set( event.clientX, event.clientY );

		} else if ( event.button === scope.mouseButtons.PAN ) {
			if ( scope.noPan === true ) return;
			state = STATE.PAN;
			panStart.set( event.clientX, event.clientY );
		}

		document.addEventListener( 'mousemove', onMouseMove, false );
		document.addEventListener( 'mouseup', onMouseUp, false );
		document.addEventListener( 'keydown', onKeyDown, false );
		scope.dispatchEvent( startEvent );
	}

	function onMouseMove( event ) {

		if ( scope.enabled === false ) return;

		event.preventDefault();

		var element = scope.domElement === document ? scope.domElement.body : scope.domElement;

		if ( state === STATE.ROTATE ) {
			if ( scope.noRotate === true ) return;

			rotateEnd.set( event.clientX, event.clientY );
			rotateDelta.subVectors( rotateEnd, rotateStart );

			// rotating across whole screen goes 360 degrees around
			scope.rotateLeft( 2 * Math.PI * rotateDelta.x / element.clientWidth * scope.rotateSpeed );

			// rotating up and down along whole screen attempts to go 360, but limited to 180
			scope.rotateUp( 2 * Math.PI * rotateDelta.y / element.clientHeight * scope.rotateSpeed );

			rotateStart.copy( rotateEnd );

		} else if ( state === STATE.LOOK_AROUND ) {
			if (scope.noRotate === true) return;

			lookAroundEnd.set(event.clientX, event.clientY);
			lookAroundDelta.subVectors(lookAroundEnd, lookAroundStart);

			// rotating across whole screen goes 360 degrees around
			scope.lookAroundLeft(2 * Math.PI * lookAroundDelta.x / element.clientWidth * scope.lookAroundSpeed);

			// rotating up and down along whole screen attempts to go 360, but limited to 180
			scope.lookAroundUp(2 * Math.PI * lookAroundDelta.y / element.clientWidth * scope.lookAroundSpeed);

			lookAroundStart.copy(lookAroundEnd);

		} else if ( state === STATE.DOLLY ) {

			if ( scope.noZoom === true ) return;

			dollyEnd.set( event.clientX, event.clientY );
			dollyDelta.subVectors( dollyEnd, dollyStart );

			if ( dollyDelta.y > 0 ) {

				scope.dollyIn(getZoomScale());

			} else {

				scope.dollyOut(getZoomScale());

			}

			dollyStart.copy( dollyEnd );

		} else if ( state === STATE.PAN ) {

			if ( scope.noPan === true ) return;

			panEnd.set( event.clientX, event.clientY );
			panDelta.subVectors( panEnd, panStart );

			scope.pan( panDelta.x, panDelta.y );

			panStart.copy( panEnd );

		}

		scope.update();

	}

	function onMouseUp( /* event */ ) {

		if ( scope.enabled === false ) return;

		document.removeEventListener( 'mousemove', onMouseMove, false );
		document.removeEventListener( 'mouseup', onMouseUp, false );
		scope.dispatchEvent( endEvent );
		state = STATE.NONE;

	}

	function onMouseWheel( event ) {

		if ( scope.enabled === false || scope.noZoom === true ) return;

		event.preventDefault();
		event.stopPropagation();

		var delta = 0;

		if ( event.wheelDelta !== undefined ) { // WebKit / Opera / Explorer 9

			delta = event.wheelDelta;

		} else if ( event.detail !== undefined ) { // Firefox

			delta = - event.detail;

		}

		if ( delta > 0 ) {

			scope.dollyOut(getZoomScale());

		} else {

			scope.dollyIn(getZoomScale());

		}

		scope.update();
		scope.dispatchEvent( startEvent );
		scope.dispatchEvent( endEvent );

	}

	function onKeyDown( event ) {

		if ( scope.enabled === false || scope.noKeys === true || scope.noPan === true ) return;

		var validKey = true;
		switch ( event.keyCode ) {
			// case scope.keys.UP:
			// 	scope.pan( 0, scope.keyPanSpeed );
			// 	scope.update();
			// 	break;

			// case scope.keys.BOTTOM:
			// 	scope.pan( 0, - scope.keyPanSpeed );
			// 	scope.update();
			// 	break;

			case scope.keys.LEFT:
				scope.pan( scope.keyPanSpeed, 0 );
				scope.update();
				break;

			case scope.keys.RIGHT:
				scope.pan( - scope.keyPanSpeed, 0 );
				scope.update();
				break;
			default:
				validKey = false;
				break;
		}
		if (validKey) event.preventDefault();
	}

	function touchstart( event ) {

		if ( scope.enabled === false ) return;

		switch ( event.touches.length ) {

			case 1:	// one-fingered touch: rotate

				if ( scope.noRotate === true ) return;

				state = STATE.TOUCH_ROTATE;

				rotateStart.set( event.touches[ 0 ].pageX, event.touches[ 0 ].pageY );
				break;

			case 2:	// two-fingered touch: dolly

				if ( scope.noZoom === true ) return;

				state = STATE.TOUCH_DOLLY;

				var dx = event.touches[ 0 ].pageX - event.touches[ 1 ].pageX;
				var dy = event.touches[ 0 ].pageY - event.touches[ 1 ].pageY;
				var distance = Math.sqrt( dx * dx + dy * dy );
				dollyStart.set( 0, distance );
				break;

			case 3: // three-fingered touch: pan

				if ( scope.noPan === true ) return;

				state = STATE.TOUCH_PAN;

				panStart.set( event.touches[ 0 ].pageX, event.touches[ 0 ].pageY );
				break;

			default:

				state = STATE.NONE;

		}

		scope.dispatchEvent( startEvent );

	}

	function touchmove( event ) {

		if ( scope.enabled === false ) return;

		event.preventDefault();
		event.stopPropagation();

		var element = scope.domElement === document ? scope.domElement.body : scope.domElement;

		switch ( event.touches.length ) {

			case 1: // one-fingered touch: rotate

				if ( scope.noRotate === true ) return;
				if ( state !== STATE.TOUCH_ROTATE ) return;

				rotateEnd.set( event.touches[ 0 ].pageX, event.touches[ 0 ].pageY );
				rotateDelta.subVectors( rotateEnd, rotateStart );

				// rotating across whole screen goes 360 degrees around
				scope.rotateLeft( 2 * Math.PI * rotateDelta.x / element.clientWidth * scope.rotateSpeed );
				// rotating up and down along whole screen attempts to go 360, but limited to 180
				scope.rotateUp( 2 * Math.PI * rotateDelta.y / element.clientHeight * scope.rotateSpeed );

				rotateStart.copy( rotateEnd );

				scope.update();
				break;

			case 2: // two-fingered touch: dolly

				if ( scope.noZoom === true ) return;
				if ( state !== STATE.TOUCH_DOLLY ) return;

				var dx = event.touches[ 0 ].pageX - event.touches[ 1 ].pageX;
				var dy = event.touches[ 0 ].pageY - event.touches[ 1 ].pageY;
				var distance = Math.sqrt( dx * dx + dy * dy );

				dollyEnd.set( 0, distance );
				dollyDelta.subVectors( dollyEnd, dollyStart );

				if ( dollyDelta.y > 0 ) {

					scope.dollyOut(getZoomScale());

				} else {

					scope.dollyIn(getZoomScale());

				}

				dollyStart.copy( dollyEnd );

				scope.update();
				break;

			case 3: // three-fingered touch: pan

				if ( scope.noPan === true ) return;
				if ( state !== STATE.TOUCH_PAN ) return;

				panEnd.set( event.touches[ 0 ].pageX, event.touches[ 0 ].pageY );
				panDelta.subVectors( panEnd, panStart );

				scope.pan( panDelta.x, panDelta.y );

				panStart.copy( panEnd );

				scope.update();
				break;

			default:

				state = STATE.NONE;

		}

	}

	function touchend( /* event */ ) {

		if ( scope.enabled === false ) return;

		scope.dispatchEvent( endEvent );
		state = STATE.NONE;

	}

	this.domElement.addEventListener( 'contextmenu', function ( event ) { event.preventDefault(); }, false );
	this.domElement.addEventListener( 'mousedown', onMouseDown, false );
	this.domElement.addEventListener( 'mousewheel', onMouseWheel, false );
	this.domElement.addEventListener( 'DOMMouseScroll', onMouseWheel, false ); // firefox

	this.domElement.addEventListener( 'touchstart', touchstart, false );
	this.domElement.addEventListener( 'touchend', touchend, false );
	this.domElement.addEventListener( 'touchmove', touchmove, false );

	// force an update at start
	this.update();

};

THREE.OrbitControls.prototype = Object.create( THREE.EventDispatcher.prototype );
