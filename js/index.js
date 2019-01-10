// C++ Game Physics
// www.amiedd.com
//Based on Collision using separting Axis Theorem. You can read more about the theorm used in game development here https://gamedevelopment.tutsplus.com/tutorials/collision-detection-using-the-separating-axis-theorem--gamedev-169

//JOUST!!

var verletEngine = {};

(function(setup, init) {
	"use strict";
	// constants
	
	var kGravity = setup.kGravity,
		kNumIterations = setup.kNumIterations,
		kFriction = setup.kFriction,
		kFrictionGround = setup.kFrictionGround,
		kViscosity = setup.kViscosity,
		kForceDrag = setup.kForceDrag;

	// Vector object
	
	var Vec2 = function(x, y) {
		this.x = x || 0.0;
		this.y = y || 0.0;
	};

	Vec2.prototype = {
		set: function(x, y) {
			this.x = x;
			this.y = y;
			return this;
		},

		copy: function(v) {
			this.x = v.x;
			this.y = v.y;
			return this;
		},

		neg: function() {
			this.x = -this.x;
			this.y = -this.y;
			return this;
		},

		sub: function(v0, v1) {
			this.x = v0.x - v1.x;
			this.y = v0.y - v1.y;
			return this;
		},

		scale: function(v, s) {
			this.x = v.x * s;
			this.y = v.y * s;
			return this;
		},

		dot: function(v) {
			return this.x * v.x + this.y * v.y;
		},

		squareDist: function(v) {
			var dx = this.x - v.x;
			var dy = this.y - v.y;
			return dx * dx + dy * dy;
		},

		length: function() {
			return Math.sqrt(this.x * this.x + this.y * this.y);
		},

		perp: function(v) {
			this.x = -v.y;
			this.y = v.x;
			return this;
		},

		normal: function(v0, v1) {
			var nx = v0.y - v1.y, ny = v1.x - v0.x;
			// normalize
			var len = 1.0 / Math.sqrt(nx * nx + ny * ny);
			this.x = nx * len;
			this.y = ny * len;
			return this;
		}
	};

  // animation
	var run = function() {
		requestAnimationFrame(run);
		ctx.clearRect(0, 0, canvas.width, canvas.height);
		integrate();
		solve();
		draw();
		dragVertex && drag();
	};

//object bodies
	var draw = function() {
		for (var i = 0, n = bodies.length; i < n; i++) {
			bodies[i].draw();
		}
	};

 //vertex
	var drag = function() {
		// draw mouse link
		ctx.beginPath();
		ctx.moveTo(dragVertex.position.x, dragVertex.position.y);
		ctx.lineTo(pointer.x, pointer.y);
		ctx.strokeStyle = "#0f0";
		ctx.stroke();

    //set position
		var s = dragVertex.parent.mass * kForceDrag;
		dragVertex.position.x += (pointer.x - dragVertex.position.x) / s;
		dragVertex.position.y += (pointer.y - dragVertex.position.y) / s;
	};

	var integrate = function() {
		for (var i = 0, n = vertices.length; i < n; i++) {
			vertices[i].integrate();
		}
	};

  //collisions
	var solve = function() {
		var nBodies = bodies.length, nConstraints = constraints.length;

		for (var n = 0; n < kNumIterations; n++) {
			
			for (var i = 0; i < nConstraints; i++) {
				constraints[i].solve();
			}

      //Caclulate the bounding area
			for (var i = 0; i < nBodies; i++) {
				bodies[i].boundingBox();
			}

      //Don't Crash(Collision Dectection)
			for (var i = 0; i < nBodies - 1; i++) {
				var b0 = bodies[i];

				for (var j = i + 1; j < nBodies; j++) {
					var b1 = bodies[j];
					collision.SAT(b0, b1) && collision.resolve();
				}
			}
		}
	};


	var Constraint = function(parent, v0, v1, edge) {
		this.parent = parent;
		this.v0 = v0;
		this.v1 = v1;
		this.p0 = v0.position;
		this.p1 = v1.position;
		this.dist = this.p0.squareDist(this.p1);
		this.edge = edge;
	};

	Constraint.prototype.solve = function() {
		var dx = this.p1.x - this.p0.x;
		var dy = this.p1.y - this.p0.y;

//Math Square Root
		var delta = this.dist / (dx * dx + dy * dy + this.dist) - 0.5;

		dx *= delta;
		dy *= delta;

		this.p1.x += dx;
		this.p1.y += dy;
		this.p0.x -= dx;
		this.p0.y -= dy;
	};

	var Body = function(body) {

    // body properties
	
		this.vCount = 0;
		this.eCount = 0;
		this.vertices = [];
		this.positions = [];
		this.edges = [];
		this.center = new Vec2();
		this.halfEx = new Vec2();
		this.min = 0;
		this.max = 0;
		this.color = body.color || "#EDF236"; /* Child of the 80s*/
		this.mass = body.mass || 1.0;

  //Node construct
		var Vertex = function(parent, vertex) {
			this.parent = parent;
			this.position = new Vec2(vertex.x, vertex.y);
			this.oldPosition = new Vec2(vertex.x, vertex.y);
		};

	
		Vertex.prototype.integrate = function() {
			var p = this.position, o = this.oldPosition, x = p.x, y = p.y;

			p.x += kViscosity * p.x - kViscosity * o.x;
			p.y += kViscosity * p.y - kViscosity * o.y + kGravity;

			o.set(x, y);

     //set the screen height/width
			if (p.y < 0) p.y = 0;
			else if (p.y > canvas.height) {
				p.x -= (p.y - canvas.height) * (p.x - o.x) * kFrictionGround;
				p.y = canvas.height;
			}

			if (p.x < 0) p.x = 0;
			else if (p.x > canvas.width) p.x = canvas.width;
		};

	
		for (var n in body.vertices) {
			var vertex = new Vertex(this, body.vertices[n]);
			body.vertices[n].ref = vertex;
			this.vertices.push(vertex);
			this.positions.push(vertex.position);
			vertices.push(vertex);
			this.vCount++;
		}

		for (var i = 0; i < body.constraints.length; i++) {
			var bci = body.constraints[i];

			var constraint = new Constraint(
				this,
				body.vertices[bci[0]].ref,
				body.vertices[bci[1]].ref,
				bci[2] || false
			);

			if (constraint.edge) {
				this.edges.push(constraint);
				this.eCount++;
			}

			constraints.push(constraint);
		}
	};

	//box bounding area
	Body.prototype.boundingBox = function() {
		var minX = 99999.0, minY = 99999.0, maxX = -99999.0, maxY = -99999.0;

		for (var i = 0; i < this.vCount; i++) {
			var p = this.positions[i];

			if (p.x > maxX) maxX = p.x;
			if (p.y > maxY) maxY = p.y;
			if (p.x < minX) minX = p.x;
			if (p.y < minY) minY = p.y;
		}

		this.center.set((minX + maxX) * 0.5, (minY + maxY) * 0.5);


		this.halfEx.set((maxX - minX) * 0.5, (maxY - minY) * 0.5);
	};


	Body.prototype.projectAxis = function(axis) {
		var d = this.positions[0].dot(axis);
		this.min = this.max = d;

		for (var i = 1; i < this.vCount; i++) {
			d = this.positions[i].dot(axis);
			if (d > this.max) this.max = d;
			if (d < this.min) this.min = d;
		}
	};


	Body.prototype.draw = function() {
		ctx.beginPath();
		var p = this.edges[0].p0;
		ctx.moveTo(p.x, p.y);

		for (var i = 1; i < this.eCount; i++) {
			p = this.edges[i].p0;
			ctx.lineTo(p.x, p.y);
		}

		ctx.closePath;
		ctx.fillStyle = this.color;
		ctx.fill();

		if (pointer.isDown && !dragVertex) {
			if (ctx.isPointInPath(pointer.x, pointer.y)) {
				var minDistance = 99999;

				for (var i = 0; i < this.vCount; i++) {
					var dist = this.positions[i].squareDist(pointer);

					if (dist < minDistance) {
						dragVertex = this.vertices[i];
						minDistance = dist;
					}
				}
			}
		}
	};

 //detect object collison
	var collision = {
		testAxis: new Vec2(),
		axis: new Vec2(),
		center: new Vec2(),
		line: new Vec2(),
		response: new Vec2(),
		relVel: new Vec2(),
		tangent: new Vec2(),
		relTanVel: new Vec2(),
		depth: 0,
		edge: null,
		vertex: null,

 //axis theorem collision
		SAT: function(B0, B1) {
			
			if (
				!(0 > Math.abs(B1.center.x - B0.center.x) - (B1.halfEx.x + B0.halfEx.x) &&
					0 > Math.abs(B1.center.y - B0.center.y) - (B1.halfEx.y + B0.halfEx.y))
			)
				return false; // return no overlap


			var minDistance = 99999, n0 = B0.eCount, n1 = B1.eCount;

			for (var i = 0, n = n0 + n1; i < n; i++) {
				// get edge
				var edge = i < n0 ? B0.edges[i] : B1.edges[i - n0];

				this.testAxis.normal(edge.p0, edge.p1);

				// Project both bodies onto the normal
				B0.projectAxis(this.testAxis);
				B1.projectAxis(this.testAxis);

				//Distance between two intervales
				var dist = B0.min < B1.min ? B1.min - B0.max : B0.min - B1.max;

				// They don't overlap? Great! Return that nothing collided
				if (dist > 0) return false;
				else if (Math.abs(dist) < minDistance) {
					minDistance = Math.abs(dist);

					// Saveeeee
					this.axis.copy(this.testAxis);
					this.edge = edge;
				}
			}

			this.depth = minDistance;

			
			if (this.edge.parent != B1) {
				var t = B1;
				B1 = B0;
				B0 = t;
			}

			var n = this.center.sub(B0.center, B1.center).dot(this.axis);
			
			if (n < 0) this.axis.neg();

			var smallestDist = 99999, v, dist;

			for (var i = 0; i < B0.vCount; i++) {
				
				v = B0.vertices[i];
				this.line.sub(v.position, B1.center);
				dist = this.axis.dot(this.line);

				// Smallest distance and the colliding vertex
				if (dist < smallestDist) {
					smallestDist = dist;
					this.vertex = v;
				}
			}

			return true;
		},
    
		resolve: function() {
			// cache vertices positions
			var p0 = this.edge.p0,
				p1 = this.edge.p1,
				o0 = this.edge.v0.oldPosition,
				o1 = this.edge.v1.oldPosition,
				vp = this.vertex.position,
				vo = this.vertex.oldPosition,
				rs = this.response;

			// reply vector
			this.response.scale(this.axis, this.depth);

			var t = Math.abs(p0.x - p1.x) > Math.abs(p0.y - p1.y)
				? (vp.x - rs.x - p0.x) / (p1.x - p0.x)
				: (vp.y - rs.y - p0.y) / (p1.y - p0.y);
			var lambda = 1 / (t * t + (1 - t) * (1 - t));

			// Math coefficient
			var m0 = this.vertex.parent.mass,
				m1 = this.edge.parent.mass,
				tm = m0 + m1,
				m0 = m0 / tm,
				m1 = m1 / tm;

			// Tell the collision who is the boss: apply the collision response
			p0.x -= rs.x * (1 - t) * lambda * m0;
			p0.y -= rs.y * (1 - t) * lambda * m0;
			p1.x -= rs.x * t * lambda * m0;
			p1.y -= rs.y * t * lambda * m0;

			vp.x += rs.x * m1;
			vp.y += rs.y * m1;

	
			this.relVel.set(
				vp.x - vo.x - (p0.x + p1.x - o0.x - o1.x) * 0.5,
				vp.y - vo.y - (p0.y + p1.y - o0.y - o1.y) * 0.5
			);

			this.tangent.perp(this.axis);

			var relTv = this.relVel.dot(this.tangent);
			var rt = this.relTanVel.set(this.tangent.x * relTv, this.tangent.y * relTv);

			// Friction Baby!!
			vo.x += rt.x * kFriction * m1;
			vo.y += rt.y * kFriction * m1;

			o0.x -= rt.x * (1 - t) * kFriction * lambda * m0;
			o0.y -= rt.y * (1 - t) * kFriction * lambda * m0;
			o1.x -= rt.x * t * kFriction * lambda * m0;
			o1.y -= rt.y * t * kFriction * lambda * m0;
		}
	};

	//API 

	this.createRectangle = function(x, y, w, h, m, c) {
		var b = new Body({
			mass: m,
			color: c,
			vertices: {
				n0: { x: x, y: y },
				n1: { x: x + w, y: y },
				n2: { x: x + w, y: y + h },
				n3: { x: x, y: y + h }
			},
			constraints: [
				["n0", "n1", true],
				["n1", "n2", true],
				["n2", "n3", true],
				["n3", "n0", true],
				["n0", "n2"],
				["n3", "n1"]
			]
		});
		bodies.push(b);
		return b;
	};

	this.createTriangle = function(x, y, w, h, m, c) {
		w /= 2;
		h /= 2;
		var b = new Body({
			x: x,
			y: y,
			mass: m,
			color: c,
			vertices: {
				n0: { x: x - w, y: y + h },
				n1: { x: x, y: y - h },
				n2: { x: x + w, y: y + h }
			},
			constraints: [["n0", "n1", true], ["n1", "n2", true], ["n2", "n0", true]]
		});
		bodies.push(b);
		return b;
	};

	this.setGravity = function(gravity) {
		kGravity = gravity;
	};

	this.reset = function() {
		bodies.length = 0;
		vertices.length = 0;
		constraints.length = 0;
		init();
	};

	this.createJoint = function(B0, v0, B1, v1) {
		var constraint = new Constraint(
			null,
			B0.vertices[v0],
			B1.vertices[v1],
			false
		);

		constraints.push(constraint);
	};


	var canvas = ge1doot.canvas("canvas");
	var ctx = canvas.ctx;
	var dragVertex = null;



	var pointer = canvas.pointer;
	pointer.up = function() {
		dragVertex = null;
	};

//arrays

	var bodies = [], vertices = [], constraints = [];

	//Game start

	init();
	run();
}.call(
	verletEngine,
	{
		kGravity: 0.0,
		kNumIterations: 5,
		kFriction: 0.2,
		kFrictionGround: 0.1,
		kViscosity: 1.0,
		kForceDrag: 5
	},

  //be a goddess create joust

	function() {
		var codepen = [
   "                *                          ",
   "      **   *****   **   **   ****  ******  ",
   "      **  **   **  **   **  **       **    ",
   "      **  **   **  **   **   ***     **    ",
   "  **  **  **   **  **   **     **    **    ",
   "   ****    *****    *****   ****     **    ",
   "                          *                "

		];

		var w = canvas.width / 35;
		var h = canvas.height / 2 - 4.5 * w;

		for (var i = 0; i < codepen.length; i++) {
			var line = codepen[i];
			for (var j = 0; j < line.length; j++) {
				var c = line.charAt(j);
				if (c != " ")
					verletEngine.createRectangle(w * 0.5 + w * j, h + w * i, w * 0.8, w * 0.8);
			}
		}

    //333 I'm only half evil
		verletEngine.createTriangle(w * 7.5, h + w * 11, w * 10, w * 2, 20, "#666");
		verletEngine.createTriangle(w * 27.5, h + w * 11, w * 10, w * 2, 20, "#666");

		verletEngine.createTriangle(
			canvas.width / 2,
			canvas.height - w * 1.8 + Math.random() * w * 0.5,
			w * 4,
			w * 4,
			10,
			"#f00"
		);
	}
));

//Hey do you know what a human's gravity is on earth!?
document.getElementById("gravity").onchange = function() {
	var gravity = this.checked ? 0.1 : 0;
	verletEngine.setGravity(gravity);
};

document.getElementById("reset").onclick = function() {
	verletEngine.reset();
};