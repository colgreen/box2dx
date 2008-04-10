using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;
using Box2DX.Collision;
using Box2DX.Dynamics;

using Tao.OpenGl;
using Tao.FreeGlut;

namespace TestBed
{
	public delegate Test TestCreateFcn();

	public class Settings
	{
		public float hz;
		public int iterationCount;
		public int drawShapes;
		public int drawJoints;
		public int drawCoreShapes;
		public int drawAABBs;
		public int drawOBBs;
		public int drawPairs;
		public int drawContactPoints;
		public int drawContactNormals;
		public int drawContactForces;
		public int drawFrictionForces;
		public int drawCOMs;
		public int drawStats;
		public int enableWarmStarting;
		public int enablePositionCorrection;
		public int enableTOI;
		public int pause;
		public int singleStep;

		public Settings()
		{
			hz = 60.0f;
			iterationCount = 10;
			drawStats = 0;
			drawShapes = 1;
			drawJoints = 1;
			drawCoreShapes = 0;
			drawAABBs = 0;
			drawOBBs = 0;
			drawPairs = 0;
			drawContactPoints = 0;
			drawContactNormals = 0;
			drawContactForces = 0;
			drawFrictionForces = 0;
			drawCOMs = 0;
			enableWarmStarting = 1;
			enablePositionCorrection = 1;
			enableTOI = 1;
			pause = 0;
			singleStep = 0;
		}
	}

	public class TestEntry
	{
		public TestEntry(string name, TestCreateFcn fcn)
		{
			Name = name;
			CreateFcn = fcn;
		}

		public string Name;
		public TestCreateFcn CreateFcn;

		public override string ToString()
		{
			return Name;
		}
	}

	public struct MyContactPoint
	{
		public Shape shape1;
		public Shape shape2;
		public Vector2 normal;
		public Vector2 position;
		public float normalForce;
		public float tangentForce;
		public int state; // 0-add, 1-persist, 2-remove
	}

	// This is called when a joint in the world is implicitly destroyed
	// because an attached body is destroyed. This gives us a chance to
	// nullify the mouse joint.
	public class MyDestructionListener : DestructionListener
	{
		public override void SayGoodbye(Shape shape) { ; }
		public override void SayGoodbye(Joint joint)
		{
			if (test._mouseJoint == joint)
			{
				test._mouseJoint = null;
			}
			else
			{
				test.JointDestroyed(joint);
			}
		}

		public Test test;
	}

	public class MyBoundaryListener : BoundaryListener
	{
		public override void Violation(Body body)
		{
			if (test._bomb != body)
			{
				test.BoundaryViolated(body);
			}
		}

		public Test test;
	}

	public class MyContactListener : ContactListener
	{
		public override void Add(ContactPoint point)
		{
			if (test._pointCount == Test.k_maxContactPoints)
			{
				return;
			}

			MyContactPoint cp = test._points[test._pointCount];
			cp.shape1 = point.Shape1;
			cp.shape2 = point.Shape2;
			cp.position = point.Position;
			cp.normal = point.Normal;
			cp.normalForce = point.NormalForce;
			cp.tangentForce = point.TangentForce;
			cp.state = 0;

			++test._pointCount;
		}

		public override void Persist(ContactPoint point)
		{
			if (test._pointCount == Test.k_maxContactPoints)
			{
				return;
			}

			MyContactPoint cp = test._points[test._pointCount];
			cp.shape1 = point.Shape1;
			cp.shape2 = point.Shape2;
			cp.position = point.Position;
			cp.normal = point.Normal;
			cp.normalForce = point.NormalForce;
			cp.tangentForce = point.TangentForce;
			cp.state = 1;

			++test._pointCount;
		}

		public override void Remove(ContactPoint point)
		{
			if (test._pointCount == Test.k_maxContactPoints)
			{
				return;
			}

			MyContactPoint cp = test._points[test._pointCount];
			cp.shape1 = point.Shape1;
			cp.shape2 = point.Shape2;
			cp.position = point.Position;
			cp.normal = point.Normal;
			cp.normalForce = point.NormalForce;
			cp.tangentForce = point.TangentForce;
			cp.state = 2;

			++test._pointCount;
		}

		public Test test;
	}

	public class Test : IDisposable
	{
		public static TestEntry[] g_testEntries = new TestEntry[]
		{
			new TestEntry("Simple Test", SimpleTest.Create),
			//new TestEntry("Pyramid", Pyramid.Create)			
		};
		public static int k_maxContactPoints = 2048;

		protected AABB _worldAABB;
		internal MyContactPoint[] _points = new MyContactPoint[k_maxContactPoints];
		internal int _pointCount;
		protected MyDestructionListener _destructionListener = new MyDestructionListener();
		protected MyBoundaryListener _boundaryListener = new MyBoundaryListener();
		protected MyContactListener _contactListener = new MyContactListener();
		internal DebugDraw _debugDraw = new OpenGLDebugDraw();
		protected int _textLine;
		internal World _world;
		internal Body _bomb;
		internal MouseJoint _mouseJoint;

		public Test()
		{
			_worldAABB = new AABB();
			_worldAABB.LowerBound.Set(-200.0f, -100.0f);
			_worldAABB.UpperBound.Set(200.0f, 200.0f);
			Vector2 gravity = new Vector2();
			gravity.Set(0.0f, -10.0f);
			bool doSleep = true;
			_world = new World(_worldAABB, gravity, doSleep);
			_bomb = null;
			_textLine = 30;
			_mouseJoint = null;
			_pointCount = 0;

			_destructionListener.test = this;
			_boundaryListener.test = this;
			_contactListener.test = this;
			_world.SetListener(_destructionListener);
			_world.SetListener(_boundaryListener);
			_world.SetListener(_contactListener);
			_world.SetDebugDraw(_debugDraw);
		}

		public void Dispose()
		{
			// By deleting the world, we delete the bomb, mouse joint, etc.
			_world.Dispose();
			_world = null;
		}

		public void SetTextLine(int line) { _textLine = line; }
		public virtual void Keyboard(System.Windows.Forms.Keys key) { ; }
		// Let derived tests know that a joint was destroyed.
		public virtual void JointDestroyed(Joint joint) { ; }
		public virtual void BoundaryViolated(Body body) { ; }

		public void MouseDown(Vector2 p)
		{
			if (_mouseJoint != null)
			{
				return;
			}

			// Make a small box.
			AABB aabb = new AABB();
			Vector2 d = new Vector2();
			d.Set(0.001f, 0.001f);
			aabb.LowerBound = p - d;
			aabb.UpperBound = p + d;

			// Query the world for overlapping shapes.
			int k_maxCount = 10;
			Shape[] shapes = new Shape[k_maxCount];
			int count = _world.Query(aabb, shapes, k_maxCount);
			Body body = null;
			for (int i = 0; i < count; ++i)
			{
				Body shapeBody = shapes[i].GetBody();
				if (shapeBody.IsStatic() == false)
				{
					bool inside = shapes[i].TestPoint(shapeBody.GetXForm(), p);
					if (inside)
					{
						body = shapes[i]._body;
						break;
					}
				}
			}

			if (body != null)
			{
				MouseJointDef md = new MouseJointDef();
				md.Body1 = _world._groundBody;
				md.Body2 = body;
				md.Target = p;
#if TARGET_FLOAT32_IS_FIXED
				md.MaxForce = (body._mass < 16.0f)? 
					(1000.0f * body._mass) : 16000.0f;
#else
				md.MaxForce = 1000.0f * body._mass;
#endif
				_mouseJoint = (MouseJoint)_world.CreateJoint(md);
				body.WakeUp();
			}
		}

		public void MouseUp()
		{
			if (_mouseJoint != null)
			{
				_world.DestroyJoint(_mouseJoint);
				_mouseJoint = null;
			}
		}

		public void MouseMove(Vector2 p)
		{
			if (_mouseJoint != null)
			{
				_mouseJoint.SetTarget(p);
			}
		}

		public void LaunchBomb()
		{
			if (_bomb != null)
			{
				_world.DestroyBody(_bomb);
				_bomb = null;
			}

			BodyDef bd = new BodyDef();
			bd.AllowSleep = true;
			bd.Position.Set(Box2DX.Common.Math.Random(-15.0f, 15.0f), 30.0f);
			bd.IsBullet = true;
			_bomb = _world.CreateDynamicBody(bd);
			_bomb.SetLinearVelocity(-5.0f * bd.Position);

			CircleDef sd = new CircleDef();
			sd.Radius = 0.3f;
			sd.Density = 20.0f;
			sd.Restitution = 0.1f;
			_bomb.CreateShape(sd);

			_bomb.SetMassFromShapes();
		}

		public void Step(Settings settings)
		{
			float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

			if (settings.pause != 0)
			{
				if (settings.singleStep != 0)
				{
					settings.singleStep = 0;
				}
				else
				{
					timeStep = 0.0f;
				}

				OpenGLDebugDraw.DrawString(5, _textLine, "****PAUSED****");
				_textLine += 15;
			}

			uint flags = 0;
			flags += (uint)settings.drawShapes * (uint)DebugDraw.DrawFlags.Shape;
			flags += (uint)settings.drawJoints * (uint)DebugDraw.DrawFlags.Joint;
			flags += (uint)settings.drawCoreShapes * (uint)DebugDraw.DrawFlags.CoreShape;
			flags += (uint)settings.drawAABBs * (uint)DebugDraw.DrawFlags.Aabb;
			flags += (uint)settings.drawOBBs * (uint)DebugDraw.DrawFlags.Obb;
			flags += (uint)settings.drawPairs * (uint)DebugDraw.DrawFlags.Pair;
			flags += (uint)settings.drawCOMs * (uint)DebugDraw.DrawFlags.CenterOfMass;
			_debugDraw.Flags = (DebugDraw.DrawFlags)flags;

			World.s_enableWarmStarting = settings.enableWarmStarting;
			World.s_enablePositionCorrection = settings.enablePositionCorrection;
			World.s_enableTOI = settings.enableTOI;

			_pointCount = 0;

			_world.Step(timeStep, settings.iterationCount);

			_world._broadPhase.Validate();

			if (_bomb != null && _bomb.IsFrozen())
			{
				_world.DestroyBody(_bomb);
				_bomb = null;
			}

			if (settings.drawStats != 0)
			{
				OpenGLDebugDraw.DrawString(5, _textLine, String.Format("proxies(max) = {0}({1}), pairs(max) = {2}({3})",
					new object[]{_world._broadPhase._proxyCount, Box2DX.Common.Settings.MaxProxies,
						_world._broadPhase._pairManager._pairCount, Box2DX.Common.Settings.MaxProxies}));
				_textLine += 15;

				OpenGLDebugDraw.DrawString(5, _textLine, String.Format("bodies/contacts/joints = {0}/{1}/{2}",
					new object[] { _world._bodyCount, _world._contactCount, _world._jointCount }));
				_textLine += 15;

				OpenGLDebugDraw.DrawString(5, _textLine, String.Format("position iterations = {0}",
					new object[] { _world._positionIterationCount }));
				_textLine += 15;
			}

			if (_mouseJoint != null)
			{
				Body body = _mouseJoint._body2;
				Vector2 p1 = body.GetWorldPoint(_mouseJoint._localAnchor);
				Vector2 p2 = _mouseJoint._target;

				Gl.glPointSize(4.0f);
				Gl.glColor3f(0.0f, 1.0f, 0.0f);
				Gl.glBegin(Gl.GL_POINTS);
				Gl.glVertex2f(p1.X, p1.Y);
				Gl.glVertex2f(p2.X, p2.Y);
				Gl.glEnd();
				Gl.glPointSize(1.0f);

				Gl.glColor3f(0.8f, 0.8f, 0.8f);
				Gl.glBegin(Gl.GL_LINES);
				Gl.glVertex2f(p1.X, p1.Y);
				Gl.glVertex2f(p2.X, p2.Y);
				Gl.glEnd();
			}

			if (settings.drawContactPoints!=0)
			{
				float k_forceScale = 0.01f;
				float k_axisScale = 0.3f;

				for (int i = 0; i < _pointCount; ++i)
				{
					MyContactPoint point = _points[i];

					if (point.state == 0)
					{
						// Add
						OpenGLDebugDraw.DrawPoint(point.position, 10.0f, new Color(0.3f, 0.95f, 0.3f));
					}
					else if (point.state == 1)
					{
						// Persist
						OpenGLDebugDraw.DrawPoint(point.position, 5.0f, new Color(0.3f, 0.3f, 0.95f));
					}
					else
					{
						// Remove
						OpenGLDebugDraw.DrawPoint(point.position, 10.0f, new Color(0.95f, 0.3f, 0.3f));
					}

					if (settings.drawContactNormals == 1)
					{
						Vector2 p1 = point.position;
						Vector2 p2 = p1 + k_axisScale * point.normal;
						OpenGLDebugDraw.DrawSegment(p1, p2, new Color(0.4f, 0.9f, 0.4f));
					}
					else if (settings.drawContactForces == 1)
					{
						Vector2 p1 = point.position;
						Vector2 p2 = p1 + k_forceScale * point.normalForce * point.normal;
						OpenGLDebugDraw.DrawSegment(p1, p2, new Color(0.9f, 0.9f, 0.3f));
					}

					if (settings.drawFrictionForces == 1)
					{
						Vector2 tangent = Vector2.Cross(point.normal, 1.0f);
						Vector2 p1 = point.position;
						Vector2 p2 = p1 + k_forceScale * point.tangentForce * tangent;
						OpenGLDebugDraw.DrawSegment(p1, p2, new Color(0.9f, 0.9f, 0.3f));
					}
				}
			}
		}

		public override string ToString()
		{
			return GetType().Name;
		}
	}
}
