using System;
using System.Collections.Generic;
using System.Text;

namespace Box2DX.Dynamics
{
	/// <summary>
	/// This holds contact filtering data.
	/// </summary>
	public struct FilterData
	{
		/// <summary>
		/// The collision category bits. Normally you would just set one bit.
		/// </summary>
		public ushort CategoryBits;

		/// <summary>
		/// The collision mask bits. This states the categories that this
		/// shape would accept for collision.
		/// </summary>
		public ushort MaskBits;

		/// <summary>
		/// Collision groups allow a certain group of objects to never collide (negative)
		/// or always collide (positive). Zero means no collision group. Non-zero group
		/// filtering always wins against the mask bits.
		/// </summary>
		public short GroupIndex;
	}

#warning "CAS"
	/// <summary>
	/// A shape definition is used to construct a shape. This class defines an
	/// abstract shape definition. You can reuse shape definitions safely.
	/// </summary>
	public class ShapeDef
	{
		/// <summary>
		/// Holds the shape type for down-casting.
		/// </summary>
		public ShapeType Type;

		/// <summary>
		/// Use this to store application specify shape data.
		/// </summary>
		public object UserData;

		/// <summary>
		/// The shape's friction coefficient, usually in the range [0,1].
		/// </summary>
		public float Friction;

		/// <summary>
		/// The shape's restitution (elasticity) usually in the range [0,1].
		/// </summary>
		public float Restitution;

		/// <summary>
		/// The shape's density, usually in kg/m^2.
		/// </summary>
		public float Density;

		/// <summary>
		/// A sensor shape collects contact information but never generates a collision
		/// response.
		/// </summary>
		public bool IsSensor;

		/// <summary>
		/// Contact filtering data.
		/// </summary>
		public FilterData Filter;

		/// <summary>
		/// The constructor sets the default shape definition values.
		/// </summary>
		public ShapeDef()
		{
			Type = ShapeType.UnknownShape;
			UserData = null;
			Friction = 0.2f;
			Restitution = 0.0f;
			Density = 0.0f;
			Filter.CategoryBits = 0x0001;
			Filter.MaskBits = 0xFFFF;
			Filter.GroupIndex = 0;
			IsSensor = false;
		}
	}

	/// <summary>
	/// Convex polygon. The vertices must be in CCW order for a right-handed
	/// coordinate system with the z-axis coming out of the screen.
	/// </summary>
	public class PolygonDef : ShapeDef
	{
		/// <summary>
		/// The number of polygon vertices.
		/// </summary>
		public int VertexCount;

		/// <summary>
		/// The polygon vertices in local coordinates.
		/// </summary>
		public Vec2[] Vertices = new Vec2[Settings.MaxPolygonVertices];

		public PolygonDef()
		{
			Type = ShapeType.PolygonShape;
			VertexCount = 0;
		}

		/// <summary>
		/// Build vertices to represent an axis-aligned box.
		/// </summary>
		/// <param name="hx">The half-width</param>
		/// <param name="hy">The half-height.</param>
		public void SetAsBox(float hx, float hy)
		{
			VertexCount = 4;
			Vertices[0].Set(-hx, -hy);
			Vertices[1].Set(hx, -hy);
			Vertices[2].Set(hx, hy);
			Vertices[3].Set(-hx, hy);
		}


		/// <summary>
		/// Build vertices to represent an oriented box.
		/// </summary>
		/// <param name="hx">The half-width</param>
		/// <param name="hy">The half-height.</param>
		/// <param name="center">The center of the box in local coordinates.</param>
		/// <param name="angle">The rotation of the box in local coordinates.</param>
		public void SetAsBox(float hx, float hy, Vec2 center, float angle)
		{
			SetAsBox(hx, hy);
			XForm xf = new XForm();
			xf.Position = center;
			xf.R.Set(angle);

			for (int i = 0; i < VertexCount; ++i)
			{
				Vertices[i] = Common.Math.Mul(xf, Vertices[i]);
			}
		}
	}
	/// <summary>
	/// This structure is used to build circle shapes.
	/// </summary>
	public class CircleDef : ShapeDef
	{
		public Vec2 LocalPosition;
		public float Radius;

		public CircleDef()
		{
			Type = ShapeType.CircleShape;
			LocalPosition = Vec2.Zero;
			Radius = 1.0f;
		}
	}

			// Sweep radius relative to the parent body's center of mass.
		protected float _sweepRadius;
		internal Body _body;
		internal Shape _next;

		/// <summary>
		/// Get the type of this shape. You can use this to down cast to the concrete shape.
		/// </summary>
		public new ShapeType ShapeType { get { return _type; } }
		
		/// <summary>
		/// Get the next shape in the parent body's shape list.
		/// </summary>
		public Shape Next { get { return _next; } }
		
		/// <summary>
		/// Get the parent body of this shape. This is NULL if the shape is not attached.
		/// </summary>
		public Body Body { get { return _body; } }
		
		/// <summary>
		/// Get the maximum radius about the parent body's center of mass.
		/// </summary>
		public float GetSweepRadius() { return _sweepRadius; }

		protected float _density;
		public float Density { get { return _density; } set { _density = value; } }

		protected float _friction;
		public float Friction { get { return _friction; } set { _friction = value; } }

		protected float _restitution;
		public float Restitution { get { return _restitution; } set { _restitution = value; } }

		protected ushort _proxyId;

		protected bool _isSensor;
		/// <summary>
		/// Is this shape a sensor (non-solid)?
		/// </summary>
		public bool IsSensor { get { return _isSensor; } }

		protected FilterData _filter;
		/// <summary>
		/// Get\Set the contact filtering data. You must call World.Refilter to correct
		/// existing contacts/non-contacts.
		/// </summary>
		public FilterData FilterData
		{
			get { return _filter; }
			set { _filter = value; }
		}

		protected object _userData;
		/// <summary>
		/// Get the user data that was assigned in the shape definition. Use this to
		/// store your application specific data.
		/// </summary>
		public object UserData
		{
			get { return _userData; }
			set { _userData = value; }
		}


	/*internal static Shape Create(ShapeDef def)
{
	switch (def.Type)
	{
		case ShapeType.CircleShape:
			{
				return new CircleShape(def);
			}

		case ShapeType.PolygonShape:
			{
				return new PolygonShape(def);
			}

		default:
			Box2DXDebug.Assert(false);
			return null;
	}
}

internal static void Destroy(ref Shape s)
{
	switch (s.GetType())
	{
		case ShapeType.CircleShape:
			s.Dispose();
			s = null;
			break;
		case ShapeType.EdgeShape:
			EdgeShape edge = (EdgeShape) s;
			if (edge._nextEdge != null) edge._nextEdge._prevEdge = null;
			if (edge._prevEdge != null) edge._prevEdge._nextEdge = null;
			s.Dispose();
			s= null;			
			break;
		case ShapeType.PolygonShape:
			s.Dispose();
			s = null;
			break;

		default:
			Box2DXDebug.Assert(false);
			break;
	}
}

internal void CreateProxy(BroadPhase broadPhase, XForm transform)
{
	Box2DXDebug.Assert(_proxyId == PairManager.NullProxy);

	AABB aabb;
	ComputeAABB(out aabb, transform);

	bool inRange = broadPhase.InRange(aabb);

	// You are creating a shape outside the world box.
	Box2DXDebug.Assert(inRange);

	if (inRange)
	{
		_proxyId = broadPhase.CreateProxy(aabb, this);
	}
	else
	{
		_proxyId = PairManager.NullProxy;
	}
}

internal void DestroyProxy(BroadPhase broadPhase)
{
	if (_proxyId != PairManager.NullProxy)
	{
		broadPhase.DestroyProxy(_proxyId);
		_proxyId = PairManager.NullProxy;
	}
}

internal bool Synchronize(BroadPhase broadPhase, XForm transform1, XForm transform2)
{
	if (_proxyId == PairManager.NullProxy)
	{
		return false;
	}

	// Compute an AABB that covers the swept shape (may miss some rotation effect).
	AABB aabb;
	ComputeSweptAABB(out aabb, transform1, transform2);

	if (broadPhase.InRange(aabb))
	{
		broadPhase.MoveProxy(_proxyId, aabb);
		return true;
	}
	else
	{
		return false;
	}
}

internal void RefilterProxy(BroadPhase broadPhase, XForm transform)
{
	if (_proxyId == PairManager.NullProxy)
	{
		return;
	}

	broadPhase.DestroyProxy(_proxyId);

	AABB aabb;
	ComputeAABB(out aabb, transform);

	bool inRange = broadPhase.InRange(aabb);

	if (inRange)
	{
		_proxyId = broadPhase.CreateProxy(aabb, this);
	}
	else
	{
		_proxyId = PairManager.NullProxy;
	}
}

public virtual void Dispose()
{
	Box2DXDebug.Assert(_proxyId == PairManager.NullProxy);
}*/
}
