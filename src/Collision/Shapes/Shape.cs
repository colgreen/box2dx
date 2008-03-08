using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Collision
{
	/// <summary>
	/// This holds the mass data computed for a shape.
	/// </summary>
	public struct MassData
	{
		/// <summary>
		/// The mass of the shape, usually in kilograms.
		/// </summary>
		public float Mass;

		/// <summary>
		/// The position of the shape's centroid relative to the shape's origin.
		/// </summary>
		public Vector2 Center;

		/// <summary>
		/// The rotational inertia of the shape.
		/// </summary>
		public float I;
	}

	/// <summary>
	/// The various collision shape types supported by Box2D.
	/// </summary>
	public enum ShapeType
	{
		UnknownShape = -1,
		CircleShape,
		PolygonShape,
		ShapeTypeCount,
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
		/// The collision category bits. Normally you would just set one bit.
		/// </summary>
		public uint CategoryBits;

		/// <summary>
		/// The collision mask bits. This states the categories that this
		/// shape would accept for collision.
		/// </summary>
		public uint MaskBits;

		/// <summary>
		/// Collision groups allow a certain group of objects to never collide (negative)
		/// or always collide (positive). Zero means no collision group. Non-zero group
		/// filtering always wins against the mask bits.
		/// </summary>
		public int GroupIndex;

		/// <summary>
		/// A sensor shape collects contact information but never generates a collision
		/// response.
		/// </summary>
		public bool IsSensor;

		/// <summary>
		/// The constructor sets the default shape definition values.
		/// </summary>
		public ShapeDef()
		{
			Type = ShapeType.UnknownShape;
			SerData = null;
			Friction = 0.2f;
			Restitution = 0.0f;
			Density = 0.0f;
			CategoryBits = 0x0001;
			MaskBits = 0xFFFF;
			GroupIndex = 0;
			IsSensor = false;
		}
	}

	/// <summary>
	/// A shape is used for collision detection. Shapes are created in World.
	/// You can use shape for collision detection before they are attached to the world.
	/// Warning: you cannot reuse shapes.
	/// </summary>
	public abstract class Shape : IDisposable
	{
		ShapeType _type;
		/// <summary>
		/// Get the type of this shape. You can use this to down cast to the concrete shape.
		/// </summary>
		public ShapeType Type { get { return _type; } set { _type = value; } }

		Shape _next;
		/// <summary>
		/// Get the next shape in the parent body's shape list.
		/// </summary>
		public Shape Next { get { return _next; } set { _next = value; } }

		Body _body;
		/// <summary>
		/// Get the parent body of this shape. This is NULL if the shape is not attached.
		/// </summary>
		public Body Body { get { return _body; } set { _body = value; } }

		float _sweepRadius;
		/// <summary>
		/// Sweep radius relative to the parent body's center of mass.
		/// </summary>
		public float SweepRadius { get { return _sweepRadius; } set { _sweepRadius = value; } }

		float _density;
		public float Density { get { return _density; } set { _density = value; } }

		float _friction;
		public float Friction { get { return _friction; } set { _friction = value; } }

		float _restitution;
		public float Restitution { get { return _restitution; } set { _restitution = value; } }

		ushort _proxyId;
		public ushort ProxyId { get { return _proxyId; } set { _proxyId = value; } }

		ushort _categoryBits;
		public ushort CategoryBits { get { return _categoryBits; } set { _categoryBits = value; } }

		ushort _maskBits;
		public ushort MaskBits { get { return _maskBits; } set { _maskBits = value; } }

		short _groupIndex;
		public short GroupIndex { get { return _groupIndex; } set { _groupIndex = value; } }

		bool _isSensor;
		/// <summary>
		/// Is this shape a sensor (non-solid)?
		/// </summary>
		public bool IsSensor { get { return _isSensor; } set { _isSensor = value; } }

		object _userData;
		/// <summary>
		/// Get the user data that was assigned in the shape definition. Use this to
		/// store your application specific data.
		/// </summary>
		public object UserData { get { return _userData; } set { _userData = value; } }

		public Shape(ShapeDef def)
		{
			_userData = def.UserData;
			_friction = def.Friction;
			_restitution = def.Restitution;
			_density = def.Density;
			_body = null;
			_sweepRadius = 0.0f;

			_next = null;

			_proxyId = PairManager.NullProxy;

			_categoryBits = def.CategoryBits;
			_maskBits = def.MaskBits;
			_groupIndex = def.GroupIndex;

			_isSensor = def.IsSensor;
		}		

		/// <summary>
		/// Test a point for containment in this shape. This only works for convex shapes.
		/// </summary>
		/// <param name="xf">The shape world transform.</param>
		/// <param name="p">A point in world coordinates.</param>
		/// <returns></returns>
		public abstract bool TestPoint(XForm xf, Vector2 p);

		/// <summary>
		/// Perform a ray cast against this shape.
		/// </summary>
		/// <param name="xf">The shape world transform.</param>
		/// <param name="lambda">Returns the hit fraction. You can use this to compute the contact point
		/// p = (1 - lambda) * segment.P1 + lambda * segment.P2.</param>
		/// <param name="normal"> Returns the normal at the contact point. If there is no intersection, 
		/// the normal is not set.</param>
		/// <param name="segment">Defines the begin and end point of the ray cast.</param>
		/// <param name="maxLambda">A number typically in the range [0,1].</param>
		/// <returns>True if there was an intersection.</returns>
		public abstract bool TestSegment(XForm xf, out float lambda, out Vector2 normal, Segment segment, float maxLambda);

		/// <summary>
		/// Given a transform, compute the associated axis aligned bounding box for this shape.
		/// </summary>
		/// <param name="aabb">Returns the axis aligned box.</param>
		/// <param name="xf">The world transform of the shape.</param>
		public abstract void ComputeAABB(out AABB aabb, XForm xf);

		/// <summary>
		/// Given two transforms, compute the associated swept axis aligned bounding box for this shape.
		/// </summary>
		/// <param name="aabb">Returns the axis aligned box.</param>
		/// <param name="xf1">The starting shape world transform.</param>
		/// <param name="xf2">The ending shape world transform.</param>
		public abstract void ComputeSweptAABB(out AABB aabb, XForm xf1, XForm xf2);

		/// <summary>
		/// Compute the mass properties of this shape using its dimensions and density.
		/// The inertia tensor is computed about the local origin, not the centroid.
		/// </summary>
		/// <param name="massData">Returns the mass data for this shape</param>
		public abstract void ComputeMass(out MassData massData);

		public abstract void UpdateSweepRadius(Vector2 center);

		public Shape Create(ShapeDef def)
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

		public void Destroy(Shape s)
		{
			switch (s.Type)
			{
				case ShapeType.CircleShape:
					if (s is IDisposable)
						(s as IDisposable).Dispose();
					s = null;
					break;

				case ShapeType.PolygonShape:
					if (s is IDisposable)
						(s as IDisposable).Dispose();
					s = null;
					break;

				default:
					Box2DXDebug.Assert(false);
			}
		}

		public void CreateProxy(BroadPhase broadPhase, XForm transform)
		{
			Box2DXDebug.Assert(_proxyId == PairManger.NullProxy);

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
				_proxyId = PairManger.NullProxy;
			}
		}

		public void DestroyProxy(BroadPhase broadPhase)
		{
			if (_proxyId != PairManger.NullProxy)
			{
				broadPhase.DestroyProxy(_proxyId);
				_proxyId = PairManger.NullProxy;
			}
		}

		public bool Synchronize(BroadPhase broadPhase, XForm transform1, XForm transform2)
		{
			if (_proxyId == PairManger.NullProxy)
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

		public void ResetProxy(BroadPhase broadPhase, XForm transform)
		{
			if (_proxyId != PairManger.NullProxy)
			{
				broadPhase.DestroyProxy(_proxyId);
			}

			AABB aabb;
			ComputeAABB(out aabb, transform);

			bool inRange = broadPhase.InRange(aabb);

			// You are affecting a shape outside the world box.
			Box2DXDebug.Assert(inRange);

			if (inRange)
			{
				_proxyId = broadPhase.CreateProxy(aabb, this);
			}
			else
			{
				_proxyId = PairManger.NullProxy;
			}
		}

		public void Dispose()
		{
			Box2DXDebug.Assert(_proxyId == PairManager.NullProxy);
		}
	}
}