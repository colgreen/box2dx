using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Collision
{
	/// <summary>
	/// This structure is used to build circle shapes.
	/// </summary>
	public class CircleDef : ShapeDef
	{
		public Vector2 LocalPosition;
		public float Radius;

		public CircleDef()
		{
			Type = ShapeType.CircleShape;
			LocalPosition = Vector2.Zero;
			Radius = 1.0f;
		}
	}

	/// <summary>
	/// A circle shape.
	/// </summary>
	public class CircleShape : Shape
	{
		// Local position in parent body
		public Vector2 _localPosition;
		// Radius of this circle.
		public float _radius;

		public CircleShape(ShapeDef def)
			: base(def)
		{
			Box2DXDebug.Assert(def.Type == ShapeType.CircleShape);
			CircleDef circleDef = (CircleDef)def;

			_type = ShapeType.CircleShape;
			_localPosition = circleDef.LocalPosition;
			_radius = circleDef.Radius;
		}

		public override void UpdateSweepRadius(Vector2 center)
		{
			// Update the sweep radius (maximum radius) as measured from
			// a local center point.
			Vector2 d = _localPosition - center;
			_sweepRadius = d.Length() + _radius - Settings.ToiSlop;
		}

		public override bool TestPoint(XForm transform, Vector2 p)
		{
			Vector2 center = transform.Position + Common.Math.Mul(transform.R, _localPosition);
			Vector2 d = p - center;
			return Vector2.Dot(d, d) <= _radius * _radius;
		}

		// Collision Detection in Interactive 3D Environments by Gino van den Bergen
		// From Section 3.1.2
		// x = s + a * r
		// norm(x) = radius
		public override bool TestSegment(XForm transform, out float lambda, out Vector2 normal, Segment segment, float maxLambda)
		{
			lambda = 0f;
			normal = Vector2.Zero;

			Vector2 position = transform.Position + Common.Math.Mul(transform.R, _localPosition);
			Vector2 s = segment.P1 - position;
			float b = Vector2.Dot(s, s) - _radius * _radius;

			// Does the segment start inside the circle?
			if (b < 0.0f)
			{
				return false;
			}

			// Solve quadratic equation.
			Vector2 r = segment.P2 - segment.P1;
			float c = Vector2.Dot(s, r);
			float rr = Vector2.Dot(r, r);
			float sigma = c * c - rr * b;

			// Check for negative discriminant and short segment.
			if (sigma < 0.0f || rr < Common.Math.FLT_EPSILON)
			{
				return false;
			}

			// Find the point of intersection of the line with the circle.
			float a = -(c + (float)System.Math.Sqrt(sigma));

			// Is the intersection point on the segment?
			if (0.0f <= a && a <= maxLambda * rr)
			{
				a /= rr;
				lambda = a;
				normal = s + a * r;
				normal.Normalize();
				return true;
			}

			return false;
		}

		public override void ComputeAABB(out AABB aabb, XForm transform)
		{
			aabb = new AABB();

			Vector2 p = transform.Position + Common.Math.Mul(transform.R, _localPosition);
			aabb.LowerBound.Set(p.X - _radius, p.Y - _radius);
			aabb.UpperBound.Set(p.X + _radius, p.Y + _radius);
		}

		public override void ComputeSweptAABB(out AABB aabb, XForm transform1, XForm transform2)
		{
			aabb = new AABB();

			Vector2 p1 = transform1.Position + Common.Math.Mul(transform1.R, _localPosition);
			Vector2 p2 = transform2.Position + Common.Math.Mul(transform2.R, _localPosition);
			Vector2 lower = Common.Math.Min(p1, p2);
			Vector2 upper = Common.Math.Max(p1, p2);

			aabb.LowerBound.Set(lower.X - _radius, lower.Y - _radius);
			aabb.UpperBound.Set(upper.X + _radius, upper.Y + _radius);
		}

		public override void ComputeMass(out MassData massData)
		{
			massData = new MassData();

			massData.Mass = _density * Settings.Pi * _radius * _radius;
			massData.Center = _localPosition;

			// inertia about the local origin
			massData.I = massData.Mass * (0.5f * _radius * _radius + Vector2.Dot(_localPosition, _localPosition));
		}
	}
}