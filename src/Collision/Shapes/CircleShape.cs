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
		public Vector2 LocalPosition;		
		// Radius of this circle.
		public float Radius;

		public CircleShape(ShapeDef def)
			: base(def)
		{
			Box2DXDebug.Assert(def.Type == ShapeType.CircleShape);
			CircleDef circleDef = (CircleDef)def;

			Type = ShapeType.CircleShape;
			LocalPosition = circleDef.LocalPosition;
			_radius = circleDef.Radius;
		}

		public void UpdateSweepRadius(Vector2 center)
		{
			// Update the sweep radius (maximum radius) as measured from
			// a local center point.
			Vector2 d = LocalPosition - center;
			SweepRadius = d.Length() + Radius - Settings.ToiSlop;
		}

		public override bool TestPoint(XForm transform, Vector2 p)
		{
			Vector2 center = transform.Position + Common.Math.Mul(transform.R, LocalPosition);
			Vector2 d = p - center;
			return Vector2.Dot(d, d) <= Radius * Radius;
		}

		// Collision Detection in Interactive 3D Environments by Gino van den Bergen
		// From Section 3.1.2
		// x = s + a * r
		// norm(x) = radius
		public override bool TestSegment(XForm xf, out float lambda, out Vector2 normal, Segment segment, float maxLambda)
		{
			Vector2 position = transform.Position + Common.Math.Mul(transform.R, LocalPosition);
			Vector2 s = segment.P1 - position;
			float b = Vector2.Dot(s, s) - Radius * Radius;

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

		public override void ComputeAABB(out AABB aabb, XForm xf)
		{
			aabb = new AABB();

			Vector2 p = transform.Position + Common.Math.Mul(transform.R, LocalPosition);
			aabb.LowerBound.Set(p.X - Radius, p.Y - Radius);
			aabb.UpperBound.Set(p.X + Radius, p.Y + Radius);
		}

		public override void ComputeSweptAABB(out AABB aabb, XForm xf1, XForm xf2)
		{
			aabb = new AABB();

			Vector2 p1 = transform1.Position + Common.Math.Mul(transform1.R, LocalPosition);
			Vector2 p2 = transform2.Position + Common.Math.Mul(transform2.R, LocalPosition);
			Vector2 lower = Common.Math.Min(p1, p2);
			Vector2 upper = Common.Math.Max(p1, p2);

			aabb.LowerBound.Set(lower.X - Radius, lower.Y - Radius);
			aabb.UpperBound.Set(upper.X + Radius, upper.Y + Radius);
		}

		public override void ComputeMass(out MassData massData)
		{
			massData = new MassData();

			massData.Mass = _density * Settings.Pi * Radius * Radius;
			massData.Center = LocalPosition;

			// inertia about the local origin
			massData.I = massData.Mass * (0.5f * Radius * Radius + Vector2.Dot(LocalPosition, LocalPosition));
		}
	}
}