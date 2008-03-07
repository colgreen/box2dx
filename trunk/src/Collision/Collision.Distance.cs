using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Collision
{
	public partial class Collision
	{
		public static int GJKIterations = 0;

		// GJK using Voronoi regions (Christer Ericson) and region selection
		// optimizations (Casey Muratori).

		// The origin is either in the region of points[1] or in the edge region. The origin is
		// not in region of points[0] because that is the old point.
		public static int ProcessTwo(out Vector2 x1, out Vector2 x2, ref Vector2[] p1s, ref Vector2[] p2s,
			ref Vector2[] points)
		{
			// If in point[1] region
			Vector2 r = -points[1];
			Vector2 d = points[0] - points[1];
			float length = d.Normalize();
			float lambda = Vector2.Dot(r, d);
			if (lambda <= 0.0f || length < Common.Math.FLT_EPSILON)
			{
				// The simplex is reduced to a point.
				x1 = p1s[1];
				x2 = p2s[1];
				p1s[0] = p1s[1];
				p2s[0] = p2s[1];
				points[0] = points[1];
				return 1;
			}

			// Else in edge region
			lambda /= length;
			x1 = p1s[1] + lambda * (p1s[0] - p1s[1]);
			x2 = p2s[1] + lambda * (p2s[0] - p2s[1]);
			return 2;
		}

		// Possible regions:
		// - points[2]
		// - edge points[0]-points[2]
		// - edge points[1]-points[2]
		// - inside the triangle
		public static int ProcessThree(out Vector2 x1, out Vector2 x2, ref Vector2[] p1s, ref Vector2[] p2s,
			ref Vector2[] points)
		{
			Vector2 a = points[0];
			Vector2 b = points[1];
			Vector2 c = points[2];

			Vector2 ab = b - a;
			Vector2 ac = c - a;
			Vector2 bc = c - b;

			float sn = -Vector2.Dot(a, ab), sd = Vector2.Dot(b, ab);
			float tn = -Vector2.Dot(a, ac), td = Vector2.Dot(c, ac);
			float un = -Vector2.Dot(b, bc), ud = Vector2.Dot(c, bc);

			// In vertex c region?
			if (td <= 0.0f && ud <= 0.0f)
			{
				// Single point
				x1 = p1s[2];
				x2 = p2s[2];
				p1s[0] = p1s[2];
				p2s[0] = p2s[2];
				points[0] = points[2];
				return 1;
			}

			// Should not be in vertex a or b region.
#warning
			//B2_NOT_USED(sd);
			//B2_NOT_USED(sn);			
			Box2DXDebug.Assert(sn > 0.0f || tn > 0.0f);
			Box2DXDebug.Assert(sd > 0.0f || un > 0.0f);

			float n = Vector2.Cross(ab, ac);

			// Should not be in edge ab region.
			float vc = n * Vector2.Cross(a, b);
			Box2DXDebug.Assert(vc > 0.0f || sn > 0.0f || sd > 0.0f);

			// In edge bc region?
			float va = n * Vector2.Cross(b, c);
			if (va <= 0.0f && un >= 0.0f && ud >= 0.0f)
			{
				Box2DXDebug.Assert(un + ud > 0.0f);
				float lambda = un / (un + ud);
				x1 = p1s[1] + lambda * (p1s[2] - p1s[1]);
				x2 = p2s[1] + lambda * (p2s[2] - p2s[1]);
				p1s[0] = p1s[2];
				p2s[0] = p2s[2];
				points[0] = points[2];
				return 2;
			}

			// In edge ac region?
			float vb = n * Vector2.Cross(c, a);
			if (vb <= 0.0f && tn >= 0.0f && td >= 0.0f)
			{
				Box2DXDebug.Assert(tn + td > 0.0f);
				float lambda = tn / (tn + td);
				x1 = p1s[0] + lambda * (p1s[2] - p1s[0]);
				x2 = p2s[0] + lambda * (p2s[2] - p2s[0]);
				p1s[1] = p1s[2];
				p2s[1] = p2s[2];
				points[1] = points[2];
				return 2;
			}

			// Inside the triangle, compute barycentric coordinates
			float denom = va + vb + vc;
			Box2DXDebug.Assert(denom > 0.0f);
			denom = 1.0f / denom;
			float u = va * denom;
			float v = vb * denom;
			float w = 1.0f - u - v;
			x1 = u * p1s[0] + v * p1s[1] + w * p1s[2];
			x2 = u * p2s[0] + v * p2s[1] + w * p2s[2];
			return 3;
		}

		public static bool InPoints(Vector2 w, Vector2[] points, int pointCount)
		{
			float k_tolerance = 100.0f * Common.Math.FLT_EPSILON;
			for (int i = 0; i < pointCount; ++i)
			{
				Vector2 d = Common.Math.Abs(w - points[i]);
				Vector2 m = Common.Math.Max(Common.Math.Abs(w), Common.Math.Abs(points[i]));

				if (d.x < k_tolerance * (m.x + 1.0f) &&
					d.y < k_tolerance * (m.y + 1.0f))
				{
					return true;
				}
			}

			return false;
		}

		public interface IGenericShape
		{
			Vector2 Support(XForm xf, Vector2 v);
			Vector2 GetFirstVertex(XForm xf);
		}

		public static float DistanceGeneric<T1, T2>(out Vector2 x1, out Vector2 x2,
						   T1 shape1_, XForm xf1, T2 shape2_, XForm xf2)
		{
			IGenericShape shape1 = shape1_ as IGenericShape;
			IGenericShape shape2 = shape2_ as IGenericShape;

			if (shape1 == null || shape2 == null)
				Box2DXDebug.Assert(false, "Can not cast some parameters to IGenericShape");

			Vector2[] p1s = new Vector2[3], p2s = new Vector2[3];
			Vector2[] points = new Vector2[3];
			int pointCount = 0;

			x1 = shape1.GetFirstVertex(xf1);
			x2 = shape2.GetFirstVertex(xf2);

			float vSqr = 0.0f;
			int maxIterations = 20;

			for (int iter = 0; iter < maxIterations; ++iter)
			{
				Vector2 v = x2 - x1;
				Vector2 w1 = shape1.Support(xf1, v);
				Vector2 w2 = shape2.Support(xf2, -v);

				vSqr = Vector2.Dot(v, v);
				Vector2 w = w2 - w1;
				float vw = Vector2.Dot(v, w);
				if (vSqr - vw <= 0.01f * vSqr || Collision.InPoints(w, points, pointCount)) // or w in points
				{
					if (pointCount == 0)
					{
						x1 = w1;
						x2 = w2;
					}
					Collision.GJKIterations = iter;
					return (float)System.Math.Sqrt(vSqr);
				}

				switch (pointCount)
				{
					case 0:
						p1s[0] = w1;
						p2s[0] = w2;
						points[0] = w;
						x1 = p1s[0];
						x2 = p2s[0];
						++pointCount;
						break;

					case 1:
						p1s[1] = w1;
						p2s[1] = w2;
						points[1] = w;
						pointCount = Collision.ProcessTwo(x1, x2, p1s, p2s, points);
						break;

					case 2:
						p1s[2] = w1;
						p2s[2] = w2;
						points[2] = w;
						pointCount = Collision.ProcessThree(x1, x2, p1s, p2s, points);
						break;
				}

				// If we have three points, then the origin is in the corresponding triangle.
				if (pointCount == 3)
				{
					Collision.GJKIterations = iter;
					return 0.0f;
				}

				float maxSqr = -Common.Math.FLT_MAX;
				for (int i = 0; i < pointCount; ++i)
				{
					maxSqr = Common.Math.Max(maxSqr, Vector2.Dot(points[i], points[i]));
				}

				if (pointCount == 3 || vSqr <= 100.0f * Common.Math.FLT_EPSILON * maxSqr)
				{
					Collision.GJKIterations = iter;
					v = x2 - x1;
					vSqr = Vector2.Dot(v, v);

					return (float)System.Math.Sqrt(vSqr);
				}
			}

			Collision.GJKIterations = maxIterations;
			return (float)System.Math.Sqrt(vSqr);
		}

		public static float DistanceCC(out Vector2 x1, out Vector2 x2,
			CircleShape circle1, XForm xf1, CircleShape circle2, XForm xf2)
		{
			Vector2 p1 = Common.Math.Mul(xf1, circle1.LocalPosition);
			Vector2 p2 = Common.Math.Mul(xf2, circle2.LocalPosition);

			Vector2 d = p2 - p1;
			float dSqr = Vector2.Dot(d, d);
			float r1 = circle1.Radius - Settings.ToiSlop;
			float r2 = circle2.Radius - Settings.ToiSlop;
			float r = r1 + r2;
			if (dSqr > r * r)
			{
				float dLen = d.Normalize();
				float distance = dLen - r;
				x1 = p1 + r1 * d;
				x2 = p2 - r2 * d;
				return distance;
			}
			else if (dSqr > Common.Math.FLT_EPSILON * Common.Math.FLT_EPSILON)
			{
				d.Normalize();
				x1 = p1 + r1 * d;
				x2 = x1;
				return 0.0f;
			}

			x1 = p1;
			x2 = x1;
			return 0.0f;
		}

#warning: "CAS"
		// This is used for polygon-vs-circle distance.
		public class Point : Collision.IGenericShape
		{
			public Vector2 p;

			public Vector2 Support(XForm xf, Vector2 v)
			{
				return p;
			}

			public Vector2 GetFirstVertex(XForm xf)
			{
				return p;
			}
		}

		// GJK is more robust with polygon-vs-point than polygon-vs-circle.
		// So we convert polygon-vs-circle to polygon-vs-point.
		public static float DistancePC(out Vector2 x1, out Vector2 x2,
			PolygonShape polygon, XForm xf1, CircleShape circle, XForm xf2)
		{
			Point point;
			point.p = Common.Math.Mul(xf2, circle.LocalPosition);

			float distance = DistanceGeneric(x1, x2, polygon, xf1, point, XForm.Identity);

			float r = circle.Radius - Settings.ToiSlop;

			if (distance > r)
			{
				distance -= r;
				Vector2 d = x2 - x1;
				d.Normalize();
				x2 -= r * d;
			}
			else
			{
				distance = 0.0f;
				x2 = x1;
			}

			return distance;
		}

		public static float Distance(out Vector2 x1, out Vector2 x2,
			Shape shape1, XForm xf1, Shape shape2, XForm xf2)
		{
			ShapeType type1 = shape1.GetType();
			ShapeType type2 = shape2.GetType();

			if (type1 == ShapeType.CircleShape && type2 == ShapeType.CircleShape)
			{
				return DistanceCC(x1, x2, (CircleShape)shape1, xf1, (CircleShape)shape2, xf2);
			}

			if (type1 == ShapeType.PolygonShape && type2 == ShapeType.CircleShape)
			{
				return DistancePC(x1, x2, (PolygonShape)shape1, xf1, (CircleShape)shape2, xf2);
			}

			if (type1 == ShapeType.CircleShape && type2 == ShapeType.PolygonShape)
			{
				return DistancePC(x2, x1, (PolygonShape)shape2, xf2, (CircleShape)shape1, xf1);
			}

			if (type1 == ShapeType.PolygonShape && type2 == ShapeType.PolygonShape)
			{
				return DistanceGeneric(x1, x2, (PolygonShape)shape1, xf1, (PolygonShape)shape2, xf2);
			}

			return 0.0f;
		}
	}
}