using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Collision
{
	public partial class Collision
	{
		public static void CollideCircles(ref Manifold manifold,
			CircleShape circle1, XForm xf1, CircleShape circle2, XForm xf2)
		{
			manifold.PointCount = 0;

			Vector2 p1 = Common.Math.Mul(xf1, circle1._localPosition);
			Vector2 p2 = Common.Math.Mul(xf2, circle2._localPosition);

			Vector2 d = p2 - p1;
			float distSqr = Vector2.Dot(d, d);
			float radiusSum = circle1._radius + circle2._radius;
			if (distSqr > radiusSum * radiusSum)
			{
				return;
			}

			float separation;
			if (distSqr < Common.Math.FLT_EPSILON)
			{
				separation = -radiusSum;
				manifold.Normal.Set(0.0f, 1.0f);
			}
			else
			{
				float dist = (float)System.Math.Sqrt(distSqr);
				separation = dist - radiusSum;
				float a = 1.0f / dist;
				manifold.Normal.X = a * d.X;
				manifold.Normal.Y = a * d.Y;
			}

			manifold.PointCount = 1;
			manifold.Points[0].ID.Key = 0;
			manifold.Points[0].Separation = separation;

			p1 += circle1._radius * manifold.Normal;
			p2 -= circle2._radius * manifold.Normal;

			Vector2 p = 0.5f * (p1 + p2);

			manifold.Points[0].LocalPoint1 = Common.Math.MulT(xf1, p);
			manifold.Points[0].LocalPoint2 = Common.Math.MulT(xf2, p);
		}

		public static void CollidePolygonAndCircle(ref Manifold manifold,
			PolygonShape polygon, XForm xf1, CircleShape circle, XForm xf2)
		{
			manifold.PointCount = 0;

			// Compute circle position in the frame of the polygon.
			Vector2 c = Common.Math.Mul(xf2, circle._localPosition);
			Vector2 cLocal = Common.Math.MulT(xf1, c);

			// Find the min separating edge.
			int normalIndex = 0;
			float separation = -Common.Math.FLT_MAX;
			float radius = circle._radius;

			for (int i = 0; i < polygon.VertexCount; ++i)
			{
				float s = Vector2.Dot(polygon._normals[i], cLocal - polygon._vertices[i]);
				if (s > radius)
				{
					// Early out.
					return;
				}

				if (s > separation)
				{
					separation = s;
					normalIndex = i;
				}
			}

			// If the center is inside the polygon ...
			if (separation < Common.Math.FLT_EPSILON)
			{
				manifold.PointCount = 1;
				manifold.Normal = Common.Math.Mul(xf1.R, polygon._normals[normalIndex]);
				manifold.Points[0].ID.Features.IncidentEdge = (byte)normalIndex;
				manifold.Points[0].ID.Features.IncidentVertex = Collision.NullFeature;
				manifold.Points[0].ID.Features.ReferenceFace = Collision.NullFeature;
				manifold.Points[0].ID.Features.Flip = 0;
				Vector2 position = c - radius * manifold.Normal;
				manifold.Points[0].LocalPoint1 = Common.Math.MulT(xf1, position);
				manifold.Points[0].LocalPoint2 = Common.Math.MulT(xf2, position);
				manifold.Points[0].Separation = separation - radius;
				return;
			}

			// Project the circle center onto the edge segment.
			int vertIndex1 = normalIndex;
			int vertIndex2 = vertIndex1 + 1 < polygon.VertexCount ? vertIndex1 + 1 : 0;
			Vector2 e = polygon._vertices[vertIndex2] - polygon._vertices[vertIndex1];
			float length = e.Normalize();

			// If the edge length is zero ...
			if (length < Common.Math.FLT_EPSILON)
			{
				Vector2 d_ = cLocal - polygon._vertices[vertIndex1];
				float dist_ = d_.Normalize();
				if (dist_ > radius)
				{
					return;
				}

				manifold.PointCount = 1;
				manifold.Normal = Common.Math.Mul(xf1.R, d_);
				manifold.Points[0].ID.Features.IncidentEdge = Collision.NullFeature;
				manifold.Points[0].ID.Features.IncidentVertex = (byte)vertIndex1;
				manifold.Points[0].ID.Features.ReferenceFace = Collision.NullFeature;
				manifold.Points[0].ID.Features.Flip = 0;
				Vector2 position = c - radius * manifold.Normal;
				manifold.Points[0].LocalPoint1 = Common.Math.MulT(xf1, position);
				manifold.Points[0].LocalPoint2 = Common.Math.MulT(xf2, position);
				manifold.Points[0].Separation = dist_ - radius;
				return;
			}

			// Project the center onto the edge.
			float u = Vector2.Dot(cLocal - polygon._vertices[vertIndex1], e);
			manifold.Points[0].ID.Features.IncidentEdge = Collision.NullFeature;
			manifold.Points[0].ID.Features.IncidentVertex = Collision.NullFeature;
			manifold.Points[0].ID.Features.ReferenceFace = Collision.NullFeature;
			manifold.Points[0].ID.Features.Flip = 0;
			Vector2 p;
			if (u <= 0.0f)
			{
				p = polygon._vertices[vertIndex1];
				manifold.Points[0].ID.Features.IncidentVertex = (byte)vertIndex1;
			}
			else if (u >= length)
			{
				p = polygon._vertices[vertIndex2];
				manifold.Points[0].ID.Features.IncidentVertex = (byte)vertIndex2;
			}
			else
			{
				p = polygon._vertices[vertIndex1] + u * e;
				manifold.Points[0].ID.Features.IncidentEdge = (byte)vertIndex1;
			}

			Vector2 d = cLocal - p;
			float dist = d.Normalize();
			if (dist > radius)
			{
				return;
			}

			manifold.PointCount = 1;
			manifold.Normal = Common.Math.Mul(xf1.R, d);
			Vector2 position_ = c - radius * manifold.Normal;
			manifold.Points[0].LocalPoint1 = Common.Math.MulT(xf1, position_);
			manifold.Points[0].LocalPoint2 = Common.Math.MulT(xf2, position_);
			manifold.Points[0].Separation = dist - radius;
		}
	}
}