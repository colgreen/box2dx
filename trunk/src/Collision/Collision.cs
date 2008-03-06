using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Collision
{
	// Structures and functions used for computing contact points, distance
	// queries, and TOI queries.

	public partial class Collision
	{
		public static readonly byte NullFeature = byte.MaxValue; //0xff
		public static readonly byte NewPoint = 0x02;
		public static readonly byte OldPoint = 0x04;

		public static bool TestOverlap(AABB a, AABB b)
		{
			Vector2 d1, d2;
			d1 = b.LowerBound - a.UpperBound;
			d2 = a.LowerBound - b.UpperBound;

			if (d1.X > 0.0f || d1.Y > 0.0f)
				return false;

			if (d2.X > 0.0f || d2.Y > 0.0f)
				return false;

			return true;
		}
	}

	/// <summary>
	/// The features that intersect to form the contact point.
	/// </summary>
	public struct Features
	{
		/// <summary>
		/// The edge that defines the contact normal
		/// </summary>
		public byte ReferenceFace;

		/// <summary>
		/// The edge most anti-parallel to the reference face
		/// </summary>
		public byte IncidentEdge;

		/// <summary>
		/// The vertex (0 or 1) on the incident edge that was clipped
		/// </summary>
		public byte IncidentVertex;

		/// <summary>
		/// Indicates that the contact normal should be flipped
		/// </summary>
		public byte Flip;
	}

	/// <summary>
	/// Contact ids to facilitate warm starting.
	/// </summary>
	[System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
	public struct ContactID
	{
		[System.Runtime.InteropServices.FieldOffset(0)]
		public Features Features;

		/// <summary>
		/// Used to quickly compare contact ids.
		/// </summary>
		[System.Runtime.InteropServices.FieldOffset(0)]
		public uint Key;
	}

	/// <summary>
	/// A manifold point is a contact point belonging to a contact
	/// manifold. It holds details related to the geometry and dynamics
	/// of the contact points.
	/// The point is stored in local coordinates because CCD
	/// requires sub-stepping in which the separation is stale.
	/// </summary>
	public struct ManifoldPoint
	{
		/// <summary>
		/// Local position of the contact point in body1.
		/// </summary>
		public Vector2 LocalPoint1;

		/// <summary>
		/// Local position of the contact point in body2.
		/// </summary>
		public Vector2 LocalPoint2;

		/// <summary>
		/// The separation of the shapes along the normal vector.
		/// </summary>
		public float Separation;

		/// <summary>
		/// The non-penetration force.
		/// </summary>
		public float NormalForce;

		/// <summary>
		/// The friction force.
		/// </summary>
		public float TangentForce;

		/// <summary>
		/// Uniquely identifies a contact point between two shapes.
		/// </summary>
		public ContactID ID;
	}

	/// <summary>
	/// A manifold for two touching convex shapes.
	/// </summary>
	public struct Manifold
	{
		/// <summary>
		/// The points of contact.
		/// </summary>
		public ManifoldPoint[/*Settings.MaxManifoldPoints*/] Points;

		/// <summary>
		/// The shared unit normal vector.
		/// </summary>
		public Vector2 Normal;

		/// <summary>
		/// The number of manifold points.
		/// </summary>
		public int PointCount;
	}

	/// <summary>
	/// A line segment.
	/// </summary>
	public struct Segment
	{
		// Collision Detection in Interactive 3D Environments by Gino van den Bergen
		// From Section 3.4.1
		// x = mu1 * p1 + mu2 * p2
		// mu1 + mu2 = 1 && mu1 >= 0 && mu2 >= 0
		// mu1 = 1 - mu2;
		// x = (1 - mu2) * p1 + mu2 * p2
		//   = p1 + mu2 * (p2 - p1)
		// x = s + a * r (s := start, r := end - start)
		// s + a * r = p1 + mu2 * d (d := p2 - p1)
		// -a * r + mu2 * d = b (b := s - p1)
		// [-r d] * [a; mu2] = b
		// Cramer's rule:
		// denom = det[-r d]
		// a = det[b d] / denom
		// mu2 = det[-r b] / denom
		/// <summary>
		/// Ray cast against this segment with another segment.        
		/// </summary>
		/// <param name="lambda"></param>
		/// <param name="normal"></param>
		/// <param name="segment"></param>
		/// <param name="maxLambda"></param>
		/// <returns></returns>
		public bool TestSegment(out float lambda, out Vector2 normal, Segment segment, float maxLambda)
		{
			lambda = 0f;
			normal = new Vector2();

			Vector2 s = segment.P1;
			Vector2 r = segment.P2 - s;
			Vector2 d = P2 - P1;
			Vector2 n = Vector2.Cross(d, 1.0f);

			float k_slop = 100.0f * Common.Math.FLT_EPSILON;
			float denom = -Vector2.Dot(r, n);

			// Cull back facing collision and ignore parallel segments.
			if (denom > k_slop)
			{
				// Does the segment intersect the infinite line associated with this segment?
				Vector2 b = s - P1;
				float a = Vector2.Dot(b, n);

				if (0.0f <= a && a <= maxLambda * denom)
				{
					float mu2 = -r.X * b.Y + r.Y * b.X;

					// Does the segment intersect this segment?
					if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
					{
						a /= denom;
						n.Normalize();
						lambda = a;
						normal = n;
						return true;
					}
				}
			}

			return false;
		}

		/// <summary>
		/// The starting point.
		/// </summary>
		public Vector2 P1;

		/// <summary>
		/// The ending point.
		/// </summary>
		public Vector2 P2;
	}

	/// <summary>
	/// An axis aligned bounding box.
	/// </summary>
	public struct AABB
	{
		/// Verify that the bounds are sorted.
		public bool IsValid
		{
			get
			{
				Vector2 d = UpperBound - LowerBound;
				bool valid = d.X >= 0.0f && d.Y >= 0;
				valid = valid && LowerBound.IsValid && UpperBound.IsValid;
				return valid;
			}
		}


		/// <summary>
		/// The lower vertex.
		/// </summary>
		public Vector2 LowerBound;

		/// <summary>
		/// The upper vertex.
		/// </summary>
		public Vector2 UpperBound;
	}

	/// <summary>
	/// An oriented bounding box.
	/// </summary>
	public struct OBB
	{
		/// <summary>
		/// The rotation matrix.
		/// </summary>
		Mat22 R;

		/// <summary>
		/// The local centroid.
		/// </summary>
		Vector2 Center;

		/// <summary>
		/// The half-widths.
		/// </summary>
		Vector2 Extents;
	}
}
