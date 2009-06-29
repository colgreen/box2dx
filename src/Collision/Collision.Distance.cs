/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Collision
{
	/// <summary>
	/// Used to warm start Distance.
	/// Set count to zero on first call.
	/// </summary>
	public unsafe struct SimplexCache
	{
		/// <summary>
		/// Length or area.
		/// </summary>
		public float Metric;
		public UInt16 Count;
		/// <summary>
		/// Vertices on shape A.
		/// </summary>
		public fixed Byte IndexA[3];
		/// <summary>
		/// Vertices on shape B.
		/// </summary>
		public fixed Byte IndexB[3];
	}

	/// <summary>
	/// Input for Distance.
	/// You have to option to use the shape radii
	/// in the computation.
	/// </summary>
	public struct DistanceInput
	{
		public XForm TransformA;
		public XForm TransformB;
		public bool UseRadii;
	}

	/// <summary>
	/// Output for Distance.
	/// </summary>
	public struct DistanceOutput
	{
		/// <summary>
		/// Closest point on shapeA.
		/// </summary>
		public Vec2 PointA;
		/// <summary>
		/// Closest point on shapeB.
		/// </summary>

		public Vec2 PointB;
		public float Distance;
		/// <summary>
		/// Number of GJK iterations used.
		/// </summary>
		public int Iterations;
	}

	// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.

	public struct SimplexVertex
	{
		public Vec2 wA;		// support point in shapeA
		public Vec2 wB;		// support point in shapeB
		public Vec2 w;		// wB - wA
		public float a;		// barycentric coordinate for closest point
		public int indexA;	// wA index
		public int indexB;	// wB index
	}

	public struct Simplex
	{
		public SimplexVertex V1, V2, V3;
		public int Count;

		public unsafe void ReadCache(SimplexCache* cache, Shape shapeA, XForm transformA, Shape shapeB, XForm transformB)
		{
			Box2DXDebug.Assert(0 <= cache->Count && cache->Count <= 3);

			// Copy data from cache.
			Count = cache->Count;
			SimplexVertex* vertices = &V1;
			for (int i = 0; i < Count; ++i)
			{
				SimplexVertex* v = vertices + i;
				v->indexA = cache->IndexA[i];
				v->indexB = cache->IndexB[i];
				Vec2 wALocal = shapeA.GetVertex(v->indexA);
				Vec2 wBLocal = shapeB.GetVertex(v->indexB);
				v->wA = Common.Math.Mul(transformA, wALocal);
				v->wB = Common.Math.Mul(transformB, wBLocal);
				v->w = v->wB - v->wA;
				v->a = 0.0f;
			}

			// Compute the new simplex metric, if it is substantially different than
			// old metric then flush the simplex.
			if (Count > 1)
			{
				float metric1 = cache->Metric;
				float metric2 = GetMetric();
				if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Common.Settings.FLT_EPSILON)
				{
					// Reset the simplex.
					Count = 0;
				}
			}

			// If the cache is empty or invalid ...
			if (Count == 0)
			{
				SimplexVertex* v = vertices + 0;
				v->indexA = 0;
				v->indexB = 0;
				Vec2 wALocal = shapeA.GetVertex(0);
				Vec2 wBLocal = shapeB.GetVertex(0);
				v->wA = Common.Math.Mul(transformA, wALocal);
				v->wB = Common.Math.Mul(transformB, wBLocal);
				v->w = v->wB - v->wA;
				Count = 1;
			}
		}

		public unsafe void WriteCache(SimplexCache* cache)
		{
			cache->Metric = GetMetric();
			cache->Count = (UInt16)Count;
			SimplexVertex* vertices = &V1;
			for (int i = 0; i < Count; ++i)
			{
				cache->IndexA[i] = (Byte)(vertices[i].indexA);
				cache->IndexB[i] = (Byte)(vertices[i].indexB);
			}
		}

		public Vec2 GetClosestPoint()
		{
			switch (Count)
			{
				case 0:
					Box2DXDebug.Assert(false);
					return Vec2.Zero;
				case 1:
					return V1.w;
				case 2:
					return V1.a * V1.w + V2.a * V2.w;
				case 3:
					return Vec2.Zero;
				default:
					Box2DXDebug.Assert(false);
					return Vec2.Zero;
			}
		}

		public unsafe void GetWitnessPoints(Vec2* pA, Vec2* pB)
		{
			switch (Count)
			{
				case 0:
					Box2DXDebug.Assert(false);
					break;

				case 1:
					*pA = V1.wA;
					*pB = V1.wB;
					break;

				case 2:
					*pA = V1.a * V1.wA + V2.a * V2.wA;
					*pB = V1.a * V1.wB + V2.a * V2.wB;
					break;

				case 3:
					*pA = V1.a * V1.wA + V2.a * V2.wA + V2.a * V2.wA;
					*pB = *pA;
					break;

				default:
					Box2DXDebug.Assert(false);
					break;
			}
		}

		public float GetMetric()
		{
			switch (Count)
			{
				case 0:
					Box2DXDebug.Assert(false);
					return 0.0f;

				case 1:
					return 0.0f;

				case 2:
					return Vec2.Distance(V1.w, V2.w);

				case 3:
					return Vec2.Cross(V2.w - V1.w, V3.w - V1.w);

				default:
					Box2DXDebug.Assert(false);
					return 0.0f;
			}
		}

		// Solve a line segment using barycentric coordinates.
		//
		// p = a1 * w1 + a2 * w2
		// a1 + a2 = 1
		//
		// The vector from the origin to the closest point on the line is
		// perpendicular to the line.
		// e12 = w2 - w1
		// dot(p, e) = 0
		// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
		//
		// 2-by-2 linear system
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		//
		// Define
		// d12_1 =  dot(w2, e12)
		// d12_2 = -dot(w1, e12)
		// d12 = d12_1 + d12_2
		//
		// Solution
		// a1 = d12_1 / d12
		// a2 = d12_2 / d12
		public void Solve2()
		{
			Vec2 w1 = V1.w;
			Vec2 w2 = V2.w;
			Vec2 e12 = w2 - w1;

			// w1 region
			float d12_2 = -Vec2.Dot(w1, e12);
			if (d12_2 <= 0.0f)
			{
				// a2 <= 0, so we clamp it to 0
				V1.a = 1.0f;
				Count = 1;
				return;
			}

			// w2 region
			float d12_1 = Vec2.Dot(w2, e12);
			if (d12_1 <= 0.0f)
			{
				// a1 <= 0, so we clamp it to 0
				V2.a = 1.0f;
				Count = 1;
				V1 = V2;
				return;
			}

			// Must be in e12 region.
			float inv_d12 = 1.0f / (d12_1 + d12_2);
			V1.a = d12_1 * inv_d12;
			V2.a = d12_2 * inv_d12;
			Count = 2;
		}
		// Possible regions:
		// - points[2]
		// - edge points[0]-points[2]
		// - edge points[1]-points[2]
		// - inside the triangle
		public void Solve3()
		{
			Vec2 w1 = V1.w;
			Vec2 w2 = V2.w;
			Vec2 w3 = V3.w;

			// Edge12
			// [1      1     ][a1] = [1]
			// [w1.e12 w2.e12][a2] = [0]
			// a3 = 0
			Vec2 e12 = w2 - w1;
			float w1e12 = Vec2.Dot(w1, e12);
			float w2e12 = Vec2.Dot(w2, e12);
			float d12_1 = w2e12;
			float d12_2 = -w1e12;

			// Edge13
			// [1      1     ][a1] = [1]
			// [w1.e13 w3.e13][a3] = [0]
			// a2 = 0
			Vec2 e13 = w3 - w1;
			float w1e13 = Vec2.Dot(w1, e13);
			float w3e13 = Vec2.Dot(w3, e13);
			float d13_1 = w3e13;
			float d13_2 = -w1e13;

			// Edge23
			// [1      1     ][a2] = [1]
			// [w2.e23 w3.e23][a3] = [0]
			// a1 = 0
			Vec2 e23 = w3 - w2;
			float w2e23 = Vec2.Dot(w2, e23);
			float w3e23 = Vec2.Dot(w3, e23);
			float d23_1 = w3e23;
			float d23_2 = -w2e23;

			// Triangle123
			float n123 = Vec2.Cross(e12, e13);

			float d123_1 = n123 * Vec2.Cross(w2, w3);
			float d123_2 = n123 * Vec2.Cross(w3, w1);
			float d123_3 = n123 * Vec2.Cross(w1, w2);

			// w1 region
			if (d12_2 <= 0.0f && d13_2 <= 0.0f)
			{
				V1.a = 1.0f;
				Count = 1;
				return;
			}

			// e12
			if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
			{
				float inv_d12 = 1.0f / (d12_1 + d12_2);
				V1.a = d12_1 * inv_d12;
				V2.a = d12_1 * inv_d12;
				Count = 2;
				return;
			}

			// e13
			if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
			{
				float inv_d13 = 1.0f / (d13_1 + d13_2);
				V1.a = d13_1 * inv_d13;
				V3.a = d13_2 * inv_d13;
				Count = 2;
				V2 = V3;
				return;
			}

			// w2 region
			if (d12_1 <= 0.0f && d23_2 <= 0.0f)
			{
				V2.a = 1.0f;
				Count = 1;
				V1 = V2;
				return;
			}

			// w3 region
			if (d13_1 <= 0.0f && d23_1 <= 0.0f)
			{
				V3.a = 1.0f;
				Count = 1;
				V1 = V3;
				return;
			}

			// e23
			if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
			{
				float inv_d23 = 1.0f / (d23_1 + d23_2);
				V2.a = d23_1 * inv_d23;
				V3.a = d23_2 * inv_d23;
				Count = 2;
				V1 = V3;
				return;
			}

			// Must be in triangle123
			float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
			V1.a = d123_1 * inv_d123;
			V2.a = d123_2 * inv_d123;
			V3.a = d123_3 * inv_d123;
			Count = 3;
		}


	}

	public partial class Collision
	{
		/// <summary>
		/// Compute the closest points between two shapes. Supports any combination of:
		/// CircleShape, PolygonShape, EdgeShape. The simplex cache is input/output.
		/// On the first call set SimplexCache.Count to zero.
		/// </summary>		
		public unsafe static void Distance(DistanceOutput* output, SimplexCache* cache, DistanceInput* input, Shape shapeA, Shape shapeB)
		{
			XForm transformA = input->TransformA;
			XForm transformB = input->TransformB;

			// Initialize the simplex.
			Simplex simplex = new Simplex();
			simplex.ReadCache(cache, shapeA, transformA, shapeB, transformB);

			// Get simplex vertices as an array.
			SimplexVertex* vertices = &simplex.V1;

			// These store the vertices of the last simplex so that we
			// can check for duplicates and prevent cycling.
			int* lastA = stackalloc int[4], lastB = stackalloc int[4];
			int lastCount = 0;

			// Main iteration loop.
			int iter = 0;
			const int k_maxIterationCount = 20;
			while (iter < k_maxIterationCount)
			{
				// Copy simplex so we can identify duplicates.
				lastCount = simplex.Count;
				for (int i = 0; i < lastCount; ++i)
				{
					lastA[i] = vertices[i].indexA;
					lastB[i] = vertices[i].indexB;
				}

				switch (simplex.Count)
				{
					case 1:
						break;

					case 2:
						simplex.Solve2();
						break;

					case 3:
						simplex.Solve3();
						break;

					default:
						Box2DXDebug.Assert(false);
						break;
				}

				// If we have 3 points, then the origin is in the corresponding triangle.
				if (simplex.Count == 3)
				{
					break;
				}

				// Compute closest point.
				Vec2 p = simplex.GetClosestPoint();
				float distanceSqr = p.LengthSquared();

				// Ensure the search direction is numerically fit.
				if (distanceSqr < Common.Settings.FLT_EPSILON * Common.Settings.FLT_EPSILON)
				{
					// The origin is probably contained by a line segment
					// or triangle. Thus the shapes are overlapped.

					// We can't return zero here even though there may be overlap.
					// In case the simplex is a point, segment, or triangle it is difficult
					// to determine if the origin is contained in the CSO or very close to it.
					break;
				}

				// Compute a tentative new simplex vertex using support points.
				SimplexVertex* vertex = vertices + simplex.Count;
				vertex->indexA = shapeA.GetSupport(Common.Math.MulT(transformA.R, p));
				vertex->wA = Common.Math.Mul(transformA, shapeA.GetVertex(vertex->indexA));
				Vec2 wBLocal;
				vertex->indexB = shapeB.GetSupport(Common.Math.MulT(transformB.R, -p));
				vertex->wB = Common.Math.Mul(transformB, shapeB.GetVertex(vertex->indexB));
				vertex->w = vertex->wB - vertex->wA;

				// Iteration count is equated to the number of support point calls.
				++iter;

				// Check for convergence.
				float lowerBound = Vec2.Dot(p, vertex->w);
				float upperBound = distanceSqr;
				const float k_relativeTolSqr = 0.01f * 0.01f;	// 1:100
				if (upperBound - lowerBound <= k_relativeTolSqr * upperBound)
				{
					// Converged!
					break;
				}

				// Check for duplicate support points.
				bool duplicate = false;
				for (int i = 0; i < lastCount; ++i)
				{
					if (vertex->indexA == lastA[i] && vertex->indexB == lastB[i])
					{
						duplicate = true;
						break;
					}
				}

				// If we found a duplicate support point we must exit to avoid cycling.
				if (duplicate)
				{
					break;
				}

				// New vertex is ok and needed.
				++simplex.Count;
			}

			// Prepare output.
			simplex.GetWitnessPoints(&output->PointA, &output->PointB);
			output->Distance = Vec2.Distance(output->PointA, output->PointB);
			output->Iterations = iter;

			// Cache the simplex.
			simplex.WriteCache(cache);

			// Apply radii if requested.
			if (input->UseRadii)
			{
				float rA = shapeA._radius;
				float rB = shapeB._radius;

				if (output->Distance > rA + rB && output->Distance > Common.Settings.FLT_EPSILON)
				{
					// Shapes are still no overlapped.
					// Move the witness points to the outer surface.
					output->Distance -= rA + rB;
					Vec2 normal = output->PointB - output->PointA;
					normal.Normalize();
					output->PointA += rA * normal;
					output->PointB -= rB * normal;
				}
				else
				{
					// Shapes are overlapped when radii are considered.
					// Move the witness points to the middle.
					Vec2 p = 0.5f * (output->PointA + output->PointB);
					output->PointA = p;
					output->PointB = p;
					output->Distance = 0.0f;
				}
			}
		}
	}
}