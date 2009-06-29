/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

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
	/// Inpute parameters for TimeOfImpact
	/// </summary>
	public struct TOIInput
	{
		public Sweep SweepA;
		public Sweep SweepB;
		public float SweepRadiusA;
		public float SweepRadiusB;
		public float Tolerance;
	}

	public partial class Collision
	{		
		public static int MaxToiIters = 0;
		public static int MaxToiRootIters = 0;

		// CCD via the secant method.
		/// <summary>
		/// Compute the time when two shapes begin to touch or touch at a closer distance.
		/// TOI considers the shape radii. It attempts to have the radii overlap by the tolerance.
		/// Iterations terminate with the overlap is within 0.5 * tolerance. The tolerance should be
		/// smaller than sum of the shape radii.
		/// Warning the sweeps must have the same time interval.
		/// </summary>
		/// <returns>
		/// The fraction between [0,1] in which the shapes first touch.
		/// fraction=0 means the shapes begin touching/overlapped, and fraction=1 means the shapes don't touch.
		/// </returns>
		public static float TimeOfImpact(TOIInput input, Shape shapeA, Shape shapeB)
		{
			Sweep sweepA = input.SweepA;
			Sweep sweepB = input.SweepB;

			Box2DXDebug.Assert(sweepA.T0 == sweepB.T0);
			Box2DXDebug.Assert(1.0f - sweepA.T0 > Common.Settings.FLT_EPSILON);

			float radius = shapeA._radius + shapeB._radius;
			float tolerance = input.Tolerance;

			float alpha = 0.0f;

			const int k_maxIterations = 1000;	// TODO_ERIN b2Settings
			int iter = 0;
			float target = 0.0f;

			// Prepare input for distance query.
			SimplexCache cache;
			cache.Count = 0;
			DistanceInput distanceInput;
			distanceInput.UseRadii = false;

			for (; ; )
			{
				XForm xfA, xfB;
				sweepA.GetTransform(out xf1, alpha);
				sweepB.GetTransform(out xf2, alpha);

				// Get the distance between shapes.
				distanceInput.TransformA = xfA;
				distanceInput.TransformB = xfB;
				DistanceOutput distanceOutput;
				Distance(ref distanceOutput, ref cache, ref distanceInput, shapeA, shapeB);

				if (distanceOutput.Distance <= 0.0f)
				{
					alpha = 1.0f;
					break;
				}

				SeparationFunction fcn;
				fcn.Initialize(ref cache, shapeA, xfA, shapeB, xfB);

				float separation = fcn.Evaluate(xfA, xfB);
				if (separation <= 0.0f)
				{
					alpha = 1.0f;
					break;
				}

				if (iter == 0)
				{
					// Compute a reasonable target distance to give some breathing room
					// for conservative advancement. We take advantage of the shape radii
					// to create additional clearance.
					if (separation > radius)
					{
						target = Common.Math.Max(radius - tolerance, 0.75f * radius);
					}
					else
					{
						target = Common.Math.Max(separation - tolerance, 0.02f * radius);
					}
				}

				if (separation - target < 0.5f * tolerance)
				{
					if (iter == 0)
					{
						alpha = 1.0f;
						break;
					}

					break;
				}

#if _FALSE
				// Dump the curve seen by the root finder
				{
					const int32 N = 100;
					float32 dx = 1.0f / N;
					float32 xs[N+1];
					float32 fs[N+1];

					float32 x = 0.0f;

					for (int32 i = 0; i <= N; ++i)
					{
						sweepA.GetTransform(&xfA, x);
						sweepB.GetTransform(&xfB, x);
						float32 f = fcn.Evaluate(xfA, xfB) - target;

						printf("%g %g\n", x, f);

						xs[i] = x;
						fs[i] = f;

						x += dx;
					}
				}
#endif

				// Compute 1D root of: f(x) - target = 0
				float newAlpha = alpha;
				{
					float x1 = alpha, x2 = 1.0f;

					float f1 = separation;

					sweepA.GetTransform(out xfA, x2);
					sweepB.GetTransform(out xfB, x2);
					float f2 = fcn.Evaluate(xfA, xfB);

					// If intervals don't overlap at t2, then we are done.
					if (f2 >= target)
					{
						alpha = 1.0f;
						break;
					}

					// Determine when intervals intersect.
					int rootIterCount = 0;
					for (; ; )
					{
						// Use a mix of the secant rule and bisection.
						float x;
						if ((rootIterCount & 1) != 0)
						{
							// Secant rule to improve convergence.
							x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
						}
						else
						{
							// Bisection to guarantee progress.
							x = 0.5f * (x1 + x2);
						}

						sweepA.GetTransform(out xfA, x);
						sweepB.GetTransform(out xfB, x);

						float f = fcn.Evaluate(xfA, xfB);

						if (Common.Math.Abs(f - target) < 0.025f * tolerance)
						{
							newAlpha = x;
							break;
						}

						// Ensure we continue to bracket the root.
						if (f > target)
						{
							x1 = x;
							f1 = f;
						}
						else
						{
							x2 = x;
							f2 = f;
						}

						++rootIterCount;

						Box2DXDebug.Assert(rootIterCount < 50);
					}

					MaxToiRootIters = Common.Math.Max(MaxToiRootIters, rootIterCount);
				}

				// Ensure significant advancement.
				if (newAlpha < (1.0f + 100.0f * Common.Settings.FLT_EPSILON) * alpha)
				{
					break;
				}

				alpha = newAlpha;

				++iter;

				if (iter == k_maxIterations)
				{
					break;
				}
			}

			MaxToiIters = Common.Math.Max(MaxToiIters, iter);

			return alpha;
		}
	}
}