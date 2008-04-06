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

using Box2DX.Collision;
using Box2DX.Common;

namespace Box2DX.Dynamics
{
#warning "CAS"
	public class ContactConstraintPoint
	{
		public Vector2 LocalAnchor1;
		public Vector2 LocalAnchor2;
		public Vector2 R1;
		public Vector2 R2;
		public float NormalForce;
		public float TangentForce;
		public float PositionImpulse;
		public float NormalMass;
		public float TangentMass;
		public float EqualizedMass;
		public float Separation;
		public float VelocityBias;
	}

#warning "CAS"
	public class ContactConstraint
	{
		public ContactConstraintPoint[] Points = new ContactConstraintPoint[Settings.MaxManifoldPoints];
		public Vector2 Normal;
		public Manifold Manifold;
		public Body Body1;
		public Body Body2;
		public float Friction;
		public float Restitution;
		public int PointCount;

		public ContactConstraint()
		{
			for (int i = 0; i < Settings.MaxManifoldPoints; i++)
				Points[i] = new ContactConstraintPoint();
		}
	}

	public class ContactSolver : IDisposable
	{
#if TARGET_FLOAT32_IS_FIXED
		public static float	FORCE_SCALE2(x){ return x<<7;}
		public static float FORCE_INV_SCALE2(x)	{return x>>7;}
#else
		public static float	FORCE_SCALE2(float x){ return x;}
		public static float FORCE_INV_SCALE2(float x)	{return x;}
#endif

		public TimeStep _step;
		public ContactConstraint[] _constraints;
		public int _constraintCount;

		public ContactSolver(TimeStep step, Contact[] contacts, int contactCount)
		{
			_step = step;

			_constraintCount = 0;
			for (int i = 0; i < contactCount; ++i)
			{
				Box2DXDebug.Assert(contacts[i].IsSolid());
				_constraintCount += contacts[i].GetManifoldCount();
			}

			_constraints = new ContactConstraint[_constraintCount];
			for (int i = 0; i < _constraintCount; i++)
				_constraints[i] = new ContactConstraint();

			int count = 0;
			for (int i = 0; i < contactCount; ++i)
			{
				Contact contact = contacts[i];

				Body b1 = contact._shape1._body;
				Body b2 = contact._shape2._body;
				int manifoldCount = contact.GetManifoldCount();
				Manifold manifolds = contact.GetManifolds();
				float friction = contact._friction;
				float restitution = contact._restitution;

				Vector2 v1 = b1._linearVelocity;
				Vector2 v2 = b2._linearVelocity;
				float w1 = b1._angularVelocity;
				float w2 = b2._angularVelocity;

#warning "manifold array"
				//for (int j = 0; j < manifoldCount; ++j)
				if(manifoldCount>0)
				{
					Manifold manifold = manifolds;

					Box2DXDebug.Assert(manifold.PointCount > 0);

					Vector2 normal = manifold.Normal;

					Box2DXDebug.Assert(count < _constraintCount);
					ContactConstraint c = _constraints[count];
					c.Body1 = b1;
					c.Body2 = b2;
					c.Manifold = manifold;
					c.Normal = normal;
					c.PointCount = manifold.PointCount;
					c.Friction = friction;
					c.Restitution = restitution;

					for (int k = 0; k < c.PointCount; ++k)
					{
						ManifoldPoint cp = manifold.Points[k];
						ContactConstraintPoint ccp = c.Points[k];

						ccp.NormalForce = cp.NormalForce;
						ccp.TangentForce = cp.TangentForce;
						ccp.Separation = cp.Separation;
						ccp.PositionImpulse = 0.0f;

						ccp.LocalAnchor1 = cp.LocalPoint1;
						ccp.LocalAnchor2 = cp.LocalPoint2;
						ccp.R1 = Common.Math.Mul(b1._xf.R, cp.LocalPoint1 - b1.GetLocalCenter());
						ccp.R2 = Common.Math.Mul(b2._xf.R, cp.LocalPoint2 - b2.GetLocalCenter());

						float r1Sqr = Vector2.Dot(ccp.R1, ccp.R1);
						float r2Sqr = Vector2.Dot(ccp.R2, ccp.R2);
						float rn1 = Vector2.Dot(ccp.R1, normal);
						float rn2 = Vector2.Dot(ccp.R2, normal);

						float kNormal = b1._invMass + b2._invMass;
						kNormal += b1._invI * (r1Sqr - rn1 * rn1) + b2._invI * (r2Sqr - rn2 * rn2);
						Box2DXDebug.Assert(kNormal > Common.Math.FLOAT32_EPSILON);
						ccp.NormalMass = 1.0f / kNormal;

						float kEqualized = b1._mass * b1._invMass + b2._mass * b2._invMass;
						kEqualized += b1._mass * b1._invI * (r1Sqr - rn1 * rn1) + b2._mass * b2._invI * (r2Sqr - rn2 * rn2);
						Box2DXDebug.Assert(kEqualized > Common.Math.FLOAT32_EPSILON);
						ccp.EqualizedMass = 1.0f / kEqualized;

						Vector2 tangent = Vector2.Cross(normal, 1.0f);

						float rt1 = Vector2.Dot(ccp.R1, tangent);
						float rt2 = Vector2.Dot(ccp.R2, tangent);
						float kTangent = b1._invMass + b2._invMass;
						kTangent += b1._invI * (r1Sqr - rt1 * rt1) + b2._invI * (r2Sqr - rt2 * rt2);
						Box2DXDebug.Assert(kTangent > Common.Math.FLOAT32_EPSILON);
						ccp.TangentMass = 1.0f / kTangent;

						// Setup a velocity bias for restitution.
						ccp.VelocityBias = 0.0f;
						if (ccp.Separation > 0.0f)
						{
							ccp.VelocityBias = -60.0f * ccp.Separation; // TODO_ERIN b2TimeStep
						}

						float vRel = Vector2.Dot(c.Normal, v2 + Vector2.Cross(w2, ccp.R2) - v1 - Vector2.Cross(w1, ccp.R1));
						if (vRel < -Settings.VelocityThreshold)
						{
							ccp.VelocityBias += -c.Restitution * vRel;
						}
					}

					++count;
				}
			}

			Box2DXDebug.Assert(count == _constraintCount);
		}

		public void Dispose()
		{
			_constraints = null;
		}

		public void InitVelocityConstraints()
		{
			// Warm start.
			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];

				Body b1 = c.Body1;
				Body b2 = c.Body2;
				float invMass1 = b1._invMass;
				float invI1 = b1._invI;
				float invMass2 = b2._invMass;
				float invI2 = b2._invI;
				Vector2 normal = c.Normal;
				Vector2 tangent = Vector2.Cross(normal, 1.0f);

				if (World.s_enableWarmStarting!=0)
				{
					for (int j = 0; j < c.PointCount; ++j)
					{
						ContactConstraintPoint ccp = c.Points[j];
						Vector2 P = FORCE_SCALE2(_step.Dt) * (ccp.NormalForce * normal + ccp.TangentForce * tangent);
						b1._angularVelocity -= invI1 * Vector2.Cross(ccp.R1, P);
						b1._linearVelocity -= invMass1 * P;
						b2._angularVelocity += invI2 * Vector2.Cross(ccp.R2, P);
						b2._linearVelocity += invMass2 * P;
					}
				}
				else
				{
					for (int j = 0; j < c.PointCount; ++j)
					{
						ContactConstraintPoint ccp = c.Points[j];
						ccp.NormalForce = 0.0f;
						ccp.TangentForce = 0.0f;
					}
				}
			}
		}

		public void SolveVelocityConstraints()
		{
			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];
				Body b1 = c.Body1;
				Body b2 = c.Body2;
				float w1 = b1._angularVelocity;
				float w2 = b2._angularVelocity;
				Vector2 v1 = b1._linearVelocity;
				Vector2 v2 = b2._linearVelocity;
				float invMass1 = b1._invMass;
				float invI1 = b1._invI;
				float invMass2 = b2._invMass;
				float invI2 = b2._invI;
				Vector2 normal = c.Normal;
				Vector2 tangent = Vector2.Cross(normal, 1.0f);
				float friction = c.Friction;
#if DEFERRED_UPDATE
				Vector2 b1_linearVelocity = b1._linearVelocity;
				float b1_angularVelocity = b1._angularVelocity;
				Vector2 b2_linearVelocity = b2._linearVelocity;
				float b2_angularVelocity = b2._angularVelocity;
#endif
				// Solve normal constraints
				for (int j = 0; j < c.PointCount; ++j)
				{
					ContactConstraintPoint ccp = c.Points[j];

					// Relative velocity at contact
					Vector2 dv = v2 + Vector2.Cross(w2, ccp.R2) - v1 - Vector2.Cross(w1, ccp.R1);

					// Compute normal force
					float vn = Vector2.Dot(dv, normal);

					float lambda = -FORCE_INV_SCALE2(_step.Inv_Dt) * ccp.NormalMass * (vn - ccp.VelocityBias);

					// b2Clamp the accumulated force
					float newForce = Common.Math.Max(ccp.NormalForce + lambda, 0.0f);
					lambda = newForce - ccp.NormalForce;

					// Apply contact impulse
					Vector2 P = FORCE_SCALE2(_step.Dt) * lambda * normal;
#if DEFERRED_UPDATE
					b1_linearVelocity -= invMass1 * P;
					b1_angularVelocity -= invI1 * Vector2.Cross(r1, P);

					b2_linearVelocity += invMass2 * P;
					b2_angularVelocity += invI2 * Vector2.Cross(r2, P);
#else
					v1 -= invMass1 * P;
					w1 -= invI1 * Vector2.Cross(ccp.R1, P);

					v2 += invMass2 * P;
					w2 += invI2 * Vector2.Cross(ccp.R2, P);
#endif
					ccp.NormalForce = newForce;
				}

#if DEFERRED_UPDATE
				b1._linearVelocity = b1_linearVelocity;
				b1._angularVelocity = b1_angularVelocity;
				b2._linearVelocity = b2_linearVelocity;
				b2._angularVelocity = b2_angularVelocity;
#endif

				// Solve tangent constraints
				for (int j = 0; j < c.PointCount; ++j)
				{
					ContactConstraintPoint ccp = c.Points[j];

					// Relative velocity at contact
					Vector2 dv = v2 + Vector2.Cross(w2, ccp.R2) - v1 - Vector2.Cross(w1, ccp.R1);

					// Compute tangent force
					float vt = Vector2.Dot(dv, tangent);
					float lambda = FORCE_INV_SCALE2(_step.Inv_Dt) * ccp.TangentMass * (-vt);

					// Clamp the accumulated force
					float maxFriction = friction * ccp.NormalForce;
					float newForce = Common.Math.Clamp(ccp.TangentForce + lambda, -maxFriction, maxFriction);
					lambda = newForce - ccp.TangentForce;

					// Apply contact impulse
					Vector2 P = FORCE_SCALE2(_step.Dt) * lambda * tangent;

					v1 -= invMass1 * P;
					w1 -= invI1 * Vector2.Cross(ccp.R1, P);

					v2 += invMass2 * P;
					w2 += invI2 * Vector2.Cross(ccp.R2, P);

					ccp.TangentForce = newForce;
				}

				b1._linearVelocity = v1;
				b1._angularVelocity = w1;
				b2._linearVelocity = v2;
				b2._angularVelocity = w2;
			}
		}

		public void FinalizeVelocityConstraints()
		{
			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];
				Manifold m = c.Manifold;

				for (int j = 0; j < c.PointCount; ++j)
				{
					m.Points[j].NormalForce = c.Points[j].NormalForce;
					m.Points[j].TangentForce = c.Points[j].TangentForce;
				}
			}
		}

		public bool SolvePositionConstraints(float baumgarte)
		{
			float minSeparation = 0.0f;

			for (int i = 0; i < _constraintCount; ++i)
			{
				ContactConstraint c = _constraints[i];
				Body b1 = c.Body1;
				Body b2 = c.Body2;
				float invMass1 = b1._mass * b1._invMass;
				float invI1 = b1._mass * b1._invI;
				float invMass2 = b2._mass * b2._invMass;
				float invI2 = b2._mass * b2._invI;

				Vector2 normal = c.Normal;

				// Solver normal constraints
				for (int j = 0; j < c.PointCount; ++j)
				{
					ContactConstraintPoint ccp = c.Points[j];

					Vector2 r1 = Common.Math.Mul(b1._xf.R, ccp.LocalAnchor1 - b1.GetLocalCenter());
					Vector2 r2 = Common.Math.Mul(b2._xf.R, ccp.LocalAnchor2 - b2.GetLocalCenter());

					Vector2 p1 = b1._sweep.C + r1;
					Vector2 p2 = b2._sweep.C + r2;
					Vector2 dp = p2 - p1;

					// Approximate the current separation.
					float separation = Vector2.Dot(dp, normal) + ccp.Separation;

					// Track max constraint error.
					minSeparation = Common.Math.Min(minSeparation, separation);

					// Prevent large corrections and allow slop.
					float C = baumgarte * Common.Math.Clamp(separation + Settings.LinearSlop, -Settings.MaxLinearCorrection, 0.0f);

					// Compute normal impulse
					float dImpulse = -ccp.EqualizedMass * C;

					// b2Clamp the accumulated impulse
					float impulse0 = ccp.PositionImpulse;
					ccp.PositionImpulse = Common.Math.Max(impulse0 + dImpulse, 0.0f);
					dImpulse = ccp.PositionImpulse - impulse0;

					Vector2 impulse = dImpulse * normal;

					b1._sweep.C -= invMass1 * impulse;
					b1._sweep.A -= invI1 * Vector2.Cross(r1, impulse);
					b1.SynchronizeTransform();

					b2._sweep.C += invMass2 * impulse;
					b2._sweep.A += invI2 * Vector2.Cross(r2, impulse);
					b2.SynchronizeTransform();
				}
			}

			// We can't expect minSpeparation >= -Settings.LinearSlop because we don't
			// push the separation above -Settings.LinearSlop.
			return minSeparation >= -1.5f * Settings.LinearSlop;
		}
	}
}
