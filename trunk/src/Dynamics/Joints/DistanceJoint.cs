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

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Dynamics
{
	/// <summary>
	/// Distance joint definition. This requires defining an
	/// anchor point on both bodies and the non-zero length of the
	/// distance joint. The definition uses local anchor points
	/// so that the initial configuration can violate the constraint
	/// slightly. This helps when saving and loading a game.
	/// @warning Do not use a zero or short length.
	/// </summary>
	public class DistanceJointDef : JointDef
	{
		public DistanceJointDef()
		{
			Type = JointType.DistanceJoint;
			LocalAnchor1.Set(0.0f, 0.0f);
			LocalAnchor2.Set(0.0f, 0.0f);
			Length = 1.0f;
		}

		/// <summary>
		/// Initialize the bodies, anchors, and length using the world anchors.
		/// </summary>
		/// <param name="body1"></param>
		/// <param name="body2"></param>
		/// <param name="anchor1"></param>
		/// <param name="anchor2"></param>
		public void Initialize(Body body1, Body body2, Vector2 anchor1, Vector2 anchor2)
		{
			Body1 = body1;
			Body2 = body2;
			LocalAnchor1 = body1.GetLocalPoint(anchor1);
			LocalAnchor2 = body2.GetLocalPoint(anchor2);
			Vector2 d = anchor2 - anchor1;
			Length = d.Length();
		}

		/// <summary>
		/// The local anchor point relative to body1's origin.
		/// </summary>
		public Vector2 LocalAnchor1;

		/// <summary>
		/// The local anchor point relative to body2's origin.
		/// </summary>
		public Vector2 LocalAnchor2;

		/// <summary>
		/// The equilibrium length between the anchor points.
		/// </summary>
		public float Length;
	}

	/// <summary>
	/// A distance joint constrains two points on two bodies
	/// to remain at a fixed distance from each other. You can view
	/// this as a massless, rigid rod.
	/// </summary>
	public class DistanceJoint : Joint
	{
		public Vector2 _localAnchor1;
		public Vector2 _localAnchor2;
		public Vector2 _u;
		public float _force;
		public float _mass;		// effective mass for the constraint.
		public float _length;

		public override Vector2 Anchor1
		{
			get { return _body1.GetWorldPoint(_localAnchor1);}
		}

		public override Vector2 Anchor2
		{
			get { return _body2.GetWorldPoint(_localAnchor2);}
		}

		public override Vector2 ReactionForce
		{
			get { return _force * _u; }
		}

		public override float ReactionTorque
		{
			get { return 0.0f; }
		}

		public DistanceJoint(DistanceJointDef def)
			: base(def)
		{
			_localAnchor1 = def.LocalAnchor1;
			_localAnchor2 = def.LocalAnchor2;
			_length = def.Length;
			_force = 0.0f;
		}

		public override void InitVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			// Compute the effective mass matrix.
			Vector2 r1 = Common.Math.Mul(b1._xf.R, _localAnchor1 - b1.GetLocalCenter());
			Vector2 r2 = Common.Math.Mul(b2._xf.R, _localAnchor2 - b2.GetLocalCenter());
			_u = b2._sweep.C + r2 - b1._sweep.C - r1;

			// Handle singularity.
			float length = _u.Length();
			if (length > Settings.LinearSlop)
			{
				_u *= 1.0f / length;
			}
			else
			{
				_u.Set(0.0f, 0.0f);
			}

			float cr1u = Vector2.Cross(r1, _u);
			float cr2u = Vector2.Cross(r2, _u);
			_mass = b1._invMass + b1._invI * cr1u * cr1u + b2._invMass + b2._invI * cr2u * cr2u;
			Box2DXDebug.Assert(_mass > Common.Math.FLT_EPSILON);
			_mass = 1.0f / _mass;

			if (World.s_enableWarmStarting!=0)
			{
				Vector2 P = step.Dt * _force * _u;
				b1._linearVelocity -= b1._invMass * P;
				b1._angularVelocity -= b1._invI * Vector2.Cross(r1, P);
				b2._linearVelocity += b2._invMass * P;
				b2._angularVelocity += b2._invI * Vector2.Cross(r2, P);
			}
			else
			{
				_force = 0.0f;
			}
		}

		public override bool SolvePositionConstraints()
		{
			Body b1 = _body1;
			Body b2 = _body2;

			Vector2 r1 = Common.Math.Mul(b1._xf.R, _localAnchor1 - b1.GetLocalCenter());
			Vector2 r2 = Common.Math.Mul(b2._xf.R, _localAnchor2 - b2.GetLocalCenter());

			Vector2 d = b2._sweep.C + r2 - b1._sweep.C - r1;

			float length = d.Normalize();
			float C = length - _length;
			C = Common.Math.Clamp(C, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);

			float impulse = -_mass * C;
			_u = d;
			Vector2 P = impulse * _u;

			b1._sweep.C -= b1._invMass * P;
			b1._sweep.A -= b1._invI * Vector2.Cross(r1, P);
			b2._sweep.C += b2._invMass * P;
			b2._sweep.A += b2._invI * Vector2.Cross(r2, P);

			b1.SynchronizeTransform();
			b2.SynchronizeTransform();

			return System.Math.Abs(C) < Settings.LinearSlop;
		}

		public override void SolveVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			Vector2 r1 = Common.Math.Mul(b1._xf.R, _localAnchor1 - b1.GetLocalCenter());
			Vector2 r2 = Common.Math.Mul(b2._xf.R, _localAnchor2 - b2.GetLocalCenter());

			// Cdot = dot(u, v + cross(w, r))
			Vector2 v1 = b1._linearVelocity + Vector2.Cross(b1._angularVelocity, r1);
			Vector2 v2 = b2._linearVelocity + Vector2.Cross(b2._angularVelocity, r2);
			float Cdot = Vector2.Dot(_u, v2 - v1);
			float force = -step.Inv_Dt * _mass * Cdot;
			_force += force;

			Vector2 P = step.Dt * force * _u;
			b1._linearVelocity -= b1._invMass * P;
			b1._angularVelocity -= b1._invI * Vector2.Cross(r1, P);
			b2._linearVelocity += b2._invMass * P;
			b2._angularVelocity += b2._invI * Vector2.Cross(r2, P);
		}
	}
}
