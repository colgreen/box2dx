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

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Dynamics
{
	/// <summary>
	/// Mouse joint definition. This requires a world target point,
	/// tuning parameters, and the time step.
	/// </summary>
	public class MouseJointDef : JointDef
	{
		public MouseJointDef()
		{
			Type = JointType.MouseJoint;
			Target.Set(0.0f, 0.0f);
			MaxForce = 0.0f;
			FrequencyHz = 5.0f;
			DampingRatio = 0.7f;
			TimeStep = 1.0f / 60.0f;
		}

		/// <summary>
		/// The initial world target point. This is assumed
		/// to coincide with the body anchor initially.
		/// </summary>
		public Vector2 Target;

		/// <summary>
		/// The maximum constraint force that can be exerted
		/// to move the candidate body. Usually you will express
		/// as some multiple of the weight (multiplier * mass * gravity).
		/// </summary>
		public float MaxForce;

		/// <summary>
		/// The response speed.
		/// </summary>
		public float FrequencyHz;

		/// <summary>
		/// The damping ratio. 0 = no damping, 1 = critical damping.
		/// </summary>
		public float DampingRatio;

		/// <summary>
		/// The time step used in the simulation.
		/// </summary>
		public float TimeStep;
	}

	/// <summary>
	/// A mouse joint is used to make a point on a body track a
	/// specified world point. This a soft constraint with a maximum
	/// force. This allows the constraint to stretch and without
	/// applying huge forces.
	/// </summary>
	public class MouseJoint : Joint
	{
		public Vector2 _localAnchor;
		public Vector2 _target;
		public Vector2 _force;

		public Mat22 _mass;		// effective mass for point-to-point constraint.
		public Vector2 _C;				// position error
		public float _maxForce;
		public float _beta;			// bias factor
		public float _gamma;		// softness

		public override Vector2 Anchor1
		{
			get { return _target; }
		}

		public override Vector2 Anchor2
		{
			get { return _body2.GetWorldPoint(_localAnchor); }
		}

		public override Vector2 ReactionForce
		{
			get { return _force; }
		}

		public override float ReactionTorque
		{
			get { return 0.0f; }
		}

		/// <summary>
		/// Use this to update the target point.
		/// </summary>
		/// <param name="target"></param>
		public void SetTarget(Vector2 target)
		{
			if (_body2.IsSleeping())
			{
				_body2.WakeUp();
			}
			_target = target;
		}

		public MouseJoint(MouseJointDef def)
			: base(def)
		{
			_target = def.Target;
			_localAnchor = Common.Math.MulT(_body2._xf, _target);

			_maxForce = def.MaxForce;
			_force.SetZero();

			float mass = _body2._mass;

			// Frequency
			float omega = 2.0f * Settings.Pi * def.FrequencyHz;

			// Damping coefficient
			float d = 2.0f * mass * def.DampingRatio * omega;

			// Spring stiffness
			float k = mass * omega * omega;

			// magic formulas
			_gamma = 1.0f / (d + def.TimeStep * k);
			_beta = def.TimeStep * k / (d + def.TimeStep * k);
		}

		public override void InitVelocityConstraints(TimeStep step)
		{
			Body b = _body2;

			// Compute the effective mass matrix.
			Vector2 r = Common.Math.Mul(b._xf.R, _localAnchor - b.GetLocalCenter());

			// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
			//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
			//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
			float invMass = b._invMass;
			float invI = b._invI;

			Mat22 K1 = new Mat22();
			K1.Col1.X = invMass; K1.Col2.X = 0.0f;
			K1.Col1.Y = 0.0f; K1.Col2.Y = invMass;

			Mat22 K2 = new Mat22();
			K2.Col1.X = invI * r.Y * r.Y; K2.Col2.X = -invI * r.X * r.Y;
			K2.Col1.Y = -invI * r.X * r.Y; K2.Col2.Y = invI * r.X * r.X;

			Mat22 K = K1 + K2;
			K.Col1.X += _gamma;
			K.Col2.Y += _gamma;

			_mass = K.Invert();

			_C = b._sweep.C + r - _target;

			// Cheat with some damping
			b._angularVelocity *= 0.98f;

			// Warm starting.
			Vector2 P = step.Dt * _force;
			b._linearVelocity += invMass * P;
			b._angularVelocity += invI * Vector2.Cross(r, P);
		}

		public override void SolveVelocityConstraints(TimeStep step)
		{
			Body b = _body2;

			Vector2 r = Common.Math.Mul(b._xf.R, _localAnchor - b.GetLocalCenter());

			// Cdot = v + cross(w, r)
			Vector2 Cdot = b._linearVelocity + Vector2.Cross(b._angularVelocity, r);
			Vector2 force = -step.Inv_Dt * Common.Math.Mul(_mass, Cdot + (_beta * step.Inv_Dt) * _C + _gamma * step.Dt * _force);

			Vector2 oldForce = _force;
			_force += force;
			float forceMagnitude = _force.Length();
			if (forceMagnitude > _maxForce)
			{
				_force *= _maxForce / forceMagnitude;
			}
			force = _force - oldForce;

			Vector2 P = step.Dt * force;
			b._linearVelocity += b._invMass * P;
			b._angularVelocity += b._invI * Vector2.Cross(r, P);
		}

		public override bool SolvePositionConstraints()
		{
			return true;
		}
	}
}