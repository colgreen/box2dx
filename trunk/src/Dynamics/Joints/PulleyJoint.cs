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

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2) >= 0
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)
//
// Limit:
// C = maxLength - length
// u = (p - s) / norm(p - s)
// Cdot = -dot(u, v + cross(w, r))
// K = invMass + invI * cross(r, u)^2
// 0 <= impulse

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Dynamics
{
	/// <summary>
	/// Pulley joint definition. This requires two ground anchors,
	/// two dynamic body anchor points, max lengths for each side,
	/// and a pulley ratio.
	/// </summary>
	public class PulleyJointDef : JointDef
	{
		public PulleyJointDef()
		{
			Type = JointType.PulleyJoint;
			GroundAnchor1.Set(-1.0f, 1.0f);
			GroundAnchor2.Set(1.0f, 1.0f);
			LocalAnchor1.Set(-1.0f, 0.0f);
			LocalAnchor2.Set(1.0f, 0.0f);
			Length1 = 0.0f;
			MaxLength1 = 0.0f;
			Length2 = 0.0f;
			MaxLength2 = 0.0f;
			Ratio = 1.0f;
			CollideConnected = true;
		}

		/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
		public void Initialize(Body body1, Body body2,
						Vector2 groundAnchor1, Vector2 groundAnchor2,
						Vector2 anchor1, Vector2 anchor2,
						float ratio)
		{
			Body1 = body1;
			Body2 = body2;
			GroundAnchor1 = groundAnchor1;
			GroundAnchor2 = groundAnchor2;
			LocalAnchor1 = body1.GetLocalPoint(anchor1);
			LocalAnchor2 = body2.GetLocalPoint(anchor2);
			Vector2 d1 = anchor1 - groundAnchor1;
			Length1 = d1.Length();
			Vector2 d2 = anchor2 - groundAnchor2;
			Length2 = d2.Length();
			Ratio = ratio;
			Box2DXDebug.Assert(ratio > Common.Math.FLOAT32_EPSILON);
			float C = Length1 + ratio * Length2;
			MaxLength1 = C - ratio * PulleyJoint.MinPulleyLength;
			MaxLength2 = (C - PulleyJoint.MinPulleyLength) / ratio;
		}

		/// <summary>
		/// The first ground anchor in world coordinates. This point never moves.
		/// </summary>
		public Vector2 GroundAnchor1;

		/// <summary>
		/// The second ground anchor in world coordinates. This point never moves.
		/// </summary>
		public Vector2 GroundAnchor2;

		/// <summary>
		/// The local anchor point relative to body1's origin.
		/// </summary>
		public Vector2 LocalAnchor1;

		/// <summary>
		/// The local anchor point relative to body2's origin.
		/// </summary>
		public Vector2 LocalAnchor2;

		/// <summary>
		/// The a reference length for the segment attached to body1.
		/// </summary>
		public float Length1;

		/// <summary>
		/// The maximum length of the segment attached to body1.
		/// </summary>
		public float MaxLength1;

		/// <summary>
		/// The a reference length for the segment attached to body2.
		/// </summary>
		public float Length2;

		/// <summary>
		/// The maximum length of the segment attached to body2.
		/// </summary>
		public float MaxLength2;

		/// <summary>
		/// The pulley ratio, used to simulate a block-and-tackle.
		/// </summary>
		public float Ratio;
	}

	/// <summary>
	/// The pulley joint is connected to two bodies and two fixed ground points.
	/// The pulley supports a ratio such that:
	/// length1 + ratio * length2 <= constant
	/// Yes, the force transmitted is scaled by the ratio.
	/// The pulley also enforces a maximum length limit on both sides. This is
	/// useful to prevent one side of the pulley hitting the top.
	/// </summary>
	public class PulleyJoint : Joint
	{
		public static readonly float MinPulleyLength = 2.0f;

		public Body _ground;
		public Vector2 _groundAnchor1;
		public Vector2 _groundAnchor2;
		public Vector2 _localAnchor1;
		public Vector2 _localAnchor2;

		public Vector2 _u1;
		public Vector2 _u2;

		public float _constant;
		public float _ratio;

		public float _maxLength1;
		public float _maxLength2;

		// Effective masses
		public float _pulleyMass;
		public float _limitMass1;
		public float _limitMass2;

		// Impulses for accumulation/warm starting.
		public float _force;
		public float _limitForce1;
		public float _limitForce2;

		// Position impulses for accumulation.
		public float _positionImpulse;
		public float _limitPositionImpulse1;
		public float _limitPositionImpulse2;

		public LimitState _state;
		public LimitState _limitState1;
		public LimitState _limitState2;

		public override Vector2 Anchor1
		{
			get { return _body1.GetWorldPoint(_localAnchor1); }
		}

		public override Vector2 Anchor2
		{
			get { return _body2.GetWorldPoint(_localAnchor2); }
		}

		public override Vector2 ReactionForce
		{
			get
			{
				Vector2 F = Settings.FORCE_SCALE(_force) * _u2;
				return F;
			}
		}

		public override float ReactionTorque
		{
			get { return 0.0f; }
		}

		/// <summary>
		/// Get the first ground anchor.
		/// </summary>
		public Vector2 GroundAnchor1
		{
			get { return _ground._xf.Position + _groundAnchor1; }
		}

		/// <summary>
		/// Get the second ground anchor.
		/// </summary>
		public Vector2 GroundAnchor2
		{
			get { return _ground._xf.Position + _groundAnchor2; }
		}

		/// <summary>
		/// Get the current length of the segment attached to body1.
		/// </summary>
		public float Length1
		{
			get
			{
				Vector2 p = _body1.GetWorldPoint(_localAnchor1);
				Vector2 s = _ground._xf.Position + _groundAnchor1;
				Vector2 d = p - s;
				return d.Length();
			}
		}

		/// <summary>
		/// Get the current length of the segment attached to body2.
		/// </summary>
		public float Length2
		{
			get
			{
				Vector2 p = _body2.GetWorldPoint(_localAnchor2);
				Vector2 s = _ground._xf.Position + _groundAnchor2;
				Vector2 d = p - s;
				return d.Length();
			}
		}

		/// <summary>
		/// Get the pulley ratio.
		/// </summary>
		public float Ratio
		{
			get { return _ratio; }
		}

		public PulleyJoint(PulleyJointDef def)
			: base(def)
		{
			_ground = _body1._world._groundBody;
			_groundAnchor1 = def.GroundAnchor1 - _ground._xf.Position;
			_groundAnchor2 = def.GroundAnchor2 - _ground._xf.Position;
			_localAnchor1 = def.LocalAnchor1;
			_localAnchor2 = def.LocalAnchor2;

			Box2DXDebug.Assert(def.Ratio != 0.0f);
			_ratio = def.Ratio;

			_constant = def.Length1 + _ratio * def.Length2;

			_maxLength1 = Common.Math.Min(def.MaxLength1, _constant - _ratio * PulleyJoint.MinPulleyLength);
			_maxLength2 = Common.Math.Min(def.MaxLength2, (_constant - PulleyJoint.MinPulleyLength) / _ratio);

			_force = 0.0f;
			_limitForce1 = 0.0f;
			_limitForce2 = 0.0f;
		}

#warning "Not Implemented Yet!"
		public override void InitVelocityConstraints(TimeStep step)
		{
			throw new NotImplementedException();
		}

		public override void SolveVelocityConstraints(TimeStep step)
		{
			throw new NotImplementedException();
		}

		public override bool SolvePositionConstraints()
		{
			throw new NotImplementedException();
		}
	}
}