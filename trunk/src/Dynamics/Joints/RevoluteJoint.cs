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

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Dynamics
{
	/// <summary>
	/// Revolute joint definition. This requires defining an
	/// anchor point where the bodies are joined. The definition
	/// uses local anchor points so that the initial configuration
	/// can violate the constraint slightly. You also need to
	/// specify the initial relative angle for joint limits. This
	/// helps when saving and loading a game.
	/// The local anchor points are measured from the body's origin
	/// rather than the center of mass because:
	/// 1. you might not know where the center of mass will be.
	/// 2. if you add/remove shapes from a body and recompute the mass,
	///    the joints will be broken.
	/// </summary>
	public class RevoluteJointDef : JointDef
	{
		public RevoluteJointDef()
		{
			Type = JointType.RevoluteJoint;
			LocalAnchor1.Set(0.0f, 0.0f);
			LocalAnchor2.Set(0.0f, 0.0f);
			ReferenceAngle = 0.0f;
			LowerAngle = 0.0f;
			UpperAngle = 0.0f;
			MaxMotorTorque = 0.0f;
			MotorSpeed = 0.0f;
			EnableLimit = false;
			EnableMotor = false;
		}

		/// <summary>
		/// Initialize the bodies, anchors, and reference angle using the world
		/// anchor.
		/// </summary>
		/// <param name="body1"></param>
		/// <param name="body2"></param>
		/// <param name="anchor"></param>
		public void Initialize(Body body1, Body body2, Vector2 anchor)
		{
			Body1 = body1;
			Body2 = body2;
			LocalAnchor1 = body1.GetLocalPoint(anchor);
			LocalAnchor2 = body2.GetLocalPoint(anchor);
			ReferenceAngle = body2.GetAngle() - body1.GetAngle();
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
		/// The body2 angle minus body1 angle in the reference state (radians).
		/// </summary>
		public float ReferenceAngle;

		/// <summary>
		/// A flag to enable joint limits.
		/// </summary>
		public bool EnableLimit;

		/// <summary>
		/// The lower angle for the joint limit (radians).
		/// </summary>
		public float LowerAngle;

		/// <summary>
		/// The upper angle for the joint limit (radians).
		/// </summary>
		public float UpperAngle;

		/// <summary>
		/// A flag to enable the joint motor.
		/// </summary>
		public bool EnableMotor;

		/// <summary>
		/// The desired motor speed. Usually in radians per second.
		/// </summary>
		public float MotorSpeed;

		/// <summary>
		/// The maximum motor torque used to achieve the desired motor speed.
		/// Usually in N-m.
		/// </summary>
		public float MaxMotorTorque;
	}

	/// <summary>
	/// A revolute joint constrains to bodies to share a common point while they
	/// are free to rotate about the point. The relative rotation about the shared
	/// point is the joint angle. You can limit the relative rotation with
	/// a joint limit that specifies a lower and upper angle. You can use a motor
	/// to drive the relative rotation about the shared point. A maximum motor torque
	/// is provided so that infinite forces are not generated.
	/// </summary>
	public class RevoluteJoint : Joint
	{
		public Vector2 _localAnchor1;	// relative
		public Vector2 _localAnchor2;
		public Vector2 _pivotForce;
		public float _motorForce;
		public float _limitForce;
		public float _limitPositionImpulse;

		public Mat22 _pivotMass;		// effective mass for point-to-point constraint.
		public float _motorMass;	// effective mass for motor/limit angular constraint.

		public bool _enableMotor;
		public float _maxMotorTorque;
		public float _motorSpeed;

		public bool _enableLimit;
		public float _referenceAngle;
		public float _lowerAngle;
		public float _upperAngle;
		public LimitState _limitState;

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
			get { return _pivotForce; }
		}

		public override float ReactionTorque
		{
			get { return _limitForce; }
		}

		/// <summary>
		/// Get the current joint angle in radians.
		/// </summary>
		public float JointAngle
		{
			get
			{
				Body b1 = _body1;
				Body b2 = _body2;
				return b2._sweep.A - b1._sweep.A - _referenceAngle;
			}
		}


		/// <summary>
		/// Get the current joint angle speed in radians per second.
		/// </summary>
		public float JointSpeed
		{
			get
			{
				Body b1 = _body1;
				Body b2 = _body2;
				return b2._angularVelocity - b1._angularVelocity;
			}
		}

		/// <summary>
		/// Is the joint limit enabled?
		/// </summary>
		public bool IsLimitEnabled
		{
			get { return _enableLimit; }
		}

		/// <summary>
		/// Enable/disable the joint limit.
		/// </summary>
		/// <param name="flag"></param>
		public void EnableLimit(bool flag)
		{
			_enableLimit = flag;
		}

		/// <summary>
		/// Get the lower joint limit in radians.
		/// </summary>
		public float LowerLimit
		{
			get { return _lowerAngle; }
		}

		/// <summary>
		/// Get the upper joint limit in radians.
		/// </summary>
		public float UpperLimit
		{
			get { return _upperAngle; }
		}

		/// <summary>
		/// Set the joint limits in radians.
		/// </summary>
		/// <param name="lower"></param>
		/// <param name="upper"></param>
		public void SetLimits(float lower, float upper)
		{
			Box2DXDebug.Assert(lower <= upper);
			_lowerAngle = lower;
			_upperAngle = upper;
		}

		/// <summary>
		/// Is the joint motor enabled?
		/// </summary>
		public bool IsMotorEnabled
		{
			get { return _enableMotor; }
		}

		/// <summary>
		/// Enable/disable the joint motor.
		/// </summary>
		/// <param name="flag"></param>
		public void EnableMotor(bool flag)
		{
			_enableMotor = flag;
		}

		/// <summary>
		/// Get\Set the motor speed in radians per second.
		/// </summary>
		public float MotorSpeed
		{
			get { return _motorSpeed; }
			set { _motorSpeed = value; }
		}

		/// <summary>
		/// Set the maximum motor torque, usually in N-m.
		/// </summary>
		/// <param name="torque"></param>
		public void SetMaxMotorTorque(float torque)
		{
			_maxMotorTorque = torque;
		}

		/// Get the current motor torque, usually in N-m.
		public float MotorTorque
		{
			get { return _motorForce; }
		}

		public RevoluteJoint(RevoluteJointDef def)
			: base(def)
		{
			_localAnchor1 = def.LocalAnchor1;
			_localAnchor2 = def.LocalAnchor2;
			_referenceAngle = def.ReferenceAngle;

			_pivotForce.Set(0.0f, 0.0f);
			_motorForce = 0.0f;
			_limitForce = 0.0f;
			_limitPositionImpulse = 0.0f;

			_lowerAngle = def.LowerAngle;
			_upperAngle = def.UpperAngle;
			_maxMotorTorque = def.MaxMotorTorque;
			_motorSpeed = def.MotorSpeed;
			_enableLimit = def.EnableLimit;
			_enableMotor = def.EnableMotor;
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