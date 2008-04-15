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

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(ay1, d)
// Cdot = dot(d, cross(w1, ay1)) + dot(ay1, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(ay1, v1) - dot(cross(d + r1, ay1), w1) + dot(ay1, v2) + dot(cross(r2, ay1), v2)
// J = [-ay1 -cross(d+r1,ay1) ay1 cross(r2,ay1)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]

// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Dynamics
{
	using Box2DXMath = Box2DX.Common.Math;
	using SystemMath = System.Math;

	/// <summary>
	/// Prismatic joint definition. This requires defining a line of
	/// motion using an axis and an anchor point. The definition uses local
	/// anchor points and a local axis so that the initial configuration
	/// can violate the constraint slightly. The joint translation is zero
	/// when the local anchor points coincide in world space. Using local
	/// anchors and a local axis helps when saving and loading a game.
	/// </summary>
	public class PrismaticJointDef : JointDef
	{
		public PrismaticJointDef()
		{
			Type = JointType.PrismaticJoint;
			LocalAnchor1.SetZero();
			LocalAnchor2.SetZero();
			LocalAxis1.Set(1.0f, 0.0f);
			ReferenceAngle = 0.0f;
			EnableLimit = false;
			LowerTranslation = 0.0f;
			UpperTranslation = 0.0f;
			EnableMotor = false;
			MaxMotorForce = 0.0f;
			MotorSpeed = 0.0f;
		}

		/// <summary>
		/// Initialize the bodies, anchors, axis, and reference angle using the world
		/// anchor and world axis.
		/// </summary>
		/// <param name="body1"></param>
		/// <param name="body2"></param>
		/// <param name="anchor"></param>
		/// <param name="axis"></param>
		public void Initialize(Body body1, Body body2, Vector2 anchor, Vector2 axis)
		{
			Body1 = body1;
			Body2 = body2;
			LocalAnchor1 = body1.GetLocalPoint(anchor);
			LocalAnchor2 = body2.GetLocalPoint(anchor);
			LocalAxis1 = body1.GetLocalVector(axis);
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
		/// The local translation axis in body1.
		/// </summary>
		public Vector2 LocalAxis1;

		/// <summary>
		/// The constrained angle between the bodies: body2_angle - body1_angle.
		/// </summary>
		public float ReferenceAngle;

		/// <summary>
		/// Enable/disable the joint limit.
		/// </summary>
		public bool EnableLimit;

		/// <summary>
		/// The lower translation limit, usually in meters.
		/// </summary>
		public float LowerTranslation;

		/// <summary>
		/// The upper translation limit, usually in meters.
		/// </summary>
		public float UpperTranslation;

		/// <summary>
		/// Enable/disable the joint motor.
		/// </summary>
		public bool EnableMotor;

		/// <summary>
		/// The maximum motor torque, usually in N-m.
		/// </summary>
		public float MaxMotorForce;

		/// <summary>
		/// The desired motor speed in radians per second.
		/// </summary>
		public float MotorSpeed;
	}

	/// <summary>
	/// A prismatic joint. This joint provides one degree of freedom: translation
	/// along an axis fixed in body1. Relative rotation is prevented. You can
	/// use a joint limit to restrict the range of motion and a joint motor to
	/// drive the motion or to model joint friction.
	/// </summary>
	public class PrismaticJoint : Joint
	{
		public Vector2 _localAnchor1;
		public Vector2 _localAnchor2;
		public Vector2 _localXAxis1;
		public Vector2 _localYAxis1;
		public float _refAngle;

		public Jacobian _linearJacobian;
		public float _linearMass;				// effective mass for point-to-line constraint.
		public float _force;

		public float _angularMass;			// effective mass for angular constraint.
		public float _torque;

		public Jacobian _motorJacobian;
		public float _motorMass;			// effective mass for motor/limit translational constraint.
		public float _motorForce;
		public float _limitForce;
		public float _limitPositionImpulse;

		public float _lowerTranslation;
		public float _upperTranslation;
		public float _maxMotorForce;
		public float _motorSpeed;

		public bool _enableLimit;
		public bool _enableMotor;
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
			get
			{
				Vector2 ax1 = Common.Math.Mul(_body1._xf.R, _localXAxis1);
				Vector2 ay1 = Common.Math.Mul(_body1._xf.R, _localYAxis1);

				return Settings.FORCE_SCALE(1.0f) * (_limitForce * ax1 + _force * ay1);
			}
		}

		public override float ReactionTorque
		{
			get { return Settings.FORCE_SCALE(_torque); }
		}

		/// <summary>
		/// Get the current joint translation, usually in meters.
		/// </summary>
		public float JointTranslation
		{
			get
			{
				Body b1 = _body1;
				Body b2 = _body2;

				Vector2 p1 = b1.GetWorldPoint(_localAnchor1);
				Vector2 p2 = b2.GetWorldPoint(_localAnchor2);
				Vector2 d = p2 - p1;
				Vector2 axis = b1.GetWorldVector(_localXAxis1);

				float translation = Vector2.Dot(d, axis);
				return translation;
			}
		}

		/// <summary>
		/// Get the current joint translation speed, usually in meters per second.
		/// </summary>
		public float JointSpeed
		{
			get
			{
				Body b1 = _body1;
				Body b2 = _body2;

				Vector2 r1 = Common.Math.Mul(b1._xf.R, _localAnchor1 - b1.GetLocalCenter());
				Vector2 r2 = Common.Math.Mul(b2._xf.R, _localAnchor2 - b2.GetLocalCenter());
				Vector2 p1 = b1._sweep.C + r1;
				Vector2 p2 = b2._sweep.C + r2;
				Vector2 d = p2 - p1;
				Vector2 axis = b1.GetWorldVector(_localXAxis1);

				Vector2 v1 = b1._linearVelocity;
				Vector2 v2 = b2._linearVelocity;
				float w1 = b1._angularVelocity;
				float w2 = b2._angularVelocity;

				float speed = Vector2.Dot(d, Vector2.Cross(w1, axis)) + Vector2.Dot(axis, v2 + Vector2.Cross(w2, r2) - v1 - Vector2.Cross(w1, r1));
				return speed;
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
		/// Get the lower joint limit, usually in meters.
		/// </summary>
		public float LowerLimit
		{
			get { return _lowerTranslation; }
		}

		/// <summary>
		/// Get the upper joint limit, usually in meters.
		/// </summary>
		public float UpperLimit
		{
			get { return _upperTranslation; }
		}

		/// <summary>
		/// Set the joint limits, usually in meters.
		/// </summary>
		/// <param name="lower"></param>
		/// <param name="upper"></param>
		public void SetLimits(float lower, float upper)
		{
			Box2DXDebug.Assert(lower <= upper);
			_lowerTranslation = lower;
			_upperTranslation = upper;
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
		/// Get\Set the motor speed, usually in meters per second.
		/// </summary>
		public float MotorSpeed
		{
			get { return _motorSpeed; }
			set { _motorSpeed = value; }
		}

		/// <summary>
		/// Set the maximum motor force, usually in N.
		/// </summary>
		/// <param name="torque"></param>
		public void SetMaxMotorForce(float force)
		{
			_maxMotorForce = Settings.FORCE_SCALE(1.0f) * force;
		}

		/// <summary>
		/// Get the current motor force, usually in N.
		/// </summary>
		public float MotorForce
		{
			get { return _motorForce; }
		}

		public PrismaticJoint(PrismaticJointDef def)
			: base(def)
		{
			_localAnchor1 = def.LocalAnchor1;
			_localAnchor2 = def.LocalAnchor2;
			_localXAxis1 = def.LocalAxis1;
			_localYAxis1 = Vector2.Cross(1.0f, _localXAxis1);
			_refAngle = def.ReferenceAngle;

			_linearJacobian.SetZero();
			_linearMass = 0.0f;
			_force = 0.0f;

			_angularMass = 0.0f;
			_torque = 0.0f;

			_motorJacobian.SetZero();
			_motorMass = 0.0f;
			_motorForce = 0.0f;
			_limitForce = 0.0f;
			_limitPositionImpulse = 0.0f;

			_lowerTranslation = def.LowerTranslation;
			_upperTranslation = def.UpperTranslation;
			_maxMotorForce = Settings.FORCE_INV_SCALE(def.MaxMotorForce);
			_motorSpeed = def.MotorSpeed;
			_enableLimit = def.EnableLimit;
			_enableMotor = def.EnableMotor;
		}

		public override void InitVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			// Compute the effective masses.
			Vector2 r1 = Box2DXMath.Mul(b1._xf.R, _localAnchor1 - b1.GetLocalCenter());
			Vector2 r2 = Box2DXMath.Mul(b2._xf.R, _localAnchor2 - b2.GetLocalCenter());

			float invMass1 = b1._invMass, invMass2 = b2._invMass;
			float invI1 = b1._invI, invI2 = b2._invI;

			// Compute point to line constraint effective mass.
			// J = [-ay1 -cross(d+r1,ay1) ay1 cross(r2,ay1)]
			Vector2 ay1 = Box2DXMath.Mul(b1._xf.R, _localYAxis1);
			Vector2 e = b2._sweep.C + r2 - b1._sweep.C;	// e = d + r1

			_linearJacobian.Set(-ay1, -Vector2.Cross(e, ay1), ay1, Vector2.Cross(r2, ay1));
			_linearMass = invMass1 + invI1 * _linearJacobian.Angular1 * _linearJacobian.Angular1 +
							invMass2 + invI2 * _linearJacobian.Angular2 * _linearJacobian.Angular2;
			Box2DXDebug.Assert(_linearMass > Box2DXMath.FLOAT32_EPSILON);
			_linearMass = 1.0f / _linearMass;

			// Compute angular constraint effective mass.
			_angularMass = invI1 + invI2;
			if (_angularMass > Box2DXMath.FLOAT32_EPSILON)
			{
				_angularMass = 1.0f / _angularMass;
			}

			// Compute motor and limit terms.
			if (_enableLimit || _enableMotor)
			{
				// The motor and limit share a Jacobian and effective mass.
				Vector2 ax1 = Box2DXMath.Mul(b1._xf.R, _localXAxis1);
				_motorJacobian.Set(-ax1, -Vector2.Cross(e, ax1), ax1, Vector2.Cross(r2, ax1));
				_motorMass = invMass1 + invI1 * _motorJacobian.Angular1 * _motorJacobian.Angular1 +
								invMass2 + invI2 * _motorJacobian.Angular2 * _motorJacobian.Angular2;
				Box2DXDebug.Assert(_motorMass > Box2DXMath.FLOAT32_EPSILON);
				_motorMass = 1.0f / _motorMass;

				if (_enableLimit)
				{
					Vector2 d = e - r1;	// p2 - p1
					float jointTranslation = Vector2.Dot(ax1, d);
					if (Box2DXMath.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.LinearSlop)
					{
						_limitState = LimitState.EqualLimits;
					}
					else if (jointTranslation <= _lowerTranslation)
					{
						if (_limitState != LimitState.AtLowerLimit)
						{
							_limitForce = 0.0f;
						}
						_limitState = LimitState.AtLowerLimit;
					}
					else if (jointTranslation >= _upperTranslation)
					{
						if (_limitState != LimitState.AtUpperLimit)
						{
							_limitForce = 0.0f;
						}
						_limitState = LimitState.AtUpperLimit;
					}
					else
					{
						_limitState = LimitState.InactiveLimit;
						_limitForce = 0.0f;
					}
				}
			}

			if (_enableMotor == false)
			{
				_motorForce = 0.0f;
			}

			if (_enableLimit == false)
			{
				_limitForce = 0.0f;
			}

			if (World.s_enableWarmStarting != 0)
			{
				Vector2 P1 = Settings.FORCE_SCALE(step.Dt) * (_force * _linearJacobian.Linear1 + (_motorForce + _limitForce) * _motorJacobian.Linear1);
				Vector2 P2 = Settings.FORCE_SCALE(step.Dt) * (_force * _linearJacobian.Linear2 + (_motorForce + _limitForce) * _motorJacobian.Linear2);
				float L1 = Settings.FORCE_SCALE(step.Dt) * (_force * _linearJacobian.Angular1 - _torque + (_motorForce + _limitForce) * _motorJacobian.Angular1);
				float L2 = Settings.FORCE_SCALE(step.Dt) * (_force * _linearJacobian.Angular2 + _torque + (_motorForce + _limitForce) * _motorJacobian.Angular2);

				b1._linearVelocity += invMass1 * P1;
				b1._angularVelocity += invI1 * L1;

				b2._linearVelocity += invMass2 * P2;
				b2._angularVelocity += invI2 * L2;
			}
			else
			{
				_force = 0.0f;
				_torque = 0.0f;
				_limitForce = 0.0f;
				_motorForce = 0.0f;
			}

			_limitPositionImpulse = 0.0f;
		}

		public override void SolveVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			float invMass1 = b1._invMass, invMass2 = b2._invMass;
			float invI1 = b1._invI, invI2 = b2._invI;

			// Solve linear constraint.
			float linearCdot = _linearJacobian.Compute(b1._linearVelocity, b1._angularVelocity, b2._linearVelocity, b2._angularVelocity);
			float force = -Settings.FORCE_INV_SCALE(step.Inv_Dt) * _linearMass * linearCdot;
			_force += force;

			float P = Settings.FORCE_SCALE(step.Dt) * force;
			b1._linearVelocity += (invMass1 * P) * _linearJacobian.Linear1;
			b1._angularVelocity += invI1 * P * _linearJacobian.Angular1;

			b2._linearVelocity += (invMass2 * P) * _linearJacobian.Linear2;
			b2._angularVelocity += invI2 * P * _linearJacobian.Angular2;

			// Solve angular constraint.
			float angularCdot = b2._angularVelocity - b1._angularVelocity;
			float torque = -Settings.FORCE_INV_SCALE(step.Inv_Dt) * _angularMass * angularCdot;
			_torque += torque;

			float L = Settings.FORCE_SCALE(step.Dt) * torque;
			b1._angularVelocity -= invI1 * L;
			b2._angularVelocity += invI2 * L;

			// Solve linear motor constraint.
			if (_enableMotor && _limitState != LimitState.EqualLimits)
			{
				float motorCdot = _motorJacobian.Compute(b1._linearVelocity, b1._angularVelocity, b2._linearVelocity, b2._angularVelocity) - _motorSpeed;
				float motorForce = -Settings.FORCE_INV_SCALE(step.Inv_Dt) * _motorMass * motorCdot;
				float oldMotorForce = _motorForce;
				_motorForce = Box2DXMath.Clamp(_motorForce + motorForce, -_maxMotorForce, _maxMotorForce);
				motorForce = _motorForce - oldMotorForce;

				float P_ = Settings.FORCE_SCALE(step.Dt) * motorForce;
				b1._linearVelocity += (invMass1 * P_) * _motorJacobian.Linear1;
				b1._angularVelocity += invI1 * P_ * _motorJacobian.Angular1;

				b2._linearVelocity += (invMass2 * P_) * _motorJacobian.Linear2;
				b2._angularVelocity += invI2 * P_ * _motorJacobian.Angular2;
			}

			// Solve linear limit constraint.
			if (_enableLimit && _limitState != LimitState.InactiveLimit)
			{
				float limitCdot = _motorJacobian.Compute(b1._linearVelocity, b1._angularVelocity, b2._linearVelocity, b2._angularVelocity);
				float limitForce = -Settings.FORCE_INV_SCALE(step.Inv_Dt) * _motorMass * limitCdot;

				if (_limitState == LimitState.EqualLimits)
				{
					_limitForce += limitForce;
				}
				else if (_limitState == LimitState.AtLowerLimit)
				{
					float oldLimitForce = _limitForce;
					_limitForce = Box2DXMath.Max(_limitForce + limitForce, 0.0f);
					limitForce = _limitForce - oldLimitForce;
				}
				else if (_limitState == LimitState.AtUpperLimit)
				{
					float oldLimitForce = _limitForce;
					_limitForce = Box2DXMath.Min(_limitForce + limitForce, 0.0f);
					limitForce = _limitForce - oldLimitForce;
				}

				float P_ = Settings.FORCE_SCALE(step.Dt) * limitForce;

				b1._linearVelocity += (invMass1 * P_) * _motorJacobian.Linear1;
				b1._angularVelocity += invI1 * P_ * _motorJacobian.Angular1;

				b2._linearVelocity += (invMass2 * P_) * _motorJacobian.Linear2;
				b2._angularVelocity += invI2 * P_ * _motorJacobian.Angular2;
			}
		}

		public override bool SolvePositionConstraints()
		{
			Body b1 = _body1;
			Body b2 = _body2;

			float invMass1 = b1._invMass, invMass2 = b2._invMass;
			float invI1 = b1._invI, invI2 = b2._invI;

			Vector2 r1 = Box2DXMath.Mul(b1._xf.R, _localAnchor1 - b1.GetLocalCenter());
			Vector2 r2 = Box2DXMath.Mul(b2._xf.R, _localAnchor2 - b2.GetLocalCenter());
			Vector2 p1 = b1._sweep.C + r1;
			Vector2 p2 = b2._sweep.C + r2;
			Vector2 d = p2 - p1;
			Vector2 ay1 = Box2DXMath.Mul(b1._xf.R, _localYAxis1);

			// Solve linear (point-to-line) constraint.
			float linearC = Vector2.Dot(ay1, d);
			// Prevent overly large corrections.
			linearC = Box2DXMath.Clamp(linearC, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);
			float linearImpulse = -_linearMass * linearC;

			b1._sweep.C += (invMass1 * linearImpulse) * _linearJacobian.Linear1;
			b1._sweep.A += invI1 * linearImpulse * _linearJacobian.Angular1;
			//b1->SynchronizeTransform(); // updated by angular constraint
			b2._sweep.C += (invMass2 * linearImpulse) * _linearJacobian.Linear2;
			b2._sweep.A += invI2 * linearImpulse * _linearJacobian.Angular2;
			//b2->SynchronizeTransform(); // updated by angular constraint

			float positionError = Box2DXMath.Abs(linearC);

			// Solve angular constraint.
			float angularC = b2._sweep.A - b1._sweep.A - _refAngle;
			// Prevent overly large corrections.
			angularC = Box2DXMath.Clamp(angularC, -Settings.MaxAngularCorrection, Settings.MaxAngularCorrection);
			float angularImpulse = -_angularMass * angularC;

			b1._sweep.A -= b1._invI * angularImpulse;
			b2._sweep.A += b2._invI * angularImpulse;

			b1.SynchronizeTransform();
			b2.SynchronizeTransform();

			float angularError = Box2DXMath.Abs(angularC);

			// Solve linear limit constraint.
			if (_enableLimit && _limitState != LimitState.InactiveLimit)
			{
				Vector2 r1_ = Box2DXMath.Mul(b1._xf.R, _localAnchor1 - b1.GetLocalCenter());
				Vector2 r2_ = Box2DXMath.Mul(b2._xf.R, _localAnchor2 - b2.GetLocalCenter());
				Vector2 p1_ = b1._sweep.C + r1_;
				Vector2 p2_ = b2._sweep.C + r2_;
				Vector2 d_ = p2_ - p1_;
				Vector2 ax1 = Box2DXMath.Mul(b1._xf.R, _localXAxis1);

				float translation = Vector2.Dot(ax1, d_);
				float limitImpulse = 0.0f;

				if (_limitState == LimitState.EqualLimits)
				{
					// Prevent large angular corrections
					float limitC = Box2DXMath.Clamp(translation, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);
					limitImpulse = -_motorMass * limitC;
					positionError = Box2DXMath.Max(positionError, Box2DXMath.Abs(angularC));
				}
				else if (_limitState == LimitState.AtLowerLimit)
				{
					float limitC = translation - _lowerTranslation;
					positionError = Box2DXMath.Max(positionError, -limitC);

					// Prevent large linear corrections and allow some slop.
					limitC = Box2DXMath.Clamp(limitC + Settings.LinearSlop, -Settings.MaxLinearCorrection, 0.0f);
					limitImpulse = -_motorMass * limitC;
					float oldLimitImpulse = _limitPositionImpulse;
					_limitPositionImpulse = Box2DXMath.Max(_limitPositionImpulse + limitImpulse, 0.0f);
					limitImpulse = _limitPositionImpulse - oldLimitImpulse;
				}
				else if (_limitState == LimitState.AtUpperLimit)
				{
					float limitC = translation - _upperTranslation;
					positionError = Box2DXMath.Max(positionError, limitC);

					// Prevent large linear corrections and allow some slop.
					limitC = Box2DXMath.Clamp(limitC - Settings.LinearSlop, 0.0f, Settings.MaxLinearCorrection);
					limitImpulse = -_motorMass * limitC;
					float oldLimitImpulse = _limitPositionImpulse;
					_limitPositionImpulse = Box2DXMath.Min(_limitPositionImpulse + limitImpulse, 0.0f);
					limitImpulse = _limitPositionImpulse - oldLimitImpulse;
				}

				b1._sweep.C += (invMass1 * limitImpulse) * _motorJacobian.Linear1;
				b1._sweep.A += invI1 * limitImpulse * _motorJacobian.Angular1;
				b2._sweep.C += (invMass2 * limitImpulse) * _motorJacobian.Linear2;
				b2._sweep.A += invI2 * limitImpulse * _motorJacobian.Angular2;

				b1.SynchronizeTransform();
				b2.SynchronizeTransform();
			}

			return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
		}
	}
}