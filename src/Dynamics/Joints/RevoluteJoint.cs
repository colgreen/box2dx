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
	using Box2DXMath = Box2DX.Common.Math;
	using SystemMath = System.Math;

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
		public void Initialize(Body body1, Body body2, Vec2 anchor)
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
		public Vec2 LocalAnchor1;

		/// <summary>
		/// The local anchor point relative to body2's origin.
		/// </summary>
		public Vec2 LocalAnchor2;

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
		public Vec2 _localAnchor1;	// relative
		public Vec2 _localAnchor2;
		public Vec2 _pivotForce;
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

#if B2_TOI_JOINTS
		publiv Vector2 _lastWarmStartingPivotForce;
#endif

		public override Vec2 Anchor1
		{
			get { return _body1.GetWorldPoint(_localAnchor1); }
		}

		public override Vec2 Anchor2
		{
			get { return _body2.GetWorldPoint(_localAnchor2); }
		}

		public override Vec2 GetReactionForce(float inv_dt)
		{
			return Settings.FORCE_SCALE(1.0f) * _pivotForce;
		}

		public override float GetReactionTorque(float inv_dt)
		{
			return _limitForce;
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

		internal override void InitVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			// Compute the effective mass matrix.
			Vec2 r1 = Box2DXMath.Mul(b1.GetXForm().R, _localAnchor1 - b1.GetLocalCenter());
			Vec2 r2 = Box2DXMath.Mul(b2.GetXForm().R, _localAnchor2 - b2.GetLocalCenter());

			// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
			//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
			//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
			float invMass1 = b1._invMass, invMass2 = b2._invMass;
			float invI1 = b1._invI, invI2 = b2._invI;

			Mat22 K1 = new Mat22();
			K1.Col1.X = invMass1 + invMass2; K1.Col2.X = 0.0f;
			K1.Col1.Y = 0.0f; K1.Col2.Y = invMass1 + invMass2;

			Mat22 K2 = new Mat22();
			K2.Col1.X = invI1 * r1.Y * r1.Y; K2.Col2.X = -invI1 * r1.X * r1.Y;
			K2.Col1.Y = -invI1 * r1.X * r1.Y; K2.Col2.Y = invI1 * r1.X * r1.X;

			Mat22 K3 = new Mat22();
			K3.Col1.X = invI2 * r2.Y * r2.Y; K3.Col2.X = -invI2 * r2.X * r2.Y;
			K3.Col1.Y = -invI2 * r2.X * r2.Y; K3.Col2.Y = invI2 * r2.X * r2.X;

			Mat22 K = K1 + K2 + K3;
			_pivotMass = K.Invert();

			_motorMass = 1.0f / (invI1 + invI2);

			if (_enableMotor == false)
			{
				_motorForce = 0.0f;
			}

			if (_enableLimit)
			{
				float jointAngle = b2._sweep.A - b1._sweep.A - _referenceAngle;
				if (Box2DXMath.Abs(_upperAngle - _lowerAngle) < 2.0f * Settings.AngularSlop)
				{
					_limitState = LimitState.EqualLimits;
				}
				else if (jointAngle <= _lowerAngle)
				{
					if (_limitState != LimitState.AtLowerLimit)
					{
						_limitForce = 0.0f;
					}
					_limitState = LimitState.AtLowerLimit;
				}
				else if (jointAngle >= _upperAngle)
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
			else
			{
				_limitForce = 0.0f;
			}

			if (step.WarmStarting)
			{
				b1._linearVelocity -= Settings.FORCE_SCALE(step.Dt) * invMass1 * _pivotForce;
				b1._angularVelocity -= Settings.FORCE_SCALE(step.Dt) * invI1 * (Vec2.Cross(r1, _pivotForce) + Settings.FORCE_INV_SCALE(_motorForce + _limitForce));

				b2._linearVelocity += Settings.FORCE_SCALE(step.Dt) * invMass2 * _pivotForce;
				b2._angularVelocity += Settings.FORCE_SCALE(step.Dt) * invI2 * (Vec2.Cross(r2, _pivotForce) + Settings.FORCE_INV_SCALE(_motorForce + _limitForce));
			}
			else
			{
				_pivotForce.SetZero();
				_motorForce = 0.0f;
				_limitForce = 0.0f;
			}

			_limitPositionImpulse = 0.0f;
		}

		internal override void SolveVelocityConstraints(TimeStep step)
		{
			Body b1 = _body1;
			Body b2 = _body2;

			Vec2 r1 = Box2DXMath.Mul(b1.GetXForm().R, _localAnchor1 - b1.GetLocalCenter());
			Vec2 r2 = Box2DXMath.Mul(b2.GetXForm().R, _localAnchor2 - b2.GetLocalCenter());

			// Solve point-to-point constraint
			Vec2 pivotCdot = b2._linearVelocity + Vec2.Cross(b2._angularVelocity, r2) - b1._linearVelocity -
				Vec2.Cross(b1._angularVelocity, r1);
			Vec2 pivotForce = -Settings.FORCE_INV_SCALE(step.Inv_Dt) * Box2DXMath.Mul(_pivotMass, pivotCdot);

#if B2_TOI_JOINTS
			if (step.WarmStarting)
			{
					_pivotForce += pivotForce;
					_lastWarmStartingPivotForce = _pivotForce;
			}
			else
			{
				_pivotForce = _lastWarmStartingPivotForce;
				//Do not update warm starting value!
			}
#else
			_pivotForce += pivotForce;
#endif

			Vec2 P = Settings.FORCE_SCALE(step.Dt) * pivotForce;
			b1._linearVelocity -= b1._invMass * P;
			b1._angularVelocity -= b1._invI * Vec2.Cross(r1, P);

			b2._linearVelocity += b2._invMass * P;
			b2._angularVelocity += b2._invI * Vec2.Cross(r2, P);

			if (_enableMotor && _limitState != LimitState.EqualLimits)
			{
				float motorCdot = b2._angularVelocity - b1._angularVelocity - _motorSpeed;
				float motorForce = -step.Inv_Dt * _motorMass * motorCdot;
				float oldMotorForce = _motorForce;
				_motorForce = Box2DXMath.Clamp(_motorForce + motorForce, -_maxMotorTorque, _maxMotorTorque);
				motorForce = _motorForce - oldMotorForce;

				float P_ = step.Dt * motorForce;
				b1._angularVelocity -= b1._invI * P_;
				b2._angularVelocity += b2._invI * P_;
			}

			if (_enableLimit && _limitState != LimitState.InactiveLimit)
			{
				float limitCdot = b2._angularVelocity - b1._angularVelocity;
				float limitForce = -step.Inv_Dt * _motorMass * limitCdot;

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

				float P_ = step.Dt * limitForce;
				b1._angularVelocity -= b1._invI * P_;
				b2._angularVelocity += b2._invI * P_;
			}
		}

		internal override bool SolvePositionConstraints()
		{
			Body b1 = _body1;
			Body b2 = _body2;

			float positionError = 0.0f;

			// Solve point-to-point position error.
			Vec2 r1 = Box2DXMath.Mul(b1.GetXForm().R, _localAnchor1 - b1.GetLocalCenter());
			Vec2 r2 = Box2DXMath.Mul(b2.GetXForm().R, _localAnchor2 - b2.GetLocalCenter());

			Vec2 p1 = b1._sweep.C + r1;
			Vec2 p2 = b2._sweep.C + r2;
			Vec2 ptpC = p2 - p1;

			positionError = ptpC.Length();

			// Prevent overly large corrections.
			//b2Vec2 dpMax(b2_maxLinearCorrection, b2_maxLinearCorrection);
			//ptpC = b2Clamp(ptpC, -dpMax, dpMax);

			float invMass1 = b1._invMass, invMass2 = b2._invMass;
			float invI1 = b1._invI, invI2 = b2._invI;

			Mat22 K1 = new Mat22();
			K1.Col1.X = invMass1 + invMass2; K1.Col2.X = 0.0f;
			K1.Col1.Y = 0.0f; K1.Col2.Y = invMass1 + invMass2;

			Mat22 K2 = new Mat22();
			K2.Col1.X = invI1 * r1.Y * r1.Y; K2.Col2.X = -invI1 * r1.X * r1.Y;
			K2.Col1.Y = -invI1 * r1.X * r1.Y; K2.Col2.Y = invI1 * r1.X * r1.X;

			Mat22 K3 = new Mat22();
			K3.Col1.X = invI2 * r2.Y * r2.Y; K3.Col2.X = -invI2 * r2.X * r2.Y;
			K3.Col1.Y = -invI2 * r2.X * r2.Y; K3.Col2.Y = invI2 * r2.X * r2.X;

			Mat22 K = K1 + K2 + K3;
			Vec2 impulse = K.Solve(-ptpC);

			b1._sweep.C -= b1._invMass * impulse;
			b1._sweep.A -= b1._invI * Vec2.Cross(r1, impulse);

			b2._sweep.C += b2._invMass * impulse;
			b2._sweep.A += b2._invI * Vec2.Cross(r2, impulse);

			b1.SynchronizeTransform();
			b2.SynchronizeTransform();

			// Handle limits.
			float angularError = 0.0f;

			if (_enableLimit && _limitState != LimitState.InactiveLimit)
			{
				float angle = b2._sweep.A - b1._sweep.A - _referenceAngle;
				float limitImpulse = 0.0f;

				if (_limitState == LimitState.EqualLimits)
				{
					// Prevent large angular corrections
					float limitC = Box2DXMath.Clamp(angle, -Settings.MaxAngularCorrection, Settings.MaxAngularCorrection);
					limitImpulse = -_motorMass * limitC;
					angularError = Box2DXMath.Abs(limitC);
				}
				else if (_limitState == LimitState.AtLowerLimit)
				{
					float limitC = angle - _lowerAngle;
					angularError = Box2DXMath.Max(0.0f, -limitC);

					// Prevent large angular corrections and allow some slop.
					limitC = Box2DXMath.Clamp(limitC + Settings.AngularSlop, -Settings.MaxAngularCorrection, 0.0f);
					limitImpulse = -_motorMass * limitC;
					float oldLimitImpulse = _limitPositionImpulse;
					_limitPositionImpulse = Box2DXMath.Max(_limitPositionImpulse + limitImpulse, 0.0f);
					limitImpulse = _limitPositionImpulse - oldLimitImpulse;
				}
				else if (_limitState == LimitState.AtUpperLimit)
				{
					float limitC = angle - _upperAngle;
					angularError = Box2DXMath.Max(0.0f, limitC);

					// Prevent large angular corrections and allow some slop.
					limitC = Box2DXMath.Clamp(limitC - Settings.AngularSlop, 0.0f, Settings.MaxAngularCorrection);
					limitImpulse = -_motorMass * limitC;
					float oldLimitImpulse = _limitPositionImpulse;
					_limitPositionImpulse = Box2DXMath.Min(_limitPositionImpulse + limitImpulse, 0.0f);
					limitImpulse = _limitPositionImpulse - oldLimitImpulse;
				}

				b1._sweep.A -= b1._invI * limitImpulse;
				b2._sweep.A += b2._invI * limitImpulse;

				b1.SynchronizeTransform();
				b2.SynchronizeTransform();
			}

			return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
		}
	}
}
