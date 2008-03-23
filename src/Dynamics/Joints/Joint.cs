using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Dynamics
{
	public enum JointType
	{
		UnknownJoint,
		RevoluteJoint,
		PrismaticJoint,
		DistanceJoint,
		PulleyJoint,
		MouseJoint,
		GearJoint
	}

	public enum LimitState
	{
		InactiveLimit,
		AtLowerLimit,
		AtUpperLimit,
		EqualLimits
	}

	public struct Jacobian
	{
		public Vector2 Linear1;
		public float Angular1;
		public Vector2 Linear2;
		public float Angular2;

		public void SetZero()
		{
			Linear1.SetZero(); Angular1 = 0.0f;
			Linear2.SetZero(); Angular2 = 0.0f;
		}

		public void Set(Vector2 x1, float a1, Vector2 x2, float a2)
		{
			Linear1 = x1; Angular1 = a1;
			Linear2 = x2; Angular2 = a2;
		}

		public float Compute(Vector2 x1, float a1, Vector2 x2, float a2)
		{
			return Vector2.Dot(Linear1, x1) + Angular1 * a1 + Vector2.Dot(Linear2, x2) + Angular2 * a2;
		}
	}

#warning "CAS"
	/// <summary>
	/// A joint edge is used to connect bodies and joints together
	/// in a joint graph where each body is a node and each joint
	/// is an edge. A joint edge belongs to a doubly linked list
	/// maintained in each attached body. Each joint has two joint
	/// nodes, one for each attached body.
	/// </summary>
	public class JointEdge
	{
		/// <summary>
		/// Provides quick access to the other body attached.
		/// </summary>
		public Body Other;

		/// <summary>
		/// The joint.
		/// </summary>
		public Joint Joint;

		/// <summary>
		/// The previous joint edge in the body's joint list.
		/// </summary>
		public JointEdge Prev;

		/// <summary>
		/// The next joint edge in the body's joint list.
		/// </summary>
		public JointEdge Next;
	}

#warning "CAS"
	/// <summary>
	/// Joint definitions are used to construct joints.
	/// </summary>
	public class JointDef
	{
		public JointDef()
		{
			Type = JointType.UnknownJoint;
			UserData = null;
			Body1 = null;
			Body2 = null;
			CollideConnected = false;
		}

		/// <summary>
		/// The joint type is set automatically for concrete joint types.
		/// </summary>
		public JointType Type;

		/// <summary>
		/// Use this to attach application specific data to your joints.
		/// </summary>
		public object UserData;

		/// <summary>
		/// The first attached body.
		/// </summary>
		public Body Body1;

		/// <summary>
		/// The second attached body.
		/// </summary>
		public Body Body2;

		/// <summary>
		/// Set this flag to true if the attached bodies should collide.
		/// </summary>
		public bool CollideConnected;
	}

	/// <summary>
	/// The base joint class. Joints are used to constraint two bodies together in
	/// various fashions. Some joints also feature limits and motors.
	/// </summary>
	public class Joint
	{
		public JointType _type;
		public Joint _prev;
		public Joint _next;
		public JointEdge _node1;
		public JointEdge _node2;
		public Body _body1;
		public Body _body2;

		public bool _islandFlag;
		public bool _collideConnected;

		public object _userData;

		/// <summary>
		/// Get the type of the concrete joint.
		/// </summary>
		/// <returns></returns>
		public JointType GetType()
		{
			return _type;
		}

		/// <summary>
		/// Get the first body attached to this joint.
		/// </summary>
		/// <returns></returns>
		public Body GetBody1()
		{
			return _body1;
		}

		/// <summary>
		/// Get the second body attached to this joint.
		/// </summary>
		/// <returns></returns>
		public Body GetBody2()
		{
			return _body2;
		}

		/// <summary>
		/// Get the anchor point on body1 in world coordinates.
		/// </summary>
		/// <returns></returns>
		public abstract Vector2 GetAnchor1();

		/// <summary>
		/// Get the anchor point on body2 in world coordinates.
		/// </summary>
		/// <returns></returns>
		public abstract Vector2 GetAnchor2();

		/// <summary>
		/// Get the reaction force on body2 at the joint anchor.
		/// </summary>
		/// <returns></returns>
		public abstract Vector2 GetReactionForce();

		/// <summary>
		/// Get the reaction torque on body2.
		/// </summary>
		/// <returns></returns>
		public abstract float GetReactionTorque();

		/// <summary>
		/// Get the next joint the world joint list.
		/// </summary>
		/// <returns></returns>
		public Joint GetNext()
		{
			return _next;
		}

		/// <summary>
		/// Get the user data pointer.
		/// </summary>
		/// <returns></returns>
		public object GetUserData()
		{
			return _userData;
		}

		public Joint(JointDef def)
		{
			_type = def.Type;
			_prev = null;
			_next = null;
			_body1 = def.Body1;
			_body2 = def.Body2;
			_collideConnected = def.CollideConnected;
			_islandFlag = false;
			_userData = def.UserData;
		}

		public static Joint Create(JointDef def)
		{
			Joint joint = null;

			switch (def.Type)
			{
				case JointType.DistanceJoint:
					{
						joint = new DistanceJoint((DistanceJointDef)def);
					}
					break;

				case JointType.MouseJoint:
					{
						joint = new MouseJoint((MouseJointDef)def);
					}
					break;

				case JointType.PrismaticJoint:
					{
						joint = new PrismaticJoint((PrismaticJointDef)def);
					}
					break;

				case JointType.RevoluteJoint:
					{
						joint = new RevoluteJoint((RevoluteJointDef)def);
					}
					break;

				case JointType.PulleyJoint:
					{
						joint = new PulleyJoint((PulleyJointDef)def);
					}
					break;

				case JointType.GearJoint:
					{
						joint = new GearJoint((GearJointDef)def);
					}
					break;

				default:
					Box2DXDebug.Assert(false);
					break;
			}

			return joint;
		}

		public static void Destroy(Joint joint)
		{
			if (joint is IDisposable)
				(Joint as IDisposable).Dispose();
			joint = null;
		}

		public abstract void InitVelocityConstraints(TimeStep step);
		public abstract void SolveVelocityConstraints(TimeStep step);

		// This returns true if the position errors are within tolerance.
		public virtual void InitPositionConstraints() { }
		public abstract bool SolvePositionConstraints();
	}
}
