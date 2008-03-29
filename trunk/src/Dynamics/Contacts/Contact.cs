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
	public delegate Contact ContactCreateFcn(Shape shape1, Shape shape2);
	public delegate void ContactDestroyFcn(Contact contact);

	public struct ContactRegister
	{
		public ContactCreateFcn CreateFcn;
		public ContactDestroyFcn DestroyFcn;
		public bool Primary;
	}

#warning "CAS"
	/// <summary>
	/// A contact edge is used to connect bodies and contacts together
	/// in a contact graph where each body is a node and each contact
	/// is an edge. A contact edge belongs to a doubly linked list
	/// maintained in each attached body. Each contact has two contact
	/// nodes, one for each attached body.
	/// </summary>
	public class ContactEdge
	{
		/// <summary>
		/// Provides quick access to the other body attached.
		/// </summary>
		public Body Other;
		/// <summary>
		/// The contact.
		/// </summary>
		public Contact Contact;
		/// <summary>
		/// The previous contact edge in the body's contact list.
		/// </summary>
		public ContactEdge Prev;
		/// <summary>
		/// The next contact edge in the body's contact list.
		/// </summary>
		public ContactEdge Next;
	}

#warning "CAS"
	/// <summary>
	/// This structure is used to report contact points.
	/// </summary>
	public class ContactPoint
	{
		/// <summary>
		/// The first shape.
		/// </summary>
		public Shape Shape1;
		/// <summary>
		/// The second shape.
		/// </summary>
		public Shape Shape2;
		/// <summary>
		/// Position in world coordinates.
		/// </summary>
		public Vector2 Position;
		/// <summary>
		/// Points from shape1 to shape2.
		/// </summary>
		public Vector2 Normal;
		/// <summary>
		/// The separation is negative when shapes are touching.
		/// </summary>
		public float Separation;
		/// <summary>
		/// The signed magnitude of the normal force.
		/// </summary>
		public float NormalForce;
		/// <summary>
		/// The signed magnitude of the tangent force.
		/// </summary>
		public float TangentForce;
		/// <summary>
		/// The contact id identifies the features in contact.
		/// </summary>
		public ContactID ID;
	}

	/// <summary>
	/// The class manages contact between two shapes. A contact exists for each overlapping
	/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
	/// that has no contact points.
	/// </summary>
	public abstract class Contact
	{
		[Flags]
		public enum CollisionFlags
		{
			NonSolid = 0x0001,
			Slow = 0x0002,
			Island = 0x0004,
			Toi = 0x0008
		}

		public static ContactRegister[,] _registers =
			new ContactRegister[(int)ShapeType.ShapeTypeCount,(int)ShapeType.ShapeTypeCount];
		public static bool _initialized = false;

		public CollisionFlags _flags;
		public int _manifoldCount;

		// World pool and list pointers.
		public Contact _prev;
		public Contact _next;

		// Nodes for connecting bodies.
		public ContactEdge _node1;
		public ContactEdge _node2;

		public Shape _shape1;
		public Shape _shape2;

		// Combined friction
		public float _friction;
		public float _restitution;

		public float _toi;

		public Contact()
		{
			_shape1 = null; _shape2 = null;
		}

		public Contact(Shape s1, Shape s2)
		{
			_flags = 0;

			if (s1.IsSensor || s2.IsSensor)
			{
				_flags |= CollisionFlags.NonSolid;
			}

			_shape1 = s1;
			_shape2 = s2;

			_manifoldCount = 0;

			_friction = (float)System.Math.Sqrt(_shape1._friction * _shape2._friction);
			_restitution = Common.Math.Max(_shape1._restitution, _shape2._restitution);
			_prev = null;
			_next = null;

			_node1.Contact = null;
			_node1.Prev = null;
			_node1.Next = null;
			_node1.Other = null;

			_node2.Contact = null;
			_node2.Prev = null;
			_node2.Next = null;
			_node2.Other = null;
		}

		public static void AddType(ContactCreateFcn createFcn, ContactDestroyFcn destoryFcn,
					  ShapeType type1, ShapeType type2)
		{
			Box2DXDebug.Assert(ShapeType.UnknownShape < type1 && type1 < ShapeType.ShapeTypeCount);
			Box2DXDebug.Assert(ShapeType.UnknownShape < type2 && type2 < ShapeType.ShapeTypeCount);

			_registers[(int)type1,(int)type2].CreateFcn = createFcn;
			_registers[(int)type1,(int)type2].DestroyFcn = destoryFcn;
			_registers[(int)type1,(int)type2].Primary = true;

			if (type1 != type2)
			{
				_registers[(int)type2,(int)type1].CreateFcn = createFcn;
				_registers[(int)type2,(int)type1].DestroyFcn = destoryFcn;
				_registers[(int)type2,(int)type1].Primary = false;
			}
		}

		public static void InitializeRegisters()
		{
			AddType(CircleContact.Create, CircleContact.Destroy, ShapeType.CircleShape, ShapeType.CircleShape);
			AddType(PolyAndCircleContact.Create, PolyAndCircleContact.Destroy, ShapeType.PolygonShape, ShapeType.CircleShape);
			AddType(PolygonContact.Create, PolygonContact.Destroy, ShapeType.PolygonShape, ShapeType.PolygonShape);
		}

		public static Contact Create(Shape shape1, Shape shape2)
		{
			if (_initialized == false)
			{
				InitializeRegisters();
				_initialized = true;
			}

			ShapeType type1 = shape1._type;
			ShapeType type2 = shape2._type;

			Box2DXDebug.Assert(ShapeType.UnknownShape < type1 && type1 < ShapeType.ShapeTypeCount);
			Box2DXDebug.Assert(ShapeType.UnknownShape < type2 && type2 < ShapeType.ShapeTypeCount);

			ContactCreateFcn createFcn = _registers[(int)type1,(int)type2].CreateFcn;
			if (createFcn != null)
			{
				if (_registers[(int)type1,(int)type2].Primary)
				{
					return createFcn(shape1, shape2);
				}
				else
				{
					Contact c = createFcn(shape2, shape1);
#warning "manifold array"
					//for (int i = 0; i < c.GetManifoldCount(); ++i)
					if(c.GetManifoldCount()>0)
					{
						Manifold m = c.GetManifolds();
						m.Normal = -m.Normal;
					}
					return c;
				}
			}
			else
			{
				return null;
			}
		}

		public static void Destroy(Contact contact)
		{
			Box2DXDebug.Assert(_initialized == true);

			if (contact.GetManifoldCount() > 0)
			{
				contact.GetShape1().GetBody().WakeUp();
				contact.GetShape2().GetBody().WakeUp();
			}

			ShapeType type1 = contact.GetShape1().GetType();
			ShapeType type2 = contact.GetShape2().GetType();

			Box2DXDebug.Assert(ShapeType.UnknownShape < type1 && type1 < ShapeType.ShapeTypeCount);
			Box2DXDebug.Assert(ShapeType.UnknownShape < type2 && type2 < ShapeType.ShapeTypeCount);

			ContactDestroyFcn destroyFcn = _registers[(int)type1,(int)type2].DestroyFcn;
			destroyFcn(contact);
		}

		/// <summary>
		/// Get the manifold array.
		/// </summary>
		/// <returns></returns>
		public abstract Manifold GetManifolds();

		/// <summary>
		/// Get the number of manifolds. This is 0 or 1 between convex shapes.
		/// This may be greater than 1 for convex-vs-concave shapes. Each
		/// manifold holds up to two contact points with a shared contact normal.
		/// </summary>
		/// <returns></returns>
		public int GetManifoldCount()
		{
			return _manifoldCount;
		}

		/// <summary>
		/// Is this contact solid?
		/// </summary>
		/// <returns>True if this contact should generate a response.</returns>
		public bool IsSolid()
		{
			return (_flags & CollisionFlags.NonSolid) == 0;
		}

		/// <summary>
		/// Get the next contact in the world's contact list.
		/// </summary>
		/// <returns></returns>
		public Contact GetNext()
		{
			return _next;
		}

		/// <summary>
		/// Get the first shape in this contact.
		/// </summary>
		/// <returns></returns>
		public Shape GetShape1()
		{
			return _shape1;
		}

		/// <summary>
		/// Get the second shape in this contact.
		/// </summary>
		/// <returns></returns>
		public Shape GetShape2()
		{
			return _shape1;
		}

		public void Update(ContactListener listener)
		{
			int oldCount = GetManifoldCount();

			Evaluate(listener);

			int newCount = GetManifoldCount();

			Body body1 = _shape1.GetBody();
			Body body2 = _shape2.GetBody();

			if (newCount == 0 && oldCount > 0)
			{
				body1.WakeUp();
				body2.WakeUp();
			}

			// Slow contacts don't generate TOI events.
			if (body1.IsStatic() || body1.IsBullet() || body2.IsStatic() || body2.IsBullet())
			{
				_flags &= ~CollisionFlags.Slow;
			}
			else
			{
				_flags |= CollisionFlags.Slow;
			}
		}

		public abstract void Evaluate(ContactListener listener);
	}
}
