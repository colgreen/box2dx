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

namespace Box2DX.Dynamics.Contacts
{
	public class PolyAndCircleContact : Contact
	{
		public Manifold _manifold = new Manifold();

		public override Manifold GetManifolds()
		{
			return _manifold;
		}

		public PolyAndCircleContact(Shape s1, Shape s2)
			: base(s1, s2)
		{
			Box2DXDebug.Assert(_shape1._type == ShapeType.PolygonShape);
			Box2DXDebug.Assert(_shape2._type == ShapeType.CircleShape);
			_manifold.PointCount = 0;
			_manifold.Points[0].NormalForce = 0.0f;
			_manifold.Points[0].TangentForce = 0.0f;
		}

		public override void Evaluate(ContactListener listener)
		{
			Body b1 = _shape1.GetBody();
			Body b2 = _shape2.GetBody();

			//memcpy(&m0, &m_manifold, sizeof(b2Manifold));
			Manifold m0 = new Manifold();
			m0.Normal = _manifold.Normal;
			m0.PointCount = _manifold.PointCount;
			m0.Points = _manifold.Points;

			Collision.Collision.CollidePolygonAndCircle(ref _manifold, (PolygonShape)_shape1, b1._xf, (CircleShape)_shape2, b2._xf);

			if (_manifold.PointCount > 0)
			{
				_manifoldCount = 1;
				if (m0.PointCount == 0)
				{
					_manifold.Points[0].ID.Features.Flip |= Collision.Collision.NewPoint;
				}
				else
				{
					_manifold.Points[0].ID.Features.Flip &= ~Collision.Collision.NewPoint;
				}
			}
			else
			{
				_manifoldCount = 0;
				if (m0.PointCount > 0 && listener != null)
				{
					ContactPoint cp = new ContactPoint();
					cp.Shape1 = _shape1;
					cp.Shape2 = _shape2;
					cp.Normal = m0.Normal;
					cp.Position = Common.Math.Mul(b1._xf, m0.Points[0].LocalPoint1);
					cp.Separation = m0.Points[0].Separation;
					cp.NormalForce = m0.Points[0].NormalForce;
					cp.TangentForce = m0.Points[0].TangentForce;
					cp.ID = m0.Points[0].ID;
					listener.Remove(cp);
				}
			}
		}

		public static Contact Create(Shape shape1, Shape shape2)
		{
			return new PolyAndCircleContact(shape1, shape2);
		}

		public static void Destroy(Contact contact)
		{
			Box2DXDebug.Assert(contact._shape1._body._contactList != contact._node1);
			Box2DXDebug.Assert(contact._shape1._body._contactList != contact._node2);
			Box2DXDebug.Assert(contact._shape2._body._contactList != contact._node1);
			Box2DXDebug.Assert(contact._shape2._body._contactList != contact._node2);

			if (contact is IDisposable)
				(contact as IDisposable).Dispose();
			contact = null;
		}
	}
}
