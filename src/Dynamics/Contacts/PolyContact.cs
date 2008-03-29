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
	public class PolygonContact : Contact
	{
		public Manifold _manifold = new Manifold();

		public override Manifold GetManifolds()
		{
			return _manifold;
		}

		public PolygonContact(Shape s1, Shape s2)
			: base(s1, s2)
		{
			Box2DXDebug.Assert(_shape1._type == ShapeType.PolygonShape);
			Box2DXDebug.Assert(_shape2._type == ShapeType.PolygonShape);
			_manifold.PointCount = 0;
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

			Collision.Collision.CollidePolygons(ref _manifold, (PolygonShape)_shape1, b1._xf, (PolygonShape)_shape2, b2._xf);

			bool[] match = new bool[] { false, false };

			// Match contact ids to facilitate warm starting.
			if (_manifold.PointCount > 0)
			{
				// Match old contact ids to new contact ids and copy the
				// stored impulses to warm start the solver.
				for (int i = 0; i < _manifold.PointCount; ++i)
				{
					ManifoldPoint cp = _manifold.Points[i];
					cp.NormalForce = 0.0f;
					cp.TangentForce = 0.0f;
					bool matched = false;
					ContactID id = cp.ID;

					for (int j = 0; j < m0.PointCount; ++j)
					{
						if (match[j] == true)
							continue;

						ManifoldPoint cp0 = m0.Points[j];
						ContactID id0 = cp0.ID;
						cp0.ID.Features.Flip &= (byte)~Collision.Collision.NewPoint;

						if (id0.Key == id.Key)
						{
							match[j] = true;
							cp.NormalForce = cp0.NormalForce;
							cp.TangentForce = cp0.TangentForce;

							// Not a new point.
							matched = true;
							break;
						}
					}

					if (matched == false)
					{
						cp.ID.Features.Flip |= Collision.Collision.NewPoint;
					}
				}

				_manifoldCount = 1;
			}
			else
			{
				_manifoldCount = 0;
			}

			// Report removed points.
			if (listener != null && m0.PointCount > 0)
			{
				ContactPoint cp = new ContactPoint();
				cp.Shape1 = _shape1;
				cp.Shape2 = _shape2;
				cp.Normal = m0.Normal;
				for (int i = 0; i < m0.PointCount; ++i)
				{
					if (match[i])
					{
						continue;
					}

					ManifoldPoint mp0 = m0.Points[i];
					cp.Position = Common.Math.Mul(b1._xf, mp0.LocalPoint1);
					cp.Separation = mp0.Separation;
					cp.NormalForce = mp0.NormalForce;
					cp.TangentForce = mp0.TangentForce;
					cp.ID = mp0.ID;
					listener.Remove(cp);
				}
			}
		}

		new public static Contact Create(Shape shape1, Shape shape2)
		{
			return new PolygonContact(shape1, shape2);
		}

		new public static void Destroy(Contact contact)
		{
			if (contact is IDisposable)
				(contact as IDisposable).Dispose();
			contact = null;
		}
	}
}