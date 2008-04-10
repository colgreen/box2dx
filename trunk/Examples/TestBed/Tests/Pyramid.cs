using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;
using Box2DX.Collision;
using Box2DX.Dynamics;

namespace TestBed
{
	public class Pyramid : Test
	{
		public Pyramid()
		{
			{
				PolygonDef sd = new PolygonDef();
				sd.SetAsBox(50.0f, 10.0f);

				BodyDef bd = new BodyDef();
				bd.Position.Set(0.0f, -10.0f);
				Body ground = _world.CreateStaticBody(bd);
				ground.CreateShape(sd);
			}

			{
				PolygonDef sd = new PolygonDef();
				float a = 0.5f;
				sd.SetAsBox(a, a);
				sd.Density = 5.0f;

				Vector2 x = new Vector2(-10.0f, 0.75f);
				Vector2 y;
				Vector2 deltaX = new Vector2(0.5625f, 2.0f);
				Vector2 deltaY = new Vector2(1.125f, 0.0f);

				for (int i = 0; i < 25; ++i)
				{
					y = x;

					for (int j = i; j < 25; ++j)
					{
						BodyDef bd = new BodyDef();
						bd.Position = y;
						Body body = _world.CreateDynamicBody(bd);
						body.CreateShape(sd);
						body.SetMassFromShapes();

						y += deltaY;
					}

					x += deltaX;
				}
			}
		}

		public static Test Create()
		{
			return new Pyramid();
		}
	}
}
