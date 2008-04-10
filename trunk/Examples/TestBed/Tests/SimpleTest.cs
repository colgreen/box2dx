using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;
using Box2DX.Collision;
using Box2DX.Dynamics;

namespace TestBed
{
	public class SimpleTest : Test
	{
		public SimpleTest()
		{
			// Define the ground body.
			BodyDef groundBodyDef = new BodyDef();
			groundBodyDef.Position.Set(0.0f, -10.0f);

			// Call the body factory which creates the ground box shape.
			// The body is also added to the world.
			Body groundBody = _world.CreateStaticBody(groundBodyDef);

			// Define the ground box shape.
			PolygonDef groundShapeDef = new PolygonDef();

			// The extents are the half-widths of the box.
			groundShapeDef.SetAsBox(50.0f, 10.0f);

			// Add the ground shape to the ground body.
			groundBody.CreateShape(groundShapeDef);

			for (int i = 0; i < 1; i++)
			{
				// Define the dynamic body. We set its position and call the body factory.
				BodyDef bodyDef = new BodyDef();
				bodyDef.Position.Set(0.0f, 4.0f *(i+1));
				Body body = _world.CreateDynamicBody(bodyDef);

				// Define another box shape for our dynamic body.
				PolygonDef shapeDef = new PolygonDef();
				shapeDef.SetAsBox(1.0f, 1.0f);

				// Set the box density to be non-zero, so it will be dynamic.
				shapeDef.Density = 1.0f;

				// Override the default friction.
				shapeDef.Friction = 0.3f;

				// Add the shape to the body.
				body.CreateShape(shapeDef);

				// Now tell the dynamic body to compute it's mass properties base
				// on its shape.
				body.SetMassFromShapes();
			}
		}

		public static Test Create()
		{
			return new SimpleTest();
		}
	}
}
