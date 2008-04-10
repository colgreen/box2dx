using System;
using System.Collections.Generic;
using System.Text;

using Tao.OpenGl;
using Tao.FreeGlut;

using Box2DX.Common;
using Box2DX.Collision;
using Box2DX.Dynamics;

namespace TestBed
{
	using Box2DXMath = Box2DX.Common.Math;
	using SysMath = System.Math;

	// This class implements debug drawing callbacks that are invoked
	// inside World.Step.
	public class OpenGLDebugDraw : DebugDraw
	{
		public override void DrawPolygon(Vector2[] vertices, int vertexCount, Color color)
		{
			Gl.glColor3f(color.R, color.G, color.B);
			Gl.glBegin(Gl.GL_LINE_LOOP);
			for (int i = 0; i < vertexCount; ++i)
			{
				Gl.glVertex2f(vertices[i].X, vertices[i].Y);
			}
			Gl.glEnd();			
		}

		public override void DrawSolidPolygon(Vector2[] vertices, int vertexCount, Color color)
		{
			Gl.glEnable(Gl.GL_BLEND);
			Gl.glBlendFunc(Gl.GL_SRC_ALPHA, Gl.GL_ONE_MINUS_SRC_ALPHA);
			Gl.glColor4f(0.5f * color.R, 0.5f * color.G, 0.5f * color.B, 0.5f);
			Gl.glBegin(Gl.GL_TRIANGLE_FAN);
			for (int i = 0; i < vertexCount; ++i)
			{
				Gl.glVertex2f(vertices[i].X, vertices[i].Y);
			}
			Gl.glEnd();
			Gl.glDisable(Gl.GL_BLEND);

			Gl.glColor4f(color.R, color.G, color.B, 1.0f);
			Gl.glBegin(Gl.GL_LINE_LOOP);
			for (int i = 0; i < vertexCount; ++i)
			{
				Gl.glVertex2f(vertices[i].X, vertices[i].Y);
			}
			Gl.glEnd();
		}

		public override void DrawCircle(Vector2 center, float radius, Color color)
		{
			float k_segments = 16.0f;
			float k_increment = 2.0f * Box2DX.Common.Settings.Pi / k_segments;
			float theta = 0.0f;
			Gl.glColor3f(color.R, color.G, color.B);
			Gl.glBegin(Gl.GL_LINE_LOOP);
			for (int i = 0; i < k_segments; ++i)
			{
				Vector2 v = center + radius * new Vector2((float)SysMath.Cos(theta), (float)SysMath.Sin(theta));
				Gl.glVertex2f(v.X, v.Y);
				theta += k_increment;
			}
			Gl.glEnd();
		}

		public override void DrawSolidCircle(Vector2 center, float radius, Vector2 axis, Color color)
		{
			float k_segments = 16.0f;
			float k_increment = 2.0f * Box2DX.Common.Settings.Pi / k_segments;
			float theta = 0.0f;
			Gl.glEnable(Gl.GL_BLEND);
			Gl.glBlendFunc(Gl.GL_SRC_ALPHA, Gl.GL_ONE_MINUS_SRC_ALPHA);
			Gl.glColor4f(0.5f * color.R, 0.5f * color.G, 0.5f * color.B, 0.5f);
			Gl.glBegin(Gl.GL_TRIANGLE_FAN);
			for (int i = 0; i < k_segments; ++i)
			{
				Vector2 v = center + radius * new Vector2((float)SysMath.Cos(theta), (float)SysMath.Sin(theta));
				Gl.glVertex2f(v.X, v.Y);
				theta += k_increment;
			}
			Gl.glEnd();
			Gl.glDisable(Gl.GL_BLEND);

			theta = 0.0f;
			Gl.glColor4f(color.R, color.G, color.B, 1.0f);
			Gl.glBegin(Gl.GL_LINE_LOOP);
			for (int i = 0; i < k_segments; ++i)
			{
				Vector2 v = center + radius * new Vector2((float)SysMath.Cos(theta), (float)SysMath.Sin(theta));
				Gl.glVertex2f(v.X, v.Y);
				theta += k_increment;
			}
			Gl.glEnd();

			Vector2 p = center + radius * axis;
			Gl.glBegin(Gl.GL_LINES);
			Gl.glVertex2f(center.X, center.Y);
			Gl.glVertex2f(p.X, p.Y);
			Gl.glEnd();
		}

		public override void DrawSegment(Vector2 p1, Vector2 p2, Color color)
		{
			Gl.glColor3f(color.R, color.G, color.B);
			Gl.glBegin(Gl.GL_LINES);
			Gl.glVertex2f(p1.X, p1.Y);
			Gl.glVertex2f(p2.X, p2.Y);
			Gl.glEnd();
		}

		public override void DrawXForm(XForm xf)
		{
			Vector2 p1 = xf.Position, p2;
			float k_axisScale = 0.4f;
			Gl.glBegin(Gl.GL_LINES);

			Gl.glColor3f(1.0f, 0.0f, 0.0f);
			Gl.glVertex2f(p1.X, p1.Y);
			p2 = p1 + k_axisScale * xf.R.Col1;
			Gl.glVertex2f(p2.X, p2.Y);

			Gl.glColor3f(0.0f, 1.0f, 0.0f);
			Gl.glVertex2f(p1.X, p1.Y);
			p2 = p1 + k_axisScale * xf.R.Col2;
			Gl.glVertex2f(p2.X, p2.Y);

			Gl.glEnd();
		}

		public static void DrawSegment(Vector2 p1, Vector2 p2, Color color, params object[] fake)
		{
			Gl.glColor3f(color.R, color.G, color.B);
			Gl.glBegin(Gl.GL_LINES);
			Gl.glVertex2f(p1.X, p1.Y);
			Gl.glVertex2f(p2.X, p2.Y);
			Gl.glEnd();
		}

		public static void DrawPoint(Vector2 p, float size, Color color)
		{
			Gl.glPointSize(size);
			Gl.glBegin(Gl.GL_POINTS);
			Gl.glColor3f(color.R, color.G, color.B);
			Gl.glVertex2f(p.X, p.Y);
			Gl.glEnd();
			Gl.glPointSize(1.0f);
		}

		public static void DrawString(int x, int y, string str)
		{
			/*System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();
			byte[] buffer = encoding.GetBytes(str);

			Gl.glMatrixMode(Gl.GL_PROJECTION);
			Gl.glPushMatrix();
			Gl.glLoadIdentity();
			//int w = Glut.glutGet(Glut.GLUT_WINDOW_WIDTH);
			//int h = Glut.glutGet(Glut.GLUT_WINDOW_HEIGHT);
			int w = 640;
			int h = 480;
			Glu.gluOrtho2D(0, w, h, 0);
			Gl.glMatrixMode(Gl.GL_MODELVIEW);
			Gl.glPushMatrix();
			Gl.glLoadIdentity();

			Gl.glColor3f(0.9f, 0.6f, 0.6f);
			Gl.glRasterPos2i(x, y);
			int length = buffer.Length;
			for (int i = 0; i < length; ++i)
			{
				Glut.glutBitmapCharacter(Glut.GLUT_BITMAP_8_BY_13, buffer[i]);
				//Glut.glutBitmapString(Glut.GLUT_BITMAP_TIMES_ROMAN_10, str);
			}

			Gl.glPopMatrix();
			Gl.glMatrixMode(Gl.GL_PROJECTION);
			Gl.glPopMatrix();
			Gl.glMatrixMode(Gl.GL_MODELVIEW);*/
		}

		public static void DrawAABB(AABB aabb, Color c)
		{
			Gl.glColor3f(c.R, c.G, c.B);
			Gl.glBegin(Gl.GL_LINE_LOOP);
			Gl.glVertex2f(aabb.LowerBound.X, aabb.LowerBound.Y);
			Gl.glVertex2f(aabb.UpperBound.X, aabb.LowerBound.Y);
			Gl.glVertex2f(aabb.UpperBound.X, aabb.UpperBound.Y);
			Gl.glVertex2f(aabb.LowerBound.X, aabb.UpperBound.Y);
			Gl.glEnd();
		}
	}
}
