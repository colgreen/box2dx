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

namespace Box2DX.Common
{
	/// <summary>
	/// A 2D column vector.
	/// </summary>
	public struct Vector2
	{
		public float X, Y;

		/// <summary>
		/// Construct using coordinates.
		/// </summary>
		/// <param name="x"></param>
		/// <param name="y"></param>
		public Vector2(float x, float y)
		{
			X = x;
			Y = y;
		}

		/// <summary>
		/// Set this vector to all zeros.
		/// </summary>
		public void SetZero() { X = 0.0f; Y = 0.0f; }

		/// <summary>
		/// Set this vector to some specified coordinates.
		/// </summary>
		/// <param name="x_"></param>
		/// <param name="y_"></param>
		public void Set(float x_, float y_) { X = x_; Y = y_; }

		/// <summary>
		///  Get the length of this vector (the norm).
		/// </summary>
		/// <returns></returns>
		public float Length()
		{
			return (float)System.Math.Sqrt(X * X + Y * Y);
		}

		/// Get the length squared. For performance, use this instead of
		/// Length (if possible).
		public float LengthSquared()
		{
			return X * X + Y * Y;
		}

		/// <summary>
		/// Convert this vector into a unit vector. Returns the length.
		/// </summary>
		public float Normalize()
		{
			float length = Length();
			if (length < Math.FLT_EPSILON)
			{
				return 0.0f;
			}
			float invLength = 1.0f / length;
			X *= invLength;
			Y *= invLength;

			return length;
		}

		/// <summary>
		/// Does this vector contain finite coordinates?
		/// </summary>
		/// <returns></returns>
		public bool IsValid
		{
			get { return Math.IsValid(X) && Math.IsValid(Y); }
		}

		/// <summary>
		/// Negate this vector.
		/// </summary>
		/// <param name="v"></param>
		/// <returns></returns>
		public static Vector2 operator -(Vector2 v1)
		{
			Vector2 v = new Vector2();
			v.Set(-v1.X, -v1.Y);
			return v;
		}

		public static Vector2 operator +(Vector2 v1, Vector2 v2)
		{
			Vector2 v = new Vector2();
			v.Set(v1.X + v2.X, v1.Y + v2.Y);
			return v;
		}

		public static Vector2 operator -(Vector2 v1, Vector2 v2)
		{
			Vector2 v = new Vector2();
			v.Set(v1.X - v2.X, v1.Y - v2.Y);
			return v;
		}

		public static Vector2 operator *(Vector2 v1, float a)
		{
			Vector2 v = new Vector2();
			v.Set(v1.X * a, v1.Y * a);
			return v;
		}

		public static Vector2 operator *(float a, Vector2 v1)
		{
			Vector2 v = new Vector2();
			v.Set(v1.X * a, v1.Y * a);
			return v;
		}

		public static bool operator ==(Vector2 a, Vector2 b)
		{
			return a.X == b.X && a.Y == b.Y;
		}

		public static bool operator !=(Vector2 a, Vector2 b)
		{
			return a.X != b.X && a.Y != b.Y;
		}

		public static Vector2 Zero { get { return new Vector2(0, 0); } }

		/// <summary>
		/// Peform the dot product on two vectors.
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		public static float Dot(Vector2 a, Vector2 b)
		{
			return a.X * b.X + a.Y * b.Y;
		}

		/// <summary>
		/// Perform the cross product on two vectors. In 2D this produces a scalar.
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		public static float Cross(Vector2 a, Vector2 b)
		{
			return a.X * b.Y - a.Y * b.X;
		}

		/// <summary>
		/// Perform the cross product on a vector and a scalar. 
		/// In 2D this produces a vector.
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		public static Vector2 Cross(Vector2 a, float s)
		{
			Vector2 v = new Vector2();
			v.Set(s * a.Y, -s * a.X);
			return v;
		}

		/// <summary>
		/// Perform the cross product on a scalar and a vector. 
		/// In 2D this produces a vector.
		/// </summary>
		/// <param name="s"></param>
		/// <param name="a"></param>
		/// <returns></returns>
		public static Vector2 Cross(float s, Vector2 a)
		{
			Vector2 v = new Vector2();
			v.Set(-s * a.Y, s * a.X);
			return v;
		}

		public static float Distance(Vector2 a, Vector2 b)
		{
			Vector2 c = a - b;
			return c.Length();
		}

		public static float DistanceSquared(Vector2 a, Vector2 b)
		{
			Vector2 c = a - b;
			return Vector2.Dot(c, c);
		}
	}
}
