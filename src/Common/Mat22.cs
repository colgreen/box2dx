using System;
using System.Collections.Generic;
using System.Text;

namespace Box2DX.Common
{
	/// <summary>
	/// A 2-by-2 matrix. Stored in column-major order.
	/// </summary>
	public struct Mat22
	{
		public Vector2 Col1, Col2;

		/// <summary>
		/// Construct this matrix using columns.
		/// </summary>
		/// <param name="c1"></param>
		/// <param name="c2"></param>
		public Mat22(Vector2 c1, Vector2 c2)
		{
			Col1 = c1;
			Col2 = c2;
		}

		/// <summary>
		/// Construct this matrix using scalars.
		/// </summary>
		/// <param name="a11"></param>
		/// <param name="a12"></param>
		/// <param name="a21"></param>
		/// <param name="a22"></param>
		public Mat22(float a11, float a12, float a21, float a22)
		{
			Col1.X = a11; Col1.Y = a21;
			Col2.X = a12; Col2.Y = a22;
		}

		/// <summary>
		/// Construct this matrix using an angle. 
		/// This matrix becomes an orthonormal rotation matrix.
		/// </summary>
		/// <param name="angle"></param>
		public Mat22(float angle)
		{
			float c = (float)System.Math.Cos(angle), s = (float)System.Math.Sin(angle);
			Col1.X = c; Col2.X = -s;
			Col1.Y = s; Col2.Y = c;
		}

		/// <summary>
		/// Initialize this matrix using columns.
		/// </summary>
		/// <param name="c1"></param>
		/// <param name="c2"></param>
		public void Set(Vector2 c1, Vector2 c2)
		{
			Col1 = c1;
			Col2 = c2;
		}

		/// <summary>
		/// Initialize this matrix using an angle.
		/// This matrix becomes an orthonormal rotation matrix.
		/// </summary>
		/// <param name="angle"></param>
		public void Set(float angle)
		{
			float c = (float)System.Math.Cos(angle), s = (float)System.Math.Sin(angle);
			Col1.X = c; Col2.X = -s;
			Col1.Y = s; Col2.Y = c;
		}

		/// <summary>
		/// Set this to the identity matrix.
		/// </summary>
		public void SetIdentity()
		{
			Col1.X = 1.0f; Col2.X = 0.0f;
			Col1.Y = 0.0f; Col2.Y = 1.0f;
		}

		/// <summary>
		/// Set this matrix to all zeros.
		/// </summary>
		public void SetZero()
		{
			Col1.X = 0.0f; Col2.X = 0.0f;
			Col1.Y = 0.0f; Col2.Y = 0.0f;
		}

		/// <summary>
		/// Extract the angle from this matrix (assumed to be a rotation matrix).
		/// </summary>
		/// <returns></returns>
		public float GetAngle()
		{
			return (float)System.Math.Atan2(Col1.Y, Col1.X);
		}

		/// <summary>
		/// Compute the inverse of this matrix, such that inv(A) * A = identity.
		/// </summary>
		/// <returns></returns>
		public Mat22 Invert()
		{
			float a = Col1.X, b = Col2.X, c = Col1.Y, d = Col2.Y;
			Mat22 B = new Mat22();
			float det = a * d - b * c;
			Box2DXDebug.Assert(det != 0.0f);
			det = 1.0f / det;
			B.Col1.X = det * d; B.Col2.X = -det * b;
			B.Col1.Y = -det * c; B.Col2.Y = det * a;
			return B;
		}

		/// <summary>
		/// Solve A * x = b, where b is a column vector. This is more efficient
		/// than computing the inverse in one-shot cases.
		/// </summary>
		/// <param name="b"></param>
		/// <returns></returns>
		public Vector2 Solve(Vector2 b)
		{
			float a11 = Col1.X, a12 = Col2.X, a21 = Col1.Y, a22 = Col2.Y;
			float det = a11 * a22 - a12 * a21;
			Box2DXDebug.Assert(det != 0.0f);
			det = 1.0f / det;
			Vector2 x = new Vector2();
			x.X = det * (a22 * b.X - a12 * b.Y);
			x.Y = det * (a11 * b.Y - a21 * b.X);
			return x;
		}

		public static Mat22 Identity { get { return new Mat22(1, 0, 0, 1); } }

		public static Mat22 operator +(Mat22 A, Mat22 B)
		{
			Mat22 C = new Mat22();
			C.Set(A.Col1 + B.Col1, A.Col2 + B.Col2);
			return C;
		}
	}
}
