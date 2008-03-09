using System;
using System.Collections.Generic;
using System.Text;

namespace Box2DX.Common
{
	public class Math
	{
		public static float FLT_EPSILON = 1.192092896e-07f; //smallest such that 1.0f+FLT_EPSILON != 1.0f
		public static float FLT_MAX = 3.402823466e+38F;
		public static ushort USHRT_MAX = 0xffff;

		/// <summary>
		/// This function is used to ensure that a floating point number is
		/// not a NaN or infinity.
		/// </summary>
		/// <param name="x"></param>
		/// <returns></returns>
		public static bool IsValid(float x)
		{
			return float.IsNaN(x) || float.IsNegativeInfinity(x) || float.IsPositiveInfinity(x);
		}

		[System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Explicit)]
		public struct Convert
		{
			[System.Runtime.InteropServices.FieldOffset(0)]
			public float x;

			[System.Runtime.InteropServices.FieldOffset(0)]
			public int i;
		}

		/// <summary>
		/// This is a approximate yet fast inverse square-root.
		/// </summary>
		/// <param name="x"></param>
		/// <returns></returns>
		public static float InvSqrt(float x)
		{
			Convert convert = new Convert();
			convert.x = x;
			float xhalf = 0.5f * x;
			convert.i = 0x5f3759df - (convert.i >> 1);
			x = convert.x;
			x = x * (1.5f - xhalf * x * x);
			return x;
		}

		/// <summary>
		/// Random number in range [-1,1]
		/// </summary>
		/// <returns></returns>
		public static float Random()
		{
			float RAND_MAX = 0x7fff;
			Random rnd = new Random();
			float r = (float)rnd.NextDouble();
			r /= RAND_MAX;
			r = 2.0f * r - 1.0f;
			return r;
		}

#warning: "check perf"
		/// <summary>
		/// Random floating point number in range [lo, hi]
		/// </summary>
		/// <param name="lo"></param>
		/// <param name="hi"></param>
		/// <returns></returns>
		public static float Random(float lo, float hi)
		{
			float RAND_MAX = 0x7fff;
			Random rnd = new Random();
			float r = (float)rnd.NextDouble();
			r /= RAND_MAX;
			r = (hi - lo) * r + lo;
			return r;
		}

		/// <summary>
		/// "Next Largest Power of 2
		/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
		/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
		/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
		/// largest power of 2. For a 32-bit value:"
		/// </summary>
		/// <param name="x"></param>
		/// <returns></returns>
		public static uint NextPowerOfTwo(uint x)
		{
			x |= (x >> 1);
			x |= (x >> 2);
			x |= (x >> 4);
			x |= (x >> 8);
			x |= (x >> 16);
			return x + 1;
		}

		public static bool IsPowerOfTwo(uint x)
		{
			bool result = x > 0 && (x & (x - 1)) == 0;
			return result;
		}

		public static float Abs(float a)
		{
			return a > 0.0f ? a : -a;
		}

		public static Vector2 Abs(Vector2 a)
		{
			Vector2 b = new Vector2();
			b.Set(Math.Abs(a.X), Math.Abs(a.X));
			return b;
		}

		public static Mat22 Abs(Mat22 A)
		{
			Mat22 B = new Mat22();
			B.Set(Math.Abs(A.Col1), Math.Abs(A.Col2));
			return B;
		}

		public static float Min(float a, float b)
		{
			return a < b ? a : b;
		}

		public static int Min(int a, int b)
		{
			return a < b ? a : b;
		}

		public static Vector2 Min(Vector2 a, Vector2 b)
		{
			Vector2 c = new Vector2();
			c.X = Math.Min(a.X, b.X);
			c.Y = Math.Min(a.Y, b.Y);
			return c;
		}

		public static float Max(float a, float b)
		{
			return a > b ? a : b;
		}

		public static int Max(int a, int b)
		{
			return a > b ? a : b;
		}

		public static Vector2 Max(Vector2 a, Vector2 b)
		{
			Vector2 c = new Vector2();
			c.X = Math.Max(a.X, b.X);
			c.Y = Math.Max(a.Y, b.Y);
			return c;
		}

		public static float Clamp(float a, float low, float high)
		{
			return Math.Max(low, Math.Min(a, high));
		}

		public static int Clamp(int a, int low, int high)
		{
			return Math.Max(low, Math.Min(a, high));
		}

		public static Vector2 Clamp(Vector2 a, Vector2 low, Vector2 high)
		{
			return Math.Max(low, Math.Min(a, high));
		}

		public static void Swap<T>(ref T a, ref T b)
		{
			T tmp = a;
			a = b;
			b = tmp;
		}

		/// <summary>
		/// Multiply a matrix times a vector. If a rotation matrix is provided,
		/// then this transforms the vector from one frame to another.
		/// </summary>
		/// <param name="A"></param>
		/// <param name="v"></param>
		/// <returns></returns>
		public static Vector2 Mul(Mat22 A, Vector2 v)
		{
			Vector2 u = new Vector2();
			u.Set(A.Col1.X * v.X + A.Col2.X * v.Y, A.Col1.Y * v.X + A.Col2.Y * v.Y);
			return u;
		}

		/// <summary>
		/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
		/// then this transforms the vector from one frame to another (inverse transform).
		/// </summary>
		/// <param name="A"></param>
		/// <param name="v"></param>
		/// <returns></returns>
		public static Vector2 MulT(Mat22 A, Vector2 v)
		{
			Vector2 u = new Vector2();
			u.Set(Vector2.Dot(v, A.Col1), Vector2.Dot(v, A.Col2));
			return u;
		}

		/// <summary>
		/// A * B
		/// </summary>
		/// <param name="A"></param>
		/// <param name="B"></param>
		/// <returns></returns>
		public static Mat22 Mul(Mat22 A, Mat22 B)
		{
			Mat22 C = new Mat22();
			C.Set(Math.Mul(A, B.Col1), Math.Mul(A, B.Col2));
			return C;
		}

		/// <summary>
		/// A^T * B
		/// </summary>
		/// <param name="A"></param>
		/// <param name="B"></param>
		/// <returns></returns>
		public static Mat22 MulT(Mat22 A, Mat22 B)
		{
			Vector2 c1 = new Vector2();
			c1.Set(Vector2.Dot(A.Col1, B.Col1), Vector2.Dot(A.Col2, B.Col1));
			Vector2 c2 = new Vector2();
			c2.Set(Vector2.Dot(A.Col1, B.Col2), Vector2.Dot(A.Col2, B.Col2));
			Mat22 C = new Mat22();
			C.Set(c1, c2);
			return C;
		}

		public static Vector2 Mul(XForm T, Vector2 v)
		{
			return T.Position + Math.Mul(T.R, v);
		}

		public static Vector2 MulT(XForm T, Vector2 v)
		{
			return Math.MulT(T.R, v - T.Position);
		}
	}
}
