using System;
using System.Text;
using System.Diagnostics;
using System.Collections.Generic;

namespace Box2DX
{
	internal static class Box2DXDebug
	{
		[Conditional("DEBUG")]
		public static void Assert(bool condition)
		{
			Debug.Assert(condition);
		}

		[Conditional("DEBUG")]
		public static void Assert(bool condition, string message)
		{
			Debug.Assert(condition, message);
		}

		[Conditional("DEBUG")]
		public static void Assert(bool condition, string message, string detailMessage)
		{
			Debug.Assert(condition, message, detailMessage);
		}

		private static void Throw(String message)
		{
			string msg = String.Format("Error: {0}", message);
			throw new Exception(msg);
		}
	}
}
