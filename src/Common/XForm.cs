using System;
using System.Collections.Generic;
using System.Text;

namespace Box2DX.Common
{
	/// <summary>
	/// A transform contains translation and rotation.
	/// It is used to represent the position and orientation of rigid frames.
	/// </summary>
	public struct XForm
	{
		public Vector2 Position;
		public Mat22 R;

		/// <summary>
		/// Initialize using a position vector and a rotation matrix.
		/// </summary>
		/// <param name="position"></param>
		/// <param name="R"></param>
		public XForm(Vector2 position, Mat22 rotation)
		{
			Position = position;
			R = rotation;
		}

		/// <summary>
		/// Set this to the identity transform.
		/// </summary>
		public void SetIdentity()
		{
			Position.SetZero();
			R.SetIdentity();
		}

		public static XForm Identity { get { return new XForm(Vector2.Zero, Mat22.Identity); } }
	}
}
