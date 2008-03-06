using System;
using System.Collections.Generic;
using System.Text;

namespace Box2DX.Common
{
	public class Settings
	{
		public static readonly float Pi = 3.14159265359f;

        // Global tuning constants based on meters-kilograms-seconds (MKS) units.

		// Collision
		public static readonly int MaxManifoldPoints = 2;
		public static readonly int MaxPolygonVertices = 8;
		public static readonly int MaxProxies = 512; // this must be a power of two
		public static readonly int MaxPairs = 8 * MaxProxies; // this must be a power of two

		// Dynamics

		/// <summary>
		/// A small length used as a collision and constraint tolerance. Usually it is
		/// chosen to be numerically significant, but visually insignificant.
		/// </summary>
		public static readonly float LinearSlop = 0.005f;	// 0.5 cm

		/// <summary>
		/// A small angle used as a collision and constraint tolerance. Usually it is
		/// chosen to be numerically significant, but visually insignificant.
		/// </summary>
		public static readonly float AngularSlop = 2.0f / 180.0f * Pi; // 2 degrees

		/// <summary>
		/// Continuous collision detection (CCD) works with core, shrunken shapes. This is amount
		/// by which shapes are automatically shrunk to work with CCD. 
		/// This must be larger than LinearSlop.
		/// </summary>
		public static readonly float ToiSlop = 8.0f * LinearSlop;

		/// <summary>
		/// Maximum number of contacts to be handled to solve a TOI island.
		/// </summary>
		public static readonly int MaxTOIContactsPerIsland = 32;

		/// <summary>
		/// A velocity threshold for elastic collisions. Any collision with a relative linear
		/// velocity below this threshold will be treated as inelastic.
		/// </summary>
		public static readonly float VelocityThreshold = 1.0f; // 1 m/s

		/// <summary>
		/// The maximum linear position correction used when solving constraints.
		/// This helps to prevent overshoot.
		/// </summary>
		public static readonly float MaxLinearCorrection = 0.2f; // 20 cm

		/// <summary>
		/// The maximum angular position correction used when solving constraints.
		/// This helps to prevent overshoot.
		/// </summary>
		public static readonly float MaxAngularCorrection = 8.0f / 180.0f * Pi; // 8 degrees

		/// <summary>
		/// The maximum linear velocity of a body. This limit is very large and is used
		/// to prevent numerical problems. You shouldn't need to adjust this.
		/// </summary>
		public static readonly float MaxLinearVelocity = 200.0f;
		public static readonly float MaxLinearVelocitySquared = MaxLinearVelocity * MaxLinearVelocity;

		/// <summary>
		/// The maximum angular velocity of a body. This limit is very large and is used
		/// to prevent numerical problems. You shouldn't need to adjust this.
		/// </summary>
		public static readonly float MaxAngularVelocity = 250.0f;
		public static readonly float MaxAngularVelocitySquared = MaxAngularVelocity * MaxAngularVelocity;

		/// <summary>
		/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
		/// that overlap is removed in one time step. However using values close to 1 often lead to overshoot.
		/// </summary>
		public static readonly float ContactBaumgarte = 0.2f;

		// Sleep

		/// <summary>
		/// The time that a body must be still before it will go to sleep.
		/// </summary>
		public static readonly float TimeToSleep = 0.5f; // half a second

		/// <summary>
		/// A body cannot sleep if its linear velocity is above this tolerance.
		/// </summary>
		public static readonly float LinearSleepTolerance = 0.01f; // 1 cm/s

		/// <summary>
		/// A body cannot sleep if its angular velocity is above this tolerance.
		/// </summary>
		public static readonly float AngularSleepTolerance = 2.0f / 180.0f; // 2 degrees/s
	}
}
