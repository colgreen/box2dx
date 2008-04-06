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

/*
This broad phase uses the Sweep and Prune algorithm as described in:
Collision Detection in Interactive 3D Environments by Gino van den Bergen
Also, some ideas, such as using integral values for fast compares comes from
Bullet (http:/www.bulletphysics.com).
*/

// Notes:
// - we use bound arrays instead of linked lists for cache coherence.
// - we use quantized integral values for fast compares.
// - we use short indices rather than pointers to save memory.
// - we use a stabbing count for fast overlap queries (less than order N).
// - we also use a time stamp on each proxy to speed up the registration of
//   overlap query results.
// - where possible, we compare bound indices instead of values to reduce
//   cache misses (TODO_ERIN).
// - no broadphase is perfect and neither is this one: it is not great for huge
//   worlds (use a multi-SAP instead), it is not great for large objects.

//#define TARGET_FLOAT32_IS_FIXED

using System;
using System.Collections.Generic;
using System.Text;

using Box2DX.Common;

namespace Box2DX.Collision
{
#warning "CAS"
	public class BoundValues
	{
		public ushort[/*2*/] LowerValues = new ushort[2];
		public ushort[/*2*/] UpperValues = new ushort[2];
	}
#warning "CAS"
	public class Bound : ICloneable
	{
		public bool IsLower { get { return (Value & (ushort)1) == (ushort)0; } }
		public bool IsUpper { get { return (Value & (ushort)1) == (ushort)1; } }

		public ushort Value;
		public ushort ProxyId;
		public ushort StabbingCount;

		public object Clone()
		{
			Bound newBound = new Bound();
			newBound.Value = this.Value;
			newBound.ProxyId = this.ProxyId;
			newBound.StabbingCount = this.StabbingCount;
			return newBound;
		}
	}
#warning "CAS"
	public class Proxy
	{
		public ushort[/*2*/] LowerBounds = new ushort[2], UpperBounds = new ushort[2];
		public ushort OverlapCount;
		public ushort TimeStamp;
		public object UserData;

		public ushort Next
		{
			get { return LowerBounds[0]; }
			set { LowerBounds[0] = value; }
		}

		public bool IsValid { get { return OverlapCount != BroadPhase.Invalid; } }
	}

	public class BroadPhase
	{
#if TARGET_FLOAT32_IS_FIXED
		public static readonly ushort BROADPHASE_MAX = (Common.Math.USHRT_MAX/2);
#else
		public static readonly ushort BROADPHASE_MAX = Common.Math.USHRT_MAX;
#endif

		public static readonly ushort Invalid = BROADPHASE_MAX;
		public static readonly ushort NullEdge = BROADPHASE_MAX;

		public PairManager _pairManager;

		public Proxy[] _proxyPool = new Proxy[Settings.MaxProxies];
		public ushort _freeProxy;

		public Bound[][] _bounds = new Bound[2][/*(2 * Settings.MaxProxies)*/];

		public ushort[] _queryResults = new ushort[Settings.MaxProxies];
		public int _queryResultCount;

		public AABB _worldAABB;
		public Vector2 _quantizationFactor;
		public int _proxyCount;
		public ushort _timeStamp;

		public static bool IsValidate = false;

		public BroadPhase(AABB worldAABB, PairCallback callback)
		{
			_pairManager = new PairManager();
			_pairManager.Initialize(this, callback);

			Box2DXDebug.Assert(worldAABB.IsValid);
			_worldAABB = worldAABB;
			_proxyCount = 0;

			Vector2 d = worldAABB.UpperBound - worldAABB.LowerBound;
			_quantizationFactor.X = (float)BROADPHASE_MAX / d.X;
			_quantizationFactor.Y = (float)BROADPHASE_MAX / d.Y;

			for (uint i = 0; i < Settings.MaxProxies - 1; ++i)
			{
				_proxyPool[i] = new Proxy();
				_proxyPool[i].Next = (ushort)(i + 1);
				_proxyPool[i].TimeStamp = 0;
				_proxyPool[i].OverlapCount = BroadPhase.Invalid;
				_proxyPool[i].UserData = null;
			}
			_proxyPool[Settings.MaxProxies - 1] = new Proxy();
			_proxyPool[Settings.MaxProxies - 1].Next = (PairManager.NullProxy);
			_proxyPool[Settings.MaxProxies - 1].TimeStamp = 0;
			_proxyPool[Settings.MaxProxies - 1].OverlapCount = BroadPhase.Invalid;
			_proxyPool[Settings.MaxProxies - 1].UserData = null;
			_freeProxy = 0;

			_timeStamp = 1;
			_queryResultCount = 0;

			for (int i = 0; i < 2; i++)
			{
				_bounds[i] = new Bound[(2 * Settings.MaxProxies)];				
			}
			for (int j = 0; j < 2; j++)
				for (int k = 0; k < (2 * Settings.MaxProxies); k++)
					_bounds[j][k] = new Bound();
		}

		// Use this to see if your proxy is in range. If it is not in range,
		// it should be destroyed. Otherwise you may get O(m^2) pairs, where m
		// is the number of proxies that are out of range.
		public bool InRange(AABB aabb)
		{
			Vector2 d = Common.Math.Max(aabb.LowerBound - _worldAABB.UpperBound, _worldAABB.LowerBound - aabb.UpperBound);
			return Common.Math.Max(d.X, d.Y) < 0.0f;
		}

		// Create and destroy proxies. These call Flush first.
		public ushort CreateProxy(AABB aabb, object userData)
		{
			Box2DXDebug.Assert(_proxyCount < Settings.MaxProxies);
			Box2DXDebug.Assert(_freeProxy != PairManager.NullProxy);

			ushort proxyId = _freeProxy;
			Proxy proxy = _proxyPool[proxyId];
			_freeProxy = proxy.Next;

			proxy.OverlapCount = 0;
			proxy.UserData = userData;

			int boundCount = 2 * _proxyCount;

			ushort[] lowerValues = new ushort[2], upperValues = new ushort[2];
			ComputeBounds(out lowerValues, out upperValues, aabb);

			for (int axis = 0; axis < 2; ++axis)
			{
				Bound[] bounds = _bounds[axis];
				int lowerIndex, upperIndex;
				Query(out lowerIndex, out upperIndex, lowerValues[axis], upperValues[axis], bounds, boundCount, axis);

#warning "Check this"
				//memmove(bounds + upperIndex + 2, bounds + upperIndex, (boundCount - upperIndex) * sizeof(b2Bound));
				//memmove(bounds + lowerIndex + 1, bounds + lowerIndex, (upperIndex - lowerIndex) * sizeof(b2Bound));
				for (int i = 0; i < (boundCount - upperIndex); i++)
				{
					bounds[upperIndex + 2 + i] = (Bound)bounds[upperIndex + i].Clone();
				}
				//Array.Copy(bounds, upperIndex, bounds, upperIndex + 2, boundCount - upperIndex);
				for (int i = 0; i < (upperIndex - lowerIndex); i++)
				{
					bounds[lowerIndex + 1 + i] = (Bound)bounds[lowerIndex + i].Clone();
				}
				//Array.Copy(bounds, lowerIndex, bounds, lowerIndex + 1, upperIndex - lowerIndex);

				// The upper index has increased because of the lower bound insertion.
				++upperIndex;

				// Copy in the new bounds.
				bounds[lowerIndex].Value = lowerValues[axis];
				bounds[lowerIndex].ProxyId = proxyId;
				bounds[upperIndex].Value = upperValues[axis];
				bounds[upperIndex].ProxyId = proxyId;

				bounds[lowerIndex].StabbingCount = lowerIndex == 0 ? (ushort)0 : bounds[lowerIndex - 1].StabbingCount;
				bounds[upperIndex].StabbingCount = bounds[upperIndex - 1].StabbingCount;

				// Adjust the stabbing count between the new bounds.
				for (int index = lowerIndex; index < upperIndex; ++index)
				{
					++bounds[index].StabbingCount;
				}

				// Adjust the all the affected bound indices.
				for (int index = lowerIndex; index < boundCount + 2; ++index)
				{
					Proxy proxy_ = _proxyPool[bounds[index].ProxyId];
					if (bounds[index].IsLower)
					{
						proxy_.LowerBounds[axis] = (ushort)index;
					}
					else
					{
						proxy_.UpperBounds[axis] = (ushort)index;
					}
				}
			}

			++_proxyCount;

			Box2DXDebug.Assert(_queryResultCount < Settings.MaxProxies);

			// Create pairs if the AABB is in range.
			for (int i = 0; i < _queryResultCount; ++i)
			{
				Box2DXDebug.Assert(_queryResults[i] < Settings.MaxProxies);
				Box2DXDebug.Assert(_proxyPool[_queryResults[i]].IsValid);

				_pairManager.AddBufferedPair(proxyId, _queryResults[i]);
			}

			_pairManager.Commit();

			if (IsValidate)
			{
				Validate();
			}

			// Prepare for next query.
			_queryResultCount = 0;
			IncrementTimeStamp();

			return proxyId;
		}

		public void DestroyProxy(int proxyId)
		{
			Box2DXDebug.Assert(0 < _proxyCount && _proxyCount <= Settings.MaxProxies);
			Proxy proxy = _proxyPool[proxyId];
			Box2DXDebug.Assert(proxy.IsValid);

			int boundCount = 2 * _proxyCount;

			for (int axis = 0; axis < 2; ++axis)
			{
				Bound[] bounds = _bounds[axis];

				int lowerIndex = proxy.LowerBounds[axis];
				int upperIndex = proxy.UpperBounds[axis];
				ushort lowerValue = bounds[lowerIndex].Value;
				ushort upperValue = bounds[upperIndex].Value;

#warning "Check this"
				//memmove(bounds + lowerIndex, bounds + lowerIndex + 1, (upperIndex - lowerIndex - 1) * sizeof(b2Bound));
				//memmove(bounds + upperIndex - 1, bounds + upperIndex + 1, (boundCount - upperIndex - 1) * sizeof(b2Bound));
				//Array.Copy(bounds, lowerIndex + 1, bounds, lowerIndex, upperIndex - lowerIndex - 1);
				for (int i = 0; i < (upperIndex - lowerIndex - 1); i++)
				{
					bounds[lowerIndex + i] = (Bound)bounds[lowerIndex + 1 + i].Clone();
				}
				//Array.Copy(bounds, upperIndex + 1, bounds, upperIndex - 1, boundCount - upperIndex - 1);
				for (int i = 0; i < (boundCount - upperIndex - 1); i++)
				{
					bounds[upperIndex - 1 + i] = (Bound)bounds[upperIndex + 1 + i].Clone();
				}

				// Fix bound indices.
				for (int index = lowerIndex; index < boundCount - 2; ++index)
				{
					Proxy proxy_ = _proxyPool[bounds[index].ProxyId];
					if (bounds[index].IsLower)
					{
						proxy_.LowerBounds[axis] = (ushort)index;
					}
					else
					{
						proxy_.UpperBounds[axis] = (ushort)index;
					}
				}

				// Fix stabbing count.
				for (int index = lowerIndex; index < upperIndex - 1; ++index)
				{
					--bounds[index].StabbingCount;
				}

				// Query for pairs to be removed. lowerIndex and upperIndex are not needed.
				Query(out lowerIndex, out upperIndex, lowerValue, upperValue, bounds, boundCount - 2, axis);
			}

			Box2DXDebug.Assert(_queryResultCount < Settings.MaxProxies);

			for (int i = 0; i < _queryResultCount; ++i)
			{
				Box2DXDebug.Assert(_proxyPool[_queryResults[i]].IsValid);
				_pairManager.RemoveBufferedPair(proxyId, _queryResults[i]);
			}

			_pairManager.Commit();

			// Prepare for next query.
			_queryResultCount = 0;
			IncrementTimeStamp();

			// Return the proxy to the pool.
			proxy.UserData = null;
			proxy.OverlapCount = BroadPhase.Invalid;
			proxy.LowerBounds[0] = BroadPhase.Invalid;
			proxy.LowerBounds[1] = BroadPhase.Invalid;
			proxy.UpperBounds[0] = BroadPhase.Invalid;
			proxy.UpperBounds[1] = BroadPhase.Invalid;

			proxy.Next = _freeProxy;
			_freeProxy = (ushort)proxyId;
			--_proxyCount;

			if (IsValidate)
			{
				Validate();
			}
		}

		// Call MoveProxy as many times as you like, then when you are done
		// call Commit to finalized the proxy pairs (for your time step).
		public void MoveProxy(int proxyId, AABB aabb)
		{
			if (proxyId == PairManager.NullProxy || Settings.MaxProxies <= proxyId)
			{
				Box2DXDebug.Assert(false);
				return;
			}

			if (aabb.IsValid == false)
			{
				Box2DXDebug.Assert(false);
				return;
			}

			int boundCount = 2 * _proxyCount;

			Proxy proxy = _proxyPool[proxyId];

			// Get new bound values
			BoundValues newValues = new BoundValues(); ;
			ComputeBounds(out newValues.LowerValues, out newValues.UpperValues, aabb);

			// Get old bound values
			BoundValues oldValues = new BoundValues();
			for (int axis = 0; axis < 2; ++axis)
			{
				oldValues.LowerValues[axis] = _bounds[axis][proxy.LowerBounds[axis]].Value;
				oldValues.UpperValues[axis] = _bounds[axis][proxy.UpperBounds[axis]].Value;
			}

			for (int axis = 0; axis < 2; ++axis)
			{
				Bound[] bounds = _bounds[axis];

				int lowerIndex = proxy.LowerBounds[axis];
				int upperIndex = proxy.UpperBounds[axis];

				ushort lowerValue = newValues.LowerValues[axis];
				ushort upperValue = newValues.UpperValues[axis];

				int deltaLower = lowerValue - bounds[lowerIndex].Value;
				int deltaUpper = upperValue - bounds[upperIndex].Value;

				bounds[lowerIndex].Value = lowerValue;
				bounds[upperIndex].Value = upperValue;

				//
				// Expanding adds overlaps
				//

				// Should we move the lower bound down?
				if (deltaLower < 0)
				{
					int index = lowerIndex;
					while (index > 0 && lowerValue < bounds[index - 1].Value)
					{
						Bound bound = bounds[index];
						Bound prevBound = bounds[index - 1];

						int prevProxyId = prevBound.ProxyId;
						Proxy prevProxy = _proxyPool[prevBound.ProxyId];

						++prevBound.StabbingCount;

						if (prevBound.IsUpper == true)
						{
							if (TestOverlap(newValues, prevProxy))
							{
								_pairManager.AddBufferedPair(proxyId, prevProxyId);
							}

							++prevProxy.UpperBounds[axis];
							++bound.StabbingCount;
						}
						else
						{
							++prevProxy.LowerBounds[axis];
							--bound.StabbingCount;
						}

						--proxy.LowerBounds[axis];
						//Common.Math.Swap<Bound>(ref bound, ref prevBound);
						Common.Math.Swap<Bound>(ref bounds[index], ref bounds[index - 1]);
						--index;
					}
				}

				// Should we move the upper bound up?
				if (deltaUpper > 0)
				{
					int index = upperIndex;
					while (index < boundCount - 1 && bounds[index + 1].Value <= upperValue)
					{
						Bound bound = bounds[index];
						Bound nextBound = bounds[index + 1];
						int nextProxyId = nextBound.ProxyId;
						Proxy nextProxy = _proxyPool[nextProxyId];

						++nextBound.StabbingCount;

						if (nextBound.IsLower == true)
						{
							if (TestOverlap(newValues, nextProxy))
							{
								_pairManager.AddBufferedPair(proxyId, nextProxyId);
							}

							--nextProxy.LowerBounds[axis];
							++bound.StabbingCount;
						}
						else
						{
							--nextProxy.UpperBounds[axis];
							--bound.StabbingCount;
						}

						++proxy.UpperBounds[axis];
						//Common.Math.Swap<Bound>(ref bound, ref nextBound);
						Common.Math.Swap<Bound>(ref bounds[index], ref bounds[index + 1]);
						++index;
					}
				}

				//
				// Shrinking removes overlaps
				//

				// Should we move the lower bound up?
				if (deltaLower > 0)
				{
					int index = lowerIndex;
					while (index < boundCount - 1 && bounds[index + 1].Value <= lowerValue)
					{
						Bound bound = bounds[index];
						Bound nextBound = bounds[index + 1];

						int nextProxyId = nextBound.ProxyId;
						Proxy nextProxy = _proxyPool[nextProxyId];

						--nextBound.StabbingCount;

						if (nextBound.IsUpper)
						{
							if (TestOverlap(oldValues, nextProxy))
							{
								_pairManager.RemoveBufferedPair(proxyId, nextProxyId);
							}

							--nextProxy.UpperBounds[axis];
							--bound.StabbingCount;
						}
						else
						{
							--nextProxy.LowerBounds[axis];
							++bound.StabbingCount;
						}

						++proxy.LowerBounds[axis];
						//Common.Math.Swap<Bound>(ref bound, ref nextBound);
						Common.Math.Swap<Bound>(ref bounds[index], ref bounds[index + 1]);
						++index;
					}
				}

				// Should we move the upper bound down?
				if (deltaUpper < 0)
				{
					int index = upperIndex;
					while (index > 0 && upperValue < bounds[index - 1].Value)
					{
						Bound bound = bounds[index];
						Bound prevBound = bounds[index - 1];

						int prevProxyId = prevBound.ProxyId;
						Proxy prevProxy = _proxyPool[prevProxyId];

						--prevBound.StabbingCount;

						if (prevBound.IsLower == true)
						{
							if (TestOverlap(oldValues, prevProxy))
							{
								_pairManager.RemoveBufferedPair(proxyId, prevProxyId);
							}

							++prevProxy.LowerBounds[axis];
							--bound.StabbingCount;
						}
						else
						{
							++prevProxy.UpperBounds[axis];
							++bound.StabbingCount;
						}

						--proxy.UpperBounds[axis];
						//Common.Math.Swap<Bound>(ref bound, ref prevBound);
						Common.Math.Swap<Bound>(ref bounds[index], ref bounds[index - 1]);
						--index;
					}
				}
			}

			if (IsValidate)
			{
				Validate();
			}
		}

		public void Commit()
		{
			_pairManager.Commit();
		}

		// Get a single proxy. Returns NULL if the id is invalid.
		public Proxy GetProxy(int proxyId)
		{
			if (proxyId == PairManager.NullProxy || _proxyPool[proxyId].IsValid == false)
			{
				return null;
			}

			return _proxyPool[proxyId];
		}

		// Query an AABB for overlapping proxies, returns the user data and
		// the count, up to the supplied maximum count.
		public int Query(AABB aabb, object[] userData, int maxCount)
		{
			ushort[] lowerValues;
			ushort[] upperValues;
			ComputeBounds(out lowerValues, out upperValues, aabb);

			int lowerIndex, upperIndex;

			Query(out lowerIndex, out upperIndex, lowerValues[0], upperValues[0], _bounds[0], 2 * _proxyCount, 0);
			Query(out lowerIndex, out upperIndex, lowerValues[1], upperValues[1], _bounds[1], 2 * _proxyCount, 1);

			Box2DXDebug.Assert(_queryResultCount < Settings.MaxProxies);

			int count = 0;
			for (int i = 0; i < _queryResultCount && count < maxCount; ++i, ++count)
			{
				Box2DXDebug.Assert(_queryResults[i] < Settings.MaxProxies);
				Proxy proxy = _proxyPool[_queryResults[i]];
				Box2DXDebug.Assert(proxy.IsValid);
				userData[i] = proxy.UserData;
			}

			// Prepare for next query.
			_queryResultCount = 0;
			IncrementTimeStamp();

			return count;
		}

		public void Validate()
		{
			for (int axis = 0; axis < 2; ++axis)
			{
				Bound[] bounds = _bounds[axis];

				int boundCount = 2 * _proxyCount;
				ushort stabbingCount = 0;

				for (int i = 0; i < boundCount; ++i)
				{
					Bound bound = bounds[i];
					Box2DXDebug.Assert(i == 0 || bounds[i - 1].Value <= bound.Value);
					Box2DXDebug.Assert(bound.ProxyId != PairManager.NullProxy);
					Box2DXDebug.Assert(_proxyPool[bound.ProxyId].IsValid);

					if (bound.IsLower == true)
					{
						Box2DXDebug.Assert(_proxyPool[bound.ProxyId].LowerBounds[axis] == i);
						++stabbingCount;
					}
					else
					{
						Box2DXDebug.Assert(_proxyPool[bound.ProxyId].UpperBounds[axis] == i);
						--stabbingCount;
					}

					Box2DXDebug.Assert(bound.StabbingCount == stabbingCount);
				}
			}
		}

		private void ComputeBounds(out ushort[] lowerValues, out ushort[] upperValues, AABB aabb)
		{
			lowerValues = new ushort[2];
			upperValues = new ushort[2];

			Box2DXDebug.Assert(aabb.UpperBound.X > aabb.LowerBound.X);
			Box2DXDebug.Assert(aabb.UpperBound.Y > aabb.LowerBound.Y);

			Vector2 minVertex = Common.Math.Clamp(aabb.LowerBound, _worldAABB.LowerBound, _worldAABB.UpperBound);
			Vector2 maxVertex = Common.Math.Clamp(aabb.UpperBound, _worldAABB.LowerBound, _worldAABB.UpperBound);

			// Bump lower bounds downs and upper bounds up. This ensures correct sorting of
			// lower/upper bounds that would have equal values.
			// TODO_ERIN implement fast float to uint16 conversion.
			lowerValues[0] = (ushort)((ushort)(_quantizationFactor.X * (minVertex.X - _worldAABB.LowerBound.X)) & (ushort)(BROADPHASE_MAX - 1));
			upperValues[0] = (ushort)((ushort)(_quantizationFactor.X * (maxVertex.X - _worldAABB.LowerBound.X)) | (ushort)1);

			lowerValues[1] = (ushort)((ushort)(_quantizationFactor.Y * (minVertex.Y - _worldAABB.LowerBound.Y)) & (ushort)(BROADPHASE_MAX - 1));
			upperValues[1] = (ushort)((ushort)(_quantizationFactor.Y * (maxVertex.Y - _worldAABB.LowerBound.Y)) | (ushort)1);
		}

		// This one is only used for validation.
		internal bool TestOverlap(Proxy p1, Proxy p2)
		{
			for (int axis = 0; axis < 2; ++axis)
			{
				Bound[] bounds = _bounds[axis];

				Box2DXDebug.Assert(p1.LowerBounds[axis] < 2 * _proxyCount);
				Box2DXDebug.Assert(p1.UpperBounds[axis] < 2 * _proxyCount);
				Box2DXDebug.Assert(p2.LowerBounds[axis] < 2 * _proxyCount);
				Box2DXDebug.Assert(p2.UpperBounds[axis] < 2 * _proxyCount);

				if (bounds[p1.LowerBounds[axis]].Value > bounds[p2.UpperBounds[axis]].Value)
					return false;

				if (bounds[p1.UpperBounds[axis]].Value < bounds[p2.LowerBounds[axis]].Value)
					return false;
			}

			return true;
		}

		internal bool TestOverlap(BoundValues b, Proxy p)
		{
			for (int axis = 0; axis < 2; ++axis)
			{
				Bound[] bounds = _bounds[axis];

				Box2DXDebug.Assert(p.LowerBounds[axis] < 2 * _proxyCount);
				Box2DXDebug.Assert(p.UpperBounds[axis] < 2 * _proxyCount);

				if (b.LowerValues[axis] > bounds[p.UpperBounds[axis]].Value)
					return false;

				if (b.UpperValues[axis] < bounds[p.LowerBounds[axis]].Value)
					return false;
			}

			return true;
		}

		private void Query(out int lowerQueryOut, out int upperQueryOut,
					   ushort lowerValue, ushort upperValue,
					   Bound[] bounds, int boundCount, int axis)
		{
			int lowerQuery = BinarySearch(bounds, boundCount, lowerValue);
			int upperQuery = BinarySearch(bounds, boundCount, upperValue);

			// Easy case: lowerQuery <= lowerIndex(i) < upperQuery
			// Solution: search query range for min bounds.
			for (int i = lowerQuery; i < upperQuery; ++i)
			{
				if (bounds[i].IsLower)
				{
					IncrementOverlapCount(bounds[i].ProxyId);
				}
			}

			// Hard case: lowerIndex(i) < lowerQuery < upperIndex(i)
			// Solution: use the stabbing count to search down the bound array.
			if (lowerQuery > 0)
			{
				int i = lowerQuery - 1;
				int s = bounds[i].StabbingCount;

				// Find the s overlaps.
				while (s != 0)
				{
					Box2DXDebug.Assert(i >= 0);

					if (bounds[i].IsLower)
					{
						Proxy proxy = _proxyPool[bounds[i].ProxyId];
						if (lowerQuery <= proxy.UpperBounds[axis])
						{
							IncrementOverlapCount(bounds[i].ProxyId);
							--s;
						}
					}
					--i;
				}
			}

			lowerQueryOut = lowerQuery;
			upperQueryOut = upperQuery;
		}

		private void IncrementOverlapCount(int proxyId)
		{
			Proxy proxy = _proxyPool[proxyId];
			if (proxy.TimeStamp < _timeStamp)
			{
				proxy.TimeStamp = _timeStamp;
				proxy.OverlapCount = 1;
			}
			else
			{
				proxy.OverlapCount = 2;
				Box2DXDebug.Assert(_queryResultCount < Settings.MaxProxies);
				_queryResults[_queryResultCount] = (ushort)proxyId;
				++_queryResultCount;
			}
		}

		private void IncrementTimeStamp()
		{
			if (_timeStamp == BROADPHASE_MAX)
			{
				for (ushort i = 0; i < Settings.MaxProxies; ++i)
				{
					_proxyPool[i].TimeStamp = 0;
				}
				_timeStamp = 1;
			}
			else
			{
				++_timeStamp;
			}
		}

		private static int BinarySearch(Bound[] bounds, int count, ushort value)
		{
			int low = 0;
			int high = count - 1;
			while (low <= high)
			{
				int mid = (low + high) >> 1;
				if (bounds[mid].Value > value)
				{
					high = mid - 1;
				}
				else if (bounds[mid].Value < value)
				{
					low = mid + 1;
				}
				else
				{
					return (ushort)mid;
				}
			}

			return low;
		}
	}
}
