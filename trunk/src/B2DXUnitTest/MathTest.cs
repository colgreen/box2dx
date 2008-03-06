using Box2DX.Common;
using Microsoft.VisualStudio.TestTools.UnitTesting;
namespace B2DXUnitTest
{
    
    
    /// <summary>
    ///This is a test class for MathTest and is intended
    ///to contain all MathTest Unit Tests
    ///</summary>
	[TestClass()]
	public class MathTest
	{


		private TestContext testContextInstance;

		/// <summary>
		///Gets or sets the test context which provides
		///information about and functionality for the current test run.
		///</summary>
		public TestContext TestContext
		{
			get
			{
				return testContextInstance;
			}
			set
			{
				testContextInstance = value;
			}
		}

		#region Additional test attributes
		// 
		//You can use the following additional attributes as you write your tests:
		//
		//Use ClassInitialize to run code before running the first test in the class
		//[ClassInitialize()]
		//public static void MyClassInitialize(TestContext testContext)
		//{
		//}
		//
		//Use ClassCleanup to run code after all tests in a class have run
		//[ClassCleanup()]
		//public static void MyClassCleanup()
		//{
		//}
		//
		//Use TestInitialize to run code before running each test
		//[TestInitialize()]
		//public void MyTestInitialize()
		//{
		//}
		//
		//Use TestCleanup to run code after each test has run
		//[TestCleanup()]
		//public void MyTestCleanup()
		//{
		//}
		//
		#endregion


		/// <summary>
		///A test for InvSqrt
		///</summary>
		[TestMethod()]
		public void InvSqrtTest()
		{
			float x = 10f; // TODO: Initialize to an appropriate value
			float expected = 0.316f; // TODO: Initialize to an appropriate value
			float actual;
			actual = Math.InvSqrt(x);
			Assert.AreEqual(expected, actual);
			Assert.Inconclusive("Verify the correctness of this test method.");
		}

        /// <summary>
        ///A test for Random
        ///</summary>
        [TestMethod()]
        public void Random1Test()
        {
            float a1 = -1;
            float a2 = 1;
            float actual;
            actual = Math.Random();
            if (!(actual > a1 && actual < a2))
                Assert.Fail(actual.ToString());
        }

        /// <summary>
        ///A test for Random
        ///</summary>
        [TestMethod()]
        public void Random2Test()
        {
            float a1 = 45;
            float a2 = 50;
            float actual;
            actual = Math.Random(a1,a2);
            if (!(actual > a1 && actual < a2))
                Assert.Fail(actual.ToString());
        }

        /// <summary>
        ///A test for IsPowerOfTwo
        ///</summary>
        [TestMethod()]
        public void IsPowerOfTwoTest()
        {
            uint x = 4;
            Assert.IsTrue(Math.IsPowerOfTwo(x));
            x = 3;
            Assert.IsFalse(Math.IsPowerOfTwo(x));
        }
	}
}
