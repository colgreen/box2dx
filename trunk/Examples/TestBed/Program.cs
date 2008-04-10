using System;
using System.Collections.Generic;
using System.Windows.Forms;

using Tao.OpenGl;
using Tao.FreeGlut;

namespace TestBed
{
	static class Program
	{
		/// <summary>
		/// The main entry point for the application.
		/// </summary>
		[STAThread]
		static void Main()
		{
			Application.EnableVisualStyles();
			Application.SetCompatibleTextRenderingDefault(false);
			Application.Run(new MainForm());
		}
	}
}
