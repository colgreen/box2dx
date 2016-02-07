# colgreen/box2dx

Box2DX is a C# port of the C++ Box2D project.

Box2DX was created by Ihar Kalasouski and originally hosted at http://code.google.com/p/box2dx

Ihar stopped contributing to the project in October 2008 shortly after releasing version 2.0.1.3-r175. Later google code shutdown although the URL is still available at time of writing (2016-02-7) for exporting the source code.

Ihar appears to have made a few commits after r175 marked with the comment "!!!Do not update to this revision, it has problems!!!". The export of the repository to this github repository contains those latest commits, therefore I have created a branch point from the last commit representing r175 (r175-stable), although it may not be exactly the right commit I think it's close enough to not have any significant differences from the r175 release.

r175-stable will remain untouched. My next step was to create branch 'resurrect' from r175-stable and check out that new branch. I have then loaded the solution into Visual Studio 2015 Update 1, updated the target framework in each C# project file to .Net 4.0 and compiled. So far so good and the test project is working as expected.

### Dependeny on Tao framework.

It should be noted that the Box2DX assembly contains a 2D physics engine only, e.g the ability to define the bounds of a world, define gravity, surfaces, objects with mass etc. There is no code in that assembly for drawing the world to screen. To render the 2D world to a window there are hook points defined, and currently the test project connects these hook points to OpenGL using the Tao framework.

The Tao framework appears to be yet another abandonned project:

 - Originally hosted at https://sourceforge.net/projects/taoframework/
 - The sourceforge page states: Superseded by OpenTK: https://sourceforge.net/projects/opentk/
 - The OpenTK main web site is live at http://www.opentk.com, but the project appears to have been maintained in recent by a different developer at https://github.com/opentk/opentk; In early 2016 the maintainer of that repository posted a README saying they are stepping down on the basis that the project owner is not available.
 
That said, the OpenTK project on github appears to be far more up-to-date than the Tao framework it replaced, therefore it is probably not a bad idea to switch Box2DX to use the latest OpenTK version available from github. I had a quick attempt at referencing OpenTK (available as a nuget package); however that appears to not be a simple drop-in replacement to Tao, so I've backed away from that for now and I'm writing this README to remind me (and to inform others) of the where I had got to in updating Box2DX.

Therefore there is still a dependency of the pre-built Tao framework binaries (included in this github repository). These are:

 - Tao.FreeGlut.dll (2.4.0.1)
 - Tao.FreeType.dll (2.3.5.0)
 - Tao.OpenGl.dll (2.1.0.4)
 - Tao.Platform.Windows.dll (1.0.0.4)
