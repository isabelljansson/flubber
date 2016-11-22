# A Maya flubber plugin
A plugin for Maya that deforms an object like flubber

Based on the tutorials by Chad Vernon:
* http://www.chadvernon.com/blog/resources/maya-api-programming/your-first-plug-in/
* http://www.chadvernon.com/blog/maya/compiling-maya-plug-ins-with-cmake/

And this tutorial for deformer plugins:
* http://www.ngreen.org/articles/implementing-a-simple-maya-python-deformer.html

And this paper about shape matching:
* https://www.cs.drexel.edu/~david/Classes/Papers/MeshlessDeformations_SIG05.pdf

---

Build on OSX:
  * mkdir build
  * cd build
  * cmake -G "Unix Makefiles" -DMAYA_VERSION=2017 ../ (the version you will use)

Compile on OSX:
  * cd in to build/ directory
  * sudo cmake --build . --config Release --target install

For OSX, the compiler will build the file sampleplugin.bundle,
  * Open Maya and go to Window -> Settings/Preferences -> Plug-in Manager
  * Browse for sampleplugin.bundle and open.

Now the Flubber function can be run from the python terminal by:
  * import maya.cmds as cmds
  * cmds.Flubber()
