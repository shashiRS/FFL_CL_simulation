The 'h' folder contains links to the more human readable
build folders done with a Windows directory junction.

We are doing this to reduce the path lengths during builds
so that we don't hit the Windows path length limit so
quickly.
This is why you will see only the 'h' folders in the build
logs.

If we come up with a better solution we might remove it
again so don't rely on it.
This is a linked folder (directory junction) and links
   D:\FFL_Closed_loop\mf_sil\conan_workarea\h\be22c1aef4d90e3a0b6d3e47b4a92643 with
  D:\FFL_Closed_loop\mf_sil\conan_workarea\build.mf_sil.entry.6.0.0-fallback.vs2017_debug\cip_build
