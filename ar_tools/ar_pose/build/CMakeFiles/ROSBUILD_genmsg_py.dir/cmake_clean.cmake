FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/ar_pose/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ar_pose/msg/__init__.py"
  "../src/ar_pose/msg/_ARMarker.py"
  "../src/ar_pose/msg/_ARMarkers.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
