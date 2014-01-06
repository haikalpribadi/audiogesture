FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/audiogesture/msg"
  "../src/audiogesture/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/audiogesture/msg/__init__.py"
  "../src/audiogesture/msg/_Strings.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
