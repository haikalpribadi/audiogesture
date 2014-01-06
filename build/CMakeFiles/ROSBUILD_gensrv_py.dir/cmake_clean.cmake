FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/audiogesture/msg"
  "../src/audiogesture/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/audiogesture/srv/__init__.py"
  "../src/audiogesture/srv/_Bextractor.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
