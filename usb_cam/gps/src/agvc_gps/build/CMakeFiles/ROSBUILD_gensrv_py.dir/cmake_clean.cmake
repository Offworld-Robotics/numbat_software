FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/agvc_gps/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/agvc_gps/srv/__init__.py"
  "../src/agvc_gps/srv/_ConvertGPSOrigin.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
