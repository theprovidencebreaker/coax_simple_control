FILE(REMOVE_RECURSE
  "../src/coax_simple_control/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/coax_simple_control/srv/__init__.py"
  "../src/coax_simple_control/srv/_SetNavMode.py"
  "../src/coax_simple_control/srv/_SetControlMode.py"
  "../src/coax_simple_control/srv/_SetWaypoint.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
