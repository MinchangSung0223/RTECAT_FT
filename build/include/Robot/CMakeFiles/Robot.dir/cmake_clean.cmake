file(REMOVE_RECURSE
  "libRobot.pdb"
  "libRobot.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/Robot.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
