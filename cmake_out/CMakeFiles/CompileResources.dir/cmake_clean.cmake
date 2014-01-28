FILE(REMOVE_RECURSE
  "CMakeFiles/CompileResources"
  "../src/gua/generated/R.inl"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/CompileResources.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
