#!/bin/bash

shopt -s globstar

style="Chromium"

types=( 
    ".cpp" 
    ".hpp" 
)

folders=( 
    "../include" 
    "../src" 
    "../editor" 
    "../examples" 
)

ignored=(
    "../src/gua/renderer/pipeline/PostFXPass.cpp"
    "../src/gua/renderer/assets/NoiseTexture.cpp"
    "../src/gua/renderer/shaders/GuaMethodsFactory.cpp"
    "../src/gua/renderer/shaders/NURBSShader.cpp"
    "../src/gua/renderer/shaders/ShadowMapMeshShader.cpp"
    "../include/gua/renderer/shaders/BoundingBoxGeometry.hpp"
    "../include/gua/renderer/shaders/EmbeddedGeometries.hpp"
    "../include/gua/renderer/shaders/StereoShaders.hpp"
)

isIgnored () {
    local e
    for e in "${ignored[@]}"; do [[ "$e" == "$1" ]] && return 0; done
    return 1
}

for folder in "${folders[@]}"
do
    for type in "${types[@]}"
    do
        for file in ${folder}/**/*${type}
        do
            if [ -f $file ]
            then
                isIgnored "$file"
                
                if [ $? -ne 0 ] 
                then
                    echo "Reformatting ${file} ..."
                    clang-format -style=$style $file > /tmp/clang_tmp
                    cat /tmp/clang_tmp > $file
                else
                    echo "Ignoring ${file} ..."
                fi
            fi
        done
    done
done
