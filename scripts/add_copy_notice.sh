#!/bin/bash

shopt -s globstar

text="/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/"

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

for folder in "${folders[@]}"
do
    for type in "${types[@]}"
    do
        for file in ${folder}/**/*${type}
        do
            if [ -f $file ]
            then
                if grep -q "uni-weimar" ${file}
                then
                    echo "Reformatting ${file} ..."
                    sed -e '1,19d' ${file} > /tmp/copyright_tmp && mv /tmp/copyright_tmp ${file}
                    echo "${text}" > /tmp/copyright_tmp
                    cat ${file} >> /tmp/copyright_tmp && mv /tmp/copyright_tmp ${file}
                fi
            fi
        done
    done
done



