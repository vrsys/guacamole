#!/bin/bash

dirlist=`ls $1*.gua_trimesh`
./example-virtual_texturing $dirlist $2

