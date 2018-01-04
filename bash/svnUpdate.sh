#!/bin/bash

for d in */ ; do
    echo "$d"
    cd "$d"
    svn st
    cd ..
done
