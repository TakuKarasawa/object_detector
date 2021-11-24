#!/bin/bash

# commit comment
if [ $# -ne 0 ]; then
    comment="$*"
else
    comment="$(date +'%Y:%m:%d-%H:%M:%S')"
fi

# branch
if [ $# -ne 1 ]; then
    branch="$*"
else
    branch="master"
fi

# management
git add .
git commit -m "$comment"
git push origin "$branch"
