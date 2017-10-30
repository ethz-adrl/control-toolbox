#!/bin/bash

# force execution in bash
if [ "$(ps -p "$$" -o comm=)" != "bash" ]; then
    # Taken from http://unix-linux.questionfor.info/q_unix-linux-programming_85038.html
    bash "$0" "$@"
    exit "$?"
fi

# brings us to the script location (in ct/ct_doc/)
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR

# get branch of ct
BRANCH=$(git rev-parse --symbolic-full-name --abbrev-ref HEAD)

# go out of ct
cd ../.. || exit 1;

rm -rf adrlab.bitbucket.io
git clone git@bitbucket.org:adrlab/adrlab.bitbucket.io.git || exit 1;

cd adrlab.bitbucket.io/ct

mkdir ${BRANCH}

cd ${BRANCH} || exit 1;
rm -rf * || exit 1;

mkdir -p ct_core/doc/html || exit 1;
mkdir -p ct_doc/doc/html || exit 1;
mkdir -p ct_models/doc/html || exit 1;
mkdir -p ct_optcon/doc/html || exit 1;
mkdir -p ct_rbd/doc/html || exit 1;

pwd

ls ../../../

cp -R ../../../ct/ct_core/doc/html/* ct_core/doc/html/ || exit 1;
cp -R ../../../ct/ct_doc/doc/html/* ct_doc/doc/html/ || exit 1;
cp -R ../../../ct/ct_models/doc/html/* ct_models/doc/html/ || exit 1;
cp -R ../../../ct/ct_optcon/doc/html/* ct_optcon/doc/html/ || exit 1;
cp -R ../../../ct/ct_rbd/doc/html/* ct_rbd/doc/html/ || exit 1;

git add . || exit 1;
git commit -a -m "automatic CT doc update for branch ${BRANCH}" || exit 1;

git push origin master  || exit 1;

cd ../../.. && rm -rf adrlab.bitbucket.io

exit 0

