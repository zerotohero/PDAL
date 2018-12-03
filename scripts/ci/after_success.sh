#!/bin/bash

TRAVIS_BUILD_DIR=$1
KEY=$2
KEY_INIT=$3

echo "publishing website"
echo "TRAVIS_BUILD_DIR=$TRAVIS_BUILD_DIR"
echo "KEY=$KEY"
echo "KEY_INIT=$KEY_INIT"

./build_docs.sh $TRAVIS_BUILD_DIR
./add_deploy_key.sh pdaldocs-private.key $KEY $KEY_INIT
./deploy_website.sh $TRAVIS_BUILD_DIR/doc/build /tmp
