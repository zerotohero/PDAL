#!/bin/bash

TRAVIS_BUILD_DIR=$1

echo "publishing website"

if [ -n $encrypted_6a5172b96922_key ]
then
    ./build_docs.sh
    # Args: keyfile, encrypted key, encrypted init vector
    ./add_deploy_key.sh pdaldocs-private.key $encrypted_6a5172b96922_key \
        $encrypted_6a5172b96922_iv
    ./deploy_website.sh $TRAVIS_BUILD_DIR/doc/build /tmp
else
    echo "No encryption key available."
fi
