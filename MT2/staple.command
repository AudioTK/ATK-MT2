#! /bin/sh

#shell script to automate notarizing

BASEDIR=$(dirname $0)
cd $BASEDIR

#---------------------------------------------------------------------------------------------------------

#variables
PLUGIN_NAME=`echo | grep "define JucePlugin_Name " JuceLibraryCode/JucePluginDefines.h`
PLUGIN_NAME=${PLUGIN_NAME/\#/}
PLUGIN_NAME=${PLUGIN_NAME/define/}
PLUGIN_NAME=${PLUGIN_NAME/JucePlugin_Name/}
PLUGIN_NAME=${PLUGIN_NAME//\"}
PLUGIN_NAME=$(echo "${PLUGIN_NAME}" | tr -d '[:space:]')

PKG="installer/build-mac-signed/$PLUGIN_NAME Installer.pkg"

arch -x86_64 xcrun stapler staple "${PKG}"
spctl -a -t install --context context:primary-signature -v "${PKG}"

echo "done"
