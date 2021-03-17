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

# work out the paths to the binaries

PKG="installer/build-mac/$PLUGIN_NAME Installer.pkg"

xcrun altool --notarize-app -f "${PKG}" --primary-bundle-id com.MatthieuBrucher.$PLUGIN_NAME.pkg -u "matthieu.brucher@gmail.com" -p "@keychain:AC_PASSWORD"

echo "done"
