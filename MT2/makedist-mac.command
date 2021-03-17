#! /bin/sh

#shell script to automate Project build, code-signing and packaging on macos

BASEDIR=$(dirname $0)
cd $BASEDIR

#---------------------------------------------------------------------------------------------------------

#variables

VERSION=`echo | grep "define JucePlugin_Version " JuceLibraryCode/JucePluginDefines.h`
VERSION=${VERSION/\#/}
VERSION=${VERSION/define/}
VERSION=${VERSION/JucePlugin_Version/}

FULL_VERSION=$(echo "${VERSION}" | tr -d '[:space:]')

PLUGIN_NAME=`echo | grep "define JucePlugin_Name " JuceLibraryCode/JucePluginDefines.h`
PLUGIN_NAME=${PLUGIN_NAME/\#/}
PLUGIN_NAME=${PLUGIN_NAME/define/}
PLUGIN_NAME=${PLUGIN_NAME/JucePlugin_Name/}
PLUGIN_NAME=${PLUGIN_NAME//\"}
PLUGIN_NAME=$(echo "${PLUGIN_NAME}" | tr -d '[:space:]')

# work out the paths to the binaries

PKG="installer/build-mac/$PLUGIN_NAME Installer.pkg"

echo "making $PLUGIN_NAME version $FULL_VERSION mac distribution..."
echo ""

#---------------------------------------------------------------------------------------------------------

#call python script to update version numbers
./update_version.py

#here you can use the touch command to force xcode to rebuild

#---------------------------------------------------------------------------------------------------------

# build xcode project. Change target to build individual formats
echo "Build"
xcodebuild -project Builds/MacOSX/$PLUGIN_NAME.xcodeproj -target "$PLUGIN_NAME - All" -configuration Release 2> ./build-mac.log

if [ -s build-mac.log ]
then
  echo "build failed due to following errors:"
  echo ""
  cat build-mac.log
  exit 1
else
 rm build-mac.log
fi

#---------------------------------------------------------------------------------------------------------

# installer, uses Packages http://s.sudre.free.fr/Software/Packages/about.html
rm -R -f installer/$PLUGIN_NAME-mac.dmg

echo "building installer"
echo ""
chmod 0777 installer
packagesbuild installer/$PLUGIN_NAME.pkgproj

#---------------------------------------------------------------------------------------------------------

echo "done"
