#!/bin/bash

if [ $# -eq 0 ]
then
    echo "run with ./createPkg.sh teamname"
    exit 0
fi

pkgName=$1
templatePkg="robot_ctrl"
echo "create package: $pkgName"
cp -r $templatePkg $pkgName
sed -i "s/$templatePkg/$pkgName/g" $pkgName/package.xml
sed -i "s/$templatePkg/$pkgName/g" $pkgName/CMakeLists.txt
sed -i "s/$templatePkg/$pkgName/g" $pkgName/launch/robot_cpp.launch
sed -i "s/$templatePkg/$pkgName/g" $pkgName/launch/robot_py.launch
