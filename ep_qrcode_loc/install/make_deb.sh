#!/bin/bash

CRTDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CurrentTime=$(date +"%Y%m%d%H%M")


PackageName=$(cat $CRTDIR/deb_package/DEBIAN/control  | grep Package: | awk -F: '{print $2}' | awk '{sub(/^[\t ]*|[\t ]*$/,"");print}')
echo "PackageName:"$PackageName

echo "注意：版本号、包名不能有空格和下划线，建议格式：1.0.0-2023-12-34"
read -p '请输入版本号:' Version
if [ -z $Version ]; then
    echo "未输入版本号，退出"
    exit 1
fi

read -p '请输入commit信息:' CommitInfo
if [ -z $CommitInfo ]; then
    echo "未输入commit信息，退出"
    exit 1
fi    

cd ../src/
git add .
git commit -m $Version--$CommitInfo
#git branch $Version
cd ../install

sed -i -r 's/^Version:.*/Version:'"$Version"'/g' $CRTDIR/deb_package/DEBIAN/control
#sed -i -r 's/^Build-Time:.*/Build-Time:'"$CurrentTime"'/g' $CRTDIR/deb_package/DEBIAN/control
if grep -q '^Build-Time:' "$CRTDIR/deb_package/DEBIAN/control"; then
    sed -i -E 's/^Build-Time:.*/Build-Time:'"$CurrentTime"'/g' "$CRTDIR/deb_package/DEBIAN/control"
else
    echo "Build-Time:$CurrentTime" >> "$CRTDIR/deb_package/DEBIAN/control"
fi

PackageSize=$((`du -b --max-depth=1 ${CRTDIR}/deb_package|awk 'END {print}'|awk '{print $1}'`))
sed -i -r 's/^Installed-Size:.*/Installed-Size:'"$PackageSize"'/g' $CRTDIR/deb_package/DEBIAN/control

#缺少该文件或者该文件不为空，会造成服务无法启动
touch     ./deb_package/opt/xmover/ros/melodic/ep_qrcode_loc/.catkin

sudo chmod 755 -R ./deb_package/DEBIAN
mkdir ./deb_output/
dpkg -b deb_package ./deb_output/$PackageName-$Version-$CurrentTime.deb

echo "构造完成 --> "./deb_output/$PackageName-$Version-$CurrentTime.deb

exit 0
