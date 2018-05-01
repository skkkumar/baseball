#set the global path, so that it can be used everywhere
PATH=$PATH:/bin:/usr/bin
export PATH

#check if cmake is installed
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' cmake|grep "install ok installed")
echo Checking for cmake: $PKG_OK
if [ "" == "$PKG_OK" ]; then
  echo "No somelib. Setting up somelib."
  sudo apt-get --force-yes --yes install cmake
fi

if [ -d "build" ]; then
    echo "build already exists"
else
    echo "creating build"
    echo $(mkdir build)
fi

echo "running ccmake"
echo $(cd build && cmake ..)
echo "running make"
echo $(cd build && make)
echo "installing"
#echo $(cd build && make install)
echo "===================================="
echo "Now to run the program simply type cd build and ./BallSpeed"
echo "===================================="
echo ""
echo $(cd build && ./BallSpeed)
echo ""
echo ""
echo "===================================="
echo "Now to run the program simply type cd build and ./BallSpeed"
echo "Or ./install.sh"
echo "===================================="
