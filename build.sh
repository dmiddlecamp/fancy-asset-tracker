mkdir temp_build

rm `basename $1`.bin
rm temp_build/*.*
cp $1/*.* temp_build

cp gps-library/*.h temp_build
cp gps-library/*.cpp temp_build

particle compile electron temp_build --saveTo `basename $1`.bin
