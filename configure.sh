year=$1
if (($#==0)); then
    echo "Must specify a year"
    exit 1
fi
if (($#>=2)); then
    namespace=$2
else
    namespace="robot$year"
fi
srcDir="src/main/java/org/team1540"
mv $srcDir/robotTemplate $srcDir/$namespace
for file in $srcDir/$namespace/*.java; do
    perl -pi -e "s/package org.team1540.robotTemplate/package org.team1540.$namespace/" $file
done
files=(
    build.gradle
    settings.gradle
)

for file in $files; do
    echo $file
    perl -pi -e "s/TEMPLATEYEAR/$year/" $file
    perl -pi -e "s/org.team1540.robotTemplate/org.team1540.$namespace/" $file
done

cd $(dirname "$0")
mkdir vendordeps
cd vendordeps
curl -SO https://software-metadata.revrobotics.com/REVLib.json
curl -SO https://www.kauailabs.com/dist/frc/$year/navx_frc.json
curl -SO https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc$year-latest.json