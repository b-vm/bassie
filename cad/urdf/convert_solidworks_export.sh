#!/bin/bash

# move files
mkdir bassie_urdf
mv bassie.SLDASM/urdf/bassie.SLDASM.urdf bassie_urdf/bassie.urdf
mv bassie.SLDASM/meshes bassie_urdf/

# fix mesh paths
sed -i 's/package:\/\/bassie\.SLDASM\/meshes\//\/meshes\//g' "bassie_urdf/bassie.urdf"

echo "Done!"